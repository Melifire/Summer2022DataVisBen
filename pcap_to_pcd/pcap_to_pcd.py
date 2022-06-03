'''
file: pcap_to_pcd.py
author: Ben Kruse
version: 1.0
date: 6/2/2022
'''

import sys
import os
import subprocess
import time


def pcap_to_bag(pcap_file_name: str, number_of_sensors: int) -> None:
    ''' Generates a bag file containing the pcap data to use in other steps

    Inputs:
    str pcap_file_name: the path of the pcap_file
    int number of sensors: the sensor count of the lidar data

    Output:
    returns: str: the path of the new bag
    Generates a bag file with the same name as the pcap in the ./temp directory
    '''
    base_pcap_name = os.path.basename(pcap_file_name)  # Gets the base name of the pcap (without the path)
    output_bag_file = os.path.abspath(f"./temp/{base_pcap_name[:-4]}bag")
    command = f'./pcaptorosbag_ws/devel/lib/pcap_to_bag/pcap_to_bag {os.path.abspath(pcap_file_name)} {output_bag_file} {number_of_sensors}'
    print(command)  # To show the command
    subprocess.run(command, shell=True)  # Runs the command in the shell
    return output_bag_file


def pcap_to_pointcloud(pcap_bag_name, json_file_name):
    ''' Retrieves the pointcloud data in the form of a bag from a pcap bag
    Inputs:
    str pcap_bag_name: the path to the pcap_bag
    str json_file_name: the path to the json_file

    Outputs:
    returns: str: the path of the new bag
    Generates a bag file named {name_of_base_bag}-pc.bag in the temp directory
    '''
    print("Starting roscore and ouster ros")
    roscore_proc = subprocess.Popen(f"roslaunch ouster_ros ouster.launch replay:=true metadata:={os.path.abspath(json_file_name)}", shell=True)
    time.sleep(10)
    
    print("Starting recording")
    recording_proc = subprocess.Popen(f"rosbag record /os_cloud_node/points -O {pcap_bag_name[:-4]}-pc __name:=recording_proc", shell=True)
    time.sleep(10)

    print("Starting playback")
    subprocess.run(f"rosbag play {pcap_bag_name}", shell=True)
    time.sleep(2)

    subprocess.run("rosnode kill /recording_proc", shell=True)
    
    return roscore_proc


def bag_to_pcd(pc_bag_name):
    ''' Creates pcd files from a bag of pointcloud2 points
    Inputs:
    str pc_bag_name: The path to the pointclound2 bag

    Outputs:
    returns None
    Creates a directory in temp called {pcap_file_name}-pcds containing all the pcd's with the timestamp as their name
    '''
    new_folder_name = os.path.basename(pc_bag_name)[:-4] + "-pcds"
    subprocess.run(f"mkdir {new_folder_name}", shell=True)
    subprocess.run(f"rosrun pcl_ros bag_to_pcd {os.path.abspath(pc_bag_name)} /os_cloud_node/points ./temp/{new_folder_name}", shell=True)
    


def main():
    ''' Command line arguments:
        pcap file name
        json file name
        number of sensors (16, 32, 64, 128)
            This is usually part of the pcap name
    '''
    '''
    if len(sys.argv) < 4:
            print("Must have pcap name, json file, and number of sensors as arguments")
            return 1
    pcap_file_name = sys.argv[1]
    json_file_name = sys.argv[2]
    number_of_sensors = sys.argv[3]
    
    # Source ros setup file (. is used for the shell python uses to source)
    subprocess.run('source /opt/ros/noetic/setup.sh', shell=True)

    # Converting the pcap file to a bag file
    pcap_bag_name = pcap_to_bag(pcap_file_name, number_of_sensors)

    # Converting the pcap bag to a pointcloud2 bag
    roscore_proc = pcap_to_pointcloud(pcap_bag_name, json_file_name)
    roscore_proc.terminate()
    subprocess.run("rostopic list", shell=True)
    '''

    bag_to_pcd("./temp/demo-pc.bag")

    print("It worked!")
    



if __name__ == "__main__":
    main()
