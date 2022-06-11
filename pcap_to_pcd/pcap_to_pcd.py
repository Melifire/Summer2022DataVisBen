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
from multiprocessing import Pool, freeze_support, Process

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
    

def batch_get_file_names(input_folder):
    '''
    '''
    onlyfiles = [f for f in os.listdir(input_folder) if os.path.isfile(input_folder + f)]
    pcaps = [os.path.splitext(f)[0] for f in onlyfiles if os.path.splitext(f)[1] == ".pcap"]
    for pcap in pcaps:
        if not os.path.exists(input_folder + pcap + ".json"):
            print(f"pcap file {pcap}.pcap does not have a matching json")
            return None
    return pcaps


def run_batch_conversions(input_folder, file_names, sensor_count):
    '''
    '''
    file_count = len(file_names)
    file_tuples = []
    for filename in file_names:
        pcap_name = input_folder + filename + ".pcap"
        json_name = input_folder + filename + ".json"
        file_tuples.append((pcap_name, json_name, sensor_count))
    if __name__ == "__main__":
        pool = Pool() 
    print("this worked")

    pool.starmap(run_conversion_test, file_tuples) 

    pool.join()


def run_conversion_test(pcap_file_name, json_file_name, number_of_sensors):
    print(f"Running conversions for {pcap_file_name}, {json_file_name}, {number_of_sensors}")
    time.sleep(5)
    print(f"Conversions done for {pcap_file_name}")

def run_conversion(pcap_file_name, json_file_name, number_of_sensors):
    '''
    '''
    # Converting the pcap file to a bag file
    pcap_bag_name = pcap_to_bag(pcap_file_name, number_of_sensors)

    # Converting the pcap bag to a pointcloud2 bag
    roscore_proc = pcap_to_pointcloud(pcap_bag_name, json_file_name)
    roscore_proc.terminate()

    bag_to_pcd("./temp/demo-pc.bag")


def main():
    ''' Command line arguments:
        
        For single file conversions
        single
        pcap file name
        json file name
        number of sensors (16, 32, 64, 128)
            This is usually part of the pcap name

        For many different files
        batch
        directory containing all pcaps and jsons. Each pair must have the same name before the extensions
        number of sensors (16, 32, 64, 128)
            This is usually part of the pcap name
    '''
    conversion_mode = sys.argv[1]  # Either 'single' or 'batch' for the mode
    
    if conversion_mode == 'single':       
        if len(sys.argv) < 5:
                print("Must have appropriate arguments")
                return 1
        pcap_file_name = sys.argv[2]
        json_file_name = sys.argv[3]
        number_of_sensors = sys.argv[4]
        
        run_conversion(pcap_file_name, json_file_name, number_of_sensors)

        print("Conversions done!")

    elif conversion_mode == 'batch':
        if len(sys.argv) < 4:
                print("Must have appropriate arguments")
                return 1
        input_folder = sys.argv[2]
        number_of_sensors = sys.argv[3]

        # Gets a list of file names (not including extensions). If there is anything wrong, will return Null.
        file_names = batch_get_file_names(input_folder)
        if file_names == None:
            print("File name error, exiting")

        run_batch_conversions(input_folder, file_names, number_of_sensors)

    else:
        print("Please select a conversion mode as your first parameter (single/batch)")

    



if __name__ == "__main__":
    freeze_support()
    main()
