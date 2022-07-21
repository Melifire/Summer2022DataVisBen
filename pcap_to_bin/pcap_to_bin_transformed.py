from pyexpat.errors import XML_ERROR_TAG_MISMATCH
import sys
import time
from ouster import client, pcap
import struct
import matplotlib.pyplot as plt
from scipy import interpolate
from scipy.spatial.transform import Rotation as R
import numpy as np
from tqdm import tqdm
import threading
import multiprocessing as mp


def integrate(data, start = 0):
    out = []
    for point in data:
        start += point
        out.append(start)
    return out

def integrate_v3(data, timestamps, start=[0,0,0]):
    assert len(data) > 0
    assert len(data[0]) == 3
    out = []
    for i, point in enumerate(data):
        if timestamps[i] == 0:
            out.append(start[:])
            continue
        start[0] += point[0]*timestamps[i]
        start[1] += point[1]*timestamps[i]
        start[2] += point[2]*timestamps[i]
        out.append(start[:])
    return out

def plot (list, x_label, y_label):
    plt.plot(list)
    plt.xlabel(x_label)
    plt.ylabel(y_label)
    plt.show()

def read_imu_packets(source: pcap.Pcap):
    ''' Reads the imu packets to generate a transform function for the data
    
    Inputs:
    source: pcap.Pcap - The pcap object to be read

    Outputs 3 functions (x, y, z) that transform points according to imu data
    '''
    accelerations = []
    a_vels = []
    timestamps = []
    timedeltas = []
    start_time = 0
    base_rot = None
    for packet in source: 
        if isinstance(packet, client.ImuPacket):
            # and access ImuPacket content
            #print(f'  acceleration = {packet.accel}')
            #print(f'  angular_velocity = {packet.angular_vel}')
            accelerations.append(packet.accel)
            if base_rot is None:
                base_rot = packet.angular_vel
            a_vels.append(packet.angular_vel-base_rot)
            if start_time == 0:
                start_time = packet.gyro_ts
            timedeltas.append((packet.gyro_ts-start_time)/(1000*1000*1000))
            timestamps.append(packet.gyro_ts)
            start_time = packet.gyro_ts
    integrated = np.asarray(integrate_v3(a_vels, timedeltas))
    integrated = integrated.T
    x_func = interpolate.interp1d(timestamps, integrated[0], fill_value='extrapolate')
    y_func = interpolate.interp1d(timestamps, integrated[1], fill_value='extrapolate')
    z_func = interpolate.interp1d(timestamps, integrated[2], fill_value='extrapolate')
    #plot(integrated, "packets", "data")
    return (x_func, y_func, z_func)
    
    
def main():
    mp.set_start_method("fork")
    if len(sys.argv) != 3:
        return 1
    _, pcap_path, metadata_path = sys.argv
    
    with open(metadata_path, 'r') as f:
        info = client.SensorInfo(f.read())
    source = pcap.Pcap(pcap_path, info)	
    
    xfunc, yfunc, zfunc = read_imu_packets(source)

    source = pcap.Pcap(pcap_path, info)
    scans = client.Scans(source)
    xyzlut = client.XYZLut(info)
    
    test_file = open("out.bin", "wb")

    all_inputs = []
    for i, scan in enumerate(scans):
        inputs = [scan, i, xyzlut, xfunc, yfunc, zfunc]
        all_inputs.append(inputs)
    
    with mp.Pool() as p:
        p.starmap(write_frame, all_inputs)

    test_file.close()

def write_frame(scan, frame_number, xyzlut, xfunc, yfunc, zfunc):
    print(f"Starting frame {frame_number}")
    out_file = open('temp/{:0>7}.bin'.format(frame_number), 'wb')

    arr = xyzlut(scan.field(client.ChanField.RANGE))
    timestamp = scan.timestamp
    #print(arr.shape)
    
    for i, w in enumerate(arr):
        row_scan_time = timestamp[i]
        x_rot = xfunc(row_scan_time)
        y_rot = yfunc(row_scan_time)
        z_rot = zfunc(row_scan_time)
        transformer = R.from_euler("xyz", [x_rot, y_rot, z_rot], degrees=True)
        for h in w:               
            x, y, z = transformer.apply(h)
            fl = struct.pack("f", x)
            out_file.write(fl)
            fl = struct.pack("f", y)
            out_file.write(fl)
            fl = struct.pack("f", z)
            out_file.write(fl)

    print(f"Done with {frame_number}")
    out_file.close()




if __name__=="__main__":
    main()
    #bs = bytes([1,2,3])
    #test_file = open("test.bin", "wb")
    #test_file.write(bs)
    #test_file.close()
    
    
