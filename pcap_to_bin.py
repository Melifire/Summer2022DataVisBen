'''
Still needs comments and some cleaning up
Run with "python pcap_to_bin.py <pcap_path> <json_path>"
'''
import sys
from ouster import client, pcap
from tqdm import tqdm
import struct

def main():
	if len(sys.argv) != 3:
		return 1
	_, pcap_path, metadata_path = sys.argv
	
	with open(metadata_path, 'r') as f:
		info = client.SensorInfo(f.read())
	source = pcap.Pcap(pcap_path, info)	
	
	scans = client.Scans(source)
	xyzlut = client.XYZLut(info)
 
	test_file = open("test.bin", "wb")
	count = 0
	point_count = 0
	for i,scan in enumerate(tqdm(scans)):
		arr = xyzlut(scan.field(client.ChanField.RANGE))
		#print(arr.shape)
		for w in arr:
			for h in w:
				point_count += 1
				for val in h:
					#print(val)
					fl = struct.pack("f", val)
					test_file.write(fl)
		count += 1
	print(count)
	print(point_count)
	test_file.close()
	

	
 


if __name__=="__main__":
	main()
