"""Minimalistic usage example for sick_scan_api

Usage: minimum_sick_scan_api_client.py launchfile

Example:
    export LD_LIBRARY_PATH=.:./build:$LD_LIBRARY_PATH
    export PYTHONPATH=.:./python/api:$PYTHONPATH
    python3 ./examples/python/minimum_sick_scan_api_client.py ./launch/sick_tim_7xx.launch

See doc/sick_scan_api/sick_scan_api.md for further information.

"""

import os
import sys
import time
import ctypes
import numpy as np

# Make sure sick_scan_api is searched in all folders configured in environment variable PYTHONPATH
from sick_scan_api import *

def pyCustomizedPointCloudMsgCb(api_handle, msg):
    """
    Implement a callback to process pointcloud messages
    Data processing to be done
    """
    print("Python PointCloudMsgCb: {} x {} pointcloud message received".format(msg.contents.width, msg.contents.height))
    print("Python PointCloudMsgCb: {} pointstep".format(msg.contents.point_step))
    #print("Python PointCloudMsgCb: {} fields".format(msg.contents.fields))
    #print ("msg.contents.data.buffer cap", msg.contents.data.capacity)
    #print ("msg.contents.data.buffer size", msg.contents.data.size)
    #print ("msg.contents.data.buffer.content bytes", bytes(msg.contents.data.buffer.contents))
    #print ("msg.contents.data.buffer.content value bytes", bytes(msg.contents.data.buffer.contents.value))
    #array_type = ctypes.c_uint8 * msg.contents.data.size
    #array = ctypes.cast(msg.contents.data.buffer, array_type)
    #python_bytes = bytes(array.contents)
    #print ("msg.contents.data.buffer.content bytes", msg.contents.data.buffer[:msg.contents.data.size])
    array = ctypes.cast(msg.contents.data.buffer, ctypes.POINTER(ctypes.c_float))
    #print ("msg.contents.data.buffer.content bytes", len(bytes(msg.contents.data.buffer[:msg.contents.data.size])))
    #print("array len", len(array[:int(msg.contents.data.size/4)]))
    version = 'VERSION .7\n'
    fields = 'FIELDS x y z\n'
    size = 'SIZE 4 4 4\n'
    type_of = 'TYPE F F F\n'
    count = 'COUNT 1 1 1\n'
    height = 'HEIGHT 1\n'
    viewpoint = 'VIEWPOINT 0 0 0 1 0 0 0\n'
    data = 'DATA binary\n'
    pdata = []
    for point in range(msg.contents.data.size):
        x = array[point*4]
        y = array[point*4+1]
        z = array[point*4+2]
        pdata.append(x)
        pdata.append(y)
        pdata.append(z)

    width = f'WIDTH {len(pdata)}\n'
    points = f'POINTS {len(pdata)}\n'
    header = f'{version}{fields}{size}{type_of}{count}{width}{height}{viewpoint}{points}{data}'
    a = np.array(pdata, dtype='f')
    h = bytes(header, 'UTF-8')

    print("pcd", h+a.tobytes())

# Pass launchfile and commandline arguments to sick_scan_library
cli_args = " ".join(sys.argv[1:])

# Load sick_scan_library
sick_scan_library = SickScanApiLoadLibrary(["build/"], "libsick_scan_shared_lib.so")

# Create a sick_scan instance and initialize a TiM-5xx
api_handle = SickScanApiCreate(sick_scan_library)
SickScanApiInitByLaunchfile(sick_scan_library, api_handle, cli_args)

# Register for pointcloud messages
cartesian_pointcloud_callback = SickScanPointCloudMsgCallback(pyCustomizedPointCloudMsgCb)
SickScanApiRegisterCartesianPointCloudMsg(sick_scan_library, api_handle, cartesian_pointcloud_callback)

# Run application or main loop
time.sleep(10)

# Close lidar and release sick_scan api
SickScanApiDeregisterCartesianPointCloudMsg(sick_scan_library, api_handle, cartesian_pointcloud_callback)
SickScanApiClose(sick_scan_library, api_handle)
SickScanApiRelease(sick_scan_library, api_handle)
SickScanApiUnloadLibrary(sick_scan_library)
