import sys
import ctypes
import numpy as np
from sick_scan_api import *

#TODO, big question is still how to share msg between cb and class, given cb's fixed format
def pyCustomizedPointCloudMsgCb(api_handle, msg):
    """
    Implement a callback to process pointcloud messages
    Data processing to be done
    """
    version = 'VERSION .7\n'
    fields = 'FIELDS x y z\n'
    size = 'SIZE 4 4 4\n'
    type_of = 'TYPE F F F\n'
    count = 'COUNT 1 1 1\n'
    height = 'HEIGHT 1\n'
    viewpoint = 'VIEWPOINT 0 0 0 1 0 0 0\n'
    data = 'DATA binary\n'
    pdata = []
    array = ctypes.cast(msg.contents.data.buffer, ctypes.POINTER(ctypes.c_float))
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
    #TODO name this function get_point_cloud, return h + a.tobytes(), CameraMimeType.PCD

#TODO parse via config arg rather than argv. here to __del__() should be new()
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


#TODO below here should be __del__()
# Close lidar and release sick_scan api
SickScanApiDeregisterCartesianPointCloudMsg(sick_scan_library, api_handle, cartesian_pointcloud_callback)
SickScanApiClose(sick_scan_library, api_handle)
SickScanApiRelease(sick_scan_library, api_handle)
SickScanApiUnloadLibrary(sick_scan_library)
