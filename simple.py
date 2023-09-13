import sick_scan_api.py

# our callbacks
def polar_callback(msg):
    print("polar", msg)

def cartesian_callback(msg):
    print("cart", msg)

# Set up lib and init via launchfile
sick_scan_lib = SickScanApiLoadLibrary(["build/", "./"], "libsick_scan_shared_lib.so")
api_handle = SickScanApiCreate(sick_scan_lib)
#TODO: confirm launch file for our lidar
SickScanApiInitByLaunchfile(sick_scan_lib, api_handle, "sick_lms_5xx.launch")

# turn call backs into call backs and registry them
cart_pointcloud_callback = SickScanPointCloudMsgCallback(cartesian_callback)
SickScanApiRegisterCartesianPointCloudMsg(sick_scan_lib, api_handle, cart_pointcloud_callback)

polar_pointcloud_callback = SickScanPointCloudMsgCallback(polar_callback)
SickScanApiRegisterPolarPointCloudMsg(sick_scan_lib, api_handle, polar_pointcloud_callback)
