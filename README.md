# sick-scan-viam
A simple module for SICK scan lidar units.
## How does this module work?
This module wraps the library produced from https://github.com/SICKAG/sick_scan_xd in order to return point clouds to viam-server.
After initializing the library with parameters provide via the component config, the module registers a callback for cartesian point clouds.
The callback simply stores the data to be returned by the component when called. It caches N responses, where N is the value of the `segments` parameter.

## Compatability
When this module is uploaded to the registry, we include a `build` directory with the compiled C library in it.
This library is architecture specific and we've seen more success with amd64 based machines (ex, intel NUC) than with arch64 based machines (ex, raspberry pi).
Currently, we believe this is a short coming of SICK's library as none of our code changes and library appears to run but not return data on arch64.

## Camera Configuration:
### Parameters:
  * "launch_file": the name of the launch file which corresponds to your lidar unit, found in SICK's repo under https://github.com/SICKAG/sick_scan_xd/tree/master/launch.
  * "host": the ip address of the lidar unit.
  * "receiver": optional, the ip of the host running viam-server, receiving data from the lidar. (only needed for multiscan lidars)
  * "segments": optional, the number of pointcloud callbacks worth of data to store locally and return when `get_point_cloud` is called.

### Example:
{
  "host": "10.1.11.188",
  "launch_file": "sick_multiscan.launch",
  "receiver": "10.1.7.101",
  "segments": 24
}

