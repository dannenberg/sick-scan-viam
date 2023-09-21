import logging
import math
import numpy as np

from sick_scan_api import *

from PIL.Image import Image
from asyncio import Lock
from typing import ClassVar, List, Mapping, Optional, Sequence, Tuple, Union
from typing_extensions import Self
from viam.components.camera import Camera, DistortionParameters, IntrinsicParameters, RawImage
from viam.logging import getLogger
from viam.media.video import NamedImage 
from viam.module.types import Reconfigurable
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import ResourceName, ResponseMetadata
from viam.resource.base import ResourceBase
from viam.resource.registry import Registry, ResourceCreatorRegistration
from viam.resource.types import Model, ModelFamily
from viam.media.video import CameraMimeType


class SickLidar(Camera, Reconfigurable):
    MODEL: ClassVar[Model] = Model(ModelFamily('viamlabs', 'sick'), 'tim-lidar')
    logger: logging.Logger
    lock: Lock
    msg: LP_SickScanPointCloudMsg
    properties: Camera.Properties

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        lidar = cls(config.name)
        lidar.logger = getLogger(f'{__name__}.{lidar.__class__.__name__}')
        lidar.properties = Camera.Properties(
            supports_pcd=True,
            intrinsic_parameters=IntrinsicParameters(width_px=0, height_px=0, focal_x_px=0.0, focal_y_px=0.0, center_x_px=0.0),
            distortion_parameters=DistortionParameters(model='')
        )
        lidar.reconfigure(config, dependencies)
        sick_scan_library = SickScanApiLoadLibrary(["build/"], "libsick_scan_shared_lib.so")
        # Create a sick_scan instance and initialize a TiM-5xx
        api_handle = SickScanApiCreate(sick_scan_library)
        SickScanApiInitByLaunchfile(sick_scan_library, api_handle, cli_args)

        # Register for pointcloud messages
        cartesian_pointcloud_callback = SickScanPointCloudMsgCallback(pyCustomizedPointCloudMsgCb)
        SickScanApiRegisterCartesianPointCloudMsg(sick_scan_library, api_handle, cartesian_pointcloud_callback)


        def foo(api_handle, msg):
            asyncio.get_event_loop().run_until_complete(lidar.update_msg(msg))

        # set up sick lib and callback
        sick_scan_lib = SickScanApiLoadLibrary(["build/"], "libsick_scan_shared_lib.so")
        api_handle = SickScanApiCreate(sick_scan_lib)
        #TODO: confirm launch file for our lidar
        SickScanApiInitByLaunchfile(sick_scan_lib, api_handle, f"launch/{config.attributes.fields['launch_file'].string_value}")
        polar_pointcloud_callback = SickScanPointCloudMsgCallback(foo)
        SickScanApiRegisterPolarPointCloudMsg(sick_scan_lib, api_handle, polar_pointcloud_callback)

        return lidar

    def __del__():
        SickScanApiDeregisterCartesianPointCloudMsg(sick_scan_library, api_handle, cartesian_pointcloud_callback)
        SickScanApiClose(sick_scan_library, api_handle)
        SickScanApiRelease(sick_scan_library, api_handle)
        SickScanApiUnloadLibrary(sick_scan_library)

    @classmethod
    def  validate_config(cls, config: ComponentConfig) -> Sequence[str]:
        launch_file = config.attributes.fields['launch_file'].string_value
        if launch_file == '':
            raise Exception('launch_file required')
        return []

    async def update_msg(self, msg):
        with self.lock:
            self.msg = msg

    def reconfigure(self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]):
        self.lock = Lock()
        self.msg = None

    def subscriber_callback(self, msg):
        with self.lock:
            self.msg = msg

    async def get_image(self, mime_type: str='', *, timeout: Optional[float]=None, **kwargs) -> Union[Image, RawImage]:
        raise NotImplementedError()

    async def get_images(self, *, timeout: Optional[float]=None, **kwargs) -> Tuple[List[NamedImage], ResponseMetadata]:
        raise NotImplementedError()

    async def get_point_cloud(self, *, timeout: Optional[float]=None, **kwargs) -> Tuple[bytes, str]:
        msg
        with self.lock:
            if self.msg is None:
                raise Exception('laserscan msg not ready')
            else:
                msg = self.msg

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

        # print("pcd", h+a.tobytes())
        return h + a.tobytes(), CameraMimeType.PCD

    async def get_properties(self, *, timeout: Optional[float] = None, **kwargs) -> Camera.Properties:
        return self.properties

Registry.register_resource_creator(
    Camera.SUBTYPE,
    SickLidar.MODEL,
    ResourceCreatorRegistration(SickLidar.new, SickLidar.validate_config)
)
