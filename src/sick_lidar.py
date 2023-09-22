import asyncio
import logging
import math
import numpy as np

from sick_scan_api import *
from ctypes import CDLL, c_void_p

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
    msg: SickScanPointCloudMsg
    properties: Camera.Properties
    sick_scan_library: CDLL
    api_handle: c_void_p

    @classmethod
    def new(cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]) -> Self:
        try:
            lidar = cls(config.name)
            lidar.logger = getLogger(f'{__name__}.{lidar.__class__.__name__}')
            lidar.properties = Camera.Properties(
                supports_pcd=True,
                intrinsic_parameters=IntrinsicParameters(width_px=0, height_px=0, focal_x_px=0.0, focal_y_px=0.0, center_x_px=0.0),
                distortion_parameters=DistortionParameters(model='')
            )
            lidar.reconfigure(config, dependencies)

            def foo(api_handle, msg):
                lidar.update_msg(msg)

            # Register for pointcloud messages
            lidar.cartesian_pointcloud_callback = SickScanPointCloudMsgCallback(foo)
            SickScanApiRegisterCartesianPointCloudMsg(lidar.sick_scan_library, lidar.api_handle, lidar.cartesian_pointcloud_callback)
            lidar.logger.info("started")

            return lidar
        except Exception as e:
            lidar.logger.info(e)

    def __del__(self):
        SickScanApiDeregisterCartesianPointCloudMsg(self.sick_scan_library, self.api_handle, self.cartesian_pointcloud_callback)
        SickScanApiClose(self.sick_scan_library, self.api_handle)
        SickScanApiRelease(self.sick_scan_library, self.api_handle)
        SickScanApiUnloadLibrary(self.sick_scan_library)

    @classmethod
    def  validate_config(cls, config: ComponentConfig) -> Sequence[str]:
        launch_file = config.attributes.fields['launch_file'].string_value
        if launch_file == '':
            raise Exception('launch_file required')
        return []

    def update_msg(self, msg):
        self.msg = msg

    def reconfigure(self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]):
        self.lock = Lock()
        self.msg = None
        self.sick_scan_library = SickScanApiLoadLibrary(["build/"], "libsick_scan_shared_lib.so")
        # Create a sick_scan instance and initialize a TiM-5xx
        self.api_handle = SickScanApiCreate(self.sick_scan_library)
        SickScanApiInitByLaunchfile(self.sick_scan_library, self.api_handle, f"launch/{config.attributes.fields['launch_file'].string_value}")

    async def get_image(self, mime_type: str='', *, timeout: Optional[float]=None, **kwargs) -> Union[Image, RawImage]:
        raise NotImplementedError()

    async def get_images(self, *, timeout: Optional[float]=None, **kwargs) -> Tuple[List[NamedImage], ResponseMetadata]:
        raise NotImplementedError()

    async def get_point_cloud(self, *, timeout: Optional[float]=None, **kwargs) -> Tuple[bytes, str]:
        msg = None
        self.logger.info('get pcd 1')
        async with self.lock:
            if self.msg is None:
                raise Exception('laserscan msg not ready')
            else:
                msg = self.msg

        self.logger.info('get pcd 2')
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
        self.logger.info('get pcd 3')
        for point in range(msg.contents.data.size):
            self.logger.info('get pcd loop')
            x = array[point*4]
            y = array[point*4+1]
            z = array[point*4+2]
            pdata.append(x)
            pdata.append(y)
            pdata.append(z)

        width = f'WIDTH {len(pdata)}\n'
        points = f'POINTS {len(pdata)}\n'
        header = f'{version}{fields}{size}{type_of}{count}{width}{height}{viewpoint}{points}{data}'
        self.logger.info('get pcd 4')
        a = np.array(pdata, dtype='f')
        h = bytes(header, 'UTF-8')

        # self.logger.info("pcd", h+a.tobytes())
        self.logger.info('get pcd 5')
        return h + a.tobytes(), CameraMimeType.PCD

    async def get_properties(self, *, timeout: Optional[float] = None, **kwargs) -> Camera.Properties:
        return self.properties

Registry.register_resource_creator(
    Camera.SUBTYPE,
    SickLidar.MODEL,
    ResourceCreatorRegistration(SickLidar.new, SickLidar.validate_config)
)
