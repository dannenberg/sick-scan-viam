import asyncio
import logging
import math
import numpy as np

from sick_scan_api import *
from ctypes import CDLL, c_void_p

from PIL.Image import Image
from threading import Lock
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
from viam.utils import struct_to_dict
from viam.media.video import CameraMimeType


class SickLidar(Camera, Reconfigurable):
    MODEL: ClassVar[Model] = Model(ModelFamily('viam-soleng', 'sick'), 'tim-lidar')
    logger: logging.Logger
    lock: Lock
    msg: SickScanPointCloudMsg
    properties: Camera.Properties
    sick_scan_library: CDLL
    api_handle: c_void_p

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

        def foo(api_handle, msg):
            lidar.update_msg(msg)

        # Register for pointcloud messages
        lidar.cartesian_pointcloud_callback = SickScanPointCloudMsgCallback(foo)
        SickScanApiRegisterCartesianPointCloudMsg(lidar.sick_scan_library, lidar.api_handle, lidar.cartesian_pointcloud_callback)

        return lidar

    def __del__(self):
        SickScanApiDeregisterCartesianPointCloudMsg(self.sick_scan_library, self.api_handle, self.cartesian_pointcloud_callback)
        SickScanApiClose(self.sick_scan_library, self.api_handle)
        SickScanApiRelease(self.sick_scan_library, self.api_handle)
        SickScanApiUnloadLibrary(self.sick_scan_library)

    @classmethod
    def  validate_config(cls, config: ComponentConfig) -> Sequence[str]:
        attributes_dict = struct_to_dict(config.attributes)
        launch_file = attributes_dict.get("launch_file", "")
        assert isinstance(launch_file, str)
        if launch_file == "":
            raise Exception("the launch_file argument is required and should contain the launch file name.")

        if "host" in attributes_dict:
            assert isinstance(attributes_dict["host"], str)

        if "receiver" in attributes_dict:
            assert isinstance(attributes_dict["receiver"], str)

        if "segments" in attributes_dict:
            assert isinstance(attributes_dict["segments"], int)
        return []

    def update_msg(self, msg):
        with self.lock:
            self.msgs.append(msg)
            if len(self.msgs) > self.num_segments:
                self.msgs = self.msgs[1:]

    def reconfigure(self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]):
        self.lock = Lock()
        self.msgs = []
        self.sick_scan_library = SickScanApiLoadLibrary(["build/"], "libsick_scan_shared_lib.so")
        # Create a sick_scan instance and initialize a TiM-5xx
        self.api_handle = SickScanApiCreate(self.sick_scan_library)
        attributes_dict = struct_to_dict(config.attributes)
        launch_file = attributes_dict.get("launch_file", "")
        host_arg = ""
        receiver_arg = ""
        launch_file_arg = f"launch/{attributes_dict['launch_file']}"
        if "host" in attributes_dict:
            host_arg = f"hostname:={attributes_dict['host']}"
        if "receiver" in attributes_dict:
            receiver_arg = f"udp_receiver_ip:={attributes_dict['receiver']}"
        if "segments" in attributes_dict:
            self.num_segments = attributes_dict['segments']
        else:
            self.num_segments = 1

        SickScanApiInitByLaunchfile(self.sick_scan_library, self.api_handle, " ".join([launch_file_arg, host_arg, receiver_arg]) )

    async def get_image(self, mime_type: str='', *, timeout: Optional[float]=None, **kwargs) -> Union[Image, RawImage]:
        raise NotImplementedError()

    async def get_images(self, *, timeout: Optional[float]=None, **kwargs) -> Tuple[List[NamedImage], ResponseMetadata]:
        raise NotImplementedError()

    async def get_point_cloud(self, *, timeout: Optional[float]=None, **kwargs) -> Tuple[bytes, str]:
        msgs = None
        with self.lock:
            if len(self.msgs) < self.num_segments:
                raise Exception('laserscan msg not ready')
            else:
                msgs = self.msgs

        version = 'VERSION .7\n'
        fields = 'FIELDS x y z\n'
        size = 'SIZE 4 4 4\n'
        type_of = 'TYPE F F F\n'
        count = 'COUNT 1 1 1\n'
        height = 'HEIGHT 1\n'
        viewpoint = 'VIEWPOINT 0 0 0 1 0 0 0\n'
        data = 'DATA binary\n'
        pdata = []
        for msg in self.msgs:
            array = ctypes.cast(msg.contents.data.buffer, ctypes.POINTER(ctypes.c_float))
            for point in range(int(msg.contents.data.size/4/4)):
                x = array[point*4]
                y = array[point*4+1]
                z = array[point*4+2]
                if x < 8589934591 and x > -8589934591 and y < 8589934591 and y > -8589934591:
                    pdata.append(x)
                    pdata.append(y)
                    pdata.append(z)
        width = f'WIDTH {len(pdata)}\n'
        points = f'POINTS {len(pdata)}\n'
        header = f'{version}{fields}{size}{type_of}{count}{width}{height}{viewpoint}{points}{data}'
        a = np.array(pdata, dtype='f')
        h = bytes(header, 'UTF-8')
        return h + a.tobytes(), CameraMimeType.PCD

    async def get_properties(self, *, timeout: Optional[float] = None, **kwargs) -> Camera.Properties:
        return self.properties

Registry.register_resource_creator(
    Camera.SUBTYPE,
    SickLidar.MODEL,
    ResourceCreatorRegistration(SickLidar.new, SickLidar.validate_config)
)
