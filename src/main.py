import asyncio
from sick_lidar import SickLidar

from viam.components.camera import Camera
from viam.module.module import Module
from viam.resource.registry import Registry, ResourceCreatorRegistration

async def main():
    module = Module.from_args()
    module.add_model_from_registry(Camera.SUBTYPE, SickLidar.MODEL)
    await module.start()

if __name__ == "__main__":
    asyncio.run(main())
