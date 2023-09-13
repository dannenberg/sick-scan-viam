import asyncio
from sick_lidar import SickLidar

from viam.module.module import Module
from viam.resource.registry import Registry, ResourceCreatorRegistration

async def main():
    Registry.register_resource_creator(Camera.SUBTYPE, SickLidar.MODEL, ResourceCreatorRegistration(SickLidar.new))

    module = Module.from_args()
    module.add_model_from_registry(Camera.SUBTYPE, SickLidar.MODEL)
    await module.start()

if __name__ == "__main__":
    asyncio.run(main())
