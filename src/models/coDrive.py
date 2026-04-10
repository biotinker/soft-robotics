import asyncio
from dataclasses import dataclass
from typing import (Any, ClassVar, Dict, Final, List, Mapping, Optional,
                    Sequence, Tuple)

from typing_extensions import Self
from viam.components.component_base import ComponentBase
from viam.components.board import Board
from viam.components.gripper import *
from viam.proto.app.robot import ComponentConfig
from viam.proto.common import Geometry, ResourceName
from viam.resource.base import ResourceBase
from viam.resource.easy_resource import EasyResource
from viam.resource.types import Model, ModelFamily
from viam.utils import ValueTypes, struct_to_dict


class Codrive(Gripper, EasyResource):
    # To enable debug-level logging, either run viam-server with the --debug option,
    # or configure your resource/machine to display debug logs.
    MODEL: ClassVar[Model] = Model(
        ModelFamily("biotinker", "soft-robotics"), "coDrive"
    )

    @classmethod
    def new(
        cls, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ) -> Self:
        """This method creates a new instance of this Gripper component.
        The default implementation sets the name from the `config` parameter and then calls `reconfigure`.

        Args:
            config (ComponentConfig): The configuration for this resource
            dependencies (Mapping[ResourceName, ResourceBase]): The dependencies (both required and optional)

        Returns:
            Self: The resource
        """
        return super().new(config, dependencies)

    @classmethod
    def validate_config(
        cls, config: ComponentConfig
    ) -> Tuple[Sequence[str], Sequence[str]]:
        """This method allows you to validate the configuration object received from the machine,
        as well as to return any required dependencies or optional dependencies based on that `config`.

        Args:
            config (ComponentConfig): The configuration for this resource

        Returns:
            Tuple[Sequence[str], Sequence[str]]: A tuple where the
                first element is a list of required dependencies and the
                second element is a list of optional dependencies
        """
        attributes = struct_to_dict(config.attributes)

        if "d1_pin" not in attributes:
            raise Exception("`d1_pin` is required in the config attributes")
        
        if "d2_pin" not in attributes:
            raise Exception("`d2_pin` is required in the config attributes")
        
        if "d3_pin" not in attributes:
            raise Exception("`d3_pin` is required in the config attributes")
        
        if "board_name" not in attributes:
            raise Exception("`board_name` is required in the config attributes")
        board_name = attributes["board_name"]

        return [board_name], []

    def reconfigure(
        self, config: ComponentConfig, dependencies: Mapping[ResourceName, ResourceBase]
    ):
        """This method allows you to dynamically update your service when it receives a new `config` object.

        Args:
            config (ComponentConfig): The new configuration
            dependencies (Mapping[ResourceName, ResourceBase]): Any dependencies (both required and optional)
        """
        attributes = struct_to_dict(config.attributes)

        self.open_pin_name = attributes.get("d1_pin")

        self.grab_pin_name = attributes.get("d2_pin")

        self.com_pin_name = attributes.get("d3_pin")

        board_name = attributes.get("board_name")
        self.board: Board = dependencies[Board.get_resource_name(board_name)]
        return super().reconfigure(config, dependencies)

    @dataclass
    class HoldingStatus(Gripper.HoldingStatus):
        pass

    async def open(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ):
        grab_pin = await self.board.gpio_pin_by_name(self.grab_pin_name)
        open_pin = await self.board.gpio_pin_by_name(self.open_pin_name)
        com_pin = await self.board.gpio_pin_by_name(self.com_pin_name)

        # Turn on generator pin
        await com_pin.set(high=True)

        await grab_pin.set(high=True)

        # Set open_pin to high
        await open_pin.set(high=True)
        # Wait for a short duration to ensure the vacuum is released
        await asyncio.sleep(2)

        # Turn off generator pin
        await com_pin.set(high=False)

    async def grab(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> bool:
        grab_pin = await self.board.gpio_pin_by_name(self.grab_pin_name)
        open_pin = await self.board.gpio_pin_by_name(self.open_pin_name)
        com_pin = await self.board.gpio_pin_by_name(self.com_pin_name)
        await open_pin.set(high=False)
        await grab_pin.set(high=False)

        # Turn on generator pin
        await com_pin.set(high=True)

        await asyncio.sleep(3)
        await com_pin.set(high=False)
        return True

    async def is_holding_something(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> HoldingStatus:
        self.logger.error("`is_holding_something` is not implemented")
        raise NotImplementedError()

    async def stop(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ):
        self.logger.error("`stop` is not implemented")
        raise NotImplementedError()

    async def is_moving(self) -> bool:
        self.logger.error("`is_moving` is not implemented")
        raise NotImplementedError()

    async def get_kinematics(
        self,
        *,
        extra: Optional[Dict[str, Any]] = None,
        timeout: Optional[float] = None,
        **kwargs
    ) -> Tuple[KinematicsFileFormat.ValueType, bytes]:
        self.logger.error("`get_kinematics` is not implemented")
        raise NotImplementedError()

    async def do_command(
        self,
        command: Mapping[str, ValueTypes],
        *,
        timeout: Optional[float] = None,
        **kwargs
    ) -> Mapping[str, ValueTypes]:
        grab_pin = await self.board.gpio_pin_by_name(self.grab_pin_name)
        open_pin = await self.board.gpio_pin_by_name(self.open_pin_name)
        com_pin = await self.board.gpio_pin_by_name(self.com_pin_name)
        await open_pin.set(high=True)
        await grab_pin.set(high=True)
        await asyncio.sleep(1)
        await open_pin.set(high=False)
        await grab_pin.set(high=False)

    async def get_geometries(
        self, *, extra: Optional[Dict[str, Any]] = None, timeout: Optional[float] = None
    ) -> List[Geometry]:
        self.logger.error("`get_geometries` is not implemented")
        raise NotImplementedError()

