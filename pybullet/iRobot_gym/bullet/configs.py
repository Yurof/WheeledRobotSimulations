from dataclasses import dataclass, field
from typing import List, Dict, Any, Tuple

from yamldataclassconfig.config import YamlDataClassConfig


@dataclass
class SensorConfig(YamlDataClassConfig):
    type: str = None
    name: str = None
    params: Dict[str, Any] = None
    frequency: float = None


@dataclass
class ActuatorConfig(YamlDataClassConfig):
    type: str
    name: str
    params: Dict[str, Any] = None


@dataclass
class VehicleConfig(YamlDataClassConfig):
    urdf_file: str = None
    debug: bool = False
    actuators: List[ActuatorConfig] = field(default_factory=lambda: [])
    sensors: List[SensorConfig] = field(default_factory=lambda: [])


@dataclass
class MapConfig(YamlDataClassConfig):
    starting_position : Tuple[float,float,float] = None
    starting_orientation : Tuple[float,float,float] = None
    goal_position : Tuple[float,float,float] = None
    goal_size : float = None


@dataclass
class SimulationConfig(YamlDataClassConfig):
    time_step: float = None
    rendering: bool = None


@dataclass
class PhysicsConfig(YamlDataClassConfig):
    gravity: float = None

# @dataclass
# class SimulationSpec(YamlDataClassConfig):
#     time_step: float = 0.01
#     rendering: bool = False
#     implementation: str = None


@dataclass
class TaskSpec(YamlDataClassConfig):
    task_name: str = None
    params: Dict[str, Any] = field(default_factory=lambda: {})


@dataclass
class VehicleSpec(YamlDataClassConfig):
    name: str = None
    sensors: List[str] = field(default_factory=lambda: [])


@dataclass
class WorldSpec(YamlDataClassConfig):
    name: str = None
    rendering: bool = False


@dataclass
class AgentSpec(YamlDataClassConfig):
    id: str
    vehicle: VehicleSpec = VehicleSpec()
    task: TaskSpec = TaskSpec()


@dataclass
class ScenarioSpec(YamlDataClassConfig):
    world: WorldSpec = None
    agents: List[AgentSpec] = None
    name: str = None
    sdf: str = None
    map: MapConfig = None
    physics: PhysicsConfig = None
    simulation: SimulationConfig = None
