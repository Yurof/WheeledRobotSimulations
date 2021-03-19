from typing import Type
from .task import Task
from .progress_based import MaximizeProgressTask

_registry = {}

def get_task(name: str) -> Type[Task]:
    return _registry[name]

def register_task(name: str, task: Type[Task]):
    if name not in _registry.keys():
        _registry[name] = task


register_task('maximize_progress', task=MaximizeProgressTask)
