"""Small nested configs that support attribute access and independent overrides."""

from copy import deepcopy
from typing import Any, Mapping


def deep_merge(left: dict, right: Mapping) -> dict:
    """Copy left, recursively replacing values supplied by right."""
    result = deepcopy(left)
    for key, value in right.items():
        result[key] = (
            deep_merge(result[key], value)
            if isinstance(result.get(key), dict) and isinstance(value, dict)
            else deepcopy(value)
        )
    return result


class ConfigNode(dict):
    """A dict whose nested values can also be read as attributes."""

    def __init__(self, mapping: Mapping | None = None):
        super().__init__(
            (key, ConfigNode(value) if isinstance(value, dict) else value)
            for key, value in (mapping or {}).items()
        )

    def __getattr__(self, name: str) -> Any:
        try:
            return self[name]
        except KeyError as error:
            raise AttributeError(name) from error

    def __setattr__(self, name: str, value: Any) -> None:
        self[name] = value

    def to_dict(self) -> dict:
        return {
            key: value.to_dict() if isinstance(value, ConfigNode) else deepcopy(value)
            for key, value in self.items()
        }

    def merge(self, overrides: Mapping) -> "ConfigNode":
        return type(self)(deep_merge(self.to_dict(), overrides))

    def copy(self) -> "ConfigNode":
        return type(self)(self.to_dict())

    merged = merge


class VehicleConfig(ConfigNode):
    """Vehicle properties shared across events."""


class EventConfig(ConfigNode):
    """Event load, surface, and run settings."""
