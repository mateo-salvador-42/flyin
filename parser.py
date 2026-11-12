"""Parser for the Fly-in map file format."""

import re
from enum import Enum
from typing import Optional
from pydantic import BaseModel, Field


class ZoneType(str, Enum):
    """Zone types defining movement cost and accessibility."""

    normal = "normal"
    restricted = "restricted"
    priority = "priority"
    blocked = "blocked"


class HubType(str, Enum):
    """Hub roles within the map."""

    start = "start_hub"
    end = "end_hub"
    regular = "hub"


class Hub(BaseModel):
    """A node in the routing graph.

    Attributes:
        name: Unique identifier for this hub.
        x: X coordinate (display only).
        y: Y coordinate (display only).
        hub_type: Whether this is a start, end, or regular hub.
        color: Optional display color name.
        max_drones: Maximum drones allowed simultaneously.
        zone: Zone type controlling movement cost.
    """

    name: str
    x: int
    y: int
    hub_type: HubType
    color: Optional[str] = None
    max_drones: int = 1
    zone: ZoneType = ZoneType.normal


class Connection(BaseModel):
    """A bidirectional edge between two hubs.

    Attributes:
        from_hub: Name of the first hub.
        to_hub: Name of the second hub.
        max_link_capacity: Maximum drones traversing simultaneously.
    """

    from_hub: str
    to_hub: str
    max_link_capacity: Optional[int] = 1


class MapData(BaseModel):
    """Complete parsed representation of a map file.

    Attributes:
        nb_drones: Number of drones to simulate.
        hubs: All hubs in the map.
        connections: All connections between hubs.
    """

    nb_drones: int = Field(ge=1)
    hubs: list[Hub]
    connections: list[Connection]

    def get_hub(self, name: str) -> Optional[Hub]:
        """Return the Hub with the given name, or None if not found.

        Args:
            name: Hub name to look up.

        Returns:
            Matching Hub instance or None.
        """
        return next((h for h in self.hubs if h.name == name), None)

    @property
    def start_hub(self) -> Hub:
        """Return the unique start hub.

        Returns:
            The hub with hub_type == HubType.start.
        """
        return next(h for h in self.hubs if h.hub_type == HubType.start)

    @property
    def end_hub(self) -> Hub:
        """Return the unique end hub.

        Returns:
            The hub with hub_type == HubType.end.
        """
        return next(h for h in self.hubs if h.hub_type == HubType.end)


def _parse_options(raw: str) -> dict[str, str]:
    """Extract key=value pairs from a bracket metadata block.

    Args:
        raw: Raw line string possibly containing [...] metadata.

    Returns:
        Dictionary of parsed key/value pairs, empty if no block found.
    """
    options: dict[str, str] = {}
    m = re.search(r'\[([^\]]+)\]', raw)
    if not m:
        return options
    for pair in m.group(1).split():
        key, _, value = pair.partition('=')
        options[key] = value
    return options


def _parse_hub_line(hub_type: HubType, rest: str,
                    line_num: int) -> Hub:
    """Parse a hub line (after the type prefix and colon).

    Args:
        hub_type: The role of this hub (start, end, or regular).
        rest: Remainder of the line after 'start_hub:', 'hub:', etc.
        line_num: Source line number for error reporting.

    Returns:
        A fully constructed Hub instance.

    Raises:
        ValueError: If the line is malformed or contains invalid values.
    """
    try:
        clean = re.sub(r'\[.*?\]', '', rest).split()
        if len(clean) < 3:
            raise ValueError("expected <name> <x> <y>")
        name = clean[0]
        x, y = int(clean[1]), int(clean[2])
    except (IndexError, ValueError) as e:
        raise ValueError(
            f"line {line_num}: invalid hub definition — {e}"
        ) from e

    opts = _parse_options(rest)

    if 'zone' in opts:
        try:
            zone = ZoneType(opts['zone'])
        except ValueError:
            valid = ', '.join(z.value for z in ZoneType)
            raise ValueError(
                f"line {line_num}: unknown zone type '{opts['zone']}'"
                f" (valid: {valid})"
            )
    else:
        zone = ZoneType.normal

    if 'max_drones' in opts:
        try:
            max_drones = int(opts['max_drones'])
            if max_drones < 1:
                raise ValueError("must be >= 1")
        except ValueError as e:
            raise ValueError(
                f"line {line_num}: invalid max_drones — {e}"
            ) from e
    else:
        max_drones = 1

    return Hub(
        name=name, x=x, y=y, hub_type=hub_type,
        color=opts.get('color'),
        max_drones=max_drones,
        zone=zone,
    )


def _parse_connection_line(rest: str, line_num: int,
                           known_hubs: set[str]) -> Connection:
    """Parse a connection line (after 'connection:').

    Args:
        rest: Remainder of the line, e.g. 'hubA-hubB [max_link_capacity=2]'.
        line_num: Source line number for error reporting.
        known_hubs: Set of hub names defined so far for validation.

    Returns:
        A fully constructed Connection instance.

    Raises:
        ValueError: If the line is malformed or references unknown hubs.
    """
    try:
        name_part = rest.split()[0]
        from_hub, to_hub = name_part.split('-', 1)
    except (IndexError, ValueError) as e:
        raise ValueError(
            f"line {line_num}: invalid connection format — {e}"
        ) from e

    if from_hub not in known_hubs:
        raise ValueError(
            f"line {line_num}: unknown hub '{from_hub}'"
        )
    if to_hub not in known_hubs:
        raise ValueError(
            f"line {line_num}: unknown hub '{to_hub}'"
        )

    opts = _parse_options(rest)

    if 'max_link_capacity' in opts:
        try:
            cap = int(opts['max_link_capacity'])
            if cap < 1:
                raise ValueError("must be >= 1")
        except ValueError as e:
            raise ValueError(
                f"line {line_num}: invalid max_link_capacity — {e}"
            ) from e
        max_link_capacity = cap
    else:
        max_link_capacity = 1

    return Connection(
        from_hub=from_hub,
        to_hub=to_hub,
        max_link_capacity=max_link_capacity,
    )


def parse_map(path: str) -> MapData:
    """Parse a map file and return a MapData model.

    Args:
        path: Path to the map file.

    Raises:
        ValueError: If nb_drones is missing, or any line is invalid.
        FileNotFoundError: If the map file does not exist.

    Returns:
        A MapData instance populated from the file.
    """
    hubs: list[Hub] = []
    connections: list[Connection] = []
    nb_drones: Optional[int] = None
    known_hubs: set[str] = set()
    seen_connections: set[frozenset[str]] = set()

    with open(path) as f:
        for line_num, raw_line in enumerate(f, start=1):
            line = raw_line.strip()
            if not line or line.startswith('#'):
                continue

            key, _, rest = line.partition(':')
            rest = rest.strip()

            if key == 'nb_drones':
                try:
                    nb_drones = int(rest)
                    if nb_drones < 1:
                        raise ValueError("must be >= 1")
                except ValueError as e:
                    raise ValueError(
                        f"line {line_num}: invalid nb_drones — {e}"
                    ) from e

            elif key in ('start_hub', 'end_hub', 'hub'):
                hub_type = HubType(key)
                hub = _parse_hub_line(hub_type, rest, line_num)
                if hub.name in known_hubs:
                    raise ValueError(
                        f"line {line_num}: duplicate hub name '{hub.name}'"
                    )
                known_hubs.add(hub.name)
                hubs.append(hub)

            elif key == 'connection':
                conn = _parse_connection_line(rest, line_num, known_hubs)
                pair: frozenset[str] = frozenset(
                    [conn.from_hub, conn.to_hub]
                )
                if pair in seen_connections:
                    raise ValueError(
                        f"line {line_num}: duplicate connection "
                        f"'{conn.from_hub}-{conn.to_hub}'"
                    )
                seen_connections.add(pair)
                connections.append(conn)

    if nb_drones is None:
        raise ValueError(f"'{path}': nb_drones is missing")

    return MapData(nb_drones=nb_drones, hubs=hubs, connections=connections)
