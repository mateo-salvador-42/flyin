"""Drone routing simulation using Dijkstra pathfinding."""

import argparse
from parser import parse_map, ZoneType
import heapq

RESET = "\033[0m"
BOLD = "\033[1m"
GREEN = "\033[32m"
CYAN = "\033[36m"
MAGENTA = "\033[35m"
YELLOW = "\033[33m"
BLUE = "\033[34m"

DRONE_COLORS = [
    "\033[91m", "\033[92m", "\033[93m", "\033[94m",
    "\033[95m", "\033[96m", "\033[97m", "\033[33m",
]

HUB_COLOR_MAP: dict[str, str] = {
    "green":   "\033[32m",
    "red":     "\033[31m",
    "blue":    "\033[34m",
    "cyan":    "\033[36m",
    "purple":  "\033[35m",
    "magenta": "\033[35m",
    "violet":  "\033[35m",
    "yellow":  "\033[33m",
    "orange":  "\033[33m",
    "gold":    "\033[93m",
    "lime":    "\033[92m",
    "brown":   "\033[33m",
    "darkred": "\033[31m",
    "maroon":  "\033[31m",
    "crimson": "\033[91m",
    "black":   "\033[90m",
    "rainbow": "\033[35m",
}


def _hub_color(color: str | None, zone: ZoneType) -> str:
    """Return the ANSI color code for a hub.

    Args:
        color: The hub's color name from the map file, or None.
        zone: The hub's zone type used as fallback.

    Returns:
        An ANSI escape code string for terminal coloring.
    """
    if color and color in HUB_COLOR_MAP:
        return HUB_COLOR_MAP[color]
    if zone == ZoneType.restricted:
        return MAGENTA
    if zone == ZoneType.priority:
        return CYAN
    return BLUE


class Graph():
    """Represents the drone routing graph parsed from a map file.

    Attributes:
        folder_path: Path to the map file.
        map: Parsed MapData containing hubs and connections.
        connection: Adjacency list mapping hub names to neighbors.
        drone_list: List of drones initialized at the start hub.
    """

    def __init__(self, path: str) -> None:
        """Initialize the graph from a map file.

        Args:
            path: Path to the map file to parse.
        """
        self.folder_path = path
        self.map = parse_map(self.folder_path)
        self.connection: dict[str, list[str]] = {}
        for connect in self.map.connections:
            self.connection.setdefault(connect.from_hub,
                                       []).append(connect.to_hub)
            self.connection.setdefault(connect.to_hub,
                                       []).append(connect.from_hub)
        self.remove_blocked()
        self.drone_list = self.create_drone()

    def remove_blocked(self) -> None:
        """Remove all blocked hubs from the adjacency list."""
        for connect in self.connection:
            self.connection[connect] = [hub for hub in
                                        self.connection[connect] if
                                        (h := self.map.get_hub(hub))
                                        is not None
                                        and h.zone != ZoneType.blocked]

    def find_best_path(self, start: str,
                       excluded: set[str]) -> list[str]:
        """Find the lowest-cost path from start to the end hub using Dijkstra.

        Movement costs: restricted=2, priority=0.5, normal=1.

        Args:
            start: Name of the hub to start from.
            excluded: Set of hub names to treat as impassable.

        Returns:
            Ordered list of hub names from start to end, or empty list
            if no path exists.
        """
        end = self.map.end_hub
        heap: list[tuple[float, str, list[str]]] = [(0, start, [start])]
        visited: set[str] = set()
        while heap:
            current_cost, current_hub, current_path = heapq.heappop(heap)
            if current_hub in visited:
                continue
            if current_hub == end.name:
                return current_path
            visited.add(current_hub)
            for connect in self.connection[current_hub]:
                if connect in excluded or connect in visited:
                    continue
                hub = self.map.get_hub(connect)
                if hub is None:
                    continue
                move_cost = (
                    2 if hub.zone == ZoneType.restricted
                    else 0.5 if hub.zone == ZoneType.priority
                    else 1
                )
                heapq.heappush(heap, (
                    current_cost + move_cost,
                    connect,
                    current_path + [connect]
                ))
        return []

    def create_drone(self) -> list['Drone']:
        """Instantiate all drones at the start hub with an initial path.

        Returns:
            List of Drone objects, one per nb_drones in the map.
        """
        result = []
        for i in range(self.map.nb_drones):
            result.append(Drone(i, self.map.start_hub.name,
                                self.find_best_path(self.map.start_hub.name,
                                                    set())))
        return result


class Drone():
    """Represents a single drone in the simulation.

    Attributes:
        drone_id: Unique identifier for the drone.
        pos: Current hub name where the drone is located.
        path: Planned route as an ordered list of hub names.
        in_transit: True when the drone is crossing a restricted zone.
        transit_dest: Destination hub name during restricted transit.
    """

    def __init__(self, drone_id: int, pos: str,
                 path: list[str]) -> None:
        """Initialize a drone at a given position with a planned path.

        Args:
            drone_id: Unique numeric identifier.
            pos: Starting hub name.
            path: Initial planned route.
        """
        self.drone_id: int = drone_id
        self.pos: str = pos
        self.path: list[str] = path
        self.in_transit: bool = False
        self.transit_dest: str = ""


class Simulation():
    """Manages the turn-by-turn drone routing simulation.

    Attributes:
        drone_list: Active drones not yet at the end hub.
        graph: The routing graph used for pathfinding.
        visual: Whether to print colored terminal output.
    """

    def __init__(self, drone_list: list[Drone], graph: Graph,
                 visual: bool = False) -> None:
        """Initialize the simulation.

        Args:
            drone_list: List of drones to simulate.
            graph: Graph containing map data and pathfinding.
            visual: Enable colored terminal output when True.
        """
        self.drone_list: list[Drone] = drone_list
        self.graph: Graph = graph
        self.visual: bool = visual

    def calculate_excluded(self) -> set[str]:
        """Return the set of hub names currently at full capacity.

        Returns:
            Set of hub names where no additional drone may enter.
        """
        result = set()
        for hub in self.graph.map.hubs:
            count = sum(1 for d in self.drone_list
                        if d.pos == hub.name and not d.in_transit)
            if count >= hub.max_drones:
                result.add(hub.name)
        return result

    def check_link_capacity(self, start: str, end: str,
                            reserved: dict[tuple[str, str], int]) -> bool:
        """Check whether a connection has remaining capacity this turn.

        Args:
            start: Name of the origin hub.
            end: Name of the destination hub.
            reserved: Current turn usage count per directed link.

        Returns:
            True if at least one slot remains on the connection.
        """
        max_cap = 1
        for connection in self.graph.map.connections:
            if ((connection.from_hub == start and connection.to_hub == end)
                    or (connection.from_hub == end
                        and connection.to_hub == start)):
                max_cap = connection.max_link_capacity or 1
                break
        return (reserved.get((start, end), 0)
                + reserved.get((end, start), 0) < max_cap)

    def _fmt_move(self, drone: Drone, dest: str) -> str:
        """Format a single drone movement for output.

        Args:
            drone: The drone that moved.
            dest: Destination hub name.

        Returns:
            Formatted string, plain or ANSI-colored depending on visual mode.
        """
        if not self.visual:
            return f'D{drone.drone_id}-{dest}'
        drone_col = DRONE_COLORS[drone.drone_id % len(DRONE_COLORS)]
        hub = self.graph.map.get_hub(dest)
        if hub is not None:
            hub_col = _hub_color(hub.color, hub.zone)
        else:
            hub_col = RESET
        return (f'{drone_col}{BOLD}D{drone.drone_id}{RESET}'
                f'-{hub_col}{dest}{RESET}')

    def run(self) -> None:
        """Run the simulation until all drones reach the end hub.

        Each turn: transit arrivals are resolved first, then remaining
        drones recalculate their path and attempt to move. Restricted
        zones cost 2 turns via the in_transit mechanism.
        """
        turn_num = 0
        while self.drone_list:
            turn_num += 1
            arrived = []
            turn = []
            reserved: set[str] = set()
            reserved_count: dict[str, int] = {}
            link_usage: dict[tuple[str, str], int] = {}

            for drone in self.drone_list:
                if drone.in_transit:
                    drone.pos = drone.transit_dest
                    drone.transit_dest = ""
                    drone.in_transit = False
                    turn.append(self._fmt_move(drone, drone.pos))
                    if drone.pos == self.graph.map.end_hub.name:
                        arrived.append(drone)

            for drone in arrived:
                self.drone_list.remove(drone)
            arrived = []

            for drone in self.drone_list:
                drone.path = self.graph.find_best_path(
                    drone.pos, self.calculate_excluded() | reserved)
                if len(drone.path) > 1:
                    next_hub = self.graph.map.get_hub(drone.path[1])
                    if next_hub is None:
                        continue
                    count = sum(1 for d in self.drone_list
                                if d.pos == next_hub.name
                                and not d.in_transit)
                    if (count < next_hub.max_drones
                            and self.check_link_capacity(
                                drone.pos, next_hub.name, link_usage)):
                        reserved_count[drone.path[1]] = (
                            reserved_count.get(drone.path[1], 0) + 1)
                        if (reserved_count[drone.path[1]]
                                >= next_hub.max_drones):
                            reserved.add(next_hub.name)
                        link_key = (drone.pos, drone.path[1])
                        link_usage[link_key] = (
                            link_usage.get(link_key, 0) + 1)
                        if next_hub.zone == ZoneType.restricted:
                            drone.in_transit = True
                            drone.transit_dest = drone.path[1]
                        else:
                            drone.pos = drone.path[1]
                            turn.append(self._fmt_move(drone, drone.pos))
                            if drone.pos == self.graph.map.end_hub.name:
                                arrived.append(drone)

            if self.visual:
                prefix = f'{YELLOW}{BOLD}Turn {turn_num:>3}{RESET}  '
                print(prefix + " ".join(turn))
            else:
                print(" ".join(turn))
            for drone in arrived:
                self.drone_list.remove(drone)

        if self.visual:
            print(f'\n{GREEN}{BOLD}Completed in {turn_num} turns.{RESET}')


def main() -> None:
    """Parse CLI arguments and launch the simulation."""
    parser = argparse.ArgumentParser(description='Drone routing simulation')
    parser.add_argument('map', help='Path to map file')
    parser.add_argument('--visual', action='store_true',
                        help='Enable colored terminal output')
    args = parser.parse_args()

    try:
        graph = Graph(args.map)
        sim = Simulation(graph.drone_list, graph, visual=args.visual)
        sim.run()
    except FileNotFoundError:
        print(f"Error: map file '{args.map}' not found.")
    except ValueError as e:
        print(f"Error: {e}")


if __name__ == '__main__':
    main()
