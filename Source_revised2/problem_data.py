from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import List, Tuple
import math


@dataclass
class ProblemData:
    number_truck: int
    number_drone: int
    truck_speed: float
    drone_speed: float
    drone_capacity: float
    drone_limit_time: float
    unloading_time: float
    coords: List[Tuple[float, float]]
    demands: List[float]
    release_dates: List[float]
    truck_time_matrix: List[List[float]]
    drone_time_matrix: List[List[float]]

    @property
    def number_of_cities(self) -> int:
        return len(self.coords)


def euclid_distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def manhattan_distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def _build_time_matrix(
    coords: List[Tuple[float, float]],
    speed: float,
    metric: str,
) -> List[List[float]]:
    if speed <= 0:
        raise ValueError("Speed must be > 0")

    n = len(coords)
    matrix = [[0.0] * n for _ in range(n)]
    for i in range(n):
        for j in range(n):
            if metric == "manhattan":
                distance = manhattan_distance(coords[i], coords[j])
            elif metric == "euclid":
                distance = euclid_distance(coords[i], coords[j])
            else:
                raise ValueError(f"Unsupported metric: {metric}")
            matrix[i][j] = distance / speed
    return matrix


def _parse_header_value(lines: List[str], key: str) -> float:
    for line in lines:
        parts = line.split()
        if not parts:
            continue
        if parts[0] == key and len(parts) >= 2:
            return float(parts[-1])
    raise ValueError(f"Missing header key: {key}")


def read_data_file(path: str | Path) -> ProblemData:
    """
    Read .dat format like test_data/data_new/1.dat and build travel-time matrices:
    - truck_time_matrix: Manhattan distance / truck_speed
    - drone_time_matrix: Euclidean distance / drone_speed
    """
    file_path = Path(path)
    if not file_path.exists():
        raise FileNotFoundError(f"File not found: {file_path}")

    raw_lines = [line.strip() for line in file_path.read_text(encoding="utf-8").splitlines() if line.strip()]
    if len(raw_lines) < 9:
        raise ValueError(f"Invalid file format (too short): {file_path}")

    header_lines = raw_lines[:8]
    data_lines = raw_lines[8:]

    if not header_lines[7].startswith("XCOORD"):
        raise ValueError("Invalid file format: expected column header line 'XCOORD YCOORD DEMAND RELEASE_DATE'")

    coords: List[Tuple[float, float]] = []
    demands: List[float] = []
    release_dates: List[float] = []
    for line in data_lines:
        parts = line.split()
        if len(parts) < 4:
            continue
        x = float(parts[0])
        y = float(parts[1])
        demand = float(parts[2])
        release = float(parts[3])
        coords.append((x, y))
        demands.append(demand)
        release_dates.append(release)

    if not coords:
        raise ValueError(f"No city rows found: {file_path}")

    number_truck = int(_parse_header_value(header_lines, "number_truck"))
    number_drone = int(_parse_header_value(header_lines, "number_drone"))
    truck_speed = float(_parse_header_value(header_lines, "truck_speed"))
    drone_speed = float(_parse_header_value(header_lines, "drone_speed"))
    drone_capacity = float(_parse_header_value(header_lines, "M_d"))
    drone_limit_time = float(_parse_header_value(header_lines, "L_d"))
    unloading_time = float(_parse_header_value(header_lines, "Sigma"))

    truck_time_matrix = _build_time_matrix(coords, speed=truck_speed, metric="manhattan")
    drone_time_matrix = _build_time_matrix(coords, speed=drone_speed, metric="euclid")

    return ProblemData(
        number_truck=number_truck,
        number_drone=number_drone,
        truck_speed=truck_speed,
        drone_speed=drone_speed,
        drone_capacity=drone_capacity,
        drone_limit_time=drone_limit_time,
        unloading_time=unloading_time,
        coords=coords,
        demands=demands,
        release_dates=release_dates,
        truck_time_matrix=truck_time_matrix,
        drone_time_matrix=drone_time_matrix,
    )
