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
    truck_manhattan_distance: List[List[float]]
    drone_euclid_distance: List[List[float]]
    truck_travel_time: List[List[float]]
    drone_travel_time: List[List[float]]

    @property
    def number_of_cities(self) -> int:
        return len(self.coords)


def euclid_distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return math.hypot(a[0] - b[0], a[1] - b[1])


def manhattan_distance(a: Tuple[float, float], b: Tuple[float, float]) -> float:
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def _build_matrix(coords: List[Tuple[float, float]], metric: str) -> List[List[float]]:
    n = len(coords)
    mat = [[0.0] * n for _ in range(n)]
    for i in range(n):
        for j in range(n):
            if metric == "manhattan":
                mat[i][j] = manhattan_distance(coords[i], coords[j])
            elif metric == "euclid":
                mat[i][j] = euclid_distance(coords[i], coords[j])
            else:
                raise ValueError(f"Unsupported metric: {metric}")
    return mat


def _to_travel_time(distance_matrix: List[List[float]], speed: float) -> List[List[float]]:
    if speed <= 0:
        raise ValueError("Speed must be > 0")
    return [[d / speed for d in row] for row in distance_matrix]


def read_dat(path: str | Path) -> ProblemData:
    p = Path(path)
    lines = [ln.strip() for ln in p.read_text(encoding="utf-8").splitlines() if ln.strip()]
    if len(lines) < 10:
        raise ValueError(f"Invalid .dat format (too short): {path}")

    header = {}
    for i in range(7):
        parts = lines[i].split()
        if len(parts) < 2:
            raise ValueError(f"Invalid header line: {lines[i]}")
        key = parts[0]
        val = parts[-1]
        header[key] = val

    # Line 8 is column header: XCOORD YCOORD DEMAND RELEASE_DATE
    data_lines = lines[8:]

    coords: List[Tuple[float, float]] = []
    demands: List[float] = []
    releases: List[float] = []
    for ln in data_lines:
        parts = ln.split()
        if len(parts) < 4:
            continue
        x = float(parts[0])
        y = float(parts[1])
        d = float(parts[2])
        r = float(parts[3])
        coords.append((x, y))
        demands.append(d)
        releases.append(r)

    if not coords:
        raise ValueError(f"No city rows found in .dat: {path}")

    truck_dist = _build_matrix(coords, metric="manhattan")
    drone_dist = _build_matrix(coords, metric="euclid")

    truck_speed = float(header.get("truck_speed",1.0 ))
    drone_speed = float(header.get("drone_speed", 1.0))

    return ProblemData(
        number_truck=int(float(header.get("number_truck", 1))),
        number_drone=int(float(header.get("number_drone", 1))),
        truck_speed=truck_speed,
        drone_speed=drone_speed,
        drone_capacity=float(header.get("M_d", 0.0)),
        drone_limit_time=float(header.get("L_d", 0.0)),
        unloading_time=float(header.get("Sigma", 0.0)),
        coords=coords,
        demands=demands,
        release_dates=releases,
        truck_manhattan_distance=truck_dist,
        drone_euclid_distance=drone_dist,
        truck_travel_time=_to_travel_time(truck_dist, truck_speed),
        drone_travel_time=_to_travel_time(drone_dist, drone_speed),
    )
