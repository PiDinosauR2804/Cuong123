from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, List


@dataclass
class TruckStop:
    city: int
    drone_customers: List[int] = field(default_factory=list)


@dataclass
class TruckRoute:
    stops: List[TruckStop] = field(default_factory=list)

    def ensure_depot_start(self) -> None:
        if not self.stops or self.stops[0].city != 0:
            self.stops.insert(0, TruckStop(city=0, drone_customers=[]))


@dataclass
class DroneLeg:
    launch_city: int
    customers: List[int] = field(default_factory=list)


@dataclass
class DroneTrip:
    legs: List[DroneLeg] = field(default_factory=list)

    @property
    def is_multi_visit(self) -> bool:
        return len(self.legs) > 1


@dataclass
class Solution:
    """
    Structured solution compatible with test_similarity.py semantics:
    - truck_routes: each route is list of [city, drone_customers]
    - drone_queue: queue of trips, each trip is list of [launch_city, customers]
    """

    truck_routes: List[TruckRoute] = field(default_factory=list)
    drone_queue: List[DroneTrip] = field(default_factory=list)

    def has_multi_visit_trip(self) -> bool:
        return any(trip.is_multi_visit for trip in self.drone_queue)

    def to_legacy(self) -> List[Any]:
        legacy_routes: List[List[List[Any]]] = []
        for route in self.truck_routes:
            legacy_route: List[List[Any]] = []
            for stop in route.stops:
                legacy_route.append([int(stop.city), [int(c) for c in stop.drone_customers]])
            legacy_routes.append(legacy_route)

        legacy_queue: List[List[List[Any]]] = []
        for trip in self.drone_queue:
            legacy_trip: List[List[Any]] = []
            for leg in trip.legs:
                legacy_trip.append([int(leg.launch_city), [int(c) for c in leg.customers]])
            legacy_queue.append(legacy_trip)

        return [legacy_routes, legacy_queue]

    @classmethod
    def from_legacy(cls, raw_solution: Any, ensure_depot_start: bool = True) -> "Solution":
        if not isinstance(raw_solution, list) or len(raw_solution) < 2:
            raise ValueError("Legacy solution must be [truck_routes, drone_queue].")

        raw_routes = raw_solution[0]
        raw_queue = raw_solution[1]

        if not isinstance(raw_routes, list):
            raise ValueError("Legacy solution[0] must be a list of truck routes.")
        if not isinstance(raw_queue, list):
            raise ValueError("Legacy solution[1] must be a list of drone trips.")

        routes: List[TruckRoute] = []
        for route in raw_routes:
            if not isinstance(route, list):
                raise ValueError(f"Invalid truck route: {route}")
            parsed_stops: List[TruckStop] = []
            for stop in route:
                if not isinstance(stop, list) or len(stop) < 2:
                    raise ValueError(f"Invalid truck stop: {stop}")
                city = int(stop[0])
                customers = [int(c) for c in (stop[1] or [])]
                parsed_stops.append(TruckStop(city=city, drone_customers=customers))
            truck_route = TruckRoute(stops=parsed_stops)
            if ensure_depot_start:
                truck_route.ensure_depot_start()
            routes.append(truck_route)

        queue: List[DroneTrip] = []
        for trip in raw_queue:
            if not isinstance(trip, list):
                raise ValueError(f"Invalid drone trip: {trip}")
            parsed_legs: List[DroneLeg] = []
            for leg in trip:
                if not isinstance(leg, list) or len(leg) < 2:
                    raise ValueError(f"Invalid drone leg: {leg}")
                launch_city = int(leg[0])
                customers = [int(c) for c in (leg[1] or [])]
                parsed_legs.append(DroneLeg(launch_city=launch_city, customers=customers))
            queue.append(DroneTrip(legs=parsed_legs))

        return cls(truck_routes=routes, drone_queue=queue)


def build_empty_solution(number_of_trucks: int) -> Solution:
    if number_of_trucks < 0:
        raise ValueError("number_of_trucks must be >= 0")
    routes = [TruckRoute(stops=[TruckStop(city=0, drone_customers=[])]) for _ in range(number_of_trucks)]
    return Solution(truck_routes=routes, drone_queue=[])
