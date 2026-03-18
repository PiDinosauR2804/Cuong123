from __future__ import annotations

from dataclasses import dataclass
from typing import Any, Dict, List, Optional, Tuple
import copy
import heapq

from .Function import ProblemData


@dataclass
class TruckStop:
    city: int
    drone_customers: List[int]


@dataclass
class DroneLeg:
    launch_city: int
    customers: List[int]


@dataclass
class SimulationResult:
    feasible: bool
    violations: List[str]
    truck_start_time: Dict[int, float]
    truck_time: Dict[int, float]
    drone_trip_start_time: Dict[int, float]
    truck_wait_by_city: Dict[int, float]
    drone_wait_by_city: Dict[int, float]
    multi_visit_trip_count: int
    system_completion_time: float


def _max_release(data: ProblemData, customers: List[int]) -> float:
    if not customers:
        return 0.0
    return max(data.release_dates[c] for c in customers)


def _sum_demand(data: ProblemData, customers: List[int]) -> float:
    return sum(data.demands[c] for c in customers)


def normalize_solution(raw_solution: Any) -> Tuple[List[List[TruckStop]], List[List[DroneLeg]]]:
    if not isinstance(raw_solution, list) or len(raw_solution) < 2:
        raise ValueError("Solution must be [truck_routes, drone_trips]")

    raw_trucks = raw_solution[0]
    raw_drone_trips = raw_solution[1]

    trucks: List[List[TruckStop]] = []
    for route in raw_trucks:
        parsed_route: List[TruckStop] = []
        for stop in route:
            if not isinstance(stop, list) or len(stop) < 2:
                raise ValueError(f"Invalid truck stop format: {stop}")
            city = int(stop[0])
            customers = [int(c) for c in (stop[1] or [])]
            parsed_route.append(TruckStop(city=city, drone_customers=customers))
        if not parsed_route or parsed_route[0].city != 0:
            parsed_route.insert(0, TruckStop(city=0, drone_customers=[]))
        trucks.append(parsed_route)

    drone_trips: List[List[DroneLeg]] = []
    for trip in raw_drone_trips:
        parsed_trip: List[DroneLeg] = []
        for leg in trip:
            if not isinstance(leg, list) or len(leg) < 2:
                raise ValueError(f"Invalid drone leg format: {leg}")
            launch_city = int(leg[0])
            customers = [int(c) for c in (leg[1] or [])]
            parsed_trip.append(DroneLeg(launch_city=launch_city, customers=customers))
        drone_trips.append(parsed_trip)

    return trucks, drone_trips


def _city_owner_truck(trucks: List[List[TruckStop]], city: int) -> Optional[int]:
    for t_idx, route in enumerate(trucks):
        for stop in route:
            if stop.city == city:
                return t_idx
    return None


def validate_solution(data: ProblemData, trucks: List[List[TruckStop]], drone_trips: List[List[DroneLeg]]) -> List[str]:
    violations: List[str] = []
    city_to_route_pos: Dict[int, Tuple[int, int]] = {}
    launch_city_owner_trip: Dict[int, int] = {}
    drone_customer_first_assignment: Dict[int, Tuple[int, int]] = {}

    visited = []
    for t_idx, route in enumerate(trucks):
        for s_idx, stop in enumerate(route):
            city_to_route_pos[stop.city] = (t_idx, s_idx)
            if s_idx == 0:
                if stop.city != 0:
                    violations.append(f"Truck {t_idx} route must start at depot 0")
            else:
                if stop.city == 0:
                    violations.append(f"Truck {t_idx} has depot inside route at position {s_idx}")
                visited.append(stop.city)

    city_set = set(range(1, data.number_of_cities))
    visited_set = set(visited)

    missing = sorted(city_set - visited_set)
    duplicates = sorted([c for c in visited_set if visited.count(c) > 1])
    if missing:
        violations.append(f"Missing customer(s) in truck routes: {missing[:10]}")
    if duplicates:
        violations.append(f"Duplicated customer(s) in truck routes: {duplicates[:10]}")

    # Queue ordering rule: for trips that first rendezvous with the same truck,
    # queue order must follow that truck's route order.
    last_first_launch_pos_by_truck: Dict[int, Tuple[int, int]] = {}

    for trip_idx, trip in enumerate(drone_trips):
        if not trip:
            continue
        trip_demand = 0.0
        seen_trucks_in_trip = set()
        first_leg_owner: Optional[int] = None
        first_leg_pos: Optional[int] = None
        for leg_idx, leg in enumerate(trip):
            prev_trip_idx = launch_city_owner_trip.get(leg.launch_city)
            if prev_trip_idx is not None and prev_trip_idx != trip_idx:
                violations.append(
                    f"Launch city {leg.launch_city} is used by multiple drone trips: {prev_trip_idx} and {trip_idx}"
                )
            else:
                launch_city_owner_trip[leg.launch_city] = trip_idx

            for customer in leg.customers:
                prev_assignment = drone_customer_first_assignment.get(customer)
                if prev_assignment is not None:
                    prev_trip, prev_leg = prev_assignment
                    violations.append(
                        f"Customer {customer} is assigned to multiple drone deliveries: "
                        f"trip {prev_trip}, leg {prev_leg} and trip {trip_idx}, leg {leg_idx}"
                    )
                else:
                    drone_customer_first_assignment[customer] = (trip_idx, leg_idx)

            leg_demand = _sum_demand(data, leg.customers)
            trip_demand += leg_demand
            if leg_demand - data.drone_capacity > 1e-9:
                violations.append(
                    f"Trip {trip_idx}, leg {leg_idx} exceeds drone capacity: {leg_demand} > {data.drone_capacity}"
                )
            owner = _city_owner_truck(trucks, leg.launch_city)
            if owner is None:
                violations.append(f"Trip {trip_idx}, leg {leg_idx} launch city not in truck route: {leg.launch_city}")
            else:
                if leg_idx == 0:
                    first_leg_owner = owner
                    route_pos = city_to_route_pos.get(leg.launch_city)
                    if route_pos is not None:
                        first_leg_pos = route_pos[1]
                if owner in seen_trucks_in_trip:
                    violations.append(
                        f"Trip {trip_idx} meets truck {owner} more than once (duplicate rendezvous in same trip)"
                    )
                else:
                    seen_trucks_in_trip.add(owner)

        if first_leg_owner is not None and first_leg_pos is not None:
            prev = last_first_launch_pos_by_truck.get(first_leg_owner)
            if prev is not None:
                prev_trip_idx, prev_pos = prev
                if first_leg_pos < prev_pos:
                    violations.append(
                        f"Queue order violates truck {first_leg_owner} itinerary: "
                        f"trip {trip_idx} first launch position {first_leg_pos} appears after "
                        f"trip {prev_trip_idx} position {prev_pos}"
                    )
            last_first_launch_pos_by_truck[first_leg_owner] = (trip_idx, first_leg_pos)

        if trip_demand - data.drone_capacity > 1e-9:
            violations.append(f"Trip {trip_idx} total demand exceeds drone capacity: {trip_demand} > {data.drone_capacity}")

    return violations


def evaluate_solution(raw_solution: Any, data: ProblemData) -> SimulationResult:
    trucks, drone_trips = normalize_solution(raw_solution)
    violations = validate_solution(data, trucks, drone_trips)

    base_path = copy.deepcopy(trucks)
    truck_positions: List[List[int]] = []
    for route in base_path:
        path = [stop.city for stop in route] + [0]
        truck_positions.append(path)

    truck_current_idx = [0] * len(base_path)
    truck_time = [0.0] * len(base_path)
    truck_start = {i: 0.0 for i in range(len(base_path))}

    drone_heap: List[Tuple[float, int]] = []
    drone_count = data.number_drone
    for i in range(drone_count):
        heapq.heappush(drone_heap, (0.0, i))

    trip_start_time: Dict[int, float] = {}
    truck_wait_by_city: Dict[int, float] = {}
    drone_wait_by_city: Dict[int, float] = {}
    city_service_ready_time: Dict[int, float] = {}

    # Initial truck move from depot to first customer.
    for i in range(len(base_path)):
        if len(truck_positions[i]) <= 2:
            continue
        first_city = truck_positions[i][1]
        depart = _max_release(data, base_path[i][0].drone_customers)
        truck_start[i] = depart
        truck_time[i] = depart + data.truck_travel_time[0][first_city]
        base_path[i][0].drone_customers.clear()
        truck_current_idx[i] = 1

    pending_trips = copy.deepcopy(drone_trips)
    trip_index = 0

    while True:
        # Trucks keep moving until they reach a point with pending drone packages.
        for i in range(len(base_path)):
            if truck_current_idx[i] >= len(base_path[i]):
                continue
            while truck_current_idx[i] < len(base_path[i]) and base_path[i][truck_current_idx[i]].drone_customers == []:
                curr_city = truck_positions[i][truck_current_idx[i]]
                if curr_city == 0:
                    break
                next_city = truck_positions[i][truck_current_idx[i] + 1]
                truck_time[i] += data.truck_travel_time[curr_city][next_city]
                if next_city != 0:
                    truck_current_idx[i] += 1
                else:
                    truck_current_idx[i] = len(base_path[i])
                    break

        finished = 0
        for i in range(len(base_path)):
            if truck_current_idx[i] >= len(base_path[i]):
                finished += 1
                continue
            curr_city = truck_positions[i][truck_current_idx[i]]
            if curr_city == 0:
                finished += 1
        if finished == len(base_path):
            break

        if not pending_trips:
            violations.append("Simulation ended with truck pending packages but no drone trip left")
            break

        # Dynamic queue policy:
        # among currently active trips, dispatch the (trip, drone) pair with
        # smallest feasible departure time from depot:
        # max(drone_ready_at_depot, release_time_of_trip, truck_arrival_at_first_launch - flight_time_depot_to_launch).
        active_candidates: List[int] = []
        for idx, trip in enumerate(pending_trips):
            if not trip:
                continue
            first_launch = trip[0].launch_city
            owner = _city_owner_truck(base_path, first_launch)
            if owner is None or truck_current_idx[owner] >= len(base_path[owner]):
                continue
            truck_city = truck_positions[owner][truck_current_idx[owner]]
            if truck_city == first_launch:
                active_candidates.append(idx)

        if not active_candidates:
            violations.append("No active drone trip can be launched from current truck positions")
            break

        heap_snapshot = sorted(drone_heap)
        best_key: Optional[Tuple[float, int, int]] = None
        chosen_trip_idx = -1
        chosen_drone_available = 0.0
        chosen_drone_id = -1
        chosen_depart_time = 0.0

        for idx in active_candidates:
            trip = pending_trips[idx]
            all_customers = [c for leg in trip for c in leg.customers]
            release_time = _max_release(data, all_customers)

            first_launch = trip[0].launch_city
            owner = _city_owner_truck(base_path, first_launch)
            if owner is None:
                continue
            truck_arrival = truck_time[owner]
            first_leg_flight = data.drone_travel_time[0][first_launch]

            for drone_available, drone_id in heap_snapshot:
                depart_time = max(drone_available, release_time, truck_arrival - first_leg_flight)
                key = (depart_time, drone_id, idx)
                if best_key is None or key < best_key:
                    best_key = key
                    chosen_trip_idx = idx
                    chosen_drone_available = drone_available
                    chosen_drone_id = drone_id
                    chosen_depart_time = depart_time

        if chosen_trip_idx < 0:
            violations.append("No dispatchable drone-trip candidate found")
            break

        trip = pending_trips.pop(chosen_trip_idx)
        drone_heap.remove((chosen_drone_available, chosen_drone_id))
        heapq.heapify(drone_heap)

        drone_available = chosen_drone_available
        drone_id = chosen_drone_id
        all_customers = [c for leg in trip for c in leg.customers]
        # Start time already accounts for ready/release/(a-b) policy.
        start = chosen_depart_time
        trip_start_time[trip_index] = start

        last_city = 0
        for leg in trip:
            owner = _city_owner_truck(base_path, leg.launch_city)
            if owner is None:
                violations.append(f"Trip {trip_index}: launch city {leg.launch_city} not found")
                continue

            truck_city = truck_positions[owner][truck_current_idx[owner]]
            if truck_city != leg.launch_city:
                violations.append(
                    f"Trip {trip_index}: truck {owner} not at launch city {leg.launch_city} (currently at {truck_city})"
                )
                continue
            start += data.drone_travel_time[last_city][truck_city]
            last_city = truck_city

            # Remove delivered customers from first future stop that contains them.
            for customer in leg.customers:
                removed = False
                for t_idx in range(len(base_path)):
                    for s_idx in range(truck_current_idx[t_idx], len(base_path[t_idx])):
                        if customer in base_path[t_idx][s_idx].drone_customers:
                            base_path[t_idx][s_idx].drone_customers.remove(customer)
                            removed = True
                            break
                    if removed:
                        break
                if not removed:
                    violations.append(f"Trip {trip_index}: customer {customer} not found in pending package lists")

            drone_arrival = start
            truck_arrival = truck_time[owner]
            # A truck handles one drone at a time at each city, so next service can only
            # start when previous one finishes.
            ready = city_service_ready_time.get(truck_city, 0.0)
            sync = max(drone_arrival, truck_arrival, ready)

            city = truck_city
            truck_wait_by_city[city] = truck_wait_by_city.get(city, 0.0) + max(0.0, sync - truck_arrival)
            drone_wait_by_city[city] = drone_wait_by_city.get(city, 0.0) + max(0.0, sync - drone_arrival)

            start = sync + data.unloading_time
            city_service_ready_time[city] = start

            first_move = True
            while truck_current_idx[owner] < len(base_path[owner]) and base_path[owner][truck_current_idx[owner]].drone_customers == []:
                curr_city = truck_positions[owner][truck_current_idx[owner]]
                if curr_city == 0:
                    break
                next_city = truck_positions[owner][truck_current_idx[owner] + 1]
                move = data.truck_travel_time[curr_city][next_city]
                if first_move:
                    truck_time[owner] = start + move
                    first_move = False
                else:
                    truck_time[owner] += move

                if next_city != 0:
                    truck_current_idx[owner] += 1
                else:
                    truck_current_idx[owner] = len(base_path[owner])
                    break

        trip_duration = start + data.drone_travel_time[last_city][0] - trip_start_time[trip_index]
        if trip_duration - data.drone_limit_time > 1e-9:
            violations.append(
                f"Trip {trip_index} exceeds drone limit time: {trip_duration:.3f} > {data.drone_limit_time}"
            )

        heapq.heappush(drone_heap, (start + data.drone_travel_time[last_city][0], drone_id))
        trip_index += 1

    drone_finish = max(t for t, _ in drone_heap) if drone_heap else 0.0
    system_completion = max(max(truck_time) if truck_time else 0.0, drone_finish)

    return SimulationResult(
        feasible=len(violations) == 0,
        violations=violations,
        truck_start_time=truck_start,
        truck_time={i: t for i, t in enumerate(truck_time)},
        drone_trip_start_time=trip_start_time,
        truck_wait_by_city=truck_wait_by_city,
        drone_wait_by_city=drone_wait_by_city,
        multi_visit_trip_count=sum(1 for trip in drone_trips if len(trip) > 1),
        system_completion_time=system_completion,
    )


def fitness(raw_solution: Any, data: ProblemData) -> float:
    """Return feasible makespan; infeasible solutions get +inf."""
    result = evaluate_solution(raw_solution, data)
    if not result.feasible:
        return float("inf")
    return result.system_completion_time


