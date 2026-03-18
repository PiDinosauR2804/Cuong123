from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Tuple
import heapq

from .problem_data import ProblemData
from .solution import Solution


EPS = 1e-9


@dataclass
class ValidationResult:
    feasible: bool
    violations: List[str] = field(default_factory=list)
    trip_flight_wait_time: Dict[int, float] = field(default_factory=dict)


def _to_solution(solution: Solution | Any) -> Solution:
    if isinstance(solution, Solution):
        return solution
    return Solution.from_legacy(solution)


def _city_index_check(city: int, n: int) -> bool:
    return 0 <= city < n


def validate_solution(
    solution: Solution | Any,
    data: ProblemData,
    include_unloading_time_in_schedule: bool = True,
) -> ValidationResult:
    """
    Validate solution with constraints requested by user:
    1) No two drone trips rendezvous at the same launch city.
    2) Total demand per drone trip does not exceed drone capacity.
    3) For each drone trip: travel time + drone waiting for truck <= drone limit time.
    4) Queue order rule for first rendezvous on the same truck must follow truck route order.
    5) A package is valid if truck carries from depot OR drone resupplies at same/before
       customer position on that truck route.
    6) A drone trip can meet multiple trucks but cannot meet one truck more than once.
    7) No customer package can be resupplied more than once in drone queue.
    8) Every customer package must appear exactly once in truck package assignment
       (depot or one truck stop), and package assignment must be on the same truck
       and not after customer position.
    """
    sol = _to_solution(solution)
    violations: List[str] = []

    if len(sol.truck_routes) != data.number_truck:
        violations.append(
            f"Number of truck routes ({len(sol.truck_routes)}) != number_truck ({data.number_truck})"
        )

    n_city = data.number_of_cities
    route_cities: List[List[int]] = []
    city_owner: Dict[int, int] = {}
    city_pos_by_truck: List[Dict[int, int]] = []

    # Build truck route ownership and route positions.
    for t_idx, route in enumerate(sol.truck_routes):
        if not route.stops:
            violations.append(f"Truck {t_idx} has empty route")
            route_cities.append([])
            city_pos_by_truck.append({})
            continue

        if route.stops[0].city != 0:
            violations.append(f"Truck {t_idx} route must start at depot 0")

        cities = [stop.city for stop in route.stops]
        route_cities.append(cities)
        pos_map: Dict[int, int] = {}
        seen_in_route = set()
        for pos, city in enumerate(cities):
            if not _city_index_check(city, n_city):
                violations.append(f"Truck {t_idx} uses out-of-range city {city}")
                continue
            if pos == 0:
                if city != 0:
                    violations.append(f"Truck {t_idx} first city must be depot 0")
            else:
                if city == 0:
                    violations.append(f"Truck {t_idx} has depot inside route at position {pos}")
                if city in seen_in_route:
                    violations.append(f"Customer city {city} appears multiple times in truck {t_idx} route")
                seen_in_route.add(city)
                prev_owner = city_owner.get(city)
                if prev_owner is not None and prev_owner != t_idx:
                    violations.append(
                        f"Customer city {city} appears in multiple truck routes: {prev_owner} and {t_idx}"
                    )
                city_owner[city] = t_idx
            # Keep first position for safety against duplicates in same route.
            if city not in pos_map:
                pos_map[city] = pos
        city_pos_by_truck.append(pos_map)

    missing_on_routes = [city for city in range(1, n_city) if city not in city_owner]
    if missing_on_routes:
        violations.append(
            f"Customers missing from truck routes: {missing_on_routes}"
        )

    package_occurrences: Dict[int, List[Tuple[int, int]]] = {}
    for t_idx, route in enumerate(sol.truck_routes):
        for s_idx, stop in enumerate(route.stops):
            for customer in stop.drone_customers:
                if not _city_index_check(customer, n_city):
                    violations.append(
                        f"Truck {t_idx}, stop {s_idx}: package customer {customer} is out of range"
                    )
                    continue
                if customer == 0:
                    violations.append(f"Truck {t_idx}, stop {s_idx}: depot cannot be a package customer")
                    continue

                package_occurrences.setdefault(customer, []).append((t_idx, s_idx))
                owner = city_owner.get(customer)
                if owner is not None and owner != t_idx:
                    violations.append(
                        f"Truck {t_idx}, stop {s_idx}: package customer {customer} belongs to truck {owner}"
                    )
                if owner is not None and s_idx > 0:
                    customer_pos = city_pos_by_truck[t_idx].get(customer)
                    if customer_pos is not None and s_idx > customer_pos:
                        violations.append(
                            f"Truck {t_idx}, stop {s_idx}: package customer {customer} is assigned after "
                            f"its own visit position {customer_pos}"
                        )

    for customer in range(1, n_city):
        occurrences = package_occurrences.get(customer, [])
        if len(occurrences) == 0:
            violations.append(f"Customer {customer} has no package assignment on any truck stop")
        elif len(occurrences) > 1:
            violations.append(
                f"Customer {customer} appears in multiple truck package assignments: {occurrences}"
            )

    # Queue-order and assignment checks.
    launch_city_owner_trip: Dict[int, int] = {}
    customer_assignment: Dict[int, Tuple[int, int]] = {}
    last_first_launch_pos_by_truck: Dict[int, Tuple[int, int]] = {}
    last_launch_pos_by_truck: Dict[int, Tuple[int, int, int]] = {}

    for trip_idx, trip in enumerate(sol.drone_queue):
        if not trip.legs:
            continue

        total_demand = 0.0
        seen_trucks_in_trip = set()
        first_owner = None
        first_pos = None

        for leg_idx, leg in enumerate(trip.legs):
            launch_city = leg.launch_city
            if not _city_index_check(launch_city, n_city):
                violations.append(f"Trip {trip_idx}, leg {leg_idx} has out-of-range launch city {launch_city}")
                continue

            previous_trip = launch_city_owner_trip.get(launch_city)
            if previous_trip is not None and previous_trip != trip_idx:
                violations.append(
                    f"Launch city {launch_city} is used by multiple drone trips: {previous_trip} and {trip_idx}"
                )
            else:
                launch_city_owner_trip[launch_city] = trip_idx

            owner = city_owner.get(launch_city)
            if owner is None:
                violations.append(
                    f"Trip {trip_idx}, leg {leg_idx} launch city {launch_city} is not on any truck route"
                )
            else:
                if owner in seen_trucks_in_trip:
                    violations.append(
                        f"Trip {trip_idx} meets truck {owner} more than once"
                    )
                seen_trucks_in_trip.add(owner)
                if leg_idx == 0:
                    first_owner = owner
                    first_pos = city_pos_by_truck[owner].get(launch_city)
                current_pos = city_pos_by_truck[owner].get(launch_city)
                if current_pos is not None:
                    prev = last_launch_pos_by_truck.get(owner)
                    if prev is not None:
                        prev_trip_idx, prev_leg_idx, prev_pos = prev
                        if current_pos < prev_pos:
                            violations.append(
                                f"Queue order violation on truck {owner}: "
                                f"trip {trip_idx}, leg {leg_idx} at position {current_pos} appears after "
                                f"trip {prev_trip_idx}, leg {prev_leg_idx} at position {prev_pos}"
                            )
                    last_launch_pos_by_truck[owner] = (trip_idx, leg_idx, current_pos)

            for customer in leg.customers:
                if not _city_index_check(customer, n_city):
                    violations.append(
                        f"Trip {trip_idx}, leg {leg_idx} uses out-of-range customer {customer}"
                    )
                    continue
                if customer == 0:
                    violations.append(f"Trip {trip_idx}, leg {leg_idx} contains depot as a customer")
                    continue

                prev_assign = customer_assignment.get(customer)
                if prev_assign is not None:
                    violations.append(
                        f"Customer {customer} is resupplied more than once: "
                        f"first at trip {prev_assign[0]}, leg {prev_assign[1]} "
                        f"and again at trip {trip_idx}, leg {leg_idx}"
                    )
                else:
                    customer_assignment[customer] = (trip_idx, leg_idx)

                total_demand += data.demands[customer]

                # Rule: customer must be resupplied at same/before its position on the SAME truck route.
                customer_owner = city_owner.get(customer)
                if owner is None or customer_owner is None:
                    continue
                if customer_owner != owner:
                    violations.append(
                        f"Trip {trip_idx}, leg {leg_idx}: customer {customer} belongs to truck {customer_owner}, "
                        f"but launch city {launch_city} belongs to truck {owner}"
                    )
                    continue

                pos_launch = city_pos_by_truck[owner].get(launch_city)
                pos_customer = city_pos_by_truck[owner].get(customer)
                if pos_launch is None or pos_customer is None:
                    continue
                if pos_launch > pos_customer:
                    violations.append(
                        f"Trip {trip_idx}, leg {leg_idx}: launch city {launch_city} is after customer {customer} "
                        f"in truck {owner} route"
                    )
                if customer not in sol.truck_routes[owner].stops[pos_launch].drone_customers:
                    violations.append(
                        f"Trip {trip_idx}, leg {leg_idx}: customer {customer} is not assigned at launch city "
                        f"{launch_city} on truck {owner} route"
                    )

        if total_demand - data.drone_capacity > EPS:
            violations.append(
                f"Trip {trip_idx} total demand exceeds drone capacity: {total_demand} > {data.drone_capacity}"
            )

        # Queue order by first rendezvous on same truck.
        if first_owner is not None and first_pos is not None:
            previous = last_first_launch_pos_by_truck.get(first_owner)
            if previous is not None:
                previous_trip_idx, previous_pos = previous
                if first_pos < previous_pos:
                    violations.append(
                        f"Queue order violation on truck {first_owner}: "
                        f"trip {trip_idx} first city position {first_pos} appears after "
                        f"trip {previous_trip_idx} position {previous_pos} in queue"
                    )
            last_first_launch_pos_by_truck[first_owner] = (trip_idx, first_pos)

    # Time check: travel + drone wait for truck <= drone_limit_time (for each trip).
    trip_flight_wait_time: Dict[int, float] = {}
    if data.number_drone <= 0:
        if sol.drone_queue:
            violations.append("number_drone <= 0 but drone_queue is not empty")
    else:
        drone_heap: List[Tuple[float, int]] = [(0.0, i) for i in range(data.number_drone)]
        heapq.heapify(drone_heap)

        truck_pos = [0 for _ in sol.truck_routes]
        truck_time = [0.0 for _ in sol.truck_routes]

        for trip_idx, trip in enumerate(sol.drone_queue):
            if not trip.legs:
                trip_flight_wait_time[trip_idx] = 0.0
                continue

            drone_available, drone_id = heapq.heappop(drone_heap)
            trip_has_unrecoverable_error = False
            metric_flight_wait = 0.0
            previous_city = 0

            # Compute delayed departure time from depot:
            # max(drone ready at depot, max release date of trip customers,
            #     first-launch truck arrival - drone flight to first launch)
            first_leg = trip.legs[0]
            first_owner = city_owner.get(first_leg.launch_city)
            if first_owner is None:
                violations.append(
                    f"Trip {trip_idx}: first launch city {first_leg.launch_city} is not on any truck route"
                )
                trip_has_unrecoverable_error = True
                drone_clock = drone_available
            else:
                first_target = city_pos_by_truck[first_owner].get(first_leg.launch_city)
                if first_target is None:
                    violations.append(
                        f"Trip {trip_idx}: cannot find first launch city {first_leg.launch_city} on truck {first_owner}"
                    )
                    trip_has_unrecoverable_error = True
                    drone_clock = drone_available
                else:
                    if first_target < truck_pos[first_owner]:
                        violations.append(
                            f"Trip {trip_idx}, leg 0: truck {first_owner} has already passed launch city "
                            f"{first_leg.launch_city}"
                        )
                        trip_has_unrecoverable_error = True
                        drone_clock = drone_available
                    else:
                        while truck_pos[first_owner] < first_target:
                            current_city = route_cities[first_owner][truck_pos[first_owner]]
                            next_city = route_cities[first_owner][truck_pos[first_owner] + 1]
                            truck_time[first_owner] += data.truck_time_matrix[current_city][next_city]
                            truck_pos[first_owner] += 1

                        all_customers = [c for leg in trip.legs for c in leg.customers if 0 <= c < n_city]
                        release_bound = max((data.release_dates[c] for c in all_customers), default=0.0)
                        first_flight = data.drone_time_matrix[0][first_leg.launch_city]
                        drone_clock = max(
                            drone_available,
                            release_bound,
                            truck_time[first_owner] - first_flight,
                        )

            for leg_idx, leg in enumerate(trip.legs):
                launch_city = leg.launch_city
                owner = city_owner.get(launch_city)
                if owner is None:
                    trip_has_unrecoverable_error = True
                    continue

                target_pos = city_pos_by_truck[owner].get(launch_city)
                if target_pos is None:
                    trip_has_unrecoverable_error = True
                    continue

                if target_pos < truck_pos[owner]:
                    violations.append(
                        f"Trip {trip_idx}, leg {leg_idx}: truck {owner} has already passed launch city {launch_city}"
                    )
                    trip_has_unrecoverable_error = True
                    continue

                # Move truck forward to launch city.
                while truck_pos[owner] < target_pos:
                    current_city = route_cities[owner][truck_pos[owner]]
                    next_city = route_cities[owner][truck_pos[owner] + 1]
                    truck_time[owner] += data.truck_time_matrix[current_city][next_city]
                    truck_pos[owner] += 1

                # Drone flies to launch city.
                flight_time = data.drone_time_matrix[previous_city][launch_city]
                drone_arrival = drone_clock + flight_time
                metric_flight_wait += flight_time

                # Drone may need to wait for truck.
                sync_time = max(drone_arrival, truck_time[owner])
                drone_wait = max(0.0, sync_time - drone_arrival)
                metric_flight_wait += drone_wait

                if include_unloading_time_in_schedule:
                    service_end = sync_time + data.unloading_time
                else:
                    service_end = sync_time

                drone_clock = service_end
                truck_time[owner] = service_end
                previous_city = launch_city

            # Return to depot.
            if not trip_has_unrecoverable_error:
                back_time = data.drone_time_matrix[previous_city][0]
                metric_flight_wait += back_time
                drone_clock += back_time

            trip_flight_wait_time[trip_idx] = metric_flight_wait
            if metric_flight_wait - data.drone_limit_time > EPS:
                violations.append(
                    f"Trip {trip_idx} exceeds drone limit time: "
                    f"{metric_flight_wait:.3f} > {data.drone_limit_time}"
                )

            heapq.heappush(drone_heap, (drone_clock, drone_id))

    return ValidationResult(
        feasible=len(violations) == 0,
        violations=violations,
        trip_flight_wait_time=trip_flight_wait_time,
    )
