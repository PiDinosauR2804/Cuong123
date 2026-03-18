from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any, Dict, List, Optional, Tuple
import heapq

from .problem_data import ProblemData
from .solution import Solution, TruckRoute, TruckStop
from .validator import validate_solution


EPS = 1e-9


@dataclass
class FitnessResult:
    feasible: bool
    objective: float
    violations: List[str] = field(default_factory=list)
    truck_departure_time: Dict[int, float] = field(default_factory=dict)
    truck_return_time: Dict[int, float] = field(default_factory=dict)
    drone_departure_time: Dict[int, float] = field(default_factory=dict)
    drone_return_time: Dict[int, float] = field(default_factory=dict)
    drone_flight_wait_energy_time: Dict[int, float] = field(default_factory=dict)


def _to_solution(solution: Solution | Any) -> Solution:
    if isinstance(solution, Solution):
        return solution
    return Solution.from_legacy(solution)


def _max_release(data: ProblemData, customers: List[int]) -> float:
    values = []
    for customer in customers:
        if 0 <= customer < data.number_of_cities:
            values.append(data.release_dates[customer])
    return max(values) if values else 0.0


def _normalize_route(route: TruckRoute) -> TruckRoute:
    # Copy to avoid mutating caller's data.
    copied = TruckRoute(stops=[TruckStop(city=s.city, drone_customers=list(s.drone_customers)) for s in route.stops])
    if not copied.stops or copied.stops[0].city != 0:
        copied.stops.insert(0, TruckStop(city=0, drone_customers=[]))
    if copied.stops[-1].city != 0:
        copied.stops.append(TruckStop(city=0, drone_customers=[]))
    return copied


def evaluate_fitness(solution: Solution | Any, data: ProblemData) -> FitnessResult:
    """
    Compute objective = max(time all trucks return depot, time all drones return depot)
    with departure/synchronization rules requested by user.
    """
    sol = _to_solution(solution)
    validation = validate_solution(sol, data)
    if not validation.feasible:
        return FitnessResult(
            feasible=False,
            objective=float("inf"),
            violations=list(validation.violations),
        )

    violations: List[str] = []

    if len(sol.truck_routes) != data.number_truck:
        violations.append(
            f"Number of truck routes ({len(sol.truck_routes)}) != number_truck ({data.number_truck})"
        )

    normalized_routes = [_normalize_route(route) for route in sol.truck_routes]
    if not normalized_routes:
        return FitnessResult(feasible=False, objective=float("inf"), violations=["No truck route found"])

    route_cities: List[List[int]] = []
    pending: List[List[List[int]]] = []
    city_owner: Dict[int, int] = {}
    city_pos_by_truck: List[Dict[int, int]] = []

    # Build maps: city -> truck, city position on truck route.
    for t_idx, route in enumerate(normalized_routes):
        cities = [stop.city for stop in route.stops]
        route_cities.append(cities)
        pending.append([list(stop.drone_customers) for stop in route.stops])

        pos_map: Dict[int, int] = {}
        for pos, city in enumerate(cities):
            if city < 0 or city >= data.number_of_cities:
                violations.append(f"Truck {t_idx} has out-of-range city {city}")
                continue
            if pos == 0 or pos == len(cities) - 1:
                if city != 0:
                    violations.append(f"Truck {t_idx} must start/end at depot 0")
                continue

            if city == 0:
                violations.append(f"Truck {t_idx} has depot inside route at position {pos}")
                continue

            if city not in pos_map:
                pos_map[city] = pos

            previous_owner = city_owner.get(city)
            if previous_owner is not None and previous_owner != t_idx:
                violations.append(
                    f"Customer city {city} appears in multiple truck routes: {previous_owner} and {t_idx}"
                )
            city_owner[city] = t_idx

        city_pos_by_truck.append(pos_map)

    truck_departure_time: Dict[int, float] = {}
    truck_return_time: Dict[int, float] = {}
    drone_departure_time: Dict[int, float] = {}
    drone_return_time: Dict[int, float] = {}
    drone_energy_time: Dict[int, float] = {}

    truck_idx = [0 for _ in normalized_routes]
    truck_time = [0.0 for _ in normalized_routes]

    # Truck starts from depot:
    # departure time = max release date of packages loaded at depot.
    for t_idx in range(len(normalized_routes)):
        depot_packages = pending[t_idx][0]
        depart = _max_release(data, depot_packages)
        truck_departure_time[t_idx] = depart
        pending[t_idx][0] = []
        truck_time[t_idx] = depart

        if len(route_cities[t_idx]) >= 2:
            next_city = route_cities[t_idx][1]
            if next_city != 0:
                truck_time[t_idx] += data.truck_time_matrix[0][next_city]
            truck_idx[t_idx] = 1

    def advance_truck_through_empty(t_idx: int) -> None:
        # Truck moves continuously while current stop has no pending drone package.
        while True:
            idx = truck_idx[t_idx]
            if idx >= len(route_cities[t_idx]) - 1:
                return
            current_city = route_cities[t_idx][idx]
            if current_city == 0:
                return
            if pending[t_idx][idx]:
                return

            next_city = route_cities[t_idx][idx + 1]
            truck_time[t_idx] += data.truck_time_matrix[current_city][next_city]
            truck_idx[t_idx] += 1

    def project_arrival_to_city(t_idx: int, target_pos: int) -> Tuple[Optional[float], Optional[int]]:
        # Return (arrival_time, blocking_city). If blocking_city is not None,
        # truck cannot currently reach target due to pending packages.
        if target_pos < truck_idx[t_idx]:
            return None, route_cities[t_idx][truck_idx[t_idx]]

        sim_idx = truck_idx[t_idx]
        sim_time = truck_time[t_idx]
        while sim_idx < target_pos:
            if sim_idx >= len(route_cities[t_idx]) - 1:
                return None, None
            current_city = route_cities[t_idx][sim_idx]
            if current_city == 0:
                return None, None
            if pending[t_idx][sim_idx]:
                return None, current_city
            next_city = route_cities[t_idx][sim_idx + 1]
            sim_time += data.truck_time_matrix[current_city][next_city]
            sim_idx += 1
        return sim_time, None

    def move_truck_to_city(t_idx: int, target_pos: int, trip_idx: int, leg_idx: int) -> bool:
        if target_pos < truck_idx[t_idx]:
            violations.append(
                f"Trip {trip_idx}, leg {leg_idx}: truck {t_idx} already passed launch city position {target_pos}"
            )
            return False

        while truck_idx[t_idx] < target_pos:
            if truck_idx[t_idx] >= len(route_cities[t_idx]) - 1:
                violations.append(
                    f"Trip {trip_idx}, leg {leg_idx}: truck {t_idx} cannot reach target position {target_pos}"
                )
                return False
            current_city = route_cities[t_idx][truck_idx[t_idx]]
            if current_city == 0:
                violations.append(
                    f"Trip {trip_idx}, leg {leg_idx}: truck {t_idx} reached depot before target position {target_pos}"
                )
                return False
            if pending[t_idx][truck_idx[t_idx]]:
                violations.append(
                    f"Trip {trip_idx}, leg {leg_idx}: truck {t_idx} blocked at city {current_city} "
                    f"with undelivered package(s) {pending[t_idx][truck_idx[t_idx]]}"
                )
                return False
            next_city = route_cities[t_idx][truck_idx[t_idx] + 1]
            truck_time[t_idx] += data.truck_time_matrix[current_city][next_city]
            truck_idx[t_idx] += 1
        return True

    if data.number_drone <= 0 and sol.drone_queue:
        violations.append("number_drone <= 0 but drone_queue is not empty")
    drone_heap: List[Tuple[float, int]] = [(0.0, i) for i in range(max(data.number_drone, 0))]
    heapq.heapify(drone_heap)

    # Process drone queue in queue order.
    for trip_idx, trip in enumerate(sol.drone_queue):
        for t in range(len(normalized_routes)):
            advance_truck_through_empty(t)

        if not trip.legs:
            drone_departure_time[trip_idx] = 0.0
            drone_return_time[trip_idx] = 0.0
            drone_energy_time[trip_idx] = 0.0
            continue

        if not drone_heap:
            violations.append(f"Trip {trip_idx}: no drone available")
            drone_departure_time[trip_idx] = float("inf")
            drone_return_time[trip_idx] = float("inf")
            drone_energy_time[trip_idx] = float("inf")
            continue

        first_leg = trip.legs[0]
        first_owner = city_owner.get(first_leg.launch_city)
        first_target_pos = None
        truck_arrive_first = None

        if first_owner is None:
            violations.append(
                f"Trip {trip_idx}: first launch city {first_leg.launch_city} is not in any truck route"
            )
        else:
            first_target_pos = city_pos_by_truck[first_owner].get(first_leg.launch_city)
            if first_target_pos is None:
                violations.append(
                    f"Trip {trip_idx}: cannot locate first launch city {first_leg.launch_city} on truck {first_owner}"
                )
            else:
                truck_arrive_first, blocking_city = project_arrival_to_city(first_owner, first_target_pos)
                if truck_arrive_first is None:
                    if blocking_city is not None:
                        violations.append(
                            f"Trip {trip_idx}: truck {first_owner} cannot reach first launch city "
                            f"{first_leg.launch_city} because it is blocked at city {blocking_city}"
                        )
                    else:
                        violations.append(
                            f"Trip {trip_idx}: truck {first_owner} cannot reach first launch city {first_leg.launch_city}"
                        )

        all_customers = []
        for leg in trip.legs:
            all_customers.extend(leg.customers)
        release_bound = _max_release(data, all_customers)

        drone_ready_time, drone_id = heapq.heappop(drone_heap)
        if truck_arrive_first is None:
            depart_time = max(drone_ready_time, release_bound)
        else:
            leg0_flight = data.drone_time_matrix[0][first_leg.launch_city]
            depart_time = max(drone_ready_time, release_bound, truck_arrive_first - leg0_flight)
        drone_departure_time[trip_idx] = depart_time

        drone_clock = depart_time
        previous_city = 0
        trip_energy = 0.0  # travel + drone waiting only (unloading excluded)

        for leg_idx, leg in enumerate(trip.legs):
            launch = leg.launch_city
            owner = city_owner.get(launch)
            if owner is None:
                violations.append(
                    f"Trip {trip_idx}, leg {leg_idx}: launch city {launch} is not in any truck route"
                )
                continue

            target_pos = city_pos_by_truck[owner].get(launch)
            if target_pos is None:
                violations.append(
                    f"Trip {trip_idx}, leg {leg_idx}: cannot locate launch city {launch} on truck {owner}"
                )
                continue

            if not move_truck_to_city(owner, target_pos, trip_idx, leg_idx):
                continue

            # Drone flies to launch city.
            fly_time = data.drone_time_matrix[previous_city][launch]
            drone_arrival = drone_clock + fly_time
            trip_energy += fly_time

            # Sync with truck at launch point.
            sync_time = max(drone_arrival, truck_time[owner])
            drone_wait = max(0.0, sync_time - drone_arrival)
            trip_energy += drone_wait

            # Unloading applies to both truck and drone schedule, but not to drone energy.
            service_end = sync_time + data.unloading_time
            truck_time[owner] = service_end
            drone_clock = service_end

            # Remove delivered packages from this launch point.
            for customer in leg.customers:
                if customer < 0 or customer >= data.number_of_cities:
                    violations.append(
                        f"Trip {trip_idx}, leg {leg_idx}: customer {customer} out of range"
                    )
                    continue

                removed = False
                if customer in pending[owner][target_pos]:
                    pending[owner][target_pos].remove(customer)
                    removed = True
                else:
                    # Fallback: find same customer in remaining stops of this truck.
                    for s in range(target_pos, len(pending[owner])):
                        if customer in pending[owner][s]:
                            pending[owner][s].remove(customer)
                            removed = True
                            break
                if not removed:
                    violations.append(
                        f"Trip {trip_idx}, leg {leg_idx}: customer {customer} not found in pending packages "
                        f"of truck {owner}"
                    )

            previous_city = launch
            advance_truck_through_empty(owner)

        # Drone returns depot.
        back_time = data.drone_time_matrix[previous_city][0]
        drone_clock += back_time
        trip_energy += back_time

        drone_return_time[trip_idx] = drone_clock
        drone_energy_time[trip_idx] = trip_energy
        heapq.heappush(drone_heap, (drone_clock, drone_id))

    # Finish truck routes to depot.
    for t_idx in range(len(normalized_routes)):
        while truck_idx[t_idx] < len(route_cities[t_idx]) - 1:
            current_city = route_cities[t_idx][truck_idx[t_idx]]
            if current_city != 0 and pending[t_idx][truck_idx[t_idx]]:
                violations.append(
                    f"Truck {t_idx} cannot leave city {current_city}; pending package(s) not supplied: "
                    f"{pending[t_idx][truck_idx[t_idx]]}"
                )
                break
            next_city = route_cities[t_idx][truck_idx[t_idx] + 1]
            truck_time[t_idx] += data.truck_time_matrix[current_city][next_city]
            truck_idx[t_idx] += 1
        truck_return_time[t_idx] = truck_time[t_idx]

    # Any pending package left means schedule incomplete.
    for t_idx in range(len(normalized_routes)):
        for s_idx in range(1, len(pending[t_idx]) - 1):
            if pending[t_idx][s_idx]:
                city = route_cities[t_idx][s_idx]
                violations.append(
                    f"Pending package(s) still remain at truck {t_idx}, city {city}: {pending[t_idx][s_idx]}"
                )

    max_truck_return = max(truck_return_time.values()) if truck_return_time else 0.0
    max_drone_return = 0.0
    if drone_heap:
        max_drone_return = max(ready for ready, _ in drone_heap)
    if drone_return_time:
        max_drone_return = max(max_drone_return, max(drone_return_time.values()))

    objective = max(max_truck_return, max_drone_return)
    feasible = len(violations) == 0
    if not feasible:
        objective = float("inf")

    return FitnessResult(
        feasible=feasible,
        objective=objective,
        violations=violations,
        truck_departure_time=truck_departure_time,
        truck_return_time=truck_return_time,
        drone_departure_time=drone_departure_time,
        drone_return_time=drone_return_time,
        drone_flight_wait_energy_time=drone_energy_time,
    )


def fitness(solution: Solution | Any, data: ProblemData) -> float:
    return evaluate_fitness(solution, data).objective
