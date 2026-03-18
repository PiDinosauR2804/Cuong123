from Source_revised.Function import ProblemData, _build_matrix, _to_travel_time
from Source_revised.Solution import normalize_solution, validate_solution


coords = [(0.0, 0.0), (1.0, 0.0), (3.0, 0.0), (2.0, 0.0)]
demands = [0.0, 1.0, 1.0, 1.0]
releases = [0.0, 0.0, 0.0, 0.0]

truck_dist = _build_matrix(coords, metric="manhattan")
drone_dist = _build_matrix(coords, metric="euclid")

data = ProblemData(
    number_truck=1,
    number_drone=1,
    truck_speed=1.0,
    drone_speed=1.0,
    drone_capacity=10.0,
    drone_limit_time=100.0,
    unloading_time=0.0,
    coords=coords,
    demands=demands,
    release_dates=releases,
    truck_manhattan_distance=truck_dist,
    drone_euclid_distance=drone_dist,
    truck_travel_time=_to_travel_time(truck_dist, 1.0),
    drone_travel_time=_to_travel_time(drone_dist, 1.0),
)

# Truck route order: 1 -> 3 -> 2.
# Drone queue intentionally violates first-sync ordering by placing trip(3) before trip(1).
raw_solution = [
    [
        [
            [0, []],
            [1, [1]],
            [3, [3]],
            [2, [2]],
        ]
    ],
    [
        [[3, [3]]],
        [[1, [1]]],
        [[2, [2]]],
    ],
]

trucks, drone_trips = normalize_solution(raw_solution)
violations = validate_solution(data, trucks, drone_trips)
print("feasible=", len(violations) == 0)
print("violations=")
for v in violations:
    print("-", v)
