import csv
import ast
from statistics import mean

# Input and output files
input_csv = "batch_result_revised.csv"
output_csv = "batch_result_revised_summary.csv"

# Helper to parse list from string
parse_list = lambda s: ast.literal_eval(s) if s and s != 'None' else []
parse_dict = lambda s: ast.literal_eval(s) if s and s != 'None' else {}

def get_trip_times(trip_distances, trip_waits=None):
    # trip_waits: list of truck wait times for each trip (same length as trip_distances)
    if not trip_distances:
        return 0, 0
    if trip_waits is None or len(trip_waits) != len(trip_distances):
        # Fallback: just use distances
        return mean(trip_distances), max(trip_distances)
    # Sum flight + truck wait for each trip
    trip_times = [d + w for d, w in zip(trip_distances, trip_waits)]
    return mean(trip_times), max(trip_times)

def get_wait_stats(wait_dict):
    if not wait_dict:
        return 0, 0
    vals = list(wait_dict.values())
    return mean(vals), max(vals)

def count_drone_trips(drone_queue):
    return len(drone_queue)

def count_multi_visit_trips(drone_queue):
    return sum(1 for trip in drone_queue if len(trip) > 1)

with open(input_csv, newline='', encoding='utf-8') as f_in, open(output_csv, 'w', newline='', encoding='utf-8') as f_out:
    reader = csv.DictReader(f_in)
    fieldnames = [
        'instance', 'drone_capacity', 'drone_limit_time', 'theta', 'run',
        'best_fitness', 'best_sol', 'best_multi_visit_fitness', 'best_multi_visit_sol',
        'best_sol_drone_trip_count', 'best_sol_multi_visit_drone_trip_count',
        'best_sol_avg_truck_wait', 'best_sol_max_truck_wait',
        'best_sol_avg_drone_wait', 'best_sol_max_drone_wait',
        'best_sol_objective', 'best_sol_avg_drone_trip_time', 'best_sol_max_drone_trip_time',
        'best_multi_visit_drone_trip_count', 'best_multi_visit_avg_truck_wait', 'best_multi_visit_max_truck_wait',
        'best_multi_visit_avg_drone_wait', 'best_multi_visit_max_drone_wait',
        'best_multi_visit_objective', 'best_multi_visit_avg_drone_trip_time', 'best_multi_visit_max_drone_trip_time',
        'best_sol_violated_trips', 'best_multi_visit_violated_trips'
    ]
    writer = csv.DictWriter(f_out, fieldnames=fieldnames)
    writer.writeheader()

    for row in reader:
        # Parse best_sol
        best_sol = parse_list(row.get('best_sol', ''))
        best_multi_visit_sol = parse_list(row.get('best_multi_visit_sol', ''))
        # Parse trip distances and truck wait by point (as list)
        best_sol_trip_distances = parse_list(row.get('best_sol_drone_trip_distances', ''))
        best_multi_visit_trip_distances = parse_list(row.get('best_multi_visit_sol_drone_trip_distances', ''))
        # Try to get per-trip truck wait times (if available as a list, else fallback to 0s)
        best_sol_truck_waits = parse_list(row.get('best_sol_truck_wait_by_point', ''))
        best_multi_visit_truck_waits = parse_list(row.get('best_multi_visit_sol_truck_wait_by_point', ''))
        # If not a list, fallback to zeros
        if not isinstance(best_sol_truck_waits, list):
            best_sol_truck_waits = [0] * len(best_sol_trip_distances)
        if not isinstance(best_multi_visit_truck_waits, list):
            best_multi_visit_truck_waits = [0] * len(best_multi_visit_trip_distances)
        # Parse wait dicts
        best_sol_truck_wait = parse_dict(row.get('best_sol_truck_wait_by_point', ''))
        best_sol_drone_wait = parse_dict(row.get('best_sol_drone_wait_by_point', ''))
        best_multi_visit_truck_wait = parse_dict(row.get('best_multi_visit_sol_truck_wait_by_point', ''))
        best_multi_visit_drone_wait = parse_dict(row.get('best_multi_visit_sol_drone_wait_by_point', ''))
        # Drone trip counts
        best_sol_drone_trip_count = count_drone_trips(best_sol[1]) if best_sol else 0
        best_sol_multi_visit_drone_trip_count = count_multi_visit_trips(best_sol[1]) if best_sol else 0
        best_multi_visit_drone_trip_count = count_drone_trips(best_multi_visit_sol[1]) if best_multi_visit_sol else 0
        # Wait stats
        best_sol_avg_truck_wait, best_sol_max_truck_wait = get_wait_stats(best_sol_truck_wait)
        best_sol_avg_drone_wait, best_sol_max_drone_wait = get_wait_stats(best_sol_drone_wait)
        best_multi_visit_avg_truck_wait, best_multi_visit_max_truck_wait = get_wait_stats(best_multi_visit_truck_wait)
        best_multi_visit_avg_drone_wait, best_multi_visit_max_drone_wait = get_wait_stats(best_multi_visit_drone_wait)
        # Trip time stats (flight + truck wait for each trip)
        best_sol_trip_times = [d + w for d, w in zip(best_sol_trip_distances, best_sol_truck_waits)]
        best_multi_visit_trip_times = [d + w for d, w in zip(best_multi_visit_trip_distances, best_multi_visit_truck_waits)]
        best_sol_avg_drone_trip_time = mean(best_sol_trip_times) if best_sol_trip_times else 0
        best_sol_max_drone_trip_time = max(best_sol_trip_times) if best_sol_trip_times else 0
        best_multi_visit_avg_drone_trip_time = mean(best_multi_visit_trip_times) if best_multi_visit_trip_times else 0
        best_multi_visit_max_drone_trip_time = max(best_multi_visit_trip_times) if best_multi_visit_trip_times else 0

        # Check for violated trips (drone trip time > drone_limit_time)
        try:
            drone_limit_time = float(row.get('drone_limit_time', 0))
        except Exception:
            drone_limit_time = 0
        best_sol_violated_trips = [i for i, t in enumerate(best_sol_trip_times) if t > drone_limit_time]
        best_multi_visit_violated_trips = [i for i, t in enumerate(best_multi_visit_trip_times) if t > drone_limit_time]
        # Objective values
        best_sol_objective = row.get('best_fitness', '')
        best_multi_visit_objective = row.get('best_multi_visit_fitness', '')
        # Write summary row
        writer.writerow({
            'instance': row.get('instance', ''),
            'drone_capacity': row.get('drone_capacity', ''),
            'drone_limit_time': row.get('drone_limit_time', ''),
            'theta': row.get('theta', ''),
            'run': row.get('run', ''),
            'best_fitness': row.get('best_fitness', ''),
            'best_sol': row.get('best_sol', ''),
            'best_multi_visit_fitness': row.get('best_multi_visit_fitness', ''),
            'best_multi_visit_sol': row.get('best_multi_visit_sol', ''),
            'best_sol_drone_trip_count': best_sol_drone_trip_count,
            'best_sol_multi_visit_drone_trip_count': best_sol_multi_visit_drone_trip_count,
            'best_sol_avg_truck_wait': best_sol_avg_truck_wait,
            'best_sol_max_truck_wait': best_sol_max_truck_wait,
            'best_sol_avg_drone_wait': best_sol_avg_drone_wait,
            'best_sol_max_drone_wait': best_sol_max_drone_wait,
            'best_sol_objective': best_sol_objective,
            'best_sol_avg_drone_trip_time': best_sol_avg_drone_trip_time,
            'best_sol_max_drone_trip_time': best_sol_max_drone_trip_time,
            'best_multi_visit_drone_trip_count': best_multi_visit_drone_trip_count,
            'best_multi_visit_avg_truck_wait': best_multi_visit_avg_truck_wait,
            'best_multi_visit_max_truck_wait': best_multi_visit_max_truck_wait,
            'best_multi_visit_avg_drone_wait': best_multi_visit_avg_drone_wait,
            'best_multi_visit_max_drone_wait': best_multi_visit_max_drone_wait,
            'best_multi_visit_objective': best_multi_visit_objective,
            'best_multi_visit_avg_drone_trip_time': best_multi_visit_avg_drone_trip_time,
            'best_multi_visit_max_drone_trip_time': best_multi_visit_max_drone_trip_time,
            'best_sol_violated_trips': best_sol_violated_trips,
            'best_multi_visit_violated_trips': best_multi_visit_violated_trips,
        })
