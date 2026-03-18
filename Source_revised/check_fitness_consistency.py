from Source_revised import read_dat
from Source_revised.Solution import evaluate_solution, fitness
from Source_revised.move_truck_with_drone import _evaluate_cached

instance = r"test_data/data_demand_random/10/C101_0.5.dat"
data = read_dat(instance)

# Feasible solution captured from current run output
raw_solution = [
    [
        [[0, [8, 10, 6, 7]], [6, []], [8, []], [10, []], [7, []]],
        [[0, [1, 2, 3, 4, 5]], [5, []], [3, []], [4, []], [1, []], [2, []], [9, [9]]],
    ],
    [
        [[9, [9]]],
    ],
]

ev_direct = evaluate_solution(raw_solution, data)
obj_direct = ev_direct.system_completion_time
obj_fitness = fitness(raw_solution, data)

cache = {}
ev_cache_first = _evaluate_cached(raw_solution, data, cache)
ev_cache_second = _evaluate_cached(raw_solution, data, cache)
obj_cache_first = ev_cache_first.system_completion_time
obj_cache_second = ev_cache_second.system_completion_time

print("feasible_direct=", ev_direct.feasible)
print("obj_direct=", obj_direct)
print("obj_fitness=", obj_fitness)
print("obj_cache_first=", obj_cache_first)
print("obj_cache_second=", obj_cache_second)

same_direct_fitness = abs(obj_direct - obj_fitness) <= 1e-9
same_direct_cache = abs(obj_direct - obj_cache_first) <= 1e-9
same_cache_repeat = abs(obj_cache_first - obj_cache_second) <= 1e-9

print("same_direct_vs_fitness=", same_direct_fitness)
print("same_direct_vs_cache=", same_direct_cache)
print("same_cache_first_vs_second=", same_cache_repeat)
