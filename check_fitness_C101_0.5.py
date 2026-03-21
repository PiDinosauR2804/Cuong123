import sys
import pprint
sys.path.append('.')

from Source_revised2.problem_data import read_data_file
from Source_revised2.fitness import evaluate_fitness
from Source_revised2.solution import Solution

# Đường dẫn file data
instance_path = 'test_data/data_demand_random_50_batch_all1_equal_cluster/C101_0.5.dat'

# Solution cần kiểm tra
solution = [
    [
        [
            [0, []],
            [11, [36, 35, 38, 37]],
            [13, []],
            [14, []],
            [12, []],
            [10, [46, 47, 48, 49]],
            [31, []],
            [32, []],
            [9, []],
            [8, []],
            [29, []],
            [30, []],
            [46, []],
            [47, []],
            [48, []],
            [49, []],
            [15, []],
            [4, []],
            [3, []],
            [36, []],
            [35, []],
            [38, []],
            [37, []],
            [28, []],
            [27, []],
            [26, []],
            [25, []],
            [24, []],
            [23, []],
            [22, []],
            [1, []]
        ],
        [
            [0, []],
            [6, []],
            [39, [50, 45, 44]],
            [21, []],
            [40, []],
            [41, []],
            [42, []],
            [43, []],
            [50, []],
            [20, []],
            [19, []],
            [18, []],
            [17, []],
            [45, []],
            [44, []],
            [34, []],
            [33, []],
            [16, []],
            [7, []],
            [5, []],
            [2, []]
        ]
    ],
    [
        [[11, [36, 35, 38, 37]]],
        [[10, [46, 47, 48, 49]]],
        [[39, [50, 45, 44]]]
    ]
]

data = read_data_file(instance_path)
sol = Solution.from_legacy(solution)
result = evaluate_fitness(sol, data)

print('Objective:', result.objective)
print('Feasible:', result.feasible)
print('Violations:')
pprint.pprint(result.violations)
print('Drone trip times (flight+wait):')
pprint.pprint(result.drone_return_time)
