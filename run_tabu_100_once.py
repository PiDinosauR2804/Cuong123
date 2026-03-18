import os
import copy
import math
import random
import time

import Data
import Function
import test_similarity
import Neighborhood
import Neighborhood10
import Neighborhood11


def run_with_limit(instance_path: str, truck_count: int, drone_count: int, theta: int = 1, max_end: int = 100):
    Data.read_data_random(instance_path)
    Data.number_of_trucks = int(truck_count)
    Data.number_of_drones = int(drone_count)
    test_similarity.theta = int(theta)

    init_solution = Function.initial_solution7()
    current_fitness, current_truck_time, current_sum_fitness = Function.fitness(init_solution)

    best_sol = init_solution
    best_fitness = current_fitness
    best_multi_visit_sol = init_solution if test_similarity.has_multi_visit(init_solution) else None
    best_multi_visit_fitness = current_fitness if best_multi_visit_sol is not None else float("inf")

    current_sol = init_solution
    solution_pack = []
    solution_pack_len = test_similarity.solution_pack_len

    END_SEGMENT = int(Data.number_of_cities / math.log10(Data.number_of_cities)) * test_similarity.theta
    END = 0
    T = 0
    epsilon = test_similarity.epsilon

    start_time = time.time()
    while END < max_end and T < 3:
        tabu_tenure = random.uniform(2 * math.log(Data.number_of_cities), Data.number_of_cities)
        tabu_tenure1 = tabu_tenure2 = tabu_tenure3 = tabu_tenure

        Tabu_Structure = [-(tabu_tenure + 1)] * Data.number_of_cities
        Tabu_Structure1 = [-(tabu_tenure + 1)] * Data.number_of_cities
        Tabu_Structure2 = [-(tabu_tenure + 1)] * Data.number_of_cities
        Tabu_Structure3 = [-(tabu_tenure + 1)] * Data.number_of_cities

        lenght_i = [0] * 6
        i = 0
        improved_in_segment = False

        while i < END_SEGMENT and END < max_end:
            choose = random.choice([0, 2, 3, 4])
            current_neighborhood = []

            if choose == 0:
                n1, solution_pack = Neighborhood.Neighborhood_combine_truck_and_drone_neighborhood_with_tabu_list_with_package(
                    name_of_truck_neiborhood=Neighborhood10.Neighborhood_one_opt_standard,
                    solution=current_sol,
                    number_of_potial_solution=1,
                    number_of_loop_drone=2,
                    tabu_list=Tabu_Structure,
                    tabu_tenure=tabu_tenure,
                    index_of_loop=lenght_i[1],
                    best_fitness=best_fitness,
                    kind_of_tabu_structure=1,
                    need_truck_time=False,
                    solution_pack=solution_pack,
                    solution_pack_len=solution_pack_len,
                    use_solution_pack=True,
                    index_consider_elite_set=0,
                )
                current_neighborhood.append([1, n1])
            elif choose == 2:
                n5, solution_pack = Neighborhood.Neighborhood_combine_truck_and_drone_neighborhood_with_tabu_list_with_package(
                    name_of_truck_neiborhood=Neighborhood11.Neighborhood_two_opt_tue,
                    solution=current_sol,
                    number_of_potial_solution=1,
                    number_of_loop_drone=2,
                    tabu_list=Tabu_Structure3,
                    tabu_tenure=tabu_tenure3,
                    index_of_loop=lenght_i[5],
                    best_fitness=best_fitness,
                    kind_of_tabu_structure=5,
                    need_truck_time=False,
                    solution_pack=solution_pack,
                    solution_pack_len=solution_pack_len,
                    use_solution_pack=True,
                    index_consider_elite_set=0,
                )
                current_neighborhood.append([5, n5])
            elif choose == 3:
                n4, solution_pack = Neighborhood.Neighborhood_combine_truck_and_drone_neighborhood_with_tabu_list_with_package(
                    name_of_truck_neiborhood=Neighborhood11.Neighborhood_move_2_1,
                    solution=current_sol,
                    number_of_potial_solution=1,
                    number_of_loop_drone=2,
                    tabu_list=Tabu_Structure2,
                    tabu_tenure=tabu_tenure2,
                    index_of_loop=lenght_i[4],
                    best_fitness=best_fitness,
                    kind_of_tabu_structure=4,
                    need_truck_time=False,
                    solution_pack=solution_pack,
                    solution_pack_len=solution_pack_len,
                    use_solution_pack=True,
                    index_consider_elite_set=0,
                )
                current_neighborhood.append([4, n4])
            else:
                n3, solution_pack = Neighborhood.Neighborhood_combine_truck_and_drone_neighborhood_with_tabu_list_with_package(
                    name_of_truck_neiborhood=Neighborhood11.Neighborhood_move_1_1_ver2,
                    solution=current_sol,
                    number_of_potial_solution=1,
                    number_of_loop_drone=2,
                    tabu_list=Tabu_Structure1,
                    tabu_tenure=tabu_tenure1,
                    index_of_loop=lenght_i[3],
                    best_fitness=best_fitness,
                    kind_of_tabu_structure=3,
                    need_truck_time=False,
                    solution_pack=solution_pack,
                    solution_pack_len=solution_pack_len,
                    use_solution_pack=True,
                    index_consider_elite_set=0,
                )
                current_neighborhood.append([3, n3])

            candidates = current_neighborhood[0][1]
            if not candidates:
                i += 1
                END += 1
                continue

            best_candidate = min(candidates, key=lambda x: x[1][0])
            current_sol = best_candidate[0]
            current_fitness = best_candidate[1][0]

            if current_fitness < best_fitness + epsilon:
                best_sol = current_sol
                best_fitness = current_fitness
                improved_in_segment = True

            if test_similarity.has_multi_visit(current_sol) and current_fitness < best_multi_visit_fitness + epsilon:
                best_multi_visit_sol = current_sol
                best_multi_visit_fitness = current_fitness

            i += 1
            END += 1

        if improved_in_segment:
            T = 0
        else:
            T += 1

    elapsed = time.time() - start_time
    return {
        "best_fitness": best_fitness,
        "best_sol": best_sol,
        "best_multi_visit_fitness": None if best_multi_visit_sol is None else best_multi_visit_fitness,
        "best_multi_visit_sol": best_multi_visit_sol,
        "best_sol_multi_visit_trip_count": sum(1 for trip in best_sol[1] if len(trip) > 1),
        "best_multi_visit_trip_count": None if best_multi_visit_sol is None else sum(1 for trip in best_multi_visit_sol[1] if len(trip) > 1),
        "end_loops": END,
        "runtime_sec": round(elapsed, 3),
    }


if __name__ == "__main__":
    target = r"test_data/data_demand_random_50_batch_all1_equal_cluster/C101_0.5.dat"
    result = run_with_limit(target, truck_count=2, drone_count=1, theta=1, max_end=100)
    print(result)
