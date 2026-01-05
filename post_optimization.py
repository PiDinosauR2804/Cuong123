import pandas as pd
import numpy as np
import os
import Data
import copy
import Function
import ast
import itertools
import time
# Path to Excel file should be configured appropriately
# EXCEL_PATH = r"C:\Users\Admin\Desktop\Final_Result\Bảng 12+13+14\Appendix Result.xlsx"
CSV_PATH = r"summary_appendix.csv"

def find_truck_for_trip(solution, trip):
    """
    Find which truck and position a trip belongs to in the solution.
    
    Args:
        solution: The solution containing truck assignments
        trip: Trip ID to find
        
    Returns:
        tuple: (truck_index, position_index)
    """
    for position_idx in range(len(solution[0])):
        for truck_idx in range(len(solution[0][position_idx])):
            if solution[0][position_idx][truck_idx][0] == trip:
                return truck_idx, position_idx


def check_solution(solution):
    """
    Check if a solution is valid by ensuring no scheduling conflicts.
    
    Args:
        solution: The solution to check
        
    Returns:
        bool: True if the solution is valid, False otherwise
    """
    position_of_truck = []
    for i in range(len(solution[1])):
        truck, position = find_truck_for_trip(solution, solution[1][i][0][0])
        position_of_truck.append([position, truck, i])
    
    # Check for scheduling conflicts
    for i in range(len(position_of_truck)):
        for j in range(i+1, len(position_of_truck)):
            if position_of_truck[i][1] == position_of_truck[j][1]:
                if position_of_truck[i][0] > position_of_truck[j][0]:
                    if position_of_truck[i][2] < position_of_truck[j][2]:
                        return False
    return True


def convert_solution_string_to_array(solution_str):
    """
    Convert solution string to a proper array format.
    
    Args:
        solution_str: String representation of the solution
        
    Returns:
        list: Solution as a properly formatted array
    """
    try:
        # Use ast.literal_eval to safely evaluate the string as a Python literal
        return ast.literal_eval(solution_str)
    except (SyntaxError, ValueError) as e:
        print(f"Error converting solution string to array: {e}")
        print(f"Solution string: {solution_str}")
        return None


def post_optimization(dataset, number_of_cities, solution, A, number_of_trucks, number_of_drones):
    """
    Perform post-optimization on a solution by generating all possible permutations
    of trips in the drone queue.
    
    Args:
        dataset: Name of the dataset
        number_of_cities: Number of cities in the problem
        solution: Initial solution to optimize
        A: Drone capacity
        number_of_trucks: Number of trucks available
        number_of_drones: Number of drones available
        
    Returns:
        tuple: (best_solution, best_fitness_value)
    """
    # Read data
    data_path = os.path.join("test_data/data_demand_random/", str(number_of_cities), dataset)
    print("--------------------------------")
    print(f"Running post-optimization for dataset: {data_path}")
    print("--------------------------------")

    Data.read_data_random(data_path)
    Data.drone_capacity = A
    Data.number_of_trucks = number_of_trucks
    Data.number_of_drones = number_of_drones
    
    # Generate all possible permutations of the drone queue
    solution_list = []
    
    # Get all drone trips from the solution
    drone_trips = solution[1]
    
    # Calculate total number of permutations
    n = len(drone_trips)
    total_permutations = 1
    for i in range(1, n+1):
        total_permutations *= i
    
    print(f"--------------------------------")
    print(f"Generating {total_permutations} permutations for {n} drone trips")
    print(f"Original drone queue: {drone_trips}")
    print(f"--------------------------------")
    
    # Generate all permutations (n!)
    for perm in itertools.permutations(range(n)):
        new_solution = copy.deepcopy(solution)
        new_drone_queue = []
        
        # Reorder the drone queue according to the permutation
        for idx in perm:
            new_drone_queue.append(drone_trips[idx])
        
        new_solution[1] = new_drone_queue
        solution_list.append(new_solution)

    
    # Evaluate solutions and find the best one
    min_fitness = float('inf')
    best_solution = copy.deepcopy(solution)
    
    # Evaluate the original solution first
    original_fitness, _, _ = Function.fitness(solution)
    if original_fitness < min_fitness:
        min_fitness = original_fitness
        
    print(f"Original solution fitness: {original_fitness}")
    
    
    for candidate_solution in solution_list:
        if check_solution(candidate_solution):
            fitness, truck_time, total_truck_time = Function.fitness(candidate_solution)
            
            if fitness < min_fitness:
                min_fitness = fitness
                best_solution = copy.deepcopy(candidate_solution)
                print(f"Found better solution with fitness: {fitness}")

    return best_solution, min_fitness


def main():
    """Main function to run post-optimization on all solutions in the Excel file."""
    # df = pd.read_excel(EXCEL_PATH)
    df = pd.read_csv(CSV_PATH)
    
    for index, row in df.iterrows():
        dataset = row['Dataset']
        number_of_cities = row['Number_of_cities']
        if number_of_cities == 100:
            number_of_drones = 3
            number_of_trucks = 3
        else:
            number_of_drones = 2
            number_of_trucks = 2

        # Convert solution string to array
        solution_str = row['Solution']
        solution = convert_solution_string_to_array(solution_str)
        
        if solution is None:
            print(f"Skipping row {index} due to solution conversion error")
            continue
            
        A = row['A']
        start_time = time.time()
        print(f"Processing row {index} for dataset {dataset}")
        optimized_solution, fitness = post_optimization(
            dataset, 
            number_of_cities, 
            solution,
            A,
            number_of_trucks,
            number_of_drones
        )
        end_time = time.time()
        df.at[index, 'New_Solution'] = str(optimized_solution)
        df.at[index, 'New_Fitness'] = float(fitness)
        df.at[index, 'Improved'] = (float(fitness) - float(row['Fitness']))/float(row['Fitness'])
        df.at[index, 'Time_Post_Optimization'] = end_time - start_time
    df.to_excel("post-optimization1.xlsx", index=False)


if __name__ == "__main__":
    main()

