import copy
import math
import random
import numpy
import queue
import time
import Data
import Neighborhood
import Neighborhood_for_TSP
import Neighborhood_drone
import os
global start_time1
global start_time
global start_time2

index_truck_of_cities = [0] * Data.number_of_cities
epsilon = (-1)*0.00001
similarity = float(os.getenv('SIMILARITY', '0.6'))
def update_per_loop(solution):
    global index_truck_of_cities
    index_truck_of_cities = [0] * Data.number_of_cities
    for i in range(len(solution[0])):
        for j in range(1, len(solution[0][i])):
            index_truck_of_cities[solution[0][i][j][0]] = i

# Initial solution for data n:n
def initial_solution():
    route = []
    restrict = [0] * Data.number_of_trucks
    for i in range(Data.number_of_trucks):
        route.append([])
    r_d = []
    for i in range(len(Data.release_date)):
        r_d.append([i, Data.release_date[i]])
    r_d.pop(0)
    r_d = sorted(r_d, key=lambda r_d: r_d[1])
    for i in range(Data.number_of_trucks):
        route[i].append(r_d[0][0])
        #       crd.append(r_d[i][0])
        r_d.pop(0)
    for i in range(Data.number_of_trucks + 1, Data.number_of_cities):
        index = -1
        min = 10000000
        next_customer_add = r_d[0][0]
        for j in range(Data.number_of_trucks):
            if Data.manhattan_move_matrix[route[j][len(route[j]) - 1]][next_customer_add] < min and i - restrict[
                j] > Data.number_of_trucks * 0.5:
                min = Data.manhattan_move_matrix[route[j][len(route[j]) - 1]][next_customer_add]
                index = j
        restrict[index] = i
        route[index].append(r_d[0][0])
        r_d.pop(0)
    route2 = []
    for i in route:
        receive_address = []
        for j in range(len(i)):
            add = random.random()
            receive_address.append(add)
        route2.append(receive_address)

    new_route1 = []
    new_route2 = []
    for i in range(Data.number_of_trucks):
        new_route1 += route[i] + [0]
        new_route2 += route2[i] + [0.0]
    new_route1.pop()
    new_route2.pop()
    solution = [new_route1, new_route2]
    new_solution = []
    new_solution.append(convert(solution))
    new_solution.append(init_queue(solution))
    return new_solution

# Update from initial solution for data n:n
def initial_solution1():
    route = []
    restrict = [0] * Data.number_of_trucks
    for i in range(Data.number_of_trucks):
        route.append([])
    r_d = []
    for i in range(len(Data.release_date)):
        r_d.append([i, Data.release_date[i]])
    r_d.pop(0)
    r_d = sorted(r_d, key=lambda r_d: r_d[1])
    truck_time = [0] * Data.number_of_trucks
    for i in range(Data.number_of_trucks):
        route[i].append(r_d[0][0])
        truck_time[i] += Data.release_date[r_d[0][0]] + Data.manhattan_move_matrix[0][r_d[0][0]]
        #       crd.append(r_d[i][0])
        r_d.pop(0)
    for i in range(Data.number_of_trucks + 1, Data.number_of_cities):
        index = -1
        min = 10000000
        ''' if crd.count(r_d[i]) != 0:
            r_d1 = copy.copy(Data.release_date)
            for p in range(crd.count(r_d[i])):
                r_d1.remove(r_d[i])
            next_customer_add = r_d1.index(r_d[i])
        else:
            next_customer_add = Data.release_date.index(r_d[i])'''
        next_customer_add = r_d[0][0]
        for j in range(Data.number_of_trucks):
            if abs(Data.euclid_flight_matrix[route[j][len(route[j])-1]][next_customer_add] + r_d[0][1] - truck_time[j] - Data.manhattan_move_matrix[route[j][len(route[j])-1]][ next_customer_add]) < min:
                min = abs(Data.euclid_flight_matrix[route[j][len(route[j])-1]][next_customer_add] + r_d[0][1] - truck_time[j] - Data.manhattan_move_matrix[route[j][len(route[j])-1]][ next_customer_add])
                index = j
        restrict[index] = i
        truck_time[index] = max(Data.release_date[r_d[0][0]] + Data.euclid_flight_matrix[0][r_d[0][0]], truck_time[index] + Data.manhattan_move_matrix[route[index][len(route[index])-1]][ next_customer_add])
        route[index].append(r_d[0][0])
        r_d.pop(0)
    route2 = []
    for i in route:
        receive_address = []
        for j in range(len(i)):
            add = random.random()
            receive_address.append(add)
        route2.append(receive_address)
    new_route1 = []
    new_route2 = []
    for i in range(Data.number_of_trucks):
        new_route1 += route[i] + [0]
        new_route2 += route2[i] + [0.0]
    new_route1.pop()
    new_route2.pop()
    solution = [new_route1, new_route2]
    new_solution = []
    new_solution.append(convert(solution))
    new_solution.append(init_queue(solution))
    return new_solution

def initial_nearest_neighbor_resupply():
    visited = [False] * Data.number_of_cities
    tour = [0]  # Start from city 0
    total_distance = 0
    visited[0] = True

    for _ in range(Data.number_of_cities - 1):
        current_city = tour[-1]
        nearest_city = None
        min_distance = float('inf')

        for city in range(Data.number_of_cities):
            if not visited[city] and Data.manhattan_move_matrix[current_city][city] < min_distance:
                nearest_city = city
                min_distance = Data.manhattan_move_matrix[current_city][city]

        tour.append(nearest_city)
        total_distance += min_distance
        visited[nearest_city] = True
    tour.pop(0)
    route = [tour]
    route2 = []
    for i in route:
        receive_address = []
        for j in range(len(i)):
            add = random.random()
            receive_address.append(add)
        route2.append(receive_address)
    new_route1 = []
    new_route2 = []
    for i in range(Data.number_of_trucks):
        new_route1 += route[i] + [0]
        new_route2 += route2[i] + [0.0]
    new_route1.pop()
    new_route2.pop()
    solution = [new_route1, new_route2]
    new_solution = []
    new_solution.append(convert(solution))
    new_solution.append(init_queue(solution))
    return new_solution
random.seed(6)
# For data 1:1
def initial_solution3():
    route = []
    restrict = [0] * Data.number_of_trucks
    for i in range(Data.number_of_trucks):
        route.append([])
    # r_d = []
    # current_point = 0
    # r_d.append(0)
    # visited = [False] * Data.number_of_cities
    # while len(r_d) < Data.number_of_cities:
    #     min_path = 99999
    #     index = -1
    #     for i in range(1, Data.number_of_cities):
    #         if visited[i] == False:
    #             if Data.manhattan_move_matrix[current_point][i] < min_path:
    #                 min_path = Data.manhattan_move_matrix[current_point][i]
    #                 index = i
    #     current_point = index
    #     visited[index] = True
    #     r_d.append(index)
    # r_d.pop(0)
    
    r_d1 = []
    for i in range(len(Data.release_date)):
        r_d1.append([i, Data.release_date[i]])
    r_d1.pop(0)
    r_d1 = sorted(r_d1, key=lambda r_d: r_d[1])
    r_d = []
    for i in range(len(r_d1)):
        r_d.append(r_d1[i][0])
    
    # print(r_d)
    for i in range(Data.number_of_trucks):
        route[i].append(r_d[0])
        #       crd.append(r_d[i][0])
        r_d.pop(0)
    for i in range(Data.number_of_trucks + 1, Data.number_of_cities):
        index = -1
        min = 10000000
        next_customer_add = r_d[0]
        for j in range(Data.number_of_trucks):
            if Data.manhattan_move_matrix[route[j][len(route[j]) - 1]][next_customer_add] < min and i - restrict[
                j] > Data.number_of_trucks * 0.5:
                min = Data.manhattan_move_matrix[route[j][len(route[j]) - 1]][next_customer_add]
                index = j
        restrict[index] = i
        route[index].append(r_d[0])
        r_d.pop(0)
    route2 = []
    for i in route:
        receive_address = []
        for j in range(len(i)):
            add = random.random()
            receive_address.append(add)
        route2.append(receive_address)

    new_route1 = []
    new_route2 = []
    for i in range(Data.number_of_trucks):
        new_route1 += route[i] + [0]
        new_route2 += route2[i] + [0.0]
    new_route1.pop()
    new_route2.pop()
    solution = [new_route1, new_route2]
    new_solution = []
    new_solution.append(convert(solution))
    new_solution.append(init_queue(solution))
    return new_solution

def initial_solution4():
    route = []
    restrict = [0] * Data.number_of_trucks
    for i in range(Data.number_of_trucks):
        route.append([])
    list = []
    visited = [False] * Data.number_of_cities
    for i in range(1, Data.number_of_cities):
        list.append([i, Data.value_tan_of_city[i]])
    list = sorted(list, key=lambda x:x[1])
    list_for_truck = []
    for i in range(Data.number_of_trucks):
        list_for_truck.append([])
    count_break = []
    for i in range(1, Data.number_of_trucks):
        count_break.append(int((Data.number_of_cities)*(i/Data.number_of_trucks)))
    count_break.append(1000)
    count = 0
    for i in range(Data.number_of_cities-1):
        list_for_truck[count].append(list[i][0])
        if i == count_break[count]:
            count += 1
    for i in range(Data.number_of_trucks):
        pre = []
        mid = []
        last = []
        list_1 = []
        for ii in range(len(list_for_truck[i])):
            list_1.append([list_for_truck[i][ii], Data.manhattan_move_matrix[0][list_for_truck[i][ii]]])
        list_1 = sorted(list_1, key=lambda x:x[1])
        onepoint3 = int(len(list_for_truck[i])/3)
        for ii in range(2 * onepoint3):
            for iii in range(ii + 1, 2* onepoint3):
                if Data.release_date[list_1[ii][0]] > Data.release_date[list_1[iii][0]]:
                    temp = list_1[ii]
                    list_1[ii] = list_1[iii]
                    list_1[iii] = temp
        for ii in range(onepoint3):
            pre.append(list_1[0][0])
            list_1.pop(0)
        for ii in range(onepoint3):
            mid.append(list_1[0][0])
            list_1.pop(0)
        for ii in range(len(list_1)):
            last.append(list_1[ii][0])
        current = 0
        for ii in range(len(pre)):
            min_in = 1000000
            index = -1
            for iii in range(len(pre)):
                if Data.manhattan_move_matrix[current][pre[iii]] < min_in:
                    min_in = Data.manhattan_move_matrix[current][pre[iii]]
                    index = iii
            route[i].append(pre[index])
            current = pre[index]
            pre.pop(index)
            
        for ii in range(len(mid)):
            min_in = 1000000
            index = -1
            for iii in range(len(mid)):
                if Data.manhattan_move_matrix[current][mid[iii]] < min_in:
                    min_in = Data.manhattan_move_matrix[current][mid[iii]]
                    index = iii
            route[i].append(mid[index])
            current = mid[index]
            mid.pop(index)
            
        for ii in range(len(last)):
            min_in = 1000000
            index = -1
            for iii in range(len(last)):
                if Data.manhattan_move_matrix[current][last[iii]] < min_in:
                    min_in = Data.manhattan_move_matrix[current][last[iii]]
                    index = iii
            route[i].append(last[index])
            current = last[index]
            last.pop(index)
    
    route2 = []
    for i in route:
        receive_address = []
        for j in range(len(i)):
            add = random.random()
            receive_address.append(add)
        route2.append(receive_address)

    new_route1 = []
    new_route2 = []
    for i in range(Data.number_of_trucks):
        new_route1 += route[i] + [0]
        new_route2 += route2[i] + [0.0]
    new_route1.pop()
    new_route2.pop()
    solution = [new_route1, new_route2]
    new_solution = []
    new_solution.append(convert(solution))
    new_solution.append(init_queue(solution))
    return new_solution
   
def initial_solution5():
    init_solution = initial_solution3()
    init_solution[1] = []
    for i in range(len(init_solution[0])):
        for j in range(len(init_solution[0][i])):
            init_solution[0][i][j][1] = []
    tabu_tenure1 = 30
    tabu_tenure2 = 30
    tabu_tenure3 = 30
    tabu_tenure4 = 30
    tabu_structure1 = [-tabu_tenure1] * Data.number_of_cities
    tabu_structure2 = [-tabu_tenure1] * Data.number_of_cities
    tabu_structure3 = [-tabu_tenure1] * Data.number_of_cities
    tabu_structure4 = [-tabu_tenure1] * Data.number_of_cities
    
    current_sol = init_solution
    current_fitness = fitness(current_sol)[0]
    best_sol = current_sol
    best_fitness = current_fitness
    for i in range(60):
        neighborhood = []
        
        a = random.random()
        if a < 0.5:
            neighborhood1 = Neighborhood_for_TSP.Neighborhood_move_1_0_no_drone(current_sol)
            neighborhood2 = Neighborhood_for_TSP.Neighborhood_move_1_1_no_drone(current_sol)
            neighborhood.append([1, neighborhood1])
            neighborhood.append([2, neighborhood2])
        else:            
            neighborhood3 = Neighborhood_for_TSP.Neighborhood_move_2_0_no_drone(current_sol)
            neighborhood4 = Neighborhood_for_TSP.Neighborhood_move_2_1_no_drone(current_sol)
            
            neighborhood.append([3, neighborhood3])
            neighborhood.append([4, neighborhood4])
        
        index = [-1] * len(neighborhood)
        min_nei = [100000] * len(neighborhood)
        
        for j in range(len(neighborhood)):
            if neighborhood[j][0] == 1:
                for k in range(len(neighborhood[j][1])):
                    cfnode = neighborhood[j][1][k][1][0]
                    if cfnode - best_fitness < epsilon:
                        min_nei[j] = cfnode
                        index[j] = k
                        best_fitness = cfnode
                        best_sol = neighborhood[j][1][k][0]
                        # if_improved = True

                    elif cfnode - min_nei[j] < epsilon and tabu_structure1[neighborhood[j][1][k][2]] + tabu_tenure1 <= i:
                        min_nei[j] = cfnode
                        index[j] = k
            elif neighborhood[j][0] == 2:
                for k in range(len(neighborhood[j][1])):    
                    cfnode = neighborhood[j][1][k][1][0]
                    if cfnode - best_fitness < epsilon:
                        min_nei[j] = cfnode
                        index[j] = k
                        best_fitness = cfnode
                        best_sol = neighborhood[j][1][k][0]
                        # if_improved = True

                    elif cfnode - min_nei[j] < epsilon and ( tabu_structure2[neighborhood[j][1][k][2][0]] + tabu_tenure2 <= i or tabu_structure2[neighborhood[j][1][k][2][1]] + tabu_tenure2 <= i ) :
                        min_nei[j] = cfnode
                        index[j] = k
            elif neighborhood[j][0] == 3:
                for k in range(len(neighborhood[j][1])):
                    cfnode = neighborhood[j][1][k][1][0]
                    if cfnode - best_fitness < epsilon:
                        min_nei[j] = cfnode
                        index[j] = k
                        best_fitness = cfnode
                        best_sol = neighborhood[j][1][k][0]
                        # if_improved = True
                                                
                    elif cfnode - min_nei[j] < epsilon and tabu_structure3[neighborhood[j][1][k][2][1]] + tabu_tenure3 <= i and tabu_structure3[neighborhood[j][1][k][2][0]] + tabu_tenure3 <= i:
                        min_nei[j] = cfnode
                        index[j] = k
            else:
                for k in range(len(neighborhood[j][1])):
                    cfnode = neighborhood[j][1][k][1][0]
                    if cfnode - best_fitness < epsilon:
                        min_nei[j] = cfnode
                        index[j] = k
                        best_fitness = cfnode
                        best_sol = neighborhood[j][1][k][0]
                        # if_improved = True
                                                
                    elif cfnode - min_nei[j] < epsilon and ( tabu_structure4[neighborhood[j][1][k][2][1]] + tabu_tenure4 <= i or tabu_structure4[neighborhood[j][1][k][2][0]] + tabu_tenure4 <= i or tabu_structure4[neighborhood[j][1][k][2][2]] + tabu_tenure4 <= i):
                        min_nei[j] = cfnode
                        index[j] = k
                        
        index_best_nei = 0
        best_fit_in_cur_loop = min_nei[0]
        for j in range(1, len(min_nei)):
            if min_nei[j] < best_fit_in_cur_loop:
                index_best_nei = j
                best_fit_in_cur_loop = min_nei[j]
        if len(neighborhood[index_best_nei][1]) == 0:
            continue
        current_sol = neighborhood[index_best_nei][1][index[index_best_nei]][0]
        current_fitness = neighborhood[index_best_nei][1][index[index_best_nei]][1][0]
        
        if neighborhood[index_best_nei][0] == 1:
            tabu_structure1[neighborhood[index_best_nei][1][index[index_best_nei]][2]] = i
        
        if neighborhood[index_best_nei][0] == 2:
            tabu_structure2[neighborhood[index_best_nei][1][index[index_best_nei]][2][1]] = i
            tabu_structure2[neighborhood[index_best_nei][1][index[index_best_nei]][2][0]] = i
        
        if neighborhood[index_best_nei][0] == 3:
            tabu_structure3[neighborhood[index_best_nei][1][index[index_best_nei]][2][1]] = i
            tabu_structure3[neighborhood[index_best_nei][1][index[index_best_nei]][2][0]] = i
        
        if neighborhood[index_best_nei][0] == 4:
            tabu_structure4[neighborhood[index_best_nei][1][index[index_best_nei]][2][1]] = i
            tabu_structure4[neighborhood[index_best_nei][1][index[index_best_nei]][2][0]] = i
            tabu_structure4[neighborhood[index_best_nei][1][index[index_best_nei]][2][2]] = i
    
    for i in range(len(best_sol[0])):
        for j in range(1, len(best_sol[0][i])):
            best_sol = Neighborhood.findLocationForDropPackage(best_sol, i, best_sol[0][i][j][0])
            # print(best_sol)
            # print("------------", j, "------------")
    
    print("stage 1: ", fitness(current_sol)[0])
    
    best_fitness = fitness(best_sol)[0]
    current_sol = best_sol
    current_fitness = best_fitness
    for i in range(30):
        neighborhood = []
        
        neighborhood1 = Neighborhood_drone.Neighborghood_change_drone_route_max_pro_plus(current_sol)
        neighborhood2 = Neighborhood_drone.Neighborhood_group_trip(current_sol)
        neighborhood3 = Neighborhood_drone.Neighborhood_change_index_trip(current_sol)
        
        neighborhood.append([1, neighborhood1])
        neighborhood.append([2, neighborhood2])
        neighborhood.append([3, neighborhood3])
        
        index = [-1] * len(neighborhood)
        min_nei = [100000] * len(neighborhood)
        
        for j in range(len(neighborhood)):
            for k in range(len(neighborhood[j][1])):
                cfnode = neighborhood[j][1][k][1][0]
                if cfnode - best_fitness < epsilon:
                    min_nei[j] = cfnode
                    index[j] = k
                    best_fitness = cfnode
                    best_sol = neighborhood[j][1][k][0]
                    # if_improved = True
                                            
                elif cfnode - min_nei[j] < epsilon:
                    min_nei[j] = cfnode
                    index[j] = k
        index_best_nei = 0
        best_fit_in_cur_loop = min_nei[0]
        for j in range(1, len(min_nei)):
            if min_nei[j] < best_fit_in_cur_loop:
                index_best_nei = j
                best_fit_in_cur_loop = min_nei[j]
        if len(neighborhood[index_best_nei][1]) == 0:
            continue
        current_sol = neighborhood[index_best_nei][1][index[index_best_nei]][0]
        current_fitness = neighborhood[index_best_nei][1][index[index_best_nei]][1][0]
            
    return best_sol   
    
def initial_solution6():
    init_solution = initial_solution3()
    init_solution[1] = []
    for i in range(len(init_solution[0])):
        for j in range(len(init_solution[0][i])):
            init_solution[0][i][j][1] = []
    
    for i in range(len(init_solution[0])):
        for j in range(1, len(init_solution[0][i])):
            init_solution = Neighborhood.findLocationForDropPackage(init_solution, i, binit_solutionst_sol[0][i][j][0])            
    return init_solution       

def initial_solution7():
    init_solution = initial_solution3()
    init_solution[1] = []
    for i in range(len(init_solution[0])):
        for j in range(len(init_solution[0][i])):
            init_solution[0][i][j][1] = []

    tabu_tenure4 = tabu_tenure1 = tabu_tenure3 = tabu_tenure2 = random.uniform(2*math.log(Data.number_of_cities), Data.number_of_cities)
    tabu_structure1 = [-tabu_tenure1] * Data.number_of_cities
    tabu_structure2 = [-tabu_tenure1] * Data.number_of_cities
    tabu_structure3 = [-tabu_tenure1] * Data.number_of_cities
    tabu_structure4 = [-tabu_tenure1] * Data.number_of_cities
    
    current_sol = init_solution
    current_fitness = fitness(current_sol)[0]
    best_sol = current_sol
    best_fitness = current_fitness
    END_SEGMENT = Data.number_of_cities
    i = 0

    while i < END_SEGMENT:
        neighborhood = []
        
        a = random.random()
        if a < 0.25:
            neighborhood1 = Neighborhood_for_TSP.Neighborhood_move_1_0_no_drone(current_sol)
            neighborhood.append([1, neighborhood1])
        elif a < 0.5:            
            neighborhood3 = Neighborhood_for_TSP.Neighborhood_move_2_0_no_drone(current_sol)
            neighborhood.append([3, neighborhood3])
            
        elif a < 0.75:
            neighborhood4 = Neighborhood_for_TSP.Neighborhood_move_2_1_no_drone(current_sol)
            neighborhood.append([4, neighborhood4])
        else:
            neighborhood2 = Neighborhood_for_TSP.Neighborhood_move_1_1_no_drone(current_sol)
            neighborhood.append([2, neighborhood2])
            
        index = [-1] * len(neighborhood)
        min_nei = [100000] * len(neighborhood)
        
        for j in range(len(neighborhood)):
            if neighborhood[j][0] == 1:
                for k in range(len(neighborhood[j][1])):
                    cfnode = neighborhood[j][1][k][1][0]
                    if cfnode - best_fitness < epsilon:
                        min_nei[j] = cfnode
                        index[j] = k
                        best_fitness = cfnode
                        best_sol = neighborhood[j][1][k][0]
                        # if_improved = True

                    elif cfnode - min_nei[j] < epsilon and tabu_structure1[neighborhood[j][1][k][2]] + tabu_tenure1 <= i:
                        min_nei[j] = cfnode
                        index[j] = k
            elif neighborhood[j][0] == 2:
                for k in range(len(neighborhood[j][1])):    
                    cfnode = neighborhood[j][1][k][1][0]
                    if cfnode - best_fitness < epsilon:
                        min_nei[j] = cfnode
                        index[j] = k
                        best_fitness = cfnode
                        best_sol = neighborhood[j][1][k][0]
                        # if_improved = True

                    elif cfnode - min_nei[j] < epsilon and ( tabu_structure2[neighborhood[j][1][k][2][0]] + tabu_tenure2 <= i or tabu_structure2[neighborhood[j][1][k][2][1]] + tabu_tenure2 <= i ) :
                        min_nei[j] = cfnode
                        index[j] = k
            elif neighborhood[j][0] == 3:
                for k in range(len(neighborhood[j][1])):
                    cfnode = neighborhood[j][1][k][1][0]
                    if cfnode - best_fitness < epsilon:
                        min_nei[j] = cfnode
                        index[j] = k
                        best_fitness = cfnode
                        best_sol = neighborhood[j][1][k][0]
                        # if_improved = True
                                                
                    elif cfnode - min_nei[j] < epsilon and tabu_structure3[neighborhood[j][1][k][2][1]] + tabu_tenure3 <= i and tabu_structure3[neighborhood[j][1][k][2][0]] + tabu_tenure3 <= i:
                        min_nei[j] = cfnode
                        index[j] = k
            else:
                for k in range(len(neighborhood[j][1])):
                    cfnode = neighborhood[j][1][k][1][0]
                    if cfnode - best_fitness < epsilon:
                        min_nei[j] = cfnode
                        index[j] = k
                        best_fitness = cfnode
                        best_sol = neighborhood[j][1][k][0]
                        # if_improved = True
                                                
                    elif cfnode - min_nei[j] < epsilon and ( tabu_structure4[neighborhood[j][1][k][2][1]] + tabu_tenure4 <= i or tabu_structure4[neighborhood[j][1][k][2][0]] + tabu_tenure4 <= i or tabu_structure4[neighborhood[j][1][k][2][2]] + tabu_tenure4 <= i):
                        min_nei[j] = cfnode
                        index[j] = k
                        
        index_best_nei = 0
        best_fit_in_cur_loop = min_nei[0]
        for j in range(1, len(min_nei)):
            if min_nei[j] < best_fit_in_cur_loop:
                index_best_nei = j
                best_fit_in_cur_loop = min_nei[j]
        if len(neighborhood[index_best_nei][1]) == 0:
            continue
        current_sol = neighborhood[index_best_nei][1][index[index_best_nei]][0]
        current_fitness = neighborhood[index_best_nei][1][index[index_best_nei]][1][0]
        
        if neighborhood[index_best_nei][0] == 1:
            tabu_structure1[neighborhood[index_best_nei][1][index[index_best_nei]][2]] = i
        
        if neighborhood[index_best_nei][0] == 2:
            tabu_structure2[neighborhood[index_best_nei][1][index[index_best_nei]][2][1]] = i
            tabu_structure2[neighborhood[index_best_nei][1][index[index_best_nei]][2][0]] = i
        
        if neighborhood[index_best_nei][0] == 3:
            tabu_structure3[neighborhood[index_best_nei][1][index[index_best_nei]][2][1]] = i
            tabu_structure3[neighborhood[index_best_nei][1][index[index_best_nei]][2][0]] = i
        
        if neighborhood[index_best_nei][0] == 4:
            tabu_structure4[neighborhood[index_best_nei][1][index[index_best_nei]][2][1]] = i
            tabu_structure4[neighborhood[index_best_nei][1][index[index_best_nei]][2][0]] = i
            tabu_structure4[neighborhood[index_best_nei][1][index[index_best_nei]][2][2]] = i
    
        i += 1
            # print(best_sol)
            # print("------------", j, "------------")
    
    # print(current_sol)
    # print("stage 1: ", fitness(current_sol)[0])
    
    for i in range(len(best_sol[0])):
        for j in range(1, len(best_sol[0][i])):
            best_sol[0][i][0][1].append(best_sol[0][i][j][0]) 
    
    best_fitness = fitness(best_sol)[0]
    current_sol = best_sol
    current_fitness = best_fitness
    
    if_improved = 0
    
    # print(current_sol)
    # print(fitness(current_sol))
    abc = 0
    
    while if_improved < 1:
        neighborhood = []
        if_improved += 1
        neighborhood1 = Neighborhood_drone.Neighborghood_change_drone_route_max_pro_plus(current_sol)
        neighborhood2 = Neighborhood_drone.Neighborhood_group_trip(current_sol)
        neighborhood3 = Neighborhood_drone.Neighborhood_change_index_trip(current_sol)
        
        neighborhood.append([1, neighborhood1])
        neighborhood.append([2, neighborhood2])
        neighborhood.append([3, neighborhood3])
        
        index = [-1] * len(neighborhood)
        min_nei = [100000] * len(neighborhood)
        
        for j in range(len(neighborhood)):
            for k in range(len(neighborhood[j][1])):
                cfnode = neighborhood[j][1][k][1][0]
                if cfnode - best_fitness < epsilon:
                    min_nei[j] = cfnode
                    index[j] = k
                    best_fitness = cfnode
                    best_sol = neighborhood[j][1][k][0]
                    if_improved = 0
                    # if_improved = True
                                            
                elif cfnode - min_nei[j] < epsilon:
                    min_nei[j] = cfnode
                    index[j] = k
        index_best_nei = 0
        best_fit_in_cur_loop = min_nei[0]
        for j in range(1, len(min_nei)):
            if min_nei[j] < best_fit_in_cur_loop:
                index_best_nei = j
                best_fit_in_cur_loop = min_nei[j]
        # if len(neighborhood[index_best_nei][1]) == 0:
        #     continue
        current_sol = neighborhood[index_best_nei][1][index[index_best_nei]][0]
        current_fitness = neighborhood[index_best_nei][1][index[index_best_nei]][1][0]
            
            
        
        return best_sol       
    
def convert(solution):
    sol = []
    temp = []
    for i in range(len(solution[0])):
        if solution[0][i] != 0:
            temp.append([solution[0][i], solution[1][i]])
        else:
            sol.append(temp)
            temp = []
    sol.append(temp)
    for i in range(len(sol)):
        sol[i].insert(0, [0, 0])
        tmp = []
        tmp.append(0) 
        for j in range(1, len(sol[i])):
            if Data.euclid_flight_matrix[0][sol[i][j][0]] * 2 < Data.drone_limit_time:
                tmp.append(sol[i][j][0])
            # Nếu pack có release_date = 0 thì nghiễm nhiên được truck mang từ depot:
            if Data.release_date[sol[i][j][0]] == 0:
                sol[i][j][1] = 0
            elif Data.city_demand[sol[i][j][0]] > Data.drone_capacity:
                sol[i][j][1] = 0
            else:
                if len(tmp) > 0:
                    if j > 3 :
                        index = tmp[0]
                        while(tmp.count(index) > 0):
                            tmp.remove(index)
                indexx = int((sol[i][j][1] * len(tmp)) + 0.5)-1
                if indexx < 0:
                    indexx = 0
                # print(indexx)
                # print(tmp)
                if len(tmp) > 0:
                    sol[i][j][1] = tmp[indexx]
                    if tmp.count(sol[i][j][1]) < 4:
                        tmp.insert(tmp.index(sol[i][j][1]),sol[i][j][1])
                else:
                    sol[i][j][1] = 0
    result = copy.deepcopy(sol)
    
    for i in range(len(result)):
        for j in range(len(result[i])):
            result[i][j][1] = []
    for i in range(len(result)):
        for j in range(len(result[i])):
            for k in range(1, len(sol[i])):
                if sol[i][k][1] == result[i][j][0]:
                    result[i][j][1].append(sol[i][k][0])
    return result

def max_release_date(point):
    if point == []: return 0
    array = []
    for i in range(len(point)):
        array.append(Data.release_date[point[i]])
    return max(array)

def max_release_date_update(package):
    temp = []
    for i in range(len(package)):
        for j in range(len(package[i][1])):
            temp.append(Data.release_date[package[i][1][j]])
    return max(temp)

def min_release_date_update(package):
    temp = []
    for i in range(len(package)):
        for j in range(len(package[i][1])):
            temp.append(Data.release_date[package[i][1][j]])
    return min(temp)

def min_release_date(point):
    if point == []: return 0
    array = []
    for i in range(len(point)):
        array.append(Data.release_date[point[i]])
    return min(array)

def avg_release_date(package):
    temp = []
    for i in range(len(package)):
        for j in range(len(package[i][1])):
            temp.append(Data.release_date[package[i][1][j]])
    avg = 0
    length = len(temp)
    for i in range(length):
        avg += temp[i] / length
    return avg

def sum_weight(package):
    sum = 0
    for i in range(len(package)):
        sum = sum + Data.city_demand[package[i]]
    return sum

def total_demand(package):
    total_demand = 0
    for i in range(len(package)):
        for j in  range(len(package[i][1])):
            total_demand += Data.city_demand[package[i][1][j]]
    return total_demand

def package_in_which_truck(solution, package):
    for i in range(len(solution)):
        for j in range(1, len(solution[i])):
            if solution[i][j][0] == package: return i

def city_in_which_truck(solution, city):
    for i in range(len(solution[0])):
        for j in range(len(solution[0][i])):
            if solution[0][i][j][0] == city: return i

def sorted_by_release_date(package):
    array = []
    for i in range(len(package)):
        array.append([Data.release_date[package[i]], package[i]])
    sortt = sorted(array, key=lambda x: x[0])
    array = []
    for i in range(len(package)):
        array.append(sortt[i][1])
    return array

def find_drone_flight_route(truck_position):
    truck_position = copy.deepcopy(truck_position)
    path = []
    current_point = 0
    while len(truck_position) != 0:
        next_point = 0
        current_len = 1000000000
        for i in range(len(truck_position)):
            if Data.euclid_flight_matrix[current_point][truck_position[i]] <= current_len:
                current_len = Data.euclid_flight_matrix[current_point][truck_position[i]]
                next_point = truck_position[i]
        current_point = next_point
        path.append(current_point)
        truck_position.remove(current_point)
    return path

def find_drone_flight_shortest(solution, package):
    route = []
    shortest_route_by_point = []
    shortest_route_by_truck = []
    for i in package:
        route.append(i[0])

    current_point = 0
    next_point = 0
    while len(route) != 0:
        current_len = 10000000000000000
        for i in range(len(route)):
            if Data.euclid_flight_matrix[current_point][route[i]] <= current_len:
                current_len = Data.euclid_flight_matrix[current_point][route[i]]
                next_point = route[i]
        current_point = next_point
        shortest_route_by_point.append(current_point)
        shortest_route_by_truck.append(city_in_which_truck(solution, current_point))

        route.remove(current_point)
    return shortest_route_by_point, shortest_route_by_truck

def find_drone_flight_shortest_by_point(points):
    shortest_route_by_point = []
    current_point = 0
    next_point = 0
    while len(points) != 0:
        current_len = 10000000000000000
        for i in range(len(points)):
            if Data.euclid_flight_matrix[current_point][points[i]] <= current_len:
                current_len = Data.euclid_flight_matrix[current_point][points[i]]
                next_point = points[i]
        current_point = next_point
        shortest_route_by_point.append(current_point)
        points.remove(current_point)
    return shortest_route_by_point

def cal_time_fly_a_trip(package):
    destinations = []
    for i in package:
        destinations.append(i[0])
    shortest_route_by_point = find_drone_flight_shortest_by_point(destinations)
    time_fly = Data.euclid_flight_matrix[0][shortest_route_by_point[0]] + Data.euclid_flight_matrix[shortest_route_by_point[-1]][0]
    for i in range(len(shortest_route_by_point) - 1):
        time_fly += Data.euclid_flight_matrix[shortest_route_by_point[i]][shortest_route_by_point[i+1]]
    time_fly += (len(shortest_route_by_point) - 1) * Data.unloading_time
    return time_fly

def init_queue(solution):
    solution = convert(solution)
    #Declare
    drone_package = []
    truck_time = [0]*Data.number_of_trucks
    truck_position = []
    temp = []
    for i in range(len(solution)):
        for j in range(0, len(solution[i])):
            temp.append(solution[i][j][0])
        temp.append(0)
        truck_position.append(temp)
        temp = []
#    print(truck_position)           #  Split solution:  [[0, 1, 3, 5, 6, 7, 0],     [0, 2, 4, 8, 0],     [0, 9, 10, 0]]
    truck_current_point = [0]*Data.number_of_trucks
    drone_queue = queue.PriorityQueue()
    for i in range(0, Data.number_of_drones):
        drone_queue.put((0, "Drone %i" % i))
    compare = [0]* Data.number_of_trucks

    #Decode

    #Truck move from depot
    for i in range(Data.number_of_trucks):
        '''print("Truck", i, "move from", truck_position[i][truck_current_point[i]], "to", truck_position[i][truck_current_point[i] + 1])'''
        truck_time[i] = max_release_date(solution[i][truck_position[i][truck_current_point[i]]][1]) + \
                                Data.manhattan_move_matrix[truck_position[i][truck_current_point[i]]][truck_position[i][truck_current_point[i] + 1]]
        solution[i][truck_current_point[i]][1] = []
        truck_current_point[i] = truck_current_point[i] + 1



    #Truck and drone move
    while True:
        number = 0

        # Check if truck finish route ?
        for i in range(Data.number_of_trucks):
            if truck_position[i][truck_current_point[i]] == 0:
                number = number + 1
        if number == Data.number_of_trucks: break


        for i in range(Data.number_of_trucks):
            # Cho xe chạy qua các điểm không có đợi hàng, cập nhật thời gian truck
            while solution[i][truck_current_point[i]][1] == []:
                if truck_position[i][truck_current_point[i]] == 0: break

                '''print("Truck", i, "move from", truck_position[i][truck_current_point[i]], "to", truck_position[i][truck_current_point[i] + 1])'''
                truck_time[i] = truck_time[i] + Data.manhattan_move_matrix[truck_position[i][truck_current_point[i]]][truck_position[i][truck_current_point[i] + 1]]

                if truck_position[i][truck_current_point[i] + 1] != 0:
                    truck_current_point[i] = truck_current_point[i] + 1
                else:
                    truck_current_point[i] = 0
                    break
            # Nếu xe tải giao hết hàng, chuyển compare[i] = 99999
            if truck_position[i][truck_current_point[i]] == 0: compare[i] = 999999

            else:
                temp = []
                size = 0
                fake = sorted_by_release_date(solution[i][truck_current_point[i]][1])
                # Kiểm tra xem capicity để lấy hàng tiếp theo.
                for j in range(len(fake)):
                    size = size + Data.city_demand[fake[j]]
                    if size <= Data.drone_capacity:
                        temp.append(fake[j])
                compare[i] = max(max_release_date(temp),
                             truck_time[i] - Data.euclid_flight_matrix[0, truck_position[i][truck_current_point[i]]])
        number = 0
        for i in range(Data.number_of_trucks):
            if truck_position[i][truck_current_point[i]] == 0:
                number = number + 1
        if number == Data.number_of_trucks: break
        index = 0
        best = compare[index]
        for i in range(Data.number_of_trucks):
            if compare[i] <= best:
                index = i
                best = compare[index]
        drone = drone_queue.get()
        start = max(drone[0], best)
        temp = []
        size = 0
        fake = sorted_by_release_date(solution[index][truck_current_point[index]][1])
        trip =[]
        for j in range(len(fake)):
            size = size + Data.city_demand[fake[j]]
            if size <= Data.drone_capacity:
                temp.append(fake[j])
        '''print(drone[1], "flight from 0 to", truck_position[index][truck_current_point[index]], "bring package", temp)'''
        # Thêm danh mục hàng vào drone package-
        trip.append([solution[index][truck_current_point[index]][0],temp])
        drone_package.append(trip)
        for j in range(len(temp)):
            solution[index][truck_current_point[index]][1].remove(temp[j])
        drone_queue.put((start + 2*Data.euclid_flight_matrix[truck_position[index][truck_current_point[index]]][0], drone[1]))
        '''print(drone[1], "flight from", truck_position[index][truck_current_point[index]], "to 0")'''
        num = 0
        while solution[index][truck_current_point[index]][1] == []:
            if truck_position[index][truck_current_point[index]] == 0: break
            '''print("Truck", index, "move from", truck_position[index][truck_current_point[index]], "to", truck_position[index][truck_current_point[index] + 1])'''
            if num == 0:
                truck_time[index] = max(truck_time[index], start) + \
                                    Data.euclid_flight_matrix[truck_position[index][truck_current_point[index]]][0] +\
                                    Data.manhattan_move_matrix[truck_position[index][truck_current_point[index]]][truck_position[index][truck_current_point[index] + 1]]
            else:
                truck_time[index] = truck_time[index] + \
                                    Data.euclid_flight_matrix[truck_position[index][truck_current_point[index]]][0] + \
                                    Data.manhattan_move_matrix[truck_position[index][truck_current_point[index]]][truck_position[index][truck_current_point[index] + 1]]
            if truck_position[index][truck_current_point[index] + 1] != 0:
                truck_current_point[index] = truck_current_point[index] + 1
            else:
                truck_current_point[index] = 0
                break
            num = num + 1
    return drone_package

def cal_distance_of_truck(solution, index_truck):
    total_distance = 0
    index_of_last_city_in_truck = len(solution[0][index_truck]) - 1
    for i in range(index_of_last_city_in_truck):
        total_distance += Data.manhattan_move_matrix[solution[0][index_truck][i][0]][solution[0][index_truck][i+1][0]]
    total_distance += Data.manhattan_move_matrix[solution[0][index_truck][index_of_last_city_in_truck][0]][solution[0][index_truck][0][0]]
    return total_distance

def fitness(solution):
    drone_package = copy.deepcopy(solution[1])
    base_path = copy.deepcopy(solution[0])
    data_truck = []
    for i in range(Data.number_of_trucks):
        temp = []
        data_truck.append(temp)
    #Declare
    truck_time = [0] * Data.number_of_trucks
    truck_position = []
    temp = []
    for i in range(len(base_path)):
        for j in range(0, len(base_path[i])):
            temp.append(base_path[i][j][0])
        temp.append(0)
        truck_position.append(temp)
        temp = []
#    print(truck_position)                      # [[0, 1, 3, 5, 6, 7, 0], [0, 2, 4, 8, 0], [0, 9, 10, 0]]
    truck_current_point = [0] * Data.number_of_trucks
    drone_queue = queue.PriorityQueue()
    for i in range(0, Data.number_of_drones):
        drone_queue.put((0, "Drone %i" % i))
#    print(drone_queue.get())       # (0, 'Drone 0')
    compare = [0] * Data.number_of_trucks
    #Decode

    #Truck move form depot
    for i in range(Data.number_of_trucks):
        '''print("Truck", i, "move from", truck_position[i][truck_current_point[i]], "to", truck_position[i][truck_current_point[i] + 1])'''
        if len(truck_position[i]) != 2:
            distance = Data.manhattan_move_matrix[truck_position[i][truck_current_point[i]]][truck_position[i][truck_current_point[i] + 1]]
            truck_time[i] = max_release_date(base_path[i][truck_position[i][truck_current_point[i]]][1]) + \
                                    distance
            data_truck[i].append(truck_time[i] - distance)
            base_path[i][truck_current_point[i]][1] = []
            truck_current_point[i] = truck_current_point[i] + 1
        else: 
            data_truck[i].append(0)
            data_truck[i].append(0)
        
    '''print("from depot:", truck_time)'''
#    print(drone_package)
    #Truck and drone move
    while True:
        for i in range(Data.number_of_trucks):
            while base_path[i][truck_current_point[i]][1] == []:
                if truck_position[i][truck_current_point[i]] == 0: break
                distance = Data.manhattan_move_matrix[truck_position[i][truck_current_point[i]]][
                    truck_position[i][truck_current_point[i] + 1]]
                '''print("Truck", i, "move from", truck_position[i][truck_current_point[i]], "to", truck_position[i][truck_current_point[i] + 1])'''
                truck_time[i] = truck_time[i] + distance
                data_truck[i].append(truck_time[i] - distance)

                if truck_position[i][truck_current_point[i] + 1] != 0:
                    truck_current_point[i] = truck_current_point[i] + 1
                else:
                    truck_current_point[i] = 0
                    break
        number = 0
        # Check stop condition
        for i in range(Data.number_of_trucks):
            if truck_position[i][truck_current_point[i]] == 0:
                number = number + 1
        if number == Data.number_of_trucks: break
        drone_pack =[]
        for loop in range(len(drone_package[0])):
            for loop1 in range(len(drone_package[0][loop][1])):
                drone_pack.append(drone_package[0][loop][1][loop1])
        pos, position = find_drone_flight_shortest(solution, drone_package[0])


        drone_package.pop(0)
        drone = drone_queue.get()       # (43.499585403736305, 'Drone 1')
        start = max(drone[0], max_release_date(drone_pack))         # Thời gian drone có thể xuất phát
        LastCityOfDrone = -1
        for i in range(len(position)):
            deliver = []
            for j in range(len(drone_pack)):
                if package_in_which_truck(base_path, drone_pack[j]) == position[i]:
                    deliver.append(drone_pack[j])
            if i == 0:
                '''print(drone[1], "flight from 0 to", truck_position[position[i]][truck_current_point[position[i]]],
                  "bring package", drone_pack, "deliver", deliver, "at", start,"take distance ",)'''
                start = start + Data.euclid_flight_matrix[0][truck_position[position[i]][truck_current_point[position[i]]]] 
                LastCityOfDrone = truck_position[position[i]][truck_current_point[position[i]]]
            else:
                '''print(drone[1], "flight from", truck_position[position[i-1]][truck_current_point[position[i-1]]],
                      "to", truck_position[position[i]][truck_current_point[position[i]]],
                      "bring package", drone_pack, "deliver", deliver,"at", start)'''
                start = start + Data.euclid_flight_matrix[truck_position[position[i-1]][truck_current_point[position[i-1]]]][truck_position[position[i]][truck_current_point[position[i]]]] 
                LastCityOfDrone = truck_position[position[i]][truck_current_point[position[i]]]
            
            for j in range(len(deliver)):
                for k in range(Data.number_of_trucks):
                    for l in range(truck_current_point[k], len(base_path[k])):
                        if deliver[j] in base_path[k][l][1]:
                            base_path[k][l][1].remove(deliver[j])
                            break
            num = 0
            start = max(start + Data.unloading_time, truck_time[position[i]] + Data.unloading_time)
            while base_path[position[i]][truck_current_point[position[i]]][1] == []:
                if truck_position[position[i]][truck_current_point[position[i]]] == 0: break
                '''print("Truck", position[i], "move from", truck_position[position[i]][truck_current_point[position[i]]],
                      "to", truck_position[position[i]][truck_current_point[position[i]] + 1])'''
                if num == 0:
                    distance = Data.manhattan_move_matrix[
                                                  truck_position[position[i]][truck_current_point[position[i]]]][
                                                  truck_position[position[i]][truck_current_point[position[i]] + 1]]
                    truck_time[position[i]] = start + distance
                    data_truck[position[i]].append(truck_time[position[i]]-distance)
                    #start = max(start + Data.unloading_time, truck_time[position[i]]-distance)

                else:
                    distance = Data.manhattan_move_matrix[
                                                  truck_position[position[i]][truck_current_point[position[i]]]][
                                                  truck_position[position[i]][truck_current_point[position[i]] + 1]]
                    truck_time[position[i]] = truck_time[position[i]] + \
                                              distance
                    data_truck[position[i]].append(truck_time[position[i]]-distance)
                num = num + 1
                if truck_position[position[i]][truck_current_point[position[i]] + 1] != 0:
                    truck_current_point[position[i]] = truck_current_point[position[i]] + 1
                else:
                    truck_current_point[position[i]] = 0
                    break
                # Cộng Data.unloading_time vào drone
        end = start + Data.euclid_flight_matrix[LastCityOfDrone][0]
        '''print("This: ",Data.euclid_flight_matrix[LastCityOfDrone][0])'''
        drone_queue.put((end, drone[1]))
        '''print(drone[1], "flight from", truck_position[position[-1]][truck_current_point[position[-1]]], "to 0")
        print("nah",truck_time)'''
    value = max(truck_time)
    return value, data_truck, sum(truck_time)

def cal_truck_time(solution):
    drone_package = copy.deepcopy(solution[1])
    base_path = copy.deepcopy(solution[0])
    data_truck = []
    for i in range(Data.number_of_trucks):
        temp = []
        data_truck.append(temp)
    #Declare
    truck_time = [0] * Data.number_of_trucks
    truck_position = []
    temp = []
    for i in range(len(base_path)):
        for j in range(0, len(base_path[i])):
            temp.append(base_path[i][j][0])
        temp.append(0)
        truck_position.append(temp)
        temp = []
#    print(truck_position)                      # [[0, 1, 3, 5, 6, 7, 0], [0, 2, 4, 8, 0], [0, 9, 10, 0]]
    truck_current_point = [0] * Data.number_of_trucks
    drone_queue = queue.PriorityQueue()
    for i in range(0, Data.number_of_drones):
        drone_queue.put((0, "Drone %i" % i))
#    print(drone_queue.get())       # (0, 'Drone 0')
    compare = [0] * Data.number_of_trucks
    #Decode

    #Truck move form depot
    for i in range(Data.number_of_trucks):
        '''print("Truck", i, "move from", truck_position[i][truck_current_point[i]], "to", truck_position[i][truck_current_point[i] + 1])'''
        if len(truck_position[i]) != 2:
            distance = Data.manhattan_move_matrix[truck_position[i][truck_current_point[i]]][truck_position[i][truck_current_point[i] + 1]]
            truck_time[i] = max_release_date(base_path[i][truck_position[i][truck_current_point[i]]][1]) + \
                                    distance
            data_truck[i].append(truck_time[i] - distance)
            base_path[i][truck_current_point[i]][1] = []
            truck_current_point[i] = truck_current_point[i] + 1
        else: 
            data_truck[i].append(0)
            data_truck[i].append(0)
        
    '''print("from depot:", truck_time)'''
#    print(drone_package)
    #Truck and drone move
    while True:
        for i in range(Data.number_of_trucks):
            while base_path[i][truck_current_point[i]][1] == []:
                if truck_position[i][truck_current_point[i]] == 0: break
                distance = Data.manhattan_move_matrix[truck_position[i][truck_current_point[i]]][
                    truck_position[i][truck_current_point[i] + 1]]
                '''print("Truck", i, "move from", truck_position[i][truck_current_point[i]], "to", truck_position[i][truck_current_point[i] + 1])'''
                truck_time[i] = truck_time[i] + distance
                data_truck[i].append(truck_time[i] - distance)

                if truck_position[i][truck_current_point[i] + 1] != 0:
                    truck_current_point[i] = truck_current_point[i] + 1
                else:
                    truck_current_point[i] = 0
                    break
        number = 0
        # Check stop condition
        for i in range(Data.number_of_trucks):
            if truck_position[i][truck_current_point[i]] == 0:
                number = number + 1
        if number == Data.number_of_trucks: break
        drone_pack =[]
        for loop in range(len(drone_package[0])):
            for loop1 in range(len(drone_package[0][loop][1])):
                drone_pack.append(drone_package[0][loop][1][loop1])
        pos, position = find_drone_flight_shortest(solution, drone_package[0])


        drone_package.pop(0)
        drone = drone_queue.get()       # (43.499585403736305, 'Drone 1')
        start = max(drone[0], max_release_date(drone_pack))         # Thời gian drone có thể xuất phát
        LastCityOfDrone = -1
        for i in range(len(position)):
            deliver = []
            for j in range(len(drone_pack)):
                if package_in_which_truck(base_path, drone_pack[j]) == position[i]:
                    deliver.append(drone_pack[j])
            if i == 0:
                '''print(drone[1], "flight from 0 to", truck_position[position[i]][truck_current_point[position[i]]],
                  "bring package", drone_pack, "deliver", deliver, "at", start,"take distance ",)'''
                start = start + Data.euclid_flight_matrix[0][truck_position[position[i]][truck_current_point[position[i]]]] 
                LastCityOfDrone = truck_position[position[i]][truck_current_point[position[i]]]
            else:
                '''print(drone[1], "flight from", truck_position[position[i-1]][truck_current_point[position[i-1]]],
                      "to", truck_position[position[i]][truck_current_point[position[i]]],
                      "bring package", drone_pack, "deliver", deliver,"at", start)'''
                start = start + Data.euclid_flight_matrix[truck_position[position[i-1]][truck_current_point[position[i-1]]]][truck_position[position[i]][truck_current_point[position[i]]]] 
                LastCityOfDrone = truck_position[position[i]][truck_current_point[position[i]]]
            
            for j in range(len(deliver)):
                for k in range(Data.number_of_trucks):
                    for l in range(truck_current_point[k], len(base_path[k])):
                        if deliver[j] in base_path[k][l][1]:
                            base_path[k][l][1].remove(deliver[j])
                            break
            num = 0
            start = max(start + Data.unloading_time, truck_time[position[i]] + Data.unloading_time)
            while base_path[position[i]][truck_current_point[position[i]]][1] == []:
                if truck_position[position[i]][truck_current_point[position[i]]] == 0: break
                '''print("Truck", position[i], "move from", truck_position[position[i]][truck_current_point[position[i]]],
                      "to", truck_position[position[i]][truck_current_point[position[i]] + 1])'''
                if num == 0:
                    distance = Data.manhattan_move_matrix[
                                                  truck_position[position[i]][truck_current_point[position[i]]]][
                                                  truck_position[position[i]][truck_current_point[position[i]] + 1]]
                    truck_time[position[i]] = start + distance
                    data_truck[position[i]].append(truck_time[position[i]]-distance)
                    #start = max(start + Data.unloading_time, truck_time[position[i]]-distance)

                else:
                    distance = Data.manhattan_move_matrix[
                                                  truck_position[position[i]][truck_current_point[position[i]]]][
                                                  truck_position[position[i]][truck_current_point[position[i]] + 1]]
                    truck_time[position[i]] = truck_time[position[i]] + \
                                              distance
                    data_truck[position[i]].append(truck_time[position[i]]-distance)
                num = num + 1
                if truck_position[position[i]][truck_current_point[position[i]] + 1] != 0:
                    truck_current_point[position[i]] = truck_current_point[position[i]] + 1
                else:
                    truck_current_point[position[i]] = 0
                    break
                # Cộng Data.unloading_time vào drone
        end = start + Data.euclid_flight_matrix[LastCityOfDrone][0]
        '''print("This: ",Data.euclid_flight_matrix[LastCityOfDrone][0])'''
        drone_queue.put((end, drone[1]))
        '''print(drone[1], "flight from", truck_position[position[-1]][truck_current_point[position[-1]]], "to 0")
        print("nah",truck_time)'''
    value = max(truck_time)
    return truck_time

def Check_if_feasible(solution):
    truck_check=[]              # Danh sách nhận hàng của các truck
    truck_check1 =[]            # Danh sách khách hàng của truck
    for i in range(Data.number_of_trucks):
        temp =[]
        temp1=[]
        truck_check.append(temp)
        truck_check1.append(temp1)
    for i in range(len(solution[0])):
        for j in range(len(solution[0][i])):
            truck_check1[i].append(solution[0][i][j][0])
            if solution[0][i][j][1] != []:
                truck_check[i].append(solution[0][i][j][0])
        if len(truck_check[i]) == 0: truck_check[i].append(0)
        truck_check[i].append(0)
        truck_check[i].pop(0)

     # Check xem nơi nhận hàng phải nằm trước vị trí người nhận hàng
    for i in range(len(solution[1])):
        for j in range(len(solution[1][i])):
            truck_index = city_in_which_truck(solution,solution[1][i][j][0])
            for k in range(len(solution[1][i][j][1])):
                if truck_check1[truck_index].index(solution[1][i][j][0]) > truck_check1[truck_index].index(solution[1][i][j][1][k]):
                    print("Nhận hàng sau vị trí khách hàng")
                    return False
    # Check xem nơi giao hàng có nằm trong phạm vi giao hàng của drone không?
    for i in range(len(solution[1])):
        route, position = find_drone_flight_shortest(solution, solution[1][i])
        total_time = 0
        total_time += Data.euclid_flight_matrix[0][route[0]]
        for j in range(len(route)):
            if j == len(route)-1:
                total_time += Data.euclid_flight_matrix[route[j]][0]
            else:
                total_time += Data.euclid_flight_matrix[route[j]][route[j+1]]
        if total_time > Data.drone_limit_time:
            print("Tồn tại địa điểm vượt quá thời gian bay của drone")
            return False
# Check chuyến hàng có vượt qua capicity_drone không ?
        total_demand = 0
        for j in range(len(solution[1][i])):
            for k in range(len(solution[1][i][j][1])):
                total_demand += Data.city_demand[solution[1][i][j][1][k]]
        if total_demand > Data.drone_capacity:
            print("Vi phạm capicity ở trip thứ ",i)
            return False

    drone_queue =[]
    for i in range(len(solution[1])):
        for j in range(len(solution[1][i])):
                drone_queue.append(solution[1][i][j][0])
    # Check if coincide customer
    city_coincide =[]
    for i in range(len(drone_queue)):
        for j in range(i+1, len(drone_queue)):
            if drone_queue[i] == drone_queue[j]:
                truck_index = city_in_which_truck(solution,drone_queue[i])
                for k in range(i+1,j):
                    if truck_index == city_in_which_truck(solution,drone_queue[k]) and drone_queue[k] != drone_queue[j]:
                        print("Thứ tự giao hàng sai")
                        print(drone_queue[i],"----",drone_queue[j])
                        return False
                if city_coincide.count(drone_queue[j]) == 0:
                    if not j in city_coincide:
                        city_coincide.append(j)
    city_coincide.sort(reverse=True)
    for i in range(len(city_coincide)):
        drone_queue.pop(city_coincide[i])

    # Check xem có đi theo tuần tự điểm nhận hàng các truck không
    for i in range(len(drone_queue)):
        check = drone_queue.pop(0)
        for j in range(len(truck_check)):
            if check == truck_check[j][0]:
                truck_check[j].pop(0)
                continue
    for i in range(len(truck_check)):
        if truck_check[i].pop() != 0:
            print("Thứ tự giao hàng và hành trình truck không trùng nhau")
            return False
    return True

def find_position(package, solution):
    position = []

    for i in range (len(solution)):
        for j in range (len(solution[i])):
            if solution[i][j][0] == package:
                position.append(i)
                position.append(j)
                break

    return position

def Compare_two_solution(solution1, solution2):
    shortcut_solution1 = []
    shortcut_solution2 = []
    for i in range(len(solution1[0])):
        temp = []
        for j in range(1, len(solution1[0][i])):
            temp.append(solution1[0][i][j][0])
        shortcut_solution1.append(temp)
    
    for i in range(len(solution2[0])):
        temp = []
        for j in range(1, len(solution2[0][i])):
            temp.append(solution2[0][i][j][0])
        shortcut_solution2.append(temp)
    
    total_score = 0
    
    for i in range(len(shortcut_solution1)):
        score = 0
        for j in range(len(shortcut_solution2)):
            cur_score = 0
            k = 0
            start_sh_sol2 = 0
            while k < len(shortcut_solution1[i]):
                k_plus = 1
                start_sh_sol2 = max(start_sh_sol2, k-3)
                go = False
                pp = 0
                for p in range(start_sh_sol2, min(len(shortcut_solution2[j]), k+4)):
                    if shortcut_solution1[i][k] == shortcut_solution2[j][p]:
                        go = True
                        pp = p
                        break
                if go:
                    while pp + 1 < len(shortcut_solution2[j]) and k + k_plus < len(shortcut_solution1[i]):
                        if shortcut_solution2[j][pp+1] == shortcut_solution1[i][k+k_plus]:
                            k_plus += 1
                            pp += 1
                            if k_plus > 2:
                                start_sh_sol2 = pp + 1
                                if k_plus == 3:
                                    cur_score += 3
                                else:
                                    cur_score += 1.5
                        else:
                            break
                k += k_plus
            score = max(cur_score, score)
        total_score += score
    # print("-----------------------------")  
    # print(solution1[0])
    # print("vs")
    # print(solution2[0])
    # print(total_score)
    if total_score > 0.7 * (Data.number_of_cities-1):
        return True
    else:
        return False
        
def Compare_two_solution_2(solution1, solution2):    

    shortcut_solution1 = []
    shortcut_solution2 = []
    for i in range(len(solution1[0])):
        temp = []
        for j in range(1, len(solution1[0][i])):
            temp.append(solution1[0][i][j][0])
        shortcut_solution1.append(temp)
    
    for i in range(len(solution2[0])):
        temp = []
        for j in range(1, len(solution2[0][i])):
            temp.append(solution2[0][i][j][0])
        shortcut_solution2.append(temp)
    total_score = 0
    array_sol_1 = [-1]*Data.number_of_cities

    for i in range(len(shortcut_solution1)):
        j = 0
        while j + 1 < len(shortcut_solution1[i]):
            array_sol_1[shortcut_solution1[i][j]] = shortcut_solution1[i][j+1]
            j += 1
    array_sol_2 = [-1]*Data.number_of_cities
    for i in range(len(shortcut_solution2)):
        j = 0
        while j + 1 < len(shortcut_solution2[i]):
            array_sol_2[shortcut_solution2[i][j]] = shortcut_solution2[i][j+1]
            j += 1
    
    for i in range(1, len(array_sol_1)):
        if array_sol_1[i] == array_sol_2[i]:
            total_score += 1
            # print(i ,"after", array_sol_1[i])
    
    # print("-----------------------------")  
    # print(solution1[0])
    # print("vs")
    # print(solution2[0])
    # print(total_score)
    if total_score > similarity * (Data.number_of_cities - 1):
        return True
    else:
        return False

def return_truck_route(solution):
    sol1 = copy.deepcopy(solution)
    for i in range(len(sol1[0])):
        for j in range(len(sol1[0][i])):
            sol1[0][i][j][1] = []
    # sol1.pop()
    return sol1
