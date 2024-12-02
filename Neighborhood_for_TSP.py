import copy
import Data
import Function
import random

DIFFERENTIAL_RATE_RELEASE_TIME = 1.2
epsilon = (-1)*0.00001

def Neighborhood_move_1_0_no_drone(solution):
    neighborhood = []
    for i in range(len(solution[0])):
        for j in range(1, len(solution[0][i])):
            for k in range(len(solution[0])):       
                for l in range(len(solution[0][k])):
                    if solution[0][i][j][0] == 0: continue
                    if i == k and (j == l or j == l + 1):
                        continue
                    else:
                        new_solution = copy.deepcopy(solution)

                        city_change = solution[0][i][j][0]
                        
                        # Xóa city_change ra khỏi truck route
                        new_solution[0][i].pop(j)
                        
                        # Chuyển city_change tới vị trí mới trên truck route - new_solution[0]
                        if i == k:
                            if j < l:
                                new_solution[0][i].insert(l, [city_change, []])
                            else:
                                new_solution[0][i].insert(l + 1, [city_change, []])
                        else:
                            new_solution[0][k].insert(l + 1, [city_change, []])

                        # print(Function.Check_if_feasible(new_solution))
                        # print(new_solution)
                        # print(Function.Check_if_feasible(new_solution))
                        pack_child = []
                        pack_child.append(new_solution)
                        a = fitness_init(new_solution)                
                        pack_child.append([a])
                        pack_child.append(city_change)
                        pack_child.append(i)
                        pack_child.append(k)
                        neighborhood.append(pack_child)
    return neighborhood

def Neighborhood_move_1_1_no_drone(solution):
    neighborhood = []
    for i in range(len(solution[0])):
        for j in range(1, len(solution[0][i])):
            for k in range(len(solution[0])):
                for l in range(1, len(solution[0][k])):
                    if i == k and j == l:
                        continue
                    new_solution = copy.deepcopy(solution)
                    pre_drop_package = []
                    city1 = solution[0][i][j][0]
                    city2 = solution[0][k][l][0]
                    
                    # Thay đổi hành trình truck
                    new_solution[0][k][l][0] = city1
                    new_solution[0][i][j][0] = city2
                    
                    # print(new_solution)
                    # print(Function.fitness(new_solution)[0])
                    # print(Function.Check_if_feasible(new_solution))
                    # print(len(neighborhood))
                    # print("---------------------------") 
                    
                    pack_child = []
                    pack_child.append(new_solution)
                    a = fitness_init(new_solution)                
                    pack_child.append([a])
                    pack_child.append([city1, city2])
                    pack_child.append(i)
                    pack_child.append(k)
                    neighborhood.append(pack_child)
                    
    return neighborhood

def Neighborhood_move_2_0_no_drone(solution):
    neighborhood = []
    for i in range(len(solution[0])):
        for j in range(1, len(solution[0][i]) - 1):
            for k in range(len(solution[0])):       
                for l in range(len(solution[0][k])):
                    if solution[0][i][j][0] == 0: continue
                    if i == k and (j == l or j == l + 1 or j == l - 1):
                        continue
                    else:
                        new_solution = copy.deepcopy(solution)

                        city_change1 = solution[0][i][j][0]
                        city_change2 = solution[0][i][j + 1][0]
                        # Xóa city_change ra khỏi truck route
                        new_solution[0][i].pop(j + 1)
                        new_solution[0][i].pop(j)
                        
                        # Chuyển city_change tới vị trí mới trên truck route - new_solution[0]
                        if i == k:
                            if j < l:
                                new_solution[0][i].insert(l - 1, [city_change1, []])
                                new_solution[0][i].insert(l, [city_change2, []])
                            else:
                                new_solution[0][i].insert(l + 1, [city_change1, []])
                                new_solution[0][i].insert(l + 2, [city_change2, []])
                        else:
                            new_solution[0][k].insert(l + 1, [city_change1, []])
                            new_solution[0][i].insert(l + 2, [city_change2, []])

                        # print(Function.Check_if_feasible(new_solution))
                        # print(i," ",j, " ", k, " ",l)
                        # print(new_solution)
                        # print(len(neighborhood))
                        # print("---------")
                        # print(Function.Check_if_feasible(new_solution))
                        pack_child = []
                        pack_child.append(new_solution)
                        a = fitness_init(new_solution)                
                        pack_child.append([a])
                        pack_child.append([city_change1, city_change2])
                        pack_child.append(i)
                        pack_child.append(k)
                        neighborhood.append(pack_child)
    return neighborhood

def Neighborhood_move_2_1_no_drone(solution):
    neighborhood = []
    for i in range(len(solution[0])):
        for j in range(1, len(solution[0][i]) - 1):
            for k in range(len(solution[0])):
                for l in range(1, len(solution[0][k])):
                    if i == k and (j == l or j - 1 == l or j == l - 1):
                        continue
                    
                    new_solution = copy.deepcopy(solution)
                    city_change1 = solution[0][i][j][0]
                    city_change2 = solution[0][i][j+1][0]
                    city_change3 = solution[0][k][l][0]
                            
                    if i == k:
                        if j < l:
                            new_solution[0][i].pop(l)
                            new_solution[0][i].pop(j+1)
                            new_solution[0][i].pop(j)
                            new_solution[0][i].insert(j, [city_change3, []])
                            new_solution[0][i].insert(l - 1, [city_change1, []])
                            new_solution[0][i].insert(l, [city_change2, []])
                        else:
                            new_solution[0][i].pop(j+1)
                            new_solution[0][i].pop(j)
                            new_solution[0][i].pop(l)
                            new_solution[0][i].insert(l, [city_change1, []])
                            new_solution[0][i].insert(l + 1, [city_change2, []])
                            new_solution[0][i].insert(j + 1, [city_change3, []])
                    else:
                        new_solution[0][k].pop(l)
                        new_solution[0][i].pop(j+1)
                        new_solution[0][i].pop(j)
                        new_solution[0][k].insert(l, [city_change1, []])
                        new_solution[0][k].insert(l + 1, [city_change2, []])
                        new_solution[0][i].insert(j, [city_change3, []])
                    
                    # print(new_solution)
                    # print(Function.fitness(new_solution)[0])
                    # print(Function.Check_if_feasible(new_solution))
                    # print(len(neighborhood))
                    # print("---------------------------") 
                            
                    pack_child = []
                    pack_child.append(new_solution)
                    a = fitness_init(new_solution)                
                    pack_child.append([a])
                    pack_child.append([city_change1, city_change2, city_change3])
                    pack_child.append(i)
                    pack_child.append(k)
                    neighborhood.append(pack_child)
                    
    return neighborhood

def fitness_init(solution):
    result = [0] * Data.number_of_trucks
    for i in range(len(solution[0])):
        current_po = solution[0][i][0][0]
        for j in range(len(solution[0][i]) - 1):
            next_po = solution[0][i][j+1][0]
            result[i] = max(result[i] + Data.manhattan_move_matrix[current_po][next_po], Data.release_date[next_po] + Data.euclid_flight_matrix[0][next_po])
            current_po = next_po
        result[i] += Data.manhattan_move_matrix[current_po][0]
    return max(result)