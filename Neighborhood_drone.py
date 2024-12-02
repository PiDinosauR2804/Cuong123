import copy
import Data
import Function
import random
import Neighborhood

epsilon = (-1)*0.00001
DIFFERENTIAL_RATE_RELEASE_TIME = Data.DIFFERENTIAL_RATE_RELEASE_TIME
B_ratio = Data.B_ratio
C_ratio = Data.C_ratio

def Neighborghood_change_drone_route(solution):
    neighborhood = []
    '''temp = []
    temp.append(solution)
    fit, tt = Function.fitness(solution)
    temp.append([fit, tt])
    neighborhood.append(temp)'''
    
    package = copy.deepcopy(solution[1])
    for i in range(Data.number_of_trucks):
        pack = []
        for j in range(len(solution[0][i][0][1])):
            pack.append(solution[0][i][0][1][j])
        package.insert(0,[[0,pack]])
    # Sắp xếp lại theo thứ tự drone
    for i in range(len(package)):
        for l in range(len(package[i])):
            for j in range(len(package[i][l][1])):
                for k in range(j+1,len(package[i][l][1])):
                    if(Data.release_date[package[i][l][1][j]] < Data.release_date[package[i][l][1][k]]):
                        temp = package[i][l][1][j];
                        package[i][l][1][j] = package[i][l][1][k]
                        package[i][l][1][k] = temp
    for i in range(len(package)):
        for j in range(len(package[i])):
            for k in range(1, len(package[i][j][1])+1):
                NewSolution = copy.deepcopy(solution)
                ChangePackages = []
                ChangeInTruck = Function.city_in_which_truck(solution, package[i][j][1][0])
                IndexOfReceiveCityInTruck = -1
                for l in range(k):
                    ChangePackage = package[i][j][1][l]
                    ChangePackages.append(ChangePackage) # Các gói hàng sẽ chuyển
                    # Xoá ở New Solution
                    if(package[i][j][0] != 0):
                        NewSolution[1][i-Data.number_of_trucks][j][1].remove(ChangePackage)
                        for ii in range(len(NewSolution[0][ChangeInTruck])):
                            if(ChangePackage in NewSolution[0][ChangeInTruck][ii][1]):
                                IndexOfReceiveCityInTruck = ii
                                NewSolution[0][ChangeInTruck][ii][1].remove(ChangePackage)
                                break
                    else:
                        for ii in range(len(NewSolution[0][ChangeInTruck])):
                            if(ChangePackage in NewSolution[0][ChangeInTruck][ii][1]):
                                IndexOfReceiveCityInTruck = ii
                                NewSolution[0][ChangeInTruck][ii][1].remove(ChangePackage)
                                break
                if (Function.sum_weight(ChangePackages) > Data.drone_capacity):
                    continue
                if (NewSolution[0][ChangeInTruck][IndexOfReceiveCityInTruck][0] in ChangePackages):
                    continue
                if (package[i][j][0] != 0):
                    if (NewSolution[1][i-Data.number_of_trucks][j][1] == []):
                        NewSolution[1][i-Data.number_of_trucks].pop(j)
                        if (NewSolution[1][i-Data.number_of_trucks] == []):
                            NewSolution[1].pop(i-Data.number_of_trucks)
                stop = False
                for l in range(IndexOfReceiveCityInTruck + 1, len(NewSolution[0][ChangeInTruck])):
                    '''print("---------------------------------")
                    print(i,", ",j,", ",k,", ",l)'''
                    New_solution1 = copy.deepcopy(NewSolution)
                    '''for iii in range(Data.number_of_trucks):
                        print(New_solution1[0][iii])
                    print(New_solution1[1])
                    print(Function.Check_if_feasible(New_solution1))
                    print("-------------------------------")'''
                    ReceiveCity = New_solution1[0][ChangeInTruck][l][0]
                    if (ReceiveCity in ChangePackages): 
                        stop = True
                        
                    if Data.euclid_flight_matrix[0][ReceiveCity] * 2 + Data.unloading_time > Data.drone_limit_time:
                        if stop:
                            break
                        else:
                            continue
                    
                    # Trường hợp 1: Điểm giao đến không phải điểm nhận hàng
                    if (New_solution1[0][ChangeInTruck][l][1] == []):
                        # Add điểm giao hàng tại truck route
                        New_solution1[0][ChangeInTruck][l][1] = New_solution1[0][ChangeInTruck][l][1] + ChangePackages
                        #Add điểm giao hàng tại drone package
                        if (Data.euclid_flight_matrix[0][ReceiveCity]*2 <= Data.drone_limit_time):
                            Neighborhood.addNewTripInDroneRoute(New_solution1, ChangePackages, ChangeInTruck, l)
                        else:
                            continue
                    # Trường hợp 2: Điểm giao đến cũng là một điểm nhận hàng
                    else:
                        # Add điểm giao hàng tại truck route
                        New_solution1[0][ChangeInTruck][l][1] = New_solution1[0][ChangeInTruck][l][1] + ChangePackages
                        #Add điểm giao hàng tại drone package
                        Neighborhood.groupTripInDroneRoute(New_solution1, ChangePackages, ChangeInTruck, l)
                    '''for iii in range(Data.number_of_trucks):
                        print(New_solution1[0][iii])
                    print(New_solution1[1])
                    print(Function.Check_if_feasible(New_solution1))
                    print("-------------------------------")'''
                    temp = []
                    temp.append(New_solution1)
                    fit, dt, tt = Function.fitness(New_solution1)
                    temp.append([fit, dt, tt])
                    temp.append(-1)
                    temp.append(-1)
                    neighborhood.append(temp)
                    if stop == True:
                        break      
    return neighborhood

def Neighborghood_change_drone_route_plus(solution):
    neighborhood = []
    
    package = copy.deepcopy(solution[1])
    for i in range(Data.number_of_trucks):
        pack = []
        for j in range(len(solution[0][i][0][1])):
            pack.append(solution[0][i][0][1][j])
        package.insert(0,[[0,pack]])
        
    for i in range(len(package)):
        for j in range(len(package[i])):
            index_truck = Function.index_truck_of_cities[package[i][j][0]]
            remained_pack = len(package[i][j][1])
            for k in range(len(solution[0][index_truck])):
                city = solution[0][index_truck][k][0]
                if city in package[i][j][1]:
                    remained_pack -= 1
                    package[i][j][1].remove(city)
                    if Data.release_date[city] > 0:
                        package[i][j][1].append(city)
                if remained_pack == 0:
                    break
        
    for i in range(len(package)):
        for j in range(len(package[i])):
            for k in range(1, len(package[i][j][1])+1):
                Total_case = takeAmountBeginAndEndPackage(package[i][j][1], k)
                for ii in range(len(Total_case)):
                    NewSolution = copy.deepcopy(solution)
                    ChangePackages = copy.deepcopy(Total_case[ii])
                    ChangeInTruck = Function.city_in_which_truck(solution, package[i][j][1][0])
                    IndexOfReceiveCityInTruck = -1
                    #print("Total: ", Total_case[ii])
                    for l in range(len(ChangePackages)):
                        #print("-----------------------------", ChangePackages)
                        ChangePackage = ChangePackages[l]

                        # Xoá ở New Solution
                        if(package[i][j][0] != 0):
                            NewSolution[1][i-Data.number_of_trucks][j][1].remove(ChangePackage)
                            for ii in range(len(NewSolution[0][ChangeInTruck])):
                                if(ChangePackage in NewSolution[0][ChangeInTruck][ii][1]):
                                    IndexOfReceiveCityInTruck = ii
                                    NewSolution[0][ChangeInTruck][ii][1].remove(ChangePackage)
                                    break
                        else:
                            for ii in range(len(NewSolution[0][ChangeInTruck])):
                                if(ChangePackage in NewSolution[0][ChangeInTruck][ii][1]):
                                    IndexOfReceiveCityInTruck = ii
                                    NewSolution[0][ChangeInTruck][ii][1].remove(ChangePackage)
                                    break
                        #print(ChangePackages)
                    #print("Change-1: ",ChangePackages)
                    if (Function.sum_weight(ChangePackages) > Data.drone_capacity):
                        continue
                    if (NewSolution[0][ChangeInTruck][IndexOfReceiveCityInTruck][0] in ChangePackages):
                        continue
                    if (package[i][j][0] != 0):
                        if (NewSolution[1][i-Data.number_of_trucks][j][1] == []):
                            NewSolution[1][i-Data.number_of_trucks].pop(j)
                            if (NewSolution[1][i-Data.number_of_trucks] == []):
                                NewSolution[1].pop(i-Data.number_of_trucks)
                    stop = False
                    for l in range(len(NewSolution[0][ChangeInTruck])):

                        if l == IndexOfReceiveCityInTruck:
                            continue
                        
                        New_solution1 = copy.deepcopy(NewSolution)

                        # for iii in range(Data.number_of_trucks):
                        #     print(New_solution1[0][iii])
                        # print(New_solution1[1])
                        # print(Function.Check_if_feasible(New_solution1))
                        # print("------")
                        
                        ReceiveCity = New_solution1[0][ChangeInTruck][l][0]
                        if ReceiveCity in ChangePackages: 
                            stop = True
                            
                        if Data.euclid_flight_matrix[0][ReceiveCity] * 2 + Data.unloading_time > Data.drone_limit_time:
                            if stop:
                                break
                            else:
                                continue
                        
                        # Trường hợp 1: Điểm giao đến không phải điểm nhận hàng
                        if New_solution1[0][ChangeInTruck][l][1] == []:
                            # Add điểm giao hàng tại truck route
                            for ll in range(len(ChangePackages)):
                                New_solution1[0][ChangeInTruck][l][1] = New_solution1[0][ChangeInTruck][l][1] + [ChangePackages[ll]]
                            #Add điểm giao hàng tại drone package
                            if Data.euclid_flight_matrix[0][ReceiveCity]*2 + Data.unloading_time <= Data.drone_limit_time:
                                if l != 0:
                                    New_solution1, index_drone_trip, index_in_trip = Neighborhood.addNewTripInDroneRoute(New_solution1, ChangePackages, ChangeInTruck, l)
                                '''if(i == 1 and j == 0 and k == 1 and l == 2):
                                    print("------------2-------------------")
                                    for iii in range(Data.number_of_trucks):
                                        print(New_solution1[0][iii])
                                    print(New_solution1[1])
                                    print(Function.Check_if_feasible(New_solution1))
                                    print("-------------------------------")'''
                            else:
                                continue
                        # Trường hợp 2: Điểm giao đến cũng là một điểm nhận hàng
                        else:
                            '''print("----3----")
                            for iii in range(Data.number_of_trucks):
                                print(New_solution1[0][iii])
                            print(New_solution1[1])
                            print(Function.Check_if_feasible(New_solution1))'''
                            # Add điểm giao hàng tại truck route
                            for ll in range(len(ChangePackages)):
                                New_solution1[0][ChangeInTruck][l][1] = New_solution1[0][ChangeInTruck][l][1] + [ChangePackages[ll]]
                            #Add điểm giao hàng tại drone package
                            if l != 0:
                                New_solution1, index_drone_trip, index_in_trip, if_group = Neighborhood.groupTripInDroneRoute(New_solution1, ChangePackages, ChangeInTruck, l)
                            '''print("----4----")
                            for iii in range(Data.number_of_trucks):
                                print(New_solution1[0][iii])
                            print(New_solution1[1])
                            print(Function.Check_if_feasible(New_solution1))'''
                        # for iii in range(Data.number_of_trucks):
                        #     print(New_solution1[0][iii])
                        # print(New_solution1[1])
                        # print(Function.Check_if_feasible(New_solution1))
                        # print("-------------------------------")
                        temp = []
                        temp.append(New_solution1)
                        fit, dt, tt = Function.fitness(New_solution1)
                        temp.append([fit, dt, tt])
                        temp.append(-1)
                        temp.append(-1)
                        neighborhood.append(temp)
                        if stop == True:
                            break              
    return neighborhood

def group_two_trip(solution, index_drone_trip1, index_in_trip1, index_drone_trip2, index_in_trip2):
    package = copy.deepcopy(solution[1][index_drone_trip2][index_in_trip2][1])
    city_remove_package = solution[1][index_drone_trip2][index_in_trip2][0]
    
    for i in range(len(package)):
        solution[1][index_drone_trip1][index_in_trip1][1] += [package[i]]
    
    chosen_city = solution[1][index_drone_trip1][index_in_trip1][0]
    index_truck = Function.index_truck_of_cities[chosen_city]
    
    solution[1][index_drone_trip2].pop(index_in_trip2)
    
    delete_trip = False
    if solution[1][index_drone_trip2] == []:
        solution[1].pop(index_drone_trip2)
        delete_trip = True
    
    for i in range(1, len(solution[0][index_truck])):
        city = solution[0][index_truck][i][0]
        # Xóa trước
        if city == city_remove_package:
            for j in reversed(range(len(solution[0][index_truck][i][1]))):
                pack = solution[0][index_truck][i][1][j]
                if pack in package:
                    solution[0][index_truck][i][1].pop(j)
        # Add sau
        if city == chosen_city:
           for j in range(len(package)):
                solution[0][index_truck][i][1] += [package[j]]
    
    if delete_trip:
        if index_drone_trip1 > index_drone_trip2:
            solution = Rearrange_index_trip(solution, index_drone_trip1 - 1, True)
        elif index_drone_trip1 < index_drone_trip2:
            solution = Rearrange_index_trip(solution, index_drone_trip1, False)
    else:
        if index_drone_trip1 > index_drone_trip2:
            solution = Rearrange_index_trip(solution, index_drone_trip1, True)
        elif index_drone_trip1 < index_drone_trip2:
            solution = Rearrange_index_trip(solution, index_drone_trip1, False)
    
    return solution
    
# Neighborhood_group_trip gộp 1 lần giao hàng tới 1 thành phố cảu drone vào một trip khác
def Neighborhood_group_trip(solution):
    neighborhood = []
    temp = []
    temp.append(solution)
    fit, dt, tt = Function.fitness(solution)
    temp.append([fit, dt, tt])
    neighborhood.append(temp)
    for i in range(len(solution[1])):
        if Data.number_of_drones < 2:
            return neighborhood
        max = -1
        min = -1
        if i <= len(solution[1]) - 4:
            max = i + 4
        else:
            max = len(solution[1])
        if i - 3 >= 0:
            min = i - 3
        else:
            min = 0
        for j in range(min, max):
            if i == j: continue
            for k in range(len(solution[1][j])):
                new_solution = copy.deepcopy(solution)
                package_need_group = []
                package_need_group_to_city = solution[1][j][k][0]
                for l in range(len(solution[1][j][k][1])):
                    package_need_group.append(solution[1][j][k][1][l])
                change_drone_flight_come_truck_number = Function.city_in_which_truck(solution,package_need_group_to_city)
                if Function.total_demand(solution[1][i]) + Function.sum_weight(package_need_group) <= Data.drone_capacity:
                    if Function.max_release_date_update(solution[1][i]) *  DIFFERENTIAL_RATE_RELEASE_TIME + Data.standard_deviation > Function.max_release_date(package_need_group) and Function.min_release_date_update(solution[1][i]) *  DIFFERENTIAL_RATE_RELEASE_TIME + B_ratio * Data.standard_deviation > Function.max_release_date(package_need_group):
                        check = False
                        check_break = False
                        for l in range(len(solution[1][i])):
                            new_city_of_package_need_group = solution[1][i][l][0]
                            come_to_truck = Function.city_in_which_truck(solution,new_city_of_package_need_group)
                            if change_drone_flight_come_truck_number == come_to_truck:
                                if j < i: 
                                    check_break = True
                                    break
                                new_solution[1][i][l][1] += package_need_group
                                # Sửa lại new_solution[0]
                                for m in range(len(new_solution[0][come_to_truck])):
                                    city_evaluate = new_solution[0][come_to_truck][m][0]
                                    if city_evaluate == new_city_of_package_need_group:
                                        new_solution[0][come_to_truck][m][1] += package_need_group
                                    if city_evaluate == package_need_group_to_city:
                                        for n in range(len(package_need_group)):
                                            new_solution[0][come_to_truck][m][1].remove(package_need_group[n])
                                check = True
                        '''if(i == 1 and j == 3 and k ==0):
                            print("-------------------------------")
                            for iii in range(Data.number_of_trucks):
                                print(new_solution[0][iii])
                            print(new_solution[1])
                            print("-------------------------------")'''
                        if check_break: continue
                        
                        if not check:
                            stop = False
                            if i < j:
                                for m in range(i+1, j):
                                    for n in range(len(new_solution[1][m])):
                                        checkCoincideTruck = Function.city_in_which_truck(new_solution, new_solution[1][m][n][0])
                                        if checkCoincideTruck == change_drone_flight_come_truck_number:
                                            stop = True
                                            break
                                    if stop: break
                            else:
                                for m in range(j+1, i):
                                    for n in range(len(new_solution[1][m])):
                                        checkCoincideTruck = Function.city_in_which_truck(new_solution, new_solution[1][m][n][0])
                                        if checkCoincideTruck == change_drone_flight_come_truck_number:
                                            stop = True
                                            break
                                    if stop: break
                            if stop: continue                                
                            new_solution[1][i].append([package_need_group_to_city, package_need_group])
                            shortest_route_by_point, shortest_route_by_truck = Function.find_drone_flight_shortest(solution, new_solution[1][i])
                            drone_fly_time = Data.euclid_flight_matrix[0][shortest_route_by_point[0]] + Data.euclid_flight_matrix[shortest_route_by_point[len(shortest_route_by_point)-1]][0]
                            for m in range(len(shortest_route_by_point)-1):
                                drone_fly_time += Data.euclid_flight_matrix[shortest_route_by_point[m]][shortest_route_by_point[m+1]]
                            if drone_fly_time + len(shortest_route_by_point) * Data.unloading_time > Data.drone_limit_time:
                                continue
                        # Gạch phần tử được gộp ra khỏi new_solution mới
                        new_solution[1][j].pop(k)
                        if new_solution[1][j] == []:
                            new_solution[1].pop(j)
                        '''if(i == 1 and j == 3 and k ==0):
                            print("-------------------------------")
                            for iii in range(Data.number_of_trucks):
                                print(new_solution[0][iii])
                            print(new_solution[1])
                            print("-------------------------------")'''
                        temp = []
                        temp.append(new_solution)
                        fit, dt, tt = Function.fitness(new_solution)
                        temp.append([fit, dt, tt])
                        #print(fit)
                        temp.append([i,j,k])
                        neighborhood.append(temp)
        '''if len(neighborhood) == 0:
        first = [solution]
        fit, tt = Function.fitness(solution)
        first.append([fit, tt])
        first.append([0, 0, 0])
        neighborhood.append(first)'''
    return neighborhood

def choose_what_to_group(solution, index_drone_trip, index_in_trip, find_in_forward):
    chosen_city = solution[1][index_drone_trip][index_in_trip][0]
    index_truck = Function.index_truck_of_cities[chosen_city] 
    index_chosen_city_in_truck = -1
    for i in range(len(solution[0][index_truck])):
        city = solution[0][index_truck][i][0]
        if city == chosen_city:
            index_chosen_city_in_truck = i
    demand_package = 0
    stone_package = solution[1][index_drone_trip][index_in_trip][1]
    for i in range(len(stone_package)):
        demand_package += Data.city_demand[stone_package[i]]
    
    if find_in_forward == False:
        index_drone_trip_choose_to_group = -1
        index_in_trip_choose_to_group = -1
        stop1368 = False
        for i in reversed(range(index_drone_trip)):
            for j in range(len(solution[1][i])):
                if Function.index_truck_of_cities[solution[1][i][j][0]] == index_truck:
                    index_drone_trip_choose_to_group = i
                    index_in_trip_choose_to_group = j
                    stop1368 = True
                    break
            if stop1368:
                break
            
        if not stop1368:
            pre_potential_package = copy.deepcopy(solution[0][index_truck][0][1])
            potential_package = []
            for i in range(len(solution[0][index_truck])):
                city = solution[0][index_truck][i][0]
                if city in pre_potential_package:
                    potential_package.append(city)
            
            for i in range(len(potential_package)):
                if Data.release_date[potential_package[i]] == 0:
                    continue
                
                continue744 = False
                
                for ii in range(index_chosen_city_in_truck):
                    if solution[0][index_truck][ii][0] in potential_package:
                        continue744 = True
                        break
                    
                if continue744:
                    continue
                
                if demand_package + Data.city_demand[potential_package[i]] <= Data.drone_capacity:
                    demand_package += Data.city_demand[potential_package[i]]
                    solution[0][index_truck][0][1].remove(potential_package[i])
                    solution[0][index_truck][index_chosen_city_in_truck][1] += [potential_package[i]]
                    solution[1][index_drone_trip][index_in_trip][1] += [potential_package[i]]
        
        else:           
            move_package = solution[1][index_drone_trip_choose_to_group][index_in_trip_choose_to_group][1]
            
            continue771 = False
            
            for i in range(index_chosen_city_in_truck):
                if solution[0][index_truck][i][0] in move_package:
                    continue771 = True
                    break
            
            if not continue771:
                if demand_package + Function.sum_weight(solution[1][index_drone_trip_choose_to_group][index_in_trip_choose_to_group][1]) <= Data.drone_capacity:
                    if Function.max_release_date(stone_package) + Data.standard_deviation > Function.max_release_date(solution[1][index_drone_trip_choose_to_group][index_in_trip_choose_to_group][1]):
                        solution = group_two_trip(solution, index_drone_trip, index_in_trip, index_drone_trip_choose_to_group, index_in_trip_choose_to_group)
    else:
        
        index_drone_trip_choose_to_group = -1
        index_in_trip_choose_to_group = -1
        stop761 = False
        for i in range(index_drone_trip + 1, len(solution[1])):
            for j in range(len(solution[1][i])):
                if Function.index_truck_of_cities[solution[1][i][j][0]] == index_truck:
                    index_drone_trip_choose_to_group = i
                    index_in_trip_choose_to_group = j
                    stop761 = True
                    break
            if stop761:
                break
        if index_drone_trip_choose_to_group != -1:
            if demand_package + Function.sum_weight(solution[1][index_drone_trip_choose_to_group][index_in_trip_choose_to_group][1]) <= Data.drone_capacity:
                if Function.max_release_date(stone_package) + Data.standard_deviation > Function.max_release_date(solution[1][index_drone_trip_choose_to_group][index_in_trip_choose_to_group][1]):
                    solution = group_two_trip(solution, index_drone_trip, index_in_trip, index_drone_trip_choose_to_group, index_in_trip_choose_to_group)

    return solution

def Neighborghood_change_drone_route_max_pro_plus(solution):
    neighborhood = []
    Function.update_per_loop(solution)
    package = copy.deepcopy(solution[1])
    
    for i in range(Data.number_of_trucks):
        pack = []
        for j in range(len(solution[0][i][0][1])):
            pack.append(solution[0][i][0][1][j])
        package.insert(0,[[0,pack]])
        
    for i in range(len(package)):
        for j in range(len(package[i])):
            index_truck = Function.index_truck_of_cities[package[i][j][0]]
            remained_pack = len(package[i][j][1])
            for k in range(len(solution[0][index_truck])):
                city = solution[0][index_truck][k][0]
                if city in package[i][j][1]:
                    remained_pack -= 1
                    package[i][j][1].remove(city)
                    if Data.release_date[city] > 0:
                        package[i][j][1].append(city)
                if remained_pack == 0:
                    break

    for i in range(len(package)):
        for j in range(len(package[i])):
            initial_demand_package = Function.sum_weight(package[i][j][1])
            for k in range(1, len(package[i][j][1])+1):
                Total_case = takeAmountBeginAndEndPackage(package[i][j][1])
                for ii in range(len(Total_case)):
                    NewSolution = copy.deepcopy(solution)
                    ChangePackages = copy.deepcopy(Total_case[ii])
                    ChangeInTruck = Function.city_in_which_truck(solution, package[i][j][1][0])
                    IndexOfReceiveCityInTruck = -1
                    #print("Total: ", Total_case[ii])
                    for l in range(len(ChangePackages)):
                        # print("hehe: ", ChangePackages)
                        #print("-----------------------------", ChangePackages)
                        ChangePackage = ChangePackages[l]
                        #ChangePackages.append(ChangePackage)
                        #print(ChangePackage)
                        #print(ChangePackages)
                        # Xoá ở New Solution
                        if package[i][j][0] != 0:
                            NewSolution[1][i-Data.number_of_trucks][j][1].remove(ChangePackage)
                            for ii in range(len(NewSolution[0][ChangeInTruck])):
                                if(ChangePackage in NewSolution[0][ChangeInTruck][ii][1]):
                                    IndexOfReceiveCityInTruck = ii
                                    NewSolution[0][ChangeInTruck][ii][1].remove(ChangePackage)
                                    break
                        else:
                            for ii in range(len(NewSolution[0][ChangeInTruck])):
                                if ChangePackage in NewSolution[0][ChangeInTruck][ii][1]:
                                    IndexOfReceiveCityInTruck = ii
                                    NewSolution[0][ChangeInTruck][ii][1].remove(ChangePackage)
                                    break
                        #print(ChangePackages)
                    #print("Change-1: ",ChangePackages)
                    if Function.sum_weight(ChangePackages) > Data.drone_capacity:
                        continue
                    if NewSolution[0][ChangeInTruck][IndexOfReceiveCityInTruck][0] in ChangePackages:
                        continue
                    
                    if package[i][j][0] != 0:
                        if NewSolution[1][i-Data.number_of_trucks][j][1] == []:
                            NewSolution[1][i-Data.number_of_trucks].pop(j)
                            if NewSolution[1][i-Data.number_of_trucks] == []:
                                NewSolution[1].pop(i-Data.number_of_trucks)
                        else:
                            NewSolution = Rearrange_index_trip(NewSolution, i-Data.number_of_trucks, False)
                    stop = False
                    for l in range(len(NewSolution[0][ChangeInTruck])):
                        
                        if l == IndexOfReceiveCityInTruck:
                            continue

                        New_solution1 = copy.deepcopy(NewSolution)
                        
                        ReceiveCity = New_solution1[0][ChangeInTruck][l][0]
                        if ReceiveCity in ChangePackages: 
                            stop = True
                        
                        if Data.euclid_flight_matrix[0][ReceiveCity] * 2 + Data.unloading_time > Data.drone_limit_time:
                            if stop:
                                break
                            else:
                                continue
                        
                        demand_change_package = Function.sum_weight(ChangePackages)
                        
                        # Trường hợp 1: Điểm giao đến không phải điểm nhận hàng
                        if New_solution1[0][ChangeInTruck][l][1] == []:
                            # Add điểm giao hàng tại truck route
                            for ll in range(len(ChangePackages)):
                                New_solution1[0][ChangeInTruck][l][1] = New_solution1[0][ChangeInTruck][l][1] + [ChangePackages[ll]]
                            #Add điểm giao hàng tại drone package
                            if Data.euclid_flight_matrix[0][ReceiveCity] * 2 + Data.unloading_time <= Data.drone_limit_time:
                                if l != 0:
                                    index_drone_trip = -1
                                    index_in_trip = -1
                                    New_solution1, index_drone_trip, index_in_trip = Neighborhood.addNewTripInDroneRoute(New_solution1, ChangePackages, ChangeInTruck, l)
                                
                                if l > IndexOfReceiveCityInTruck:
                                    if demand_change_package <= 0.5 * Data.drone_capacity:
                                        New_solution1 = choose_what_to_group(New_solution1, index_drone_trip, index_in_trip, True)
                                    if initial_demand_package - demand_change_package <= 0.5 * Data.drone_capacity:
                                        if initial_demand_package != demand_change_package:
                                            if i - Data.number_of_trucks >= 0:
                                                New_solution1 = choose_what_to_group(New_solution1, i - Data.number_of_trucks, j, False)
                                else:
                                    if initial_demand_package != demand_change_package:
                                        if initial_demand_package - demand_change_package <= 0.5 * Data.drone_capacity:
                                            if l != 0:
                                                New_solution1 = choose_what_to_group(New_solution1, i - Data.number_of_trucks + 1, j, True)
                                            else:
                                                New_solution1 = choose_what_to_group(New_solution1, i - Data.number_of_trucks, j, True)
                                    if demand_change_package <= 0.5 * Data.drone_capacity:
                                        if l != 0:
                                            New_solution1 = choose_what_to_group(New_solution1, index_drone_trip, index_in_trip, False)
                                
                            else:
                                continue
                        # Trường hợp 2: Điểm giao đến cũng là một điểm nhận hàng
                        else:
                            
                            # Add điểm giao hàng tại truck route
                            for ll in range(len(ChangePackages)):
                                New_solution1[0][ChangeInTruck][l][1] = New_solution1[0][ChangeInTruck][l][1] + [ChangePackages[ll]]
                            #Add điểm giao hàng tại drone package
                            if_group = True
                            
                            if l != 0: 
                                index_drone_trip = -1
                                index_in_trip = -1
                                New_solution1, index_drone_trip, index_in_trip, if_group = Neighborhood.groupTripInDroneRoute(New_solution1, ChangePackages, ChangeInTruck, l)
                                if if_group:
                                    NewSolution = Rearrange_index_trip(NewSolution, index_drone_trip, True)
                                       
                            if if_group:    # Gộp thành công
                                
                                if l > IndexOfReceiveCityInTruck:
                                    if Function.sum_weight(New_solution1[1][index_drone_trip][index_in_trip][1]) <= 0.5 * Data.drone_capacity:
                                        New_solution1 = choose_what_to_group(New_solution1, index_drone_trip, index_in_trip, True)
                                    if initial_demand_package != demand_change_package:
                                        if initial_demand_package - demand_change_package <= 0.5 * Data.drone_capacity:
                                            if i - Data.number_of_trucks >= 0:
                                                New_solution1 = choose_what_to_group(New_solution1, i - Data.number_of_trucks, j, False)
                                else:
                                    if initial_demand_package != demand_change_package:
                                        if initial_demand_package - demand_change_package <= 0.5 * Data.drone_capacity:
                                            New_solution1 = choose_what_to_group(New_solution1, i - Data.number_of_trucks, j, True)
                                        
                                    if l != 0:
                                        if Function.sum_weight(New_solution1[1][index_drone_trip][index_in_trip][1]) <= 0.5 * Data.drone_capacity:
                                            New_solution1 = choose_what_to_group(New_solution1, index_drone_trip, index_in_trip, False)
                           
                            else:           # Gộp thất bại, tạo trip mới riêng
                                if l > IndexOfReceiveCityInTruck:
                                    if demand_change_package <= 0.5 * Data.drone_capacity:
                                        New_solution1 = choose_what_to_group(New_solution1, index_drone_trip, index_in_trip, True)
                                    if i - Data.number_of_trucks >= 0:
                                        if initial_demand_package != demand_change_package:
                                            if initial_demand_package - demand_change_package <= 0.5 * Data.drone_capacity:
                                                New_solution1 = choose_what_to_group(New_solution1, i - Data.number_of_trucks, j, False)
                                else:
                                    if initial_demand_package != demand_change_package:
                                        if initial_demand_package - demand_change_package <= 0.5 * Data.drone_capacity:
                                            New_solution1 = choose_what_to_group(New_solution1, i - Data.number_of_trucks + 1, j, True)
                                    if demand_change_package <= 0.5 * Data.drone_capacity:
                                        if l != 0:
                                            New_solution1 = choose_what_to_group(New_solution1, index_drone_trip, index_in_trip, False)
                                    
                        # for iii in range(Data.number_of_trucks):
                        #     print(solution[0][iii])
                        # print(solution[1])
                        # print("----")    
                        # for iii in range(Data.number_of_trucks):
                        #     print(New_solution1[0][iii])
                        # print(New_solution1[1])
                        # # print(New_solution1)
                        # print(Function.cal_truck_time(New_solution1))
                        # print(len(neighborhood))
                        # print("-------------------------------")
                        
                        temp = []
                        temp.append(New_solution1)
                        fit, dt, tt = Function.fitness(New_solution1)
                        temp.append([fit, dt, tt])
                        temp.append(-1)
                        temp.append(-1)
                        neighborhood.append(temp)
                        if stop == True:
                            break              
    return neighborhood

def Neighborghood_change_drone_route_max_pro_plus_for_specific_truck(solution, accept_truck_list):
    # print("solution0000: ", solution)
    neighborhood = []
    minus_for_number_of_trucks = Data.number_of_trucks
    Function.update_per_loop(solution)
    package = copy.deepcopy(solution[1])
    
    for i in range(Data.number_of_trucks):
        if i in accept_truck_list:
            pack = []
            for j in range(len(solution[0][i][0][1])):
                pack.append(solution[0][i][0][1][j])
            package.insert(0,[[0,pack]])
        else:
            minus_for_number_of_trucks -= 1
        
    # print("solution1111: ", solution)
    for i in reversed(range(len(package))):
        for j in reversed(range(len(package[i]))):
            if package[i][j][0] == 0:
                continue
            index_truck = Function.index_truck_of_cities[package[i][j][0]]
            
            remained_pack = len(package[i][j][1])
            if index_truck in accept_truck_list:
                for k in range(len(solution[0][index_truck])):
                    city = solution[0][index_truck][k][0]
                    if city in package[i][j][1]:
                        remained_pack -= 1
                        package[i][j][1].remove(city)
                        if Data.release_date[city] > 0:
                            package[i][j][1].append(city)
                    if remained_pack == 0:
                        break    
        
    # print(package)
    # print("solution111: ", solution)
    for i in range(len(package)):
        for j in range(len(package[i])):
            if package[i][j][0] != 0:
                if Function.index_truck_of_cities[package[i][j][0]] not in accept_truck_list:
                    continue
            initial_demand_package = Function.sum_weight(package[i][j][1]) 
            Total_case = takeAmountBeginAndEndPackage(package[i][j][1])
            for ii in range(len(Total_case)):
                NewSolution = copy.deepcopy(solution)
                ChangePackages = Total_case[ii]
                ChangeInTruck = Function.city_in_which_truck(solution, package[i][j][1][0])
                # print("ChangePackages: ", ChangePackages)
                # print("solution: ", NewSolution)
                IndexOfReceiveCityInTruck = -1
                # print("Package: ", Total_case[ii])
                
                for l in range(len(ChangePackages)):
                    # print("hehe: ", ChangePackages)
                    #print("-----------------------------", ChangePackages)
                    ChangePackage = ChangePackages[l]
                    
                    #ChangePackages.append(ChangePackage)
                    #print(ChangePackage)
                    #print(ChangePackages)
                    # Xoá ở New Solution
                    if package[i][j][0] != 0:
                        NewSolution[1][i-minus_for_number_of_trucks][j][1].remove(ChangePackage)
                        for ii in range(len(NewSolution[0][ChangeInTruck])):
                            if ChangePackage in NewSolution[0][ChangeInTruck][ii][1]:
                                IndexOfReceiveCityInTruck = ii
                                NewSolution[0][ChangeInTruck][ii][1].remove(ChangePackage)
                                break
                    else:
                        for ii in range(len(NewSolution[0][ChangeInTruck])):
                            if ChangePackage in NewSolution[0][ChangeInTruck][ii][1]:
                                IndexOfReceiveCityInTruck = ii
                                NewSolution[0][ChangeInTruck][ii][1].remove(ChangePackage)
                                break
                    #print(ChangePackages)
                #print("Change-1: ",ChangePackages)
                # print("heheee: ", IndexOfReceiveCityInTruck)
                if Function.sum_weight(ChangePackages) > Data.drone_capacity:
                    continue
                if NewSolution[0][ChangeInTruck][IndexOfReceiveCityInTruck][0] in ChangePackages:
                    continue
                if package[i][j][0] != 0:
                    if NewSolution[1][i-minus_for_number_of_trucks][j][1] == []:
                        NewSolution[1][i-minus_for_number_of_trucks].pop(j)
                        if NewSolution[1][i-minus_for_number_of_trucks] == []:
                            NewSolution[1].pop(i-minus_for_number_of_trucks)
                stop = False
                for l in range(len(NewSolution[0][ChangeInTruck])):
                    
                    if l == IndexOfReceiveCityInTruck:
                        continue

                    New_solution1 = copy.deepcopy(NewSolution)
                    # print("Package: ", ChangePackages)
                    
                    # for iii in range(Data.number_of_trucks):
                    #     print(New_solution1[0][iii])
                    # print(New_solution1[1])
                    # print("------")
                    
                    demand_change_package = Function.sum_weight(ChangePackages)
                    
                    ReceiveCity = New_solution1[0][ChangeInTruck][l][0]
                    if ReceiveCity in ChangePackages: 
                        stop = True
                    
                    if Data.euclid_flight_matrix[0][ReceiveCity] * 2 + Data.unloading_time > Data.drone_limit_time:
                        if stop:
                            break
                        else:
                            continue
                    
                    # Trường hợp 1: Điểm giao đến không phải điểm nhận hàng
                    if New_solution1[0][ChangeInTruck][l][1] == []:
                        # Add điểm giao hàng tại truck route
                        for ll in range(len(ChangePackages)):
                            New_solution1[0][ChangeInTruck][l][1] = New_solution1[0][ChangeInTruck][l][1] + [ChangePackages[ll]]
                        #Add điểm giao hàng tại drone package
                        if Data.euclid_flight_matrix[0][ReceiveCity] * 2 <= Data.drone_limit_time:
                            if l != 0:
                                index_drone_trip = -1
                                index_in_trip = -1
                                New_solution1, index_drone_trip, index_in_trip = Neighborhood.addNewTripInDroneRoute(New_solution1, ChangePackages, ChangeInTruck, l)
                            
                            # print(New_solution1)
                            # print("----after1-----")
                            
                            if l > IndexOfReceiveCityInTruck:
                                if demand_change_package <= 0.5 * Data.drone_capacity:
                                    New_solution1 = choose_what_to_group(New_solution1, index_drone_trip, index_in_trip, True)
                                if initial_demand_package - demand_change_package <= 0.5 * Data.drone_capacity:
                                    if initial_demand_package != demand_change_package:
                                        if i - minus_for_number_of_trucks >= 0:
                                            New_solution1 = choose_what_to_group(New_solution1, i - minus_for_number_of_trucks, j, False)
                            else:
                                if initial_demand_package != demand_change_package:
                                    if initial_demand_package - demand_change_package <= 0.5 * Data.drone_capacity:
                                        if l != 0:
                                            New_solution1 = choose_what_to_group(New_solution1, i - minus_for_number_of_trucks + 1, j, True)
                                        else:
                                            New_solution1 = choose_what_to_group(New_solution1, i - minus_for_number_of_trucks, j, True)
                                if demand_change_package <= 0.5 * Data.drone_capacity:
                                    if l != 0:
                                        New_solution1 = choose_what_to_group(New_solution1, index_drone_trip, index_in_trip, False)

                                
                            
                        else:
                            continue
                    # Trường hợp 2: Điểm giao đến cũng là một điểm nhận hàng
                    else:
                        
                        # Add điểm giao hàng tại truck route
                        for ll in range(len(ChangePackages)):
                            New_solution1[0][ChangeInTruck][l][1] = New_solution1[0][ChangeInTruck][l][1] + [ChangePackages[ll]]
                        #Add điểm giao hàng tại drone package
                        if_group = True
                        if l != 0: 
                            index_drone_trip = -1
                            index_in_trip = -1
                            New_solution1, index_drone_trip, index_in_trip, if_group = Neighborhood.groupTripInDroneRoute(New_solution1, ChangePackages, ChangeInTruck, l)
                        
                        # print(New_solution1)
                        # print("----after1-----")
                        
                        if if_group:    # Gộp thành công
                            
                            if l > IndexOfReceiveCityInTruck:
                                # print("IndexOfReceiveCityInTruck: ", IndexOfReceiveCityInTruck)
                                # print("l: ", l)
                                # print("index_drone_trip: ", index_drone_trip)
                                if Function.sum_weight(New_solution1[1][index_drone_trip][index_in_trip][1]) <= 0.5 * Data.drone_capacity:
                                    New_solution1 = choose_what_to_group(New_solution1, index_drone_trip, index_in_trip, True)
                                if initial_demand_package - demand_change_package <= 0.5 * Data.drone_capacity:
                                    if initial_demand_package != demand_change_package:
                                        if i - minus_for_number_of_trucks >= 0:
                                            New_solution1 = choose_what_to_group(New_solution1, i - minus_for_number_of_trucks, j, False)
                            else:
                                if initial_demand_package != demand_change_package:
                                    if initial_demand_package - demand_change_package <= 0.5 * Data.drone_capacity:
                                        New_solution1 = choose_what_to_group(New_solution1, i - minus_for_number_of_trucks, j, True)
                                if l != 0:
                                    if Function.sum_weight(New_solution1[1][index_drone_trip][index_in_trip][1]) <= 0.5 * Data.drone_capacity:
                                        New_solution1 = choose_what_to_group(New_solution1, index_drone_trip, index_in_trip, False)
                        
                        else:           # Gộp thất bại, tạo trip mới riêng
                            if l > IndexOfReceiveCityInTruck:
                                if demand_change_package <= 0.5 * Data.drone_capacity:
                                    New_solution1 = choose_what_to_group(New_solution1, index_drone_trip, index_in_trip, True)
                                if i - minus_for_number_of_trucks >= 0:
                                    if initial_demand_package != demand_change_package:
                                        if initial_demand_package - demand_change_package <= 0.5 * Data.drone_capacity:
                                            New_solution1 = choose_what_to_group(New_solution1, i - minus_for_number_of_trucks, j, False)
                            else:
                                if initial_demand_package != demand_change_package:
                                    if initial_demand_package - demand_change_package <= 0.5 * Data.drone_capacity:
                                        # print("index_drone_trip11: ", i - Data.number_of_trucks, "index_in_trip1: ", j)
                                        # print("second: ", New_solution1)
                                        New_solution1 = choose_what_to_group(New_solution1, i - minus_for_number_of_trucks + 1, j, True)
                                    
                                if demand_change_package <= 0.5 * Data.drone_capacity:
                                    if l != 0:
                                        New_solution1 = choose_what_to_group(New_solution1, index_drone_trip, index_in_trip, False)
                    # print(New_solution1)
                    # print(Function.fitness(New_solution1)[0])
                    # print(len(neighborhood))
                    # print("-------------------------------")
                    
                    temp = []
                    temp.append(New_solution1)
                    fit, dt, tt = Function.fitness(New_solution1)
                    temp.append([fit, dt, tt])
                    temp.append(-1)
                    temp.append(-1)
                    neighborhood.append(temp)
                    if stop == True:
                        break              
    return neighborhood

def takeRetrieval(Package, ChangePackes, PackageTrip, CurrentNumber, CurrentAmount, TakenAmount):
    if CurrentAmount == 0:
        for i in range(len(PackageTrip)- (TakenAmount - CurrentAmount) + 1):          
            Package[CurrentAmount] = PackageTrip[i]
            CurrentNumber = i + 1
            if CurrentAmount == TakenAmount - 1:
                temp = copy.deepcopy(Package)
                ChangePackes.append(temp)
            else:
                takeRetrieval(Package, ChangePackes, PackageTrip, CurrentNumber, CurrentAmount + 1, TakenAmount)
    else:
        for i in range(CurrentNumber, len(PackageTrip)- (TakenAmount - CurrentAmount) + 1):
            Package[CurrentAmount] = PackageTrip[i]
            CurrentNumber = i + 1
            if CurrentAmount == TakenAmount - 1:
                temp = copy.deepcopy(Package)
                ChangePackes.append(temp)
            else:
                takeRetrieval(Package, ChangePackes, PackageTrip, CurrentNumber, CurrentAmount + 1, TakenAmount)

def takePermutationPackage(PackageTrip, TakenAmount):
    ChangePackages = []
    Package = [-1] * TakenAmount
    takeRetrieval(Package, ChangePackages, PackageTrip, 0, 0, TakenAmount)
    return ChangePackages

def takeAmountBeginAndEndPackage(PackageTrip):
    ChangePackages = []
    for i in range(1, len(PackageTrip) + 1):
        if len(PackageTrip) == i:
            package = copy.deepcopy(PackageTrip)
            ChangePackages.append(package)
        else:
            package1 = copy.deepcopy(PackageTrip[:i])
            ChangePackages.append(package1)
            package2 = copy.deepcopy(PackageTrip[-i:])
            ChangePackages.append(package2)
    return ChangePackages

def Rearrange_index_trip(solution, index_trip, forward):
    max_release_date_time = Function.max_release_date_update(solution[1][index_trip])
    shortest_route_by_point, shortest_route_by_truck = Function.find_drone_flight_shortest(solution, solution[1][index_trip])
    time_fly = Data.euclid_flight_matrix[0][shortest_route_by_point[0]] + Data.euclid_flight_matrix[shortest_route_by_point[-1]][0]
    for i in range(len(shortest_route_by_point) - 1):
        time_fly += Data.euclid_flight_matrix[shortest_route_by_point[i]][shortest_route_by_point[i+1]]
    total_time = time_fly + max_release_date_time 
    next_index = index_trip
    index_truck = []
    for i in range(len(solution[1][index_trip])):
        index_truck.append(Function.index_truck_of_cities[solution[1][index_trip][i][0]])
    max_index = len(solution[1]) - 1
    stop_loop = False
    if forward:
        convert = True
        stop = False
        while not stop:
            if next_index == max_index:
                break
            for i in range(len(solution[1][next_index + 1])):
                if Function.index_truck_of_cities[solution[1][next_index + 1][i][0]] in index_truck:
                    stop_loop = True
            if stop_loop:
                break
            next_index += 1
            compared_time_fly = Function.cal_time_fly_a_trip(solution[1][next_index])
            if compared_time_fly + Function.max_release_date_update(solution[1][next_index]) < total_time:
                convert = False
            else:
                stop = True
        if convert:
            stop = False
            while not stop:
                if next_index == 0:
                    break
                for i in range(len(solution[1][next_index - 1])):
                    if Function.index_truck_of_cities[solution[1][next_index - 1][i][0]] in index_truck:
                        stop_loop = True
                if stop_loop:
                    break
                next_index -= 1
                compared_time_fly = Function.cal_time_fly_a_trip(solution[1][next_index])
                if compared_time_fly + Function.max_release_date_update(solution[1][next_index]) < total_time:
                    stop = True
    else:
        convert = True
        stop = False
        while not stop:
            if next_index == 0:
                break
            for i in range(len(solution[1][next_index - 1])):
                if Function.index_truck_of_cities[solution[1][next_index - 1][i][0]] in index_truck:
                    stop_loop = True
            if stop_loop:
                break
            next_index -= 1
            compared_time_fly = Function.cal_time_fly_a_trip(solution[1][next_index])
            if compared_time_fly + Function.max_release_date_update(solution[1][next_index]) > total_time:
                convert = False
            else:
                stop = True
        if convert:
            stop = False
            while not stop:
                if next_index == max_index:
                    break
                for i in range(len(solution[1][next_index + 1])):
                    if Function.index_truck_of_cities[solution[1][next_index + 1][i][0]] in index_truck:
                        stop_loop = True
                if stop_loop:
                    break
                next_index += 1
                compared_time_fly = Function.cal_time_fly_a_trip(solution[1][next_index])
                if compared_time_fly + Function.max_release_date_update(solution[1][next_index]) > total_time:
                    stop = True
                    
    if next_index > index_trip:
        temp = solution[1][index_trip]
        solution[1].pop(index_trip)
        solution[1].insert(next_index, temp)
    elif next_index < index_trip:
        temp = solution[1][index_trip]
        solution[1].pop(index_trip)
        solution[1].insert(next_index, temp)
                
    return solution

def Change_index_trip(solution, index_of_trip, to_index):
    temp = solution[1][index_of_trip]
    solution[1].pop(index_of_trip)
    if index_of_trip > to_index:
        solution[1].insert(to_index, temp)
    else:
        solution[1].insert(to_index - 1, temp)
    return solution

def Neighborhood_change_index_trip(solution):
    Function.update_per_loop(solution)
    neighborhood = []
    if Data.number_of_trucks < 2:
        return neighborhood
    for i in range(len(solution[1])):
        below = i
        above = i
        list_city_stop = []
        for j in range(len(solution[1][i])):
            list_city_stop.append(Function.index_truck_of_cities[solution[1][i][j][0]])
        for j in range(i + 1, len(solution[1])):
            stop = False
            for k in range(len(solution[1][j])):
                if Function.index_truck_of_cities[solution[1][j][k][0]] in list_city_stop:
                    stop = True
            if stop:
                break
            else:
                above += 1
        for j in range(i - 1, -1, -1):
            stop = False
            for k in range(len(solution[1][j])):
                if Function.index_truck_of_cities[solution[1][j][k][0]] in list_city_stop:
                    stop = True
            if stop:
                break
            else:
                below -= 1
        # print(i, ": ", below, " ", above)
        for j in range(below, above + 1):
            if j == i or j == i + 1:
                continue
            new_solution = copy.deepcopy(solution)
            new_solution = Change_index_trip(new_solution, i, j)
            
            
            # print(new_solution[0])
            # print(new_solution[1])
            # print(Function.Check_if_feasible(new_solution))
            # print(Function.fitness(new_solution)[0])
            # print(len(neighborhood))
            # print("--------------")
            temp = []
            temp.append(new_solution)
            fit, dt, tt = Function.fitness(new_solution)
            temp.append([fit, dt, tt])
            temp.append(-1)
            temp.append(-1)
            neighborhood.append(temp)
    return neighborhood
