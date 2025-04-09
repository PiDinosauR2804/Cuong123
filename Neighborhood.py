import copy
import Data
import Function
import random
import Neighborhood11
import Neighborhood10
import Neighborhood_drone

epsilon = (-1)*0.00001
DIFFERENTIAL_RATE_RELEASE_TIME = Data.DIFFERENTIAL_RATE_RELEASE_TIME
B_ratio = Data.B_ratio
C_ratio = Data.C_ratio

# Phục vụ cho nei Neighborghood_change_drone_route, đọc lúc sau
def addNewTripInDroneRoute(solution, ChangesPackages, ChangeInTruck, IndexOfNewReceiveCity):
    if IndexOfNewReceiveCity == 0:
        return solution, -1, -1
    NewReceiveCity = solution[0][ChangeInTruck][IndexOfNewReceiveCity][0]
    min_point = -1  # Chiếu theo điểm nhận hàng
    max_point = -1
    for a in range(IndexOfNewReceiveCity - 1, 0, -1):  # Tìm min_point_check trên new_solution1[1]
        if solution[0][ChangeInTruck][a][1] != []:
            min_point = solution[0][ChangeInTruck][a][0]
            break
    for a in range(IndexOfNewReceiveCity + 1, len(solution[0][ChangeInTruck])):  # Tìm max_point_check trên new_solution1[1]
        if solution[0][ChangeInTruck][a][1] != []:
            max_point = solution[0][ChangeInTruck][a][0]
            break
    if min_point != -1:  # Kiểm tra và điều chỉnh min_point
        stop_here = False
        for a in range(len(solution[1]) - 1, -1, -1):
            for b in range(len(solution[1][a])):
                if solution[1][a][b][0] == min_point:
                    start_check_point_in_drone_queue = a + 1
                    stop_here = True
                    break
            if stop_here:
                break
    else:
        start_check_point_in_drone_queue = 0
        
    number_in_the_drone_queue_of_drop_package = -1
    check_max = False
    time_fly = Function.cal_time_fly_a_trip([[NewReceiveCity, []]])
    total_time = time_fly + Function.max_release_date(ChangesPackages)
    
    if max_point != -1:  # Add trip mới vào thứ tự giao hàng của drone
        for a in range(start_check_point_in_drone_queue, len(solution[1])):
            for b in range(len(solution[1][a])):
                if solution[1][a][b][0] == max_point:
                    number_in_the_drone_queue_of_drop_package = a
                    check_max = True
                    break
            if  total_time < Function.max_release_date_update(solution[1][a]) + Function.cal_time_fly_a_trip(solution[1][a]):
                number_in_the_drone_queue_of_drop_package = a
                check_max = True
                break
            if check_max: break
    else:
        for a in range(start_check_point_in_drone_queue, len(solution[1])):
            if total_time < Function.max_release_date_update(solution[1][a]) + Function.cal_time_fly_a_trip(solution[1][a]):
                number_in_the_drone_queue_of_drop_package = a
                check_max = True
                break
        if number_in_the_drone_queue_of_drop_package == -1:
            number_in_the_drone_queue_of_drop_package = len(solution[1])
    package = []
    for i in range(len(ChangesPackages)):
        package.append(ChangesPackages[i])
    solution[1].insert(number_in_the_drone_queue_of_drop_package,
                            [[NewReceiveCity, package]])
    index_in_trip = 0
    return solution, number_in_the_drone_queue_of_drop_package, index_in_trip

# Phục vụ cho nei Neighborghood_change_drone_route, đọc lúc sau
def groupTripInDroneRoute(solution, ChangesPackages, ChangeInTruck, IndexOfNewReceiveCity):
    ReceiveCity = solution[0][ChangeInTruck][IndexOfNewReceiveCity][0]
    count = len(solution[0][ChangeInTruck][IndexOfNewReceiveCity][1]) - len(ChangesPackages)
    index_drone_trip = -1
    index_in_trip = -1
    #print("Change: ",ChangesPackages)
    for i in range(len(solution[1])):
        for j in range(len(solution[1][i])):
            if solution[1][i][j][0] == ReceiveCity:
                if Function.total_demand(solution[1][i]) + Function.sum_weight(ChangesPackages) <= Data.drone_capacity:
                    index_drone_trip = i
                    index_in_trip = j
                    for k in range(len(ChangesPackages)):
                        solution[1][i][j][1] = solution[1][i][j][1] + [ChangesPackages[k]]
                    return solution, index_drone_trip, index_in_trip, True
                count = count - len(solution[1][i][j][1])
            if count == 0:
                break
        if count == 0:
            break
    # Change Method: Turn Group into addNewTrip
    solution, index_drone_trip, index_in_trip = addNewTripInDroneRoute(solution, ChangesPackages, ChangeInTruck, IndexOfNewReceiveCity)
    return solution, index_drone_trip, index_in_trip, False

def findIndexOfDropPackage(new_solution, solution, city_change, truck_time, i, j, k, l, drop_package):
    demand_drop_package = Data.city_demand[drop_package]
    # Tính toán demand của phần hàng bị rớt
    list_check = []
    index = -1          # thứ tự trên truck mới mà drop package sẽ được đưa tới
    check = False
    stop = False
    stop1 = False
    max_index = - 1     # thành phố cuối cùng trên truck đang xét mà drop package có thể tới để feasible lời giải
    if demand_drop_package == 0:
        new_solution[0][i][0][1] += [drop_package] 
        stop = True
    elif Function.max_release_date(new_solution[0][i][0][1]) * DIFFERENTIAL_RATE_RELEASE_TIME + Data.standard_deviation > \
            Data.release_date[drop_package] and Function.min_release_date(new_solution[0][i][0][1]) * \
            DIFFERENTIAL_RATE_RELEASE_TIME + C_ratio * Data.standard_deviation > Data.release_date[drop_package]:
        new_solution[0][i][0][1] += [drop_package] 
        stop = True
    else:
        for m in range(len(new_solution[0][i])):
            a = new_solution[0][i][m][0]
            if check == False:
                if new_solution[0][i][m][1] != []:
                    list_check.append(a)
                if a == drop_package:
                    check = True
            else:
                if new_solution[0][i][m][1] != []:
                    max_index = a
                    break
        # Duyệt có điểm nhận hàng nào gộp đợc với drop_package không
        # Duyệt trên drone route - new_solution[1]
        for m in range(len(new_solution[1])): 
            for n in range(len(new_solution[1][m])):
                a = new_solution[1][m][n][0]
                if a == max_index:  # nếu tới max_index thì dừng xét và và chuyển sang tạo trip cho drone
                    stop1 = True
                    break
                if a in list_check:
                    # Kiểm tra điều kiện để có thể gộp
                    if Function.total_demand(new_solution[1][m]) + demand_drop_package <= Data.drone_capacity \
                            and Function.max_release_date_update(new_solution[1][m]) *  DIFFERENTIAL_RATE_RELEASE_TIME \
                                + Data.standard_deviation > Function.max_release_date([drop_package]) and Function.min_release_date_update(new_solution[1][m]) *  DIFFERENTIAL_RATE_RELEASE_TIME \
                                + B_ratio * Data.standard_deviation > Function.max_release_date([drop_package])  :
                        stop1 = True
                        stop = True
                        number_of_add_city_in_truck = -1
                        for c in range(len(new_solution[0][i])):
                            if new_solution[0][i][c][0] == a:
                                number_of_add_city_in_truck = c
                                break
                        new_solution[0][i][number_of_add_city_in_truck][
                            1] += [drop_package]
                        new_solution[1][m][n][1] += [drop_package]
                        break

            if stop1: break
    # Nếu không tìm được điểm để gộp hàng thì sẽ tạo trip mới cho drop package
    # Tìm điểm nhận hàng mới cho drop_package
    if not stop:
        # Tính toán criterion cho từng điểm để chọn điểm phù hợp nhất tạo trip mới
        # Criterion[i] = |release_date + time_drone_fly – truck_time[i] | 
        if i == k:      # Trường hợp khi city change vẫn nằm tại truck cũ
            if j < l:   # Trường hợp city change được chuyển ra sau
                where_city_come_after_by_truck_route = l
                minus = truck_time[i][j + 1] - truck_time[i][j] - \
                        Data.manhattan_move_matrix[
                            solution[0][i][j - 1][0]][solution[0][i][j + 1][0]] + \
                        Data.manhattan_move_matrix[city_change][solution[0][i][j + 1][0]]
                add1 = Data.manhattan_move_matrix[solution[0][i][l][0]][
                    city_change]
                add2 = 0
                if l != len(new_solution[0][i]) - 1:
                    add2 = add1 + Data.manhattan_move_matrix[city_change][
                        solution[0][i][l + 1][0]]
                minn = 100000000000
                criterion = 0
                for a in range(j - 1, len(new_solution[0][i])):
                    point_evaluate = new_solution[0][i][a][0]
                    if a > where_city_come_after_by_truck_route:
                        if Data.euclid_flight_matrix[0][
                            point_evaluate] * 2 + Data.unloading_time <= Data.drone_limit_time:
                            criterion = abs(
                                Function.max_release_date([drop_package]) +
                                Data.euclid_flight_matrix[0][point_evaluate] - (
                                        truck_time[i][a] - minus + add2))
                            if criterion < minn:
                                index = a
                                minn = criterion
                    elif a == where_city_come_after_by_truck_route:
                        if Data.euclid_flight_matrix[0][
                            point_evaluate] * 2 + Data.unloading_time <= Data.drone_limit_time:
                            criterion = abs(
                                Function.max_release_date([drop_package]) +
                                Data.euclid_flight_matrix[0][point_evaluate] - (
                                        truck_time[i][a] - minus + add1))
                            if criterion < minn:
                                index = a
                                minn = criterion
                    elif a < j:
                        if Data.euclid_flight_matrix[0][
                            point_evaluate] * 2 + Data.unloading_time <= Data.drone_limit_time:
                            criterion = abs(
                                Function.max_release_date([drop_package]) +
                                Data.euclid_flight_matrix[0][
                                    point_evaluate] - truck_time[i][a])
                            if criterion < minn:
                                index = a
                                minn = criterion
                    else:
                        if Data.euclid_flight_matrix[0][
                            point_evaluate] * 2 + Data.unloading_time <= Data.drone_limit_time:
                            criterion = abs(
                                Function.max_release_date([drop_package]) + \
                                Data.euclid_flight_matrix[0][
                                    point_evaluate] - (
                                        truck_time[i][a + 1] - minus))
                            if criterion < minn:
                                index = a
                                minn = criterion
                    if point_evaluate == drop_package: break
            else:   # Trường hợp city change được chuyển ra trước
                where_city_come_after_by_truck_route = l + 1
                minus = truck_time[i][j + 1] - truck_time[i][j] - \
                        Data.manhattan_move_matrix[
                            solution[0][i][j - 1][0]][solution[0][i][j + 1][0]] + \
                        Data.manhattan_move_matrix[solution[0][i][j][0]][
                            solution[0][i][j + 1][0]]
                add1 = Data.manhattan_move_matrix[solution[0][i][l][0]][
                    city_change]
                add2 = 0
                if l != len(new_solution[0][i]) - 1:
                    add2 = add1 + Data.manhattan_move_matrix[city_change][
                        solution[0][i][l + 1][0]]
                minn = 1000000000
                for a in range(j - 1, len(new_solution[0][i])):
                    point_evaluate = new_solution[0][i][a][0]
                    if a > where_city_come_after_by_truck_route:
                        if Data.euclid_flight_matrix[0][
                            point_evaluate] * 2 + Data.unloading_time <= Data.drone_limit_time:
                            criterion = abs(
                                Function.max_release_date([drop_package]) +
                                Data.euclid_flight_matrix[0][point_evaluate] - (
                                        truck_time[i][a] - minus + add2))
                            if criterion < minn:
                                index = a
                                minn = criterion
                    elif a == where_city_come_after_by_truck_route:
                        if Data.euclid_flight_matrix[0][
                            point_evaluate] * 2 + Data.unloading_time <= Data.drone_limit_time:
                            criterion = abs(
                                Function.max_release_date([drop_package]) +
                                Data.euclid_flight_matrix[0][point_evaluate] - (
                                        truck_time[i][a - 1] - minus + add1))
                            if criterion < minn:
                                index = a
                                minn = criterion
                    elif a <= l:
                        if Data.euclid_flight_matrix[0][
                            point_evaluate] * 2 + Data.unloading_time <= Data.drone_limit_time:
                            criterion = abs(
                                Function.max_release_date([drop_package]) +
                                Data.euclid_flight_matrix[0][
                                    point_evaluate] - truck_time[i][a])
                            if criterion < minn:
                                index = a
                                minn = criterion
                    else:
                        if Data.euclid_flight_matrix[0][
                            point_evaluate] * 2 + Data.unloading_time <= Data.drone_limit_time:
                            criterion = Function.max_release_date(
                                [drop_package]) + \
                                        Data.euclid_flight_matrix[0][point_evaluate] - (
                                                truck_time[i][a - 1] - minus)
                            if criterion < minn:
                                index = a
                                minn = criterion
                    if point_evaluate == drop_package: break
        else:   # Trường hợp city change được chuyển tơi truck mới
            minus = truck_time[i][j + 1] - truck_time[i][j] - Data.manhattan_move_matrix[
                solution[0][i][j - 1][0]][solution[0][i][j + 1][0]] + \
                    Data.manhattan_move_matrix[city_change][solution[0][i][j + 1][0]]
            minn = 1000000
            for a in range(j - 1, len(new_solution[0][i])):
                point_evaluate = new_solution[0][i][a][0]
                if a < j:
                    if Data.euclid_flight_matrix[0][
                        point_evaluate] * 2 + Data.unloading_time <= Data.drone_limit_time:
                        criterion = abs(
                            Function.max_release_date([drop_package]) +
                            Data.euclid_flight_matrix[0][
                                point_evaluate] - truck_time[i][a])
                        if criterion < minn:
                            index = a
                            minn = criterion
                else:
                    if Data.euclid_flight_matrix[0][
                        point_evaluate] * 2 + Data.unloading_time <= Data.drone_limit_time:
                        criterion = Function.max_release_date([drop_package]) + \
                                    Data.euclid_flight_matrix[0][
                                        point_evaluate] - (
                                            truck_time[i][a + 1] - minus)
                        if criterion < minn:
                            index = a
                            minn = criterion
                if point_evaluate == drop_package: break
        if index != -1: # Nếu tìm được index
            new_solution[0][i][index][1] += [drop_package]
            new_solution, index_drone_trip, index_in_trip = addNewTripInDroneRoute(new_solution, [drop_package], i, index)
        else:     
            new_solution[0][i][0][1] += [drop_package]
    return new_solution

def swap_two_array(solution):
    solution_pack = []
    neighborhood = []
    for aa in range(len(solution[0])):
        length = len(solution[0][aa]) - 1
        middle = int(length/2)
        # print("middle:", middle)
        ranged = []
        for i in range(2, middle + 1):
            ranged.append(i)
        # print(length, middle, ranged)
        if len(ranged) == 0:
            continue
        ran = ranged[int(random.random()*len(ranged))]
        # print(ran)
        # print(ran)
        # ran = 3
        # print(ran)
        for i in range(1, middle+1):
            for j in range(i + ran - 1, middle+1):
                for k in range(middle+1, length+1):
                    for l in range(k + ran - 1, length+1):
                        new_solution = copy.deepcopy(solution)
                        # print("++++++++++new")
                        # print(new_solution)
                        # print(solution[0][aa][i][0],"  ",solution[0][aa][j][0], "  ",solution[0][aa][k][0], "  ",solution[0][aa][l][0])
                        # print("----")
                        pre_drop_package = []
                        drop_package = []
                        # print(i,", ",j,", ",k, ",", l)
                        for x in range(i, j+1):
                            pre_drop_package.append(solution[0][aa][x][0])
                        for x in range(k, l+1):
                            pre_drop_package.append(solution[0][aa][x][0])
                        # print(drop_package)
                        for x in range(i, j + 1):
                            for y in range(len(solution[0][aa][x][1])):
                                pack = solution[0][aa][x][1][y]
                                if pack not in pre_drop_package:
                                    pre_drop_package.append(pack)
                        for x in range(k, l + 1):
                            for y in range(len(solution[0][aa][x][1])):
                                pack = solution[0][aa][x][1][y]
                                if pack not in pre_drop_package:
                                    pre_drop_package.append(pack)
                        # print(pre_drop_package)
                        for x in range(length+1):
                            for y in reversed(range(len(solution[0][aa][x][1]))):
                                pack = solution[0][aa][x][1][y]
                                if pack in pre_drop_package:
                                    new_solution[0][aa][x][1].remove(pack)
                        new_solution[0][aa] = new_solution[0][aa][:i] + new_solution[0][aa][k:l+1] + new_solution[0][aa][j+1:k] + new_solution[0][aa][i:j+1] + new_solution[0][aa][l+1:]
                        
                        for x in reversed(range(len(new_solution[1]))):
                            for y in reversed(range(len(new_solution[1][x]))):
                                for z in reversed(range(len(new_solution[1][x][y][1]))):
                                    pack = new_solution[1][x][y][1][z]
                                    if pack in pre_drop_package:
                                        new_solution[1][x][y][1].remove(pack)
                                    if z == 0:
                                        if new_solution[1][x][y][1] == []:
                                            new_solution[1][x].pop(y)
                                            if new_solution[1][x] == []:
                                                new_solution[1].pop(x)

                        for x in range(len(new_solution[0][aa])):
                            city = new_solution[0][aa][x][0]
                            if city in pre_drop_package:
                                drop_package.append(city)
                        # print(drop_package)
                        # print(new_solution)
                        for x in range(len(drop_package)):
                            # print("----------")
                            # print(aa)
                            # print(drop_package[x])
                            # print(new_solution)
                            new_solution = findLocationForDropPackage(new_solution, aa, drop_package[x])
                        # print(new_solution[0])
                        # print("hehe")
                        # print(new_solution)
                        # print(Function.Check_if_feasible(new_solution))
                        # print(new_solution[1])
                        # print("End: ",new_solution)
                        pack_child = []
                        pack_child.append(new_solution)
                        a, b, c = Function.fitness(new_solution)
                        pack_child.append([a, b, c])
                        pack_child.append(-1)
                        pack_child.append(aa)
                        pack_child.append(aa)
                        neighborhood.append(pack_child)
    return neighborhood, solution_pack

def findLocationForDropPackage(new_solution, index_truck, drop_package):
    list_check = []  # Tập hợp điểm nhận hàng trên truck
    check = False
    stop = False
    stop1 = False
    max_index = - 1  # Tìm điểm cuối cùng có thể nhận được new_package hợp lệ trên route_truck
    if Data.release_date[drop_package] == 0 or Data.city_demand[drop_package] > Data.drone_capacity:
        new_solution[0][index_truck][0][1] += [drop_package]
        stop = True
    elif Function.max_release_date(new_solution[0][index_truck][0][1]) * DIFFERENTIAL_RATE_RELEASE_TIME + Data.standard_deviation > \
            Data.release_date[drop_package] and Function.min_release_date(new_solution[0][index_truck][0][1]) * \
            DIFFERENTIAL_RATE_RELEASE_TIME + C_ratio * Data.standard_deviation > Data.release_date[drop_package]:
        new_solution[0][index_truck][0][1] += [drop_package] 
        stop = True
    else:
        for m in range(len(new_solution[0][index_truck])):
            # print(new_solution)
            # print(new_solution[0][index_truck])
            # print(m)
            a = new_solution[0][index_truck][m][0]
            if check == False:
                if new_solution[0][index_truck][m][1] != []:
                    list_check.append(a)
                if a == drop_package:
                    check = True
            else:
                if new_solution[0][index_truck][m][1] != []:
                    max_index = a
                    break
        for m in range(len(new_solution[1])):  # Duyệt có điểm nhận hàng nào gộp đợc với new_package không
            for n in range(len(new_solution[1][m])):
                a = new_solution[1][m][n][0]
                if a == max_index:
                    stop1 = True
                    break
                if a in list_check:
                    if Function.total_demand(new_solution[1][m]) + Data.city_demand[drop_package] <= Data.drone_capacity \
                            and Function.max_release_date_update(new_solution[1][m]) * DIFFERENTIAL_RATE_RELEASE_TIME + \
                                Data.standard_deviation > Data.release_date[drop_package] and Function.min_release_date_update(new_solution[1][m]) * DIFFERENTIAL_RATE_RELEASE_TIME + \
                                B_ratio * Data.standard_deviation > Data.release_date[drop_package]:
                        stop = True
                        stop1 = True
                        number_of_city_change_in_new_truck = -1
                        for c in range(len(new_solution[0][index_truck])):
                            if new_solution[0][index_truck][c][0] == a:
                                number_of_city_change_in_new_truck = c
                                break
                        new_solution[0][index_truck][number_of_city_change_in_new_truck][
                            1] += [drop_package]
                        new_solution[1][m][n][1] += [drop_package]
                        break
            if stop1: break
    if not stop:
        index = -1
        city_received = drop_package
        for i in reversed(range(len(new_solution[0][index_truck]))):
            index_of_city = new_solution[0][index_truck][i][0]
            if index_of_city == drop_package:
                index = i
                break
        while Data.euclid_flight_matrix[0][city_received] * 2 + Data.unloading_time > Data.drone_limit_time:
            index -= 1
            city_received = new_solution[0][index_truck][index][0]
            
        new_solution[0][index_truck][index][1] += [drop_package]
        if index != 0:
            # new_solution, successed = groupToATrip(new_solution, [drop_package], index_truck, index)
            new_solution, number_in_the_drone_queue_of_drop_package, index_in_trip = addNewTripInDroneRoute(new_solution, [drop_package], index_truck, index)
    return new_solution  

def find_shortest_path_by_greedy_1(start, end, list):
    new_list= []
    new_list.append(start)
    new_list.append(end)
    total = Data.manhattan_move_matrix[start][end]
    for i in range(len(list)):
        city = list[i]
        index = -1
        min_distance = 10000000
        for j in range(1, len(new_list)):
            city_before = new_list[j-1]
            city_after = new_list[j]
            criteria = Data.manhattan_move_matrix[city_before][city] + Data.manhattan_move_matrix[city][city_after] - Data.manhattan_move_matrix[city_before][city_after]
            if criteria < min_distance:
                min_distance = criteria
                index = j
        new_list.insert(index, city)
        total += criteria
    new_list.remove(end)
    new_list.remove(start)
    return [new_list, total]

def find_shortest_path_by_greedy_2(start, end, list):
    new_list= []
    uncovered_list = copy.deepcopy(list)
    current = start
    total = 0
    for i in range(len(uncovered_list)):
        next_city = -1
        min_distance = 100000
        for j in range(len(uncovered_list)):
            city = uncovered_list[j]
            distance = Data.manhattan_move_matrix[current][city]
            if distance < min_distance:
                next_city = city
                min_distance = distance
            if distance == min_distance:
                if Data.manhattan_move_matrix[0][current] < Data.manhattan_move_matrix[0][city]:
                    next_city = city
                    min_distance = distance
        current = next_city
        total += min_distance
        new_list.append(next_city)
        uncovered_list.remove(next_city)
    total += Data.manhattan_move_matrix[current][end]
    return [new_list, total]

def one_opt_and_change_truck_route_after(solution, truck_time):
    neighborhood = []
    for i in range(len(solution[0])):
        for j in range(1, len(solution[0][i])):
            for k in range(len(solution[0])):
                for l in range(len(solution[0][k]) - 3):
                    # First check
                    if i == k:
                        if j == l or j == l + 1:
                            continue
                    
                    new_solution = copy.deepcopy(solution)
                    
                    city_change = solution[0][i][j][0]
                    drop_package = copy.deepcopy(solution[0][i][j][1])
                    if city_change in drop_package: drop_package.remove(city_change)
                    # Xóa city_change ra khỏi truck route
                    new_solution[0][i].pop(j)
                    stop3 = False
                    # Xóa gói hàng của city_change ở điểm giao hàng trên truck route
                    for m in range(len(new_solution[0][i])):
                        if city_change in new_solution[0][i][m][1]:
                            new_solution[0][i][m][1].remove(city_change)
                                                                     
                    # Xóa chuyến hàng giao hàng mang city_change tại new_solution[1] - drone route
                    for m in range(len(new_solution[1])):  
                        for n in range(len(new_solution[1][m])):
                            if city_change in new_solution[1][m][n][1]:
                                new_solution[1][m][n][1].remove(city_change)
                                if new_solution[1][m][n][1] == []:
                                    new_solution[1][m].pop(n)
                                    if new_solution[1][m] == []:
                                        new_solution[1].pop(m)
                                stop3 = True
                                break
                        if stop3: break
                    # Chuyển city_change tới vị trí mới trên truck route - new_solution[0]
                    begin_index_change_route = -1
                    if i == k:
                        if j < l:
                            begin_index_change_route = l + 1
                        else:
                            begin_index_change_route = l + 2
                    else:
                        begin_index_change_route = l + 2
                    
                    new_solution[0][k].insert(begin_index_change_route - 1, [city_change, []])
                    
                    if drop_package != []:  # Nếu điểm bị xóa là điểm nhận hàng
                        # Xóa chuyến hàng giao hàng mang drop_package ; Xóa theo từng gói hàng trên drop package chứ
                        # không phải theo từng lần giao hàng có tại drop package
                        for m in reversed(range(len(new_solution[1]))): 
                            for n in reversed(range(len(new_solution[1][m]))):
                                if new_solution[1][m][n][0] == city_change:
                                    new_solution[1][m].pop(n)
                                    if new_solution[1][m] == []:
                                        new_solution[1].pop(m)
                                                                         
                        # Tìm nơi nhận hàng mới cho từng drop_package
                        # Xét từng góp hàng trong drop package
                        for mm in range(len(drop_package)):
                            new_solution = findIndexOfDropPackage(new_solution, solution, city_change, truck_time, i, j, k, l, drop_package[mm])
                    # New
                    # print("-----")    
                    # print(new_solution[0])
                    # print(new_solution[1])
                    next_drop_package = []
                    re_arranged_list = []
                    
                    for m in reversed(range(begin_index_change_route, len(new_solution[0][k]))):
                        city = new_solution[0][k][m][0]
                        re_arranged_list.append(city)
                        next_drop_package.append(city)
                        for n in range(len(new_solution[0][k][m][1])):
                            package = new_solution[0][k][m][1][n]
                            if package not in next_drop_package:
                                next_drop_package.append(package)
                        new_solution[0][k].pop(m)
                    
                    if city_change not in next_drop_package:
                        next_drop_package.append(city_change)
                    
                    for m in range(len(new_solution[0][k])):
                        for n in reversed(range(len(new_solution[0][k][m][1]))):
                            city = new_solution[0][k][m][1][n]
                            if city in re_arranged_list:
                                new_solution[0][k][m][1].pop(n)
                    
                    for m in reversed(range(len(new_solution[1]))):
                        for n in reversed(range(len(new_solution[1][m]))):
                            for p in reversed(range(len(new_solution[1][m][n][1]))):
                                package = new_solution[1][m][n][1][p]
                                if package in next_drop_package:
                                    new_solution[1][m][n][1].pop(p)
                            if new_solution[1][m][n][1] == []:
                                new_solution[1][m].pop(n)
                                if new_solution[1][m] == []:
                                    new_solution[1].pop(m)
                    
                    a, b = find_shortest_path_by_greedy_2(new_solution[0][k][begin_index_change_route-1][0], 0, re_arranged_list)
                    c, d = find_shortest_path_by_greedy_1(new_solution[0][k][begin_index_change_route-1][0], 0, re_arranged_list)
                    if b < d:
                        re_arranged_list = a
                    else:
                        re_arranged_list = c
                    # print("re: ", re_arranged_list)
                    pre_re_arranged_list = find_shortest_path_by_greedy_2(new_solution[0][k][begin_index_change_route-1][0], 0, re_arranged_list)
                    # print("re: ", re_arranged_list)
                    for m in range(len(new_solution[0][k])):
                        city = new_solution[0][k][m][0]
                        if city in pre_re_arranged_list:
                            re_arranged_list.append(city)
                
                    for m in range(len(re_arranged_list)):
                        new_solution[0][k].insert(len(new_solution[0][k]), [re_arranged_list[m], []])
                    
                    for m in reversed(range(len(next_drop_package))):
                        new_solution = findLocationForDropPackage(new_solution, k, next_drop_package[m])
                    # print("--------------------------")    
                    # print(new_solution[0])
                    # print(new_solution[1])
                    # print(Function.Check_if_feasible(new_solution))
                    
                    pack_child = []
                    pack_child.append(new_solution)
                    a, b, c = Function.fitness(new_solution)
                    pack_child.append([a, b, c])
                    pack_child.append(city_change)
                    pack_child.append(i)
                    pack_child.append(k)
                    neighborhood.append(pack_child)
                    # print(len(neighborhood))
    return neighborhood
 
def minimal_change(solution):
    rs = []
    for i in range(len(solution[0])):
        rs.append([])
        for j in range(1, len(solution[0][i])):
            if len(solution[0][i][j][1]) >=2:
                rs[i].append([j, solution[0][i][j][1]])
                
    neighbors = []
    for i in range(len(rs)):
        for j in range(len(rs[i])):
            for u in range(len(rs[i][j][1])):
                #for v in range(u+1, len(rs[i][j][1])):
                city_change1 = solution[0][i][rs[i][j][0]][0]
                city_change2 = rs[i][j][1][u]
                if city_change1 != city_change2: 
                    #print(city_change1, city_change2)

                    pos_1 = rs[i][j][0]
                    pos_2 = Function.find_position(city_change2, solution[0])[1]

                    newsolution = copy.deepcopy(solution)
                    
                    package_needchange = []

                    if newsolution[0][i][pos_2][1] != []:
                        package_needchange = package_needchange + newsolution[0][i][pos_2][1]

                    newsolution[0][i][pos_1][0] = city_change2
                    newsolution[0][i][pos_2][0] = city_change1

                    newsolution[0][i][pos_2][1].clear()
                    for k in range(len(package_needchange)):
                        newsolution[0][i] = Neighborhood11.rearrange_package(package_needchange[k], newsolution[0][i], Function.find_position(package_needchange[k], newsolution[0])[1])
                        #newsolution = Neighborhood.findLocationForDropPackage(newsolution, i, package_needchange[k])

                    newsolution = Neighborhood11.fix_drone_queue(newsolution)

                    pack_child = []
                    pack_child.append(newsolution)
                    a, b, c = Function.fitness(newsolution)
                    pack_child.append([a, b, c])
                    neighbors.append(pack_child)      
    return neighbors

def Neighborhood_combine_truck_and_drone_neighborhood(name_of_truck_neiborhood, solution, number_of_potiential_solution, number_of_loop_drone, whether_use_truck_time):
    potential_solution = []
    if whether_use_truck_time:
        current_neighborhood = name_of_truck_neiborhood(solution, Function.fitness(solution)[1])
    else:
        current_neighborhood = name_of_truck_neiborhood(solution)
        
    for i in range(len(current_neighborhood)):
        num = len(potential_solution)
        while num != 0:
            if current_neighborhood[i][1][0] < potential_solution[num-1][1][0]:
                num -= 1
            else:
                break
        potential_solution.insert(num, current_neighborhood[i])
        if len(potential_solution) > number_of_potiential_solution:
            potential_solution.pop()
        
    for i in range(len(potential_solution)):
        j = 0
        list_accept_truck = [potential_solution[i][3], potential_solution[i][4]]
        sol = copy.deepcopy(potential_solution[i])
        min_to_improve = potential_solution[i][1][0]
        # list_neighborhood = [Neighborhood_drone.Neighborghood_change_drone_route_max_pro_plus_for_specific_truck, Neighborhood_drone.Neighborhood_group_trip]
        list_neighborhood = [Neighborhood_drone.Neighborghood_change_drone_route_max_pro_plus_for_specific_truck, Neighborhood_drone.Neighborhood_change_index_trip]

        while j < number_of_loop_drone:
            j += 1
            for k in range(len(list_neighborhood)):
                if k == 0:
                    drone_neighborhood = list_neighborhood[k](sol[0], list_accept_truck)
                else:
                    drone_neighborhood = list_neighborhood[k](sol[0])
                min_in_loop = 100000000
                next_index = 0
                if len(drone_neighborhood) == 0:
                    continue
                for l in range(len(drone_neighborhood)):
                    cfnode = drone_neighborhood[l][1][0]
                    if cfnode - min_to_improve < epsilon:
                        potential_solution[i] = drone_neighborhood[l]
                        next_index = l
                        min_in_loop = cfnode
                        min_to_improve = cfnode
                        j = 0
                    elif cfnode - min_in_loop < epsilon:
                        next_index = l
                        min_in_loop = cfnode
                sol = drone_neighborhood[next_index]
        
    return potential_solution

def Neighborhood_combine_truck_and_drone_neighborhood_with_tabu_list(name_of_truck_neiborhood, solution, number_of_potial_solution, number_of_loop_drone, tabu_list, tabu_tenure, index_of_loop, best_fitness, kind_of_tabu_structure, need_truck_time):
    potential_solution = []
    if need_truck_time:
        current_neighborhood = name_of_truck_neiborhood(solution, Function.fitness(solution)[1])
    else:
        current_neighborhood = name_of_truck_neiborhood(solution)
    for i in range(len(current_neighborhood)):
        num = len(potential_solution)
        cfsol = current_neighborhood[i][1][0]
        continue2564 = False
        if kind_of_tabu_structure in [1, 2]:
            # print(index_of_loop - tabu_list[current_neighborhood[0][2]] > tabu_tenure)
            # print(current_neighborhood[0][2], "->", tabu_list[current_neighborhood[0][2]])
            if cfsol - best_fitness < epsilon or index_of_loop - tabu_list[current_neighborhood[i][2]] > tabu_tenure: 
                continue2564 = True
        elif kind_of_tabu_structure in [3, 5]:
            if cfsol - best_fitness < epsilon or (index_of_loop - tabu_list[current_neighborhood[i][2][0]] > tabu_tenure or index_of_loop - tabu_list[current_neighborhood[i][2][1]] > tabu_tenure): 
                continue2564 = True
        elif kind_of_tabu_structure in [4]:
            if cfsol - best_fitness < epsilon or (index_of_loop - tabu_list[current_neighborhood[i][2][0]] > tabu_tenure or index_of_loop - tabu_list[current_neighborhood[i][2][1]] > tabu_tenure or index_of_loop - tabu_list[current_neighborhood[i][2][2]] > tabu_tenure): 
                continue2564 = True
        if continue2564:
            while num != 0:
                if cfsol < potential_solution[num-1][1][0]:
                    num -= 1
                else:
                    break
            potential_solution.insert(num, current_neighborhood[i])
            if len(potential_solution) > number_of_potial_solution:
                potential_solution.pop()
        
    restrict_next_loop = []
    for i in range(len(potential_solution)):
        restrict_next_loop.append(potential_solution[i][2])
    for i in range(len(potential_solution)):
        j = 0
        list_accept_truck = [potential_solution[i][3], potential_solution[i][4]]
        sol = copy.deepcopy(potential_solution[i])
        min_to_improve = potential_solution[i][1][0]
        # list_neighborhood = [Neighborhood_drone.Neighborghood_change_drone_route_max_pro_plus_for_specific_truck, Neighborhood_drone.Neighborhood_group_trip]
        list_neighborhood = [Neighborhood_drone.Neighborghood_change_drone_route_max_pro_plus_for_specific_truck, Neighborhood_drone.Neighborhood_change_index_trip, Neighborhood_drone.Neighborhood_group_trip]       
        while j < number_of_loop_drone:
            j += 1
            # print("i: ",i," j: ", j)
            # print(sol[0])
            for k in range(len(list_neighborhood)):
                if k == 0:
                    drone_neighborhood = list_neighborhood[k](sol[0], list_accept_truck)
                else:
                    drone_neighborhood = list_neighborhood[k](sol[0])
                min_in_loop = 1000000
                next_index = 0
                if len(drone_neighborhood) == 0:
                    continue
                for l in range(len(drone_neighborhood)):
                    cfnode = drone_neighborhood[l][1][0]
                    if cfnode - min_to_improve < epsilon:
                        potential_solution[i] = drone_neighborhood[l]
                        next_index = l
                        min_in_loop = cfnode
                        min_to_improve = cfnode
                        j = 0
                    elif cfnode - min_in_loop < epsilon:
                        next_index = l
                        min_in_loop = cfnode
                sol = drone_neighborhood[next_index]
    for i in range(len(potential_solution)):
        potential_solution[i][2] = restrict_next_loop[i]
    # print(restrict_next_loop)
    return potential_solution

def Optimize_initial_solution_in_drone(solution):
    i = 0
    best_fit = Function.fitness(solution)[0]
    best_sol = solution
    cur_sol = solution
    while i < 2:
        i += 1
        neighborhood = Neighborhood_drone.Neighborghood_change_drone_route_max_pro_plus(cur_sol)
        index = -1
        min_in = 100000
        for i in range(len(neighborhood)):
            cfsol = neighborhood[i][1][0]
            if cfsol - best_fit < epsilon:
                i = 0
                index = i
                min_in = cfsol
                best_fit = cfsol
                best_sol = neighborhood[i][0]
            elif cfsol - min_in < epsilon:
                index = i
                min_in = cfsol
        cur_sol = neighborhood[index][0]
    return best_sol

def Reverse_truck_route(solution, index_truck):
    Function.update_per_loop(solution)
    new_solution = copy.deepcopy(solution)
    package = []
    for i in range(len(new_solution[0][index_truck])):
        package.append(new_solution[0][index_truck][i][0])
        new_solution[0][index_truck][i][1] = []
    package.pop(0)
    for i in reversed(range(len(new_solution[1]))):
        for j in reversed(range(len(new_solution[1][i]))):
            if Function.index_truck_of_cities[new_solution[1][i][j][0]] == index_truck:
                new_solution[1][i].pop(j)
        if new_solution[1][i] == []:
            new_solution[1].pop(i)
    new_solution[0][index_truck] = new_solution[0][index_truck][0:1] + new_solution[0][index_truck][1:][::-1]
    for i in reversed(range(len(package))):
        new_solution = findLocationForDropPackage(new_solution, index_truck, package[i])
    return new_solution

def find_if_truck_route_need_reverse(solution):
    current_solution = solution
    current_fitness = Function.fitness(solution)[0]
    if_improved = False
    for i in range(Data.number_of_trucks):
        new_solution = copy.deepcopy(current_solution)
        new_solution = Reverse_truck_route(new_solution, i)
        new_solution, new_fitness = sub_tabu_search(new_solution, 6, i)
        if new_fitness < current_fitness:
            current_solution = new_solution
            current_fitness = new_fitness
            if_improved =True
    return current_solution, current_fitness, if_improved

def sub_tabu_search(solution, loop, index_truck):
    # if_improved = False
    sub_tabu_tenure1 = 5
    sub_tabu_tenure2 = 5
    sub_tabu_structure1 = [-5] * Data.number_of_cities
    sub_tabu_structure2 = []
    for i in range(Data.number_of_cities):
        add = [-5] * Data.number_of_cities
        sub_tabu_structure2.append(add)
    current_sol = solution
    current_fitness = Function.fitness(current_sol)[0]
    best_sol = current_sol
    best_fitness = current_fitness
    for i in range(loop):
        neighborhood  = []
        if i % 3 < 2:
            neighborhood1 = Neighborhood10.Neighborhood_one_otp_fix_for_specific_truck(current_sol, index_truck)
            neighborhood2 = Neighborhood10.Neighborhood_move_1_1_ver2_for_specific_truck(current_sol, index_truck)
            neighborhood.append([1, neighborhood1])
            neighborhood.append([2, neighborhood2])
        else:
            neighborhood3 = Neighborhood_drone.Neighborghood_change_drone_route_max_pro_plus_for_specific_truck(current_sol, [index_truck])
            # neighborhood4 = Neighborhood_drone.Neighborhood_group_trip(current_sol)
            neighborhood.append([3, neighborhood3])
            # neighborhood.append([4, neighborhood4])
        
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

                    elif cfnode - min_nei[j] < epsilon and sub_tabu_structure1[neighborhood[j][1][k][2]] + sub_tabu_tenure1 <= i:
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

                    elif cfnode - min_nei[j] < epsilon and sub_tabu_structure2[neighborhood[j][1][k][2][0]][neighborhood[j][1][k][2][1]] + sub_tabu_tenure2 <= i:
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
        
        if neighborhood[index_best_nei][0] == 1:
            sub_tabu_structure1[neighborhood[index_best_nei][1][index[index_best_nei]][2]] = i
        
        if neighborhood[index_best_nei][0] == 2:
            sub_tabu_structure2[neighborhood[index_best_nei][1][index[index_best_nei]][2][0]][neighborhood[index_best_nei][1][index[index_best_nei]][2][1]] = i
            sub_tabu_structure2[neighborhood[index_best_nei][1][index[index_best_nei]][2][1]][neighborhood[index_best_nei][1][index[index_best_nei]][2][0]] = i
        
    return best_sol, best_fitness

def sub_local_search_optimize(solution):
    nei = []
    current_sol = solution
    current_fitness, current_truck_time, current_sum = Function.fitness(current_sol)
    best_sol = current_sol
    best_fitness = current_fitness
    loop_improved = 0
    i = 0
    while i - loop_improved > 20:
        neighborhood = one_opt_and_change_truck_route_after(solution, current_truck_time)
        index = -1
        min_in = 1000000
        for i in range(len(neighborhood)):
            cfsol = neighborhood[i][1][0]
            if cfsol - best_fitness < epsilon:
                best_sol = neighborhood[i][0]
                best_fitness = cfsol
                index = i
                min_in = cfsol
                loop_improved = i
            elif cfsol - min_in < epsilon:
                index = i
                min_in = cfsol
        current_sol = neighborhood[index][0]
        current_fitness, current_truck_time = neighborhood[index][1]
    nei = []
    temp = []
    temp.append(best_sol)
    temp.append(Function.fitness(best_sol))
    temp.append(-1)
    temp.append(-1)
    nei.append(temp)
    return nei

def Neighborhood_combine_truck_and_drone_neighborhood_with_package(name_of_truck_neiborhood, solution, number_of_potiential_solution, number_of_loop_drone, whether_use_truck_time, solution_pack, solution_pack_len, use_solution_pack, index_consider_elite_set):
    potential_solution = []
    if whether_use_truck_time:
        current_neighborhood = name_of_truck_neiborhood(solution, Function.fitness(solution)[1])
    else:
        current_neighborhood = name_of_truck_neiborhood(solution)
        
    for i in range(len(current_neighborhood)):
        num = len(potential_solution)
        while num != 0:
            if current_neighborhood[i][1][0] < potential_solution[num-1][1][0]:
                num -= 1
            else:
                break
        potential_solution.insert(num, current_neighborhood[i])
        consider = False
        if len(potential_solution) > number_of_potiential_solution:
            consider = True
            potential_solution.pop()
        
        # if use_solution_pack:
        #     if consider:
        #         have_same_shape = False
        #         for jjj in range(len(solution_pack)):
        #             if Function.Compare_two_solution_2(current_neighborhood[i][0], solution_pack[jjj][0]):
        #                 have_same_shape = True
        #                 if current_neighborhood[i][1][0] < solution_pack[jjj][1][0]:
        #                     add_solution = copy.deepcopy(current_neighborhood[i])
        #                     solution_pack[jjj] = add_solution
        #                 break
        #         if not have_same_shape:
        #             num1 = len(solution_pack)
        #             while num1 != 0:
        #                 if current_neighborhood[i][1][0] < solution_pack[num1-1][1][0]:
        #                     num1 -= 1
        #                 else:
        #                     break
        #             add_solution = copy.deepcopy(current_neighborhood[i])
        #             solution_pack.insert(num1, add_solution)
        #             if len(solution_pack) > solution_pack_len:
        #                 solution_pack.pop()
    if len(solution_pack) != 0:
        min_solution_pack = solution_pack[-1][1][0]
    else:
        min_solution_pack = 10000000000
    if len(potential_solution) != 0:                      
        if index_consider_elite_set < solution_pack_len:
            if potential_solution[0][1][0] - min_solution_pack < epsilon:
                have_same_shape = False
                for jjj in range(len(solution_pack)):
                    if jjj + 1 > index_consider_elite_set:
                        if Function.Compare_two_solution_2(potential_solution[0][0], solution_pack[jjj][0]):
                            have_same_shape = True
                            if potential_solution[0][1][0] < solution_pack[jjj][1][0]:
                                add_solution = copy.deepcopy(potential_solution[0])
                                solution_pack[jjj] = add_solution
                            break
                if not have_same_shape:
                    num1 = len(solution_pack)
                    while num1 > index_consider_elite_set:
                        if potential_solution[0][1][0] < solution_pack[num1-1][1][0]:
                            num1 -= 1
                        else:
                            break
                    add_solution = copy.deepcopy(potential_solution[0])
                    solution_pack.insert(num1, add_solution)
                    if len(solution_pack) > solution_pack_len:
                        solution_pack.pop()
            
    for i in range(len(potential_solution)):
        j = 0
        list_accept_truck = [potential_solution[i][3], potential_solution[i][4]]
        sol = copy.deepcopy(potential_solution[i])
        min_to_improve = potential_solution[i][1][0]
        # list_neighborhood = [Neighborhood_drone.Neighborghood_change_drone_route_max_pro_plus_for_specific_truck, Neighborhood_drone.Neighborhood_group_trip]
        list_neighborhood = [Neighborhood_drone.Neighborghood_change_drone_route_max_pro_plus_for_specific_truck, Neighborhood_drone.Neighborhood_change_index_trip]

        while j < number_of_loop_drone:
            j += 1
            for k in range(len(list_neighborhood)):
                if k == 0:
                    drone_neighborhood = list_neighborhood[k](sol[0], list_accept_truck)
                else:
                    drone_neighborhood = list_neighborhood[k](sol[0])
                min_in_loop = 100000000
                next_index = 0
                if len(drone_neighborhood) == 0:
                    continue
                for l in range(len(drone_neighborhood)):
                    cfnode = drone_neighborhood[l][1][0]
                    if cfnode - min_to_improve < epsilon:
                        potential_solution[i] = drone_neighborhood[l]
                        next_index = l
                        min_in_loop = cfnode
                        min_to_improve = cfnode
                        j = 0
                    elif cfnode - min_in_loop < epsilon:
                        next_index = l
                        min_in_loop = cfnode
                sol = drone_neighborhood[next_index]
        
    return potential_solution, solution_pack

def Neighborhood_combine_truck_and_drone_neighborhood_with_tabu_list_with_package(name_of_truck_neiborhood, solution, number_of_potial_solution, number_of_loop_drone, tabu_list, tabu_tenure, index_of_loop, best_fitness, kind_of_tabu_structure, need_truck_time, solution_pack, solution_pack_len, use_solution_pack, index_consider_elite_set):
    potential_solution = []
    if need_truck_time:
        current_neighborhood = name_of_truck_neiborhood(solution, Function.fitness(solution)[1])
    else:
        current_neighborhood = name_of_truck_neiborhood(solution)
    # chosen_fitness = current_neighborhood[i][1][0]
    # chosen_sol = current_neighborhood[i][0]
    for i in range(len(current_neighborhood)):
        num = len(potential_solution)
        num1 = len(solution_pack)
        cfsol = current_neighborhood[i][1][0]
        continue2564 = False
        if kind_of_tabu_structure in [1, 2]:
            # print(index_of_loop - tabu_list[current_neighborhood[0][2]] > tabu_tenure)
            # print(current_neighborhood[0][2], "->", tabu_list[current_neighborhood[0][2]])
            if cfsol - best_fitness < epsilon or index_of_loop - tabu_list[current_neighborhood[i][2]] > tabu_tenure: 
                continue2564 = True
        elif kind_of_tabu_structure in [3, 5]:
            if cfsol - best_fitness < epsilon or (index_of_loop - tabu_list[current_neighborhood[i][2][0]] > tabu_tenure or index_of_loop - tabu_list[current_neighborhood[i][2][1]] > tabu_tenure): 
                continue2564 = True
        elif kind_of_tabu_structure in [4]:
            if cfsol - best_fitness < epsilon or (index_of_loop - tabu_list[current_neighborhood[i][2][0]] > tabu_tenure or index_of_loop - tabu_list[current_neighborhood[i][2][1]] > tabu_tenure or index_of_loop - tabu_list[current_neighborhood[i][2][2]] > tabu_tenure): 
                continue2564 = True
        consider = False
        if continue2564:
            while num != 0:
                if cfsol < potential_solution[num-1][1][0]:
                    num -= 1
                else:
                    break
            potential_solution.insert(num, current_neighborhood[i])
            if len(potential_solution) > number_of_potial_solution:
                consider = True
                potential_solution.pop()
                
        # if use_solution_pack:    
        #     if consider:
        #         have_same_shape = False
        #         for jjj in range(len(solution_pack)):
        #             if Function.Compare_two_solution_2(current_neighborhood[i][0], solution_pack[jjj][0]):
        #                 have_same_shape = True
        #                 if current_neighborhood[i][1][0] < solution_pack[jjj][1][0]:
        #                     add_solution = copy.deepcopy(current_neighborhood[i])
        #                     solution_pack[jjj] = add_solution
        #                 break
        #         if not have_same_shape:
        #             num1 = len(solution_pack)
        #             while num1 != 0:
        #                 if current_neighborhood[i][1][0] < solution_pack[num1-1][1][0]:
        #                     num1 -= 1
        #                 else:
        #                     break
        #             add_solution = copy.deepcopy(current_neighborhood[i])
        #             solution_pack.insert(num1, add_solution)
        #             if len(solution_pack) > solution_pack_len:
        #                 solution_pack.pop()
    if len(solution_pack) != 0:
        min_solution_pack = solution_pack[-1][1][0]
    else:
        min_solution_pack = 10000000000
    # if len(potential_solution) != 0:
    #     if index_consider_elite_set < solution_pack_len:
    #         if potential_solution[0][1][0] - min_solution_pack < epsilon:
    #             have_same_shape = False
    #             for jjj in range(len(solution_pack)):
    #                 if jjj + 1 > index_consider_elite_set:
    #                     if Function.Compare_two_solution_2(potential_solution[0][0], solution_pack[jjj][0]):
    #                         have_same_shape = True
    #                         if potential_solution[0][1][0] < solution_pack[jjj][1][0]:
    #                             add_solution = copy.deepcopy(potential_solution[0])
    #                             solution_pack[jjj] = add_solution
    #                         break
    #             if not have_same_shape:
    #                 num1 = len(solution_pack)
    #                 while num1 > index_consider_elite_set:
    #                     if potential_solution[0][1][0] < solution_pack[num1-1][1][0]:
    #                         num1 -= 1
    #                     else:
    #                         break
    #                 add_solution = copy.deepcopy(potential_solution[0])
    #                 solution_pack.insert(num1, add_solution)
    #                 if len(solution_pack) > solution_pack_len:
    #                     solution_pack.pop()
    if len(potential_solution) != 0:
        if index_consider_elite_set < solution_pack_len:
            if potential_solution[0][1][0] - min_solution_pack < epsilon:
                have_same_shape = False
                for jjj in range(len(solution_pack)):
                    if Function.Compare_two_solution_2(potential_solution[0][0], solution_pack[jjj][0]):
                        have_same_shape = True
                        # Nếu như đang xét 
                        if jjj + 1 > index_consider_elite_set:
                            if potential_solution[0][1][0] < solution_pack[jjj][1][0] + epsilon:
                                add_solution = copy.deepcopy(potential_solution[0])
                                solution_pack[jjj] = add_solution
                                break
                        # Nếu như đang xét 
                        else:
                            if potential_solution[0][1][0] < solution_pack[jjj][1][0] + epsilon:
                                have_same_shape = False
                                
                if not have_same_shape:
                    num1 = len(solution_pack)
                    while num1 > index_consider_elite_set:
                        if potential_solution[0][1][0] < solution_pack[num1-1][1][0] + epsilon:
                            num1 -= 1
                        else:
                            break
                    add_solution = copy.deepcopy(potential_solution[0])
                    solution_pack.insert(num1, add_solution)
                    if len(solution_pack) > solution_pack_len:
                        solution_pack.pop()
    restrict_next_loop = []
    for i in range(len(potential_solution)):
        restrict_next_loop.append(potential_solution[i][2])
    for i in range(len(potential_solution)):
        j = 0
        list_accept_truck = [potential_solution[i][3], potential_solution[i][4]]
        sol = copy.deepcopy(potential_solution[i])
        min_to_improve = potential_solution[i][1][0]
        # list_neighborhood = [Neighborhood_drone.Neighborghood_change_drone_route_max_pro_plus_for_specific_truck, Neighborhood_drone.Neighborhood_group_trip]
        list_neighborhood = [Neighborhood_drone.Neighborghood_change_drone_route_max_pro_plus_for_specific_truck, Neighborhood_drone.Neighborhood_change_index_trip, Neighborhood_drone.Neighborhood_group_trip]
        # list_neighborhood = [Neighborhood_drone.Neighborghood_change_drone_route_max_pro_plus_for_specific_truck, Neighborhood_drone.Neighborhood_change_index_trip]             
        while j < number_of_loop_drone:
            j += 1
            # print("i: ",i," j: ", j)
            # print(sol[0])
            for k in range(len(list_neighborhood)):
                if k == 0:
                    drone_neighborhood = list_neighborhood[k](sol[0], list_accept_truck)
                else:
                    drone_neighborhood = list_neighborhood[k](sol[0])
                min_in_loop = 1000000
                next_index = 0
                if len(drone_neighborhood) == 0:
                    continue
                for l in range(len(drone_neighborhood)):
                    cfnode = drone_neighborhood[l][1][0]
                    if cfnode - min_to_improve < epsilon:
                        potential_solution[i] = drone_neighborhood[l]
                        next_index = l
                        min_in_loop = cfnode
                        min_to_improve = cfnode
                        j = 0
                    elif cfnode - min_in_loop < epsilon:
                        next_index = l
                        min_in_loop = cfnode
                sol = drone_neighborhood[next_index]
    for i in range(len(potential_solution)):
        potential_solution[i][2] = restrict_next_loop[i]
    # print(restrict_next_loop)
    return potential_solution, solution_pack

def Neighborhood_combine_truck_and_drone_neighborhood_with_package_1(name_of_truck_neiborhood, solution, number_of_potiential_solution, number_of_loop_drone, whether_use_truck_time, solution_pack, solution_pack_len, use_solution_pack, index_consider_elite_set):
    potential_solution = []
    if whether_use_truck_time:
        current_neighborhood = name_of_truck_neiborhood(solution, Function.fitness(solution)[1])
    else:
        current_neighborhood = name_of_truck_neiborhood(solution)
        
    for i in range(len(current_neighborhood)):
        num = len(potential_solution)
        while num != 0:
            if current_neighborhood[i][1][0] < potential_solution[num-1][1][0]:
                num -= 1
            else:
                break
        potential_solution.insert(num, current_neighborhood[i])
        consider = False
        if len(potential_solution) > number_of_potiential_solution:
            consider = True
            potential_solution.pop()
        
        if use_solution_pack:
            if consider:
                have_same_shape = False
                for jjj in range(len(solution_pack)):
                    if Function.Compare_two_solution_2(current_neighborhood[i][0], solution_pack[jjj][0]):
                        have_same_shape = True
                        if current_neighborhood[i][1][0] < solution_pack[jjj][1][0]:
                            add_solution = copy.deepcopy(current_neighborhood[i])
                            solution_pack[jjj] = add_solution
                        break
                if not have_same_shape:
                    num1 = len(solution_pack)
                    while num1 != 0:
                        if current_neighborhood[i][1][0] < solution_pack[num1-1][1][0]:
                            num1 -= 1
                        else:
                            break
                    add_solution = copy.deepcopy(current_neighborhood[i])
                    solution_pack.insert(num1, add_solution)
                    if len(solution_pack) > solution_pack_len:
                        solution_pack.pop()
                    # print("---------new------------")
                    # print(Function.return_truck_route(add_solution[0]))
                    # print("---------", len(solution_pack), "---------------")
                    # for t in range(len(solution_pack)):
                    #     print(Function.return_truck_route(solution_pack[t][0]))
                    #     print("++++++++")
                    
        
    # if len(solution_pack) != 0:
    #     min_solution_pack = solution_pack[-1][1][0]
    # else:
    #     min_solution_pack = 10000000000
    # if len(potential_solution) != 0:                      
    #     if index_consider_elite_set < solution_pack_len:
    #         if potential_solution[0][1][0] - min_solution_pack < epsilon:
    #             have_same_shape = False
    #             for jjj in range(len(solution_pack)):
    #                 if jjj + 1 > index_consider_elite_set:
    #                     if Function.Compare_two_solution_2(potential_solution[0][0], solution_pack[jjj][0]):
    #                         have_same_shape = True
    #                         if potential_solution[0][1][0] < solution_pack[jjj][1][0]:
    #                             add_solution = copy.deepcopy(potential_solution[0])
    #                             solution_pack[jjj] = add_solution
    #                         break
    #             if not have_same_shape:
    #                 num1 = len(solution_pack)
    #                 while num1 > index_consider_elite_set:
    #                     if potential_solution[0][1][0] < solution_pack[num1-1][1][0]:
    #                         num1 -= 1
    #                     else:
    #                         break
    #                 add_solution = copy.deepcopy(potential_solution[0])
    #                 solution_pack.insert(num1, add_solution)
    #                 if len(solution_pack) > solution_pack_len:
    #                     solution_pack.pop()
            
    for i in range(len(potential_solution)):
        j = 0
        list_accept_truck = [potential_solution[i][3], potential_solution[i][4]]
        sol = copy.deepcopy(potential_solution[i])
        min_to_improve = potential_solution[i][1][0]
        # list_neighborhood = [Neighborhood_drone.Neighborghood_change_drone_route_max_pro_plus_for_specific_truck, Neighborhood_drone.Neighborhood_group_trip]
        list_neighborhood = [Neighborhood_drone.Neighborghood_change_drone_route_max_pro_plus_for_specific_truck, Neighborhood_drone.Neighborhood_change_index_trip]

        while j < number_of_loop_drone:
            j += 1
            for k in range(len(list_neighborhood)):
                if k == 0:
                    drone_neighborhood = list_neighborhood[k](sol[0], list_accept_truck)
                else:
                    drone_neighborhood = list_neighborhood[k](sol[0])
                min_in_loop = 100000000
                next_index = 0
                if len(drone_neighborhood) == 0:
                    continue
                for l in range(len(drone_neighborhood)):
                    cfnode = drone_neighborhood[l][1][0]
                    if cfnode - min_to_improve < epsilon:
                        potential_solution[i] = drone_neighborhood[l]
                        next_index = l
                        min_in_loop = cfnode
                        min_to_improve = cfnode
                        j = 0
                    elif cfnode - min_in_loop < epsilon:
                        next_index = l
                        min_in_loop = cfnode
                sol = drone_neighborhood[next_index]
        
    return potential_solution, solution_pack

def Neighborhood_combine_truck_and_drone_neighborhood_with_tabu_list_with_package_1(name_of_truck_neiborhood, solution, number_of_potial_solution, number_of_loop_drone, tabu_list, tabu_tenure, index_of_loop, best_fitness, kind_of_tabu_structure, need_truck_time, solution_pack, solution_pack_len, use_solution_pack, index_consider_elite_set):
    potential_solution = []
    if need_truck_time:
        current_neighborhood = name_of_truck_neiborhood(solution, Function.fitness(solution)[1])
    else:
        current_neighborhood = name_of_truck_neiborhood(solution)
    # chosen_fitness = current_neighborhood[i][1][0]
    # chosen_sol = current_neighborhood[i][0]
    for i in range(len(current_neighborhood)):
        num = len(potential_solution)
        num1 = len(solution_pack)
        cfsol = current_neighborhood[i][1][0]
        continue2564 = False
        if kind_of_tabu_structure in [1, 2]:
            # print(index_of_loop - tabu_list[current_neighborhood[0][2]] > tabu_tenure)
            # print(current_neighborhood[0][2], "->", tabu_list[current_neighborhood[0][2]])
            if cfsol - best_fitness < epsilon or index_of_loop - tabu_list[current_neighborhood[i][2]] > tabu_tenure: 
                continue2564 = True
        elif kind_of_tabu_structure in [3, 5]:
            if cfsol - best_fitness < epsilon or (index_of_loop - tabu_list[current_neighborhood[i][2][0]] > tabu_tenure or index_of_loop - tabu_list[current_neighborhood[i][2][1]] > tabu_tenure): 
                continue2564 = True
        elif kind_of_tabu_structure in [4]:
            if cfsol - best_fitness < epsilon or (index_of_loop - tabu_list[current_neighborhood[i][2][0]] > tabu_tenure or index_of_loop - tabu_list[current_neighborhood[i][2][1]] > tabu_tenure or index_of_loop - tabu_list[current_neighborhood[i][2][2]] > tabu_tenure): 
                continue2564 = True
        consider = False
        if continue2564:
            while num != 0:
                if cfsol < potential_solution[num-1][1][0]:
                    num -= 1
                else:
                    break
            potential_solution.insert(num, current_neighborhood[i])
            if len(potential_solution) > number_of_potial_solution:
                consider = True
                potential_solution.pop()
                
        if use_solution_pack:    
            if consider:
                have_same_shape = False
                for jjj in range(len(solution_pack)):
                    if Function.Compare_two_solution_2(current_neighborhood[i][0], solution_pack[jjj][0]):
                        have_same_shape = True
                        if current_neighborhood[i][1][0] < solution_pack[jjj][1][0]:
                            add_solution = copy.deepcopy(current_neighborhood[i])
                            solution_pack[jjj] = add_solution
                        break
                if not have_same_shape:
                    num1 = len(solution_pack)
                    while num1 != 0:
                        if current_neighborhood[i][1][0] < solution_pack[num1-1][1][0]:
                            num1 -= 1
                        else:
                            break
                    add_solution = copy.deepcopy(current_neighborhood[i])
                    solution_pack.insert(num1, add_solution)
                    if len(solution_pack) > solution_pack_len:
                        solution_pack.pop()
                    # print("---------new------------")
                    # print(Function.return_truck_route(add_solution[0]))
                    # print("---------", len(solution_pack), "---------------")
                    # for t in range(len(solution_pack)):
                    #     print(Function.return_truck_route(solution_pack[t][0]))
                    #     print("++++++++")
        
        
    # if len(solution_pack) != 0:
    #     min_solution_pack = solution_pack[-1][1][0]
    # else:
    #     min_solution_pack = 10000000000
    # if len(potential_solution) != 0:
    #     if index_consider_elite_set < solution_pack_len:
    #         if potential_solution[0][1][0] - min_solution_pack < epsilon:
    #             have_same_shape = False
    #             for jjj in range(len(solution_pack)):
    #                 if jjj + 1 > index_consider_elite_set:
    #                     if Function.Compare_two_solution_2(potential_solution[0][0], solution_pack[jjj][0]):
    #                         have_same_shape = True
    #                         if potential_solution[0][1][0] < solution_pack[jjj][1][0]:
    #                             add_solution = copy.deepcopy(potential_solution[0])
    #                             solution_pack[jjj] = add_solution
    #                         break
    #             if not have_same_shape:
    #                 num1 = len(solution_pack)
    #                 while num1 > index_consider_elite_set:
    #                     if potential_solution[0][1][0] < solution_pack[num1-1][1][0]:
    #                         num1 -= 1
    #                     else:
    #                         break
    #                 add_solution = copy.deepcopy(potential_solution[0])
    #                 solution_pack.insert(num1, add_solution)
    #                 if len(solution_pack) > solution_pack_len:
    #                     solution_pack.pop()
            
    restrict_next_loop = []
    for i in range(len(potential_solution)):
        restrict_next_loop.append(potential_solution[i][2])
    for i in range(len(potential_solution)):
        j = 0
        list_accept_truck = [potential_solution[i][3], potential_solution[i][4]]
        sol = copy.deepcopy(potential_solution[i])
        min_to_improve = potential_solution[i][1][0]
        # list_neighborhood = [Neighborhood_drone.Neighborghood_change_drone_route_max_pro_plus_for_specific_truck, Neighborhood_drone.Neighborhood_group_trip]
        list_neighborhood = [Neighborhood_drone.Neighborghood_change_drone_route_max_pro_plus_for_specific_truck, Neighborhood_drone.Neighborhood_change_index_trip, Neighborhood_drone.Neighborhood_group_trip]       
        while j < number_of_loop_drone:
            j += 1
            # print("i: ",i," j: ", j)
            # print(sol[0])
            for k in range(len(list_neighborhood)):
                if k == 0:
                    drone_neighborhood = list_neighborhood[k](sol[0], list_accept_truck)
                else:
                    drone_neighborhood = list_neighborhood[k](sol[0])
                min_in_loop = 1000000
                next_index = 0
                if len(drone_neighborhood) == 0:
                    continue
                for l in range(len(drone_neighborhood)):
                    cfnode = drone_neighborhood[l][1][0]
                    if cfnode - min_to_improve < epsilon:
                        potential_solution[i] = drone_neighborhood[l]
                        next_index = l
                        min_in_loop = cfnode
                        min_to_improve = cfnode
                        j = 0
                    elif cfnode - min_in_loop < epsilon:
                        next_index = l
                        min_in_loop = cfnode
                sol = drone_neighborhood[next_index]
    for i in range(len(potential_solution)):
        potential_solution[i][2] = restrict_next_loop[i]
    # print(restrict_next_loop)
    return potential_solution, solution_pack