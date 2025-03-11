import copy
import Data
import Function
import random
import Neighborhood

DIFFERENTIAL_RATE_RELEASE_TIME = Data.DIFFERENTIAL_RATE_RELEASE_TIME
epsilon = (-1)*0.00001
B_ratio = Data.B_ratio
C_ratio = Data.C_ratio

# Neighborhood_one_otp đưa một thành phố ra sau một thành phố khác
def Neighborhood_one_otp(solution, truck_time):
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
                        # for pp in range(len(new_solution[0])):
                        #     print(new_solution[0][pp])
                        # print(new_solution[1])
                        # print("-------------1--------------")
                        city_change = solution[0][i][j][0]
                        package_need_change = copy.deepcopy(solution[0][i][j][1])
                        # Xóa city change trong drop_package (drop_package là các gói hàng nhận tại city_change)
                        if city_change in package_need_change: package_need_change.remove(city_change)
                        # Xóa city_change ra khỏi truck route
                        new_solution[0][i].pop(j)
                        where_city_come_after_by_truck_route = l
                        stop3 = False
                        # Xóa gói hàng của city_change ở điểm giao hàng trên truck route
                        for m in range(len(new_solution[0][i])):
                            if city_change in new_solution[0][i][m][1]:
                                new_solution[0][i][m][1].remove(city_change)
                          
                        # Khi thay thế đoạn dưới vào ba dòng trên thì có thể fix qua một bug khi gộp neighborhood của em và Cường ạ
                        '''array = copy.deepcopy(new_solution[0][i])
                        for m in range(len(array)):  # Xóa gói hàng của city_change ở điểm giao hàng
                            if city_change in array[m][1]:
                                array[m][1].remove(city_change)
                        new_solution[0][i] = copy.deepcopy(array)'''
                        # Khi thay thế đoạn trên vào ba dòng trên thì có thể fix qua một bug khi gộp neighborhood của em và Cường ạ
                                                
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
                        if i == k:
                            if j < l:
                                new_solution[0][i].insert(l, [city_change, []])
                            else:
                                new_solution[0][i].insert(l + 1, [city_change, []])
                        else:
                            new_solution[0][k].insert(l + 1, [city_change, []])
                        # Tiến hành chọn vị trí nhận hàng mới cho drop_package
                        
                        if package_need_change != []:  # Nếu điểm bị xóa là điểm nhận hàng
                            erase_list = []
                            package_need_change1 = []
                            # Xóa chuyến hàng giao hàng mang drop_package ; Xóa theo từng gói hàng trên drop package chứ
                            # không phải theo từng lần giao hàng có tại drop package
                            for m in range(len(new_solution[1])): 
                                for n in range(len(new_solution[1][m])):
                                    if new_solution[1][m][n][0] == city_change:
                                        temp = []
                                        for p in range(len(new_solution[1][m][n][1])):
                                            temp.append(new_solution[1][m][n][1][p])
                                            package_need_change1.append(new_solution[1][m][n][1][p])
                                        #package_need_change1.append(temp)
                                        erase_list.append([m, n])
                            for m in range(len(erase_list) - 1, -1, -1):
                                del1 = erase_list[m][0]
                                del2 = erase_list[m][1]
                                new_solution[1][del1].pop(del2)
                                if new_solution[1][del1] == []:
                                    new_solution[1].pop(del1)
                            
                            # Tìm nơi nhận hàng mới cho từng drop_package
                            # Xét từng góp hàng trong drop package
                            for mm in range(len(package_need_change1)):
                                new_solution = Neighborhood.findIndexOfDropPackage(new_solution, solution, city_change, truck_time, i, j, k, l, package_need_change1[mm])
                            # for pp in range(len(new_solution[0])):
                            #     print(new_solution[0][pp])
                            # print(new_solution[1])
                            # print("---------------------------")
                            
                        # Xử lý gói hàng của city change được giao ở đâu tại trip drone nào ?
                        list_check = []  # Tập hợp điểm nhận hàng trên truck
                        check = False
                        stop = False
                        stop1 = False
                        max_index = - 1  # Tìm điểm cuối cùng có thể nhận được new_package hợp lệ trên route_truck
                        if Data.release_date[city_change] == 0:
                            new_solution[0][k][0][1] += [city_change]
                            stop = True
                        elif Function.max_release_date(new_solution[0][k][0][1]) * DIFFERENTIAL_RATE_RELEASE_TIME + Data.standard_deviation > \
                                Data.release_date[city_change] and Function.min_release_date(new_solution[0][k][0][1]) * \
                                DIFFERENTIAL_RATE_RELEASE_TIME + C_ratio * Data.standard_deviation > Data.release_date[city_change]:
                            new_solution[0][k][0][1] += [city_change] 
                            stop = True
                        else:
                            for m in range(len(new_solution[0][k])):
                                a = new_solution[0][k][m][0]
                                if check == False:
                                    if new_solution[0][k][m][1] != []:
                                        list_check.append(a)
                                    if a == city_change:
                                        check = True
                                else:
                                    if new_solution[0][k][m][1] != []:
                                        max_index = a
                                        break
                            for m in range(len(new_solution[1])):  # Duyệt có điểm nhận hàng nào gộp đợc với new_package không
                                for n in range(len(new_solution[1][m])):
                                    a = new_solution[1][m][n][0]
                                    if a == max_index:
                                        stop1 = True
                                        break
                                    if a in list_check:
                                        if Function.total_demand(new_solution[1][m]) + Data.city_demand[city_change] <= Data.drone_capacity \
                                                and Function.max_release_date_update(new_solution[1][m]) *  DIFFERENTIAL_RATE_RELEASE_TIME + \
                                                    Data.standard_deviation > Data.release_date[city_change] and Function.min_release_date_update(new_solution[1][m]) *  DIFFERENTIAL_RATE_RELEASE_TIME + \
                                                B_ratio * Data.standard_deviation > Data.release_date[city_change]:
                                            stop = True
                                            stop1 = True
                                            number_of_city_change_in_new_truck = -1
                                            for c in range(len(new_solution[0][k])):
                                                if new_solution[0][k][c][0] == a:
                                                    number_of_city_change_in_new_truck = c
                                                    break
                                            new_solution[0][k][number_of_city_change_in_new_truck][
                                                1] += [city_change]
                                            new_solution[1][m][n][1] += [city_change]
                                            break
                                if stop1: break
                                
                        # Tính toán criterion cho từng điểm để chọn điểm phù hợp nhất tạo trip mới
                        # Criterion[i] = |release_date + time_drone_fly – truck_time[i] | 
                        if not stop:  # Nếu việc gộp vào các gói hàng trước không được, tạo điểm nhận hàng mới cho new_package
                            add1 = Data.manhattan_move_matrix[solution[0][k][l][0]][city_change]
                            add2 = 0
                            if l != len(solution[0][k]) - 1:
                                add2 = add1 + Data.manhattan_move_matrix[city_change][solution[0][k][l + 1][0]]
                            minn = 100000
                            index = -1
                            for a in range(len(new_solution[0][k])):
                                point_evaluate = new_solution[0][k][a][0]
                                if a > l + 1:
                                    if Data.euclid_flight_matrix[0][point_evaluate] * 2 + Data.unloading_time <= Data.drone_limit_time:
                                        criterion = abs(Data.city_demand[city_change] +
                                                        Data.euclid_flight_matrix[0][
                                                            point_evaluate] - (truck_time[k][a - 1] + add2))
                                        if criterion < minn:
                                            index = a
                                            minn = criterion
                                elif a == l + 1:
                                    if Data.euclid_flight_matrix[0][
                                        point_evaluate] * 2 + Data.unloading_time <= Data.drone_limit_time:
                                        criterion = abs(Data.city_demand[city_change] +
                                                        Data.euclid_flight_matrix[0][
                                                            point_evaluate] - (truck_time[k][a - 1] + add1))
                                        if criterion < minn:
                                            index = a
                                            minn = criterion
                                else:
                                    if Data.euclid_flight_matrix[0][
                                        point_evaluate] * 2 + Data.unloading_time <= Data.drone_limit_time:
                                        criterion = abs(Data.city_demand[city_change] + \
                                                        Data.euclid_flight_matrix[0][
                                                            point_evaluate] - (
                                                            truck_time[k][a]))
                                        if criterion < minn:
                                            index = a
                                            minn = criterion
                                if new_solution[0][k][a][0] == city_change: break
                            # Sau đoạn trên đã tìm được thành phố nhận hàng city change

                            # Tìm xem gói hàng city change được giao ở trip drone nào
                            new_solution[0][k][index][1] += [city_change]
                            if index != 0:
                                new_solution, number_in_the_drone_queue_of_drop_package, index_in_trip = Neighborhood.addNewTripInDroneRoute(new_solution, [city_change], k, index)
                        #print(Function.Check_if_feasible(new_solution))
                        # print(new_solution)
                        # print(Function.Check_if_feasible(new_solution))
                        pack_child = []
                        pack_child.append(new_solution)
                        a, b, c = Function.fitness(new_solution)
                        pack_child.append([a, b, c])
                        pack_child.append(city_change)
                        pack_child.append(i)
                        pack_child.append(k)
                        neighborhood.append(pack_child)
    return neighborhood

def Neighborhood_one_opt_standard(solution):
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
                        
                        pre_drop_package = copy.deepcopy(solution[0][i][j][1])
                        # Xóa city change trong drop_package (drop_package là các gói hàng nhận tại city_change)
                        if city_change in pre_drop_package: pre_drop_package.remove(city_change)
                        # Xóa city_change ra khỏi truck route
                        new_solution[0][i].pop(j)
                        
                        stop3 = False
                        # Xóa gói hàng của city_change ở điểm giao hàng trên truck route
                        for m in range(len(new_solution[0][i])):
                            if city_change in new_solution[0][i][m][1]:
                                new_solution[0][i][m][1].remove(city_change)
                                break
                                                                              
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
                        if i == k:
                            if j < l:
                                new_solution[0][i].insert(l, [city_change, []])
                            else:
                                new_solution[0][i].insert(l + 1, [city_change, []])
                        else:
                            new_solution[0][k].insert(l + 1, [city_change, []])
                        # Tiến hành chọn vị trí nhận hàng mới cho drop_package
                    
                        drop_package = []
                        # Xóa chuyến hàng giao hàng mang drop_package ; Xóa theo từng gói hàng trên drop package 
                        for m in reversed(range(len(new_solution[1]))):
                            for mm in reversed(range(len(new_solution[1][m]))):
                                for mmm in reversed(range(len(new_solution[1][m][mm][1]))):
                                    city = new_solution[1][m][mm][1][mmm]
                                    if city in pre_drop_package:
                                        new_solution[1][m][mm][1].pop(mmm)
                                        if new_solution[1][m][mm][1] == []:
                                            new_solution[1][m].pop(mm)
                                            if new_solution[1][m] == []:
                                                new_solution[1].pop(m)
                        for m in range(len(new_solution[0][i])):
                            city = new_solution[0][i][m][0]
                            if city in pre_drop_package:
                                drop_package.append(city)
                        
                        # Tìm nơi nhận hàng mới cho từng drop_package
                        # Xét từng góp hàng trong drop package
                        new_solution1 = copy.deepcopy(new_solution)
                        for m in range(len(drop_package)):
                            new_solution, new_solution1 = Neighborhood.findLocationForDropPackage(new_solution, i, drop_package[m])
                            
                        # Xử lý gói hàng của city change được giao ở đâu tại trip drone nào ?
                        new_solution, new_solution1 = Neighborhood.findLocationForDropPackage(new_solution, k, city_change)
                        # print(new_solution)
                        # print(Function.Check_if_feasible(new_solution))
                        pack_child = []
                        pack_child.append(new_solution)
                        a, b, c = Function.fitness(new_solution)
                        pack_child.append([a, b, c])
                        pack_child.append(city_change)
                        pack_child.append(i)
                        pack_child.append(k)
                        neighborhood.append(pack_child)

                        if new_solution1 != new_solution:
                            pack_child1 = []
                            pack_child1.append(new_solution1)
                            a1, b1, c1 = Function.fitness(new_solution1)
                            pack_child1.append([a1, b1, c1])
                            pack_child1.append(city_change)
                            pack_child1.append(i)
                            pack_child1.append(k)
                            neighborhood.append(pack_child1)
    return neighborhood

def Neighborhood_one_otp_plus(solution, truck_time):
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
                        #print(i, ", ", j, ", ", k, ", ", l)
                        # for pp in range(len(new_solution[0])):
                        #     print(new_solution[0][pp])
                        # print(new_solution[1])
                        # print("-------------1--------------")

                        city_change = solution[0][i][j][0]
                        drop_package = copy.deepcopy(solution[0][i][j][1])
                        # Xóa city change trong drop_package (drop_package là các gói hàng nhận tại city_change)
                        if city_change in drop_package: drop_package.remove(city_change)
                        # Xóa city_change ra khỏi truck route
                        new_solution[0][i].pop(j)
                        where_city_come_after_by_truck_route = l
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
                        IndexOfNewReceiveCity = -1
                        # Chuyển city_change tới vị trí mới trên truck route - new_solution[0]
                        if i == k:
                            if j < l:
                                new_solution[0][i].insert(l, [city_change, []])
                                IndexOfNewReceiveCity = j
                            else:
                                new_solution[0][i].insert(l + 1, [city_change, []])
                                IndexOfNewReceiveCity = j + 1
                        else:
                            new_solution[0][k].insert(l + 1, [city_change, []])
                            IndexOfNewReceiveCity = j
                        # Tiến hành chọn vị trí nhận hàng mới cho drop_package
                        
                        if drop_package != []:  # Nếu điểm bị xóa là điểm nhận hàng
                            erase_list = []
                            drop_package1 = []
                            # Xóa chuyến hàng giao hàng mang drop_package ; Xóa theo từng gói hàng trên drop package chứ
                            # không phải theo từng lần giao hàng có tại drop package
                            for m in range(len(new_solution[1])): 
                                for n in range(len(new_solution[1][m])):
                                    if new_solution[1][m][n][0] == city_change:
                                        temp = []
                                        for p in range(len(new_solution[1][m][n][1])):
                                            temp.append(new_solution[1][m][n][1][p])
                                            drop_package1.append([new_solution[1][m][n][1][p]])
                                        #drop_package1.append(temp)
                                        erase_list.append([m, n])
                            for m in range(len(erase_list) - 1, -1, -1):
                                del1 = erase_list[m][0]
                                del2 = erase_list[m][1]
                                new_solution[1][del1].pop(del2)
                                if new_solution[1][del1] == []:
                                    new_solution[1].pop(del1)
                            
                            # Tìm nơi nhận hàng mới cho từng drop_package
                            # Xét từng góp hàng trong drop package
                            
                            city_received = new_solution[0][i][IndexOfNewReceiveCity][0]
                            while Data.euclid_flight_matrix[0][city_received] * 2 + Data.unloading_time > Data.drone_limit_time:
                                IndexOfNewReceiveCity -= 1
                                city_received = new_solution[0][i][IndexOfNewReceiveCity][0]
                                
                            for mm in range(len(drop_package1)):
                                
                                new_solution[0][i][IndexOfNewReceiveCity][1] = new_solution[0][i][IndexOfNewReceiveCity][1] + drop_package1[mm]
                                #Add điểm giao hàng tại drone package
                                if IndexOfNewReceiveCity != 0:
                                    Neighborhood.groupTripInDroneRoute(new_solution, drop_package1[mm], i, IndexOfNewReceiveCity)
                                
                            # for pp in range(len(new_solution[0])):
                            #     print(new_solution[0][pp])
                            # print(new_solution[1])
                            # print("---------------------------")
                            
                        # Xử lý gói hàng của city change được giao ở đâu tại trip drone nào ?
                        list_check = []  # Tập hợp điểm nhận hàng trên truck
                        check = False
                        stop = False
                        stop1 = False
                        max_index = - 1  # Tìm điểm cuối cùng có thể nhận được new_package hợp lệ trên route_truck
                        if Data.release_date[city_change] == 0:
                            new_solution[0][k][0][1] += [city_change]
                            stop = True
                        elif Function.max_release_date(new_solution[0][k][0][1]) * DIFFERENTIAL_RATE_RELEASE_TIME + Data.standard_deviation > \
                                Data.release_date[city_change] and Function.min_release_date(new_solution[0][k][0][1]) * \
                                DIFFERENTIAL_RATE_RELEASE_TIME + C_ratio * Data.standard_deviation > Data.release_date[city_change]:
                            new_solution[0][k][0][1] += [city_change] 
                            stop = True
                        else:
                            for m in range(len(new_solution[0][k])):
                                a = new_solution[0][k][m][0]
                                if check == False:
                                    if new_solution[0][k][m][1] != []:
                                        list_check.append(a)
                                    if a == city_change:
                                        check = True
                                else:
                                    if new_solution[0][k][m][1] != []:
                                        max_index = a
                                        break
                            for m in range(len(new_solution[1])):  # Duyệt có điểm nhận hàng nào gộp đợc với new_package không
                                for n in range(len(new_solution[1][m])):
                                    a = new_solution[1][m][n][0]
                                    if a == max_index:
                                        stop1 = True
                                        break
                                    if a in list_check:
                                        if Function.total_demand(new_solution[1][m]) + Data.city_demand[city_change] <= Data.drone_capacity \
                                            and Function.max_release_date_update(new_solution[1][m]) *  DIFFERENTIAL_RATE_RELEASE_TIME + \
                                                Data.standard_deviation > Data.release_date[city_change] and Function.min_release_date_update(new_solution[1][m]) *  DIFFERENTIAL_RATE_RELEASE_TIME + \
                                                B_ratio * Data.standard_deviation > Data.release_date[city_change]:
                                            stop = True
                                            stop1 = True
                                            number_of_city_change_in_new_truck = -1
                                            for c in range(len(new_solution[0][k])):
                                                if new_solution[0][k][c][0] == a:
                                                    number_of_city_change_in_new_truck = c
                                                    break
                                            new_solution[0][k][number_of_city_change_in_new_truck][
                                                1] += [city_change]
                                            new_solution[1][m][n][1] += [city_change]
                                            break
                                if stop1: break
                                
                        # Tính toán criterion cho từng điểm để chọn điểm phù hợp nhất tạo trip mới
                        # Criterion[i] = |release_date + time_drone_fly – truck_time[i] | 
                        if not stop:  # Nếu việc gộp vào các gói hàng trước không được, tạo điểm nhận hàng mới cho new_package
                            add1 = Data.manhattan_move_matrix[solution[0][k][l][0]][city_change]
                            add2 = 0
                            if l != len(solution[0][k]) - 1:
                                add2 = add1 + Data.manhattan_move_matrix[city_change][solution[0][k][l + 1][0]]
                            minn = 100000
                            index = -1
                            for a in range(len(new_solution[0][k])):
                                point_evaluate = new_solution[0][k][a][0]
                                if a > l + 1:
                                    if Data.euclid_flight_matrix[0][point_evaluate] * 2 + Data.unloading_time <= Data.drone_limit_time:
                                        criterion = abs(Data.city_demand[city_change] +
                                                        Data.euclid_flight_matrix[0][
                                                            point_evaluate] - (truck_time[k][a - 1] + add2))
                                        if criterion < minn:
                                            index = a
                                            minn = criterion
                                elif a == l + 1:
                                    if Data.euclid_flight_matrix[0][
                                        point_evaluate] * 2 + Data.unloading_time <= Data.drone_limit_time:
                                        criterion = abs(Data.city_demand[city_change] +
                                                        Data.euclid_flight_matrix[0][
                                                            point_evaluate] - (truck_time[k][a - 1] + add1))
                                        if criterion < minn:
                                            index = a
                                            minn = criterion
                                else:
                                    if Data.euclid_flight_matrix[0][
                                        point_evaluate] * 2 + Data.unloading_time <= Data.drone_limit_time:
                                        criterion = abs(Data.city_demand[city_change] + \
                                                        Data.euclid_flight_matrix[0][
                                                            point_evaluate] - (
                                                            truck_time[k][a]))
                                        if criterion < minn:
                                            index = a
                                            minn = criterion
                                if new_solution[0][k][a][0] == city_change: break
                            # Sau đoạn trên đã tìm được thành phố nhận hàng city change

                            # Tìm xem gói hàng city change được giao ở trip drone nào
                            new_solution[0][k][index][1] += [city_change]
                            if index != 0:
                                new_solution, number_in_the_drone_queue_of_drop_package, index_in_trip = Neighborhood.addNewTripInDroneRoute(new_solution, [city_change], k, index)
                        # print(new_solution)
                        # print(Function.Check_if_feasible(new_solution))
                        pack_child = []
                        pack_child.append(new_solution)
                        a, b, c = Function.fitness(new_solution)
                        pack_child.append([a, b, c])
                        pack_child.append(city_change)  # City change
                        pack_child.append(i)            # Truck mà city change vừa rời
                        pack_child.append(k)
                        neighborhood.append(pack_child)
    return neighborhood

def Neighborhood_move_depot(solution):
    neighborhood = []
    for i in range(len(solution[0])):
        for j in range(1, len(solution[0][i]) - 1):
            new_solution = copy.deepcopy(solution)
            pre_drop_package = []
            
            for k in reversed(range(1, j + 1)):
                pre_drop_package.append(new_solution[0][i][k][0])
                for l in reversed(range(len(new_solution[0][i][k][1]))):
                    city = new_solution[0][i][k][1][l]
                    if city not in pre_drop_package:
                        pre_drop_package.append(city)
                new_solution[0][i][k][1] = []
            
            for k in reversed(range(len(new_solution[0][i][0][1]))):
                city = new_solution[0][i][0][1][k]
                if city in pre_drop_package:
                    new_solution[0][i][0][1].pop(k)
            
            for x in reversed(range(len(new_solution[1]))):
                for y in reversed(range(len(new_solution[1][x]))):
                    for z in reversed(range(len(new_solution[1][x][y][1]))):
                        city = new_solution[1][x][y][1][z]
                        if city in pre_drop_package:
                            new_solution[1][x][y][1].pop(z)
                            if new_solution[1][x][y][1] == []:
                                new_solution[1][x].pop(y)
                                if new_solution[1][x] == []:
                                    new_solution[1].pop(x)
            new_solution[0][i] = new_solution[0][i][:1] + new_solution[0][i][j+1:] + new_solution[0][i][1:j+1]
            drop_package = []
            for k in range(len(new_solution[0][i])):
                city = new_solution[0][i][k][0]
                if city in pre_drop_package:
                    drop_package.append(city)
            for k in range(len(drop_package)):
                new_solution = Neighborhood.findLocationForDropPackage(new_solution, i, drop_package[k])
            pack_child = []
            pack_child.append(new_solution)
            a, b, c = Function.fitness(new_solution)
            pack_child.append([a, b, c])
            neighborhood.append(pack_child)
    return neighborhood

def Neighborhood_one_otp_fix_for_specific_truck(solution, index_truck):
    neighborhood = []
    for i in range(len(solution[0])):
        if i != index_truck:
            continue
        for j in range(1, len(solution[0][i])):
            for k in range(len(solution[0])):
                if k != index_truck:
                    continue
                for l in range(len(solution[0][k])):
                    if solution[0][i][j][0] == 0: continue
                    if i == k and (j == l or j == l + 1):
                        continue
                    else:
                        new_solution = copy.deepcopy(solution)

                        city_change = solution[0][i][j][0]
                        
                        pre_drop_package = copy.deepcopy(solution[0][i][j][1])
                        # Xóa city change trong drop_package (drop_package là các gói hàng nhận tại city_change)
                        if city_change in pre_drop_package: pre_drop_package.remove(city_change)
                        # Xóa city_change ra khỏi truck route
                        new_solution[0][i].pop(j)
                        
                        stop3 = False
                        # Xóa gói hàng của city_change ở điểm giao hàng trên truck route
                        for m in range(len(new_solution[0][i])):
                            if city_change in new_solution[0][i][m][1]:
                                new_solution[0][i][m][1].remove(city_change)
                                break
                                                                              
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
                        if i == k:
                            if j < l:
                                new_solution[0][i].insert(l, [city_change, []])
                            else:
                                new_solution[0][i].insert(l + 1, [city_change, []])
                        else:
                            new_solution[0][k].insert(l + 1, [city_change, []])
                        # Tiến hành chọn vị trí nhận hàng mới cho drop_package
                    
                        drop_package = []
                        # Xóa chuyến hàng giao hàng mang drop_package ; Xóa theo từng gói hàng trên drop package 
                        for m in reversed(range(len(new_solution[1]))):
                            for mm in reversed(range(len(new_solution[1][m]))):
                                for mmm in reversed(range(len(new_solution[1][m][mm][1]))):
                                    city = new_solution[1][m][mm][1][mmm]
                                    if city in pre_drop_package:
                                        new_solution[1][m][mm][1].pop(mmm)
                                        if new_solution[1][m][mm][1] == []:
                                            new_solution[1][m].pop(mm)
                                            if new_solution[1][m] == []:
                                                new_solution[1].pop(m)
                        for m in range(len(new_solution[0][i])):
                            city = new_solution[0][i][m][0]
                            if city in pre_drop_package:
                                drop_package.append(city)
                        
                        # Tìm nơi nhận hàng mới cho từng drop_package
                        # Xét từng góp hàng trong drop package
                        for m in range(len(drop_package)):
                            new_solution = Neighborhood.findLocationForDropPackage(new_solution, i, drop_package[m])
                            
                        # Xử lý gói hàng của city change được giao ở đâu tại trip drone nào ?
                        new_solution = Neighborhood.findLocationForDropPackage(new_solution, k, city_change)
                        # print(Function.Check_if_feasible(new_solution))
                        # print(new_solution)
                        # print(Function.Check_if_feasible(new_solution))
                        pack_child = []
                        pack_child.append(new_solution)
                        a, b, c = Function.fitness(new_solution)
                        pack_child.append([a, b, c])
                        pack_child.append(city_change)
                        pack_child.append(i)
                        pack_child.append(k)
                        neighborhood.append(pack_child)
    return neighborhood

def Neighborhood_move_1_1_ver2_for_specific_truck(solution, index_truck):
    neighborhood = []
    for i in range(len(solution[0])):
        if i != index_truck:
            continue
        for j in range(1, len(solution[0][i])):
            for k in range(len(solution[0])):
                if k != index_truck:
                    continue
                for l in range(1, len(solution[0][k])):
                    if i == k and j == l:
                        continue
                    new_solution = copy.deepcopy(solution)
                    pre_drop_package = []
                    city1 = solution[0][i][j][0]
                    city2 = solution[0][k][l][0]
                    pre_drop_package.append(city1)
                    pre_drop_package.append(city2)
                    
                    for ii in reversed(range(len(new_solution[0][i][j][1]))):
                        city = new_solution[0][i][j][1][ii]
                        if city not in pre_drop_package:
                            pre_drop_package.append(city)
                        new_solution[0][i][j][1].pop(ii)
                    for ii in reversed(range(len(new_solution[0][k][l][1]))):
                        city = new_solution[0][k][l][1][ii]
                        if city not in pre_drop_package:
                            pre_drop_package.append(city)
                        new_solution[0][k][l][1].pop(ii)
                    
                    # Thay đổi hành trình truck
                    new_solution[0][k][l][0] = city1
                    new_solution[0][i][j][0] = city2
                    
                    # Xóa hàng 
                    drop_package1 = []
                    drop_package2 = []
                    for ii in range(len(new_solution[0][i])):
                        city = new_solution[0][i][ii][0]
                        if city in pre_drop_package:
                            drop_package1.append(city)
                        for iii in reversed(range(len(new_solution[0][i][ii][1]))):
                            pack = new_solution[0][i][ii][1][iii]
                            if pack in pre_drop_package:
                                new_solution[0][i][ii][1].pop(iii)
                    if i != k:
                        for ii in range(len(new_solution[0][k])):
                            city = new_solution[0][k][ii][0]
                            if city in pre_drop_package:
                                drop_package2.append(city)
                            for iii in reversed(range(len(new_solution[0][k][ii][1]))):
                                pack = new_solution[0][k][ii][1][iii]
                                if pack in pre_drop_package:
                                    new_solution[0][k][ii][1].pop(iii)
                    for ii in reversed(range(len(new_solution[1]))):
                        for iii in reversed(range(len(new_solution[1][ii]))):
                            city = new_solution[1][ii][iii][0]
                            if city == city1 or city == city2:
                                new_solution[1][ii].pop(iii)
                            else:
                                for jj in reversed(range(len(new_solution[1][ii][iii][1]))):
                                    pack = new_solution[1][ii][iii][1][jj]
                                    if pack in pre_drop_package:
                                        new_solution[1][ii][iii][1].pop(jj)
                                if new_solution[1][ii][iii][1] == []:
                                    new_solution[1][ii].pop(iii)
                        if new_solution[1][ii] == []:
                            new_solution[1].pop(ii)
                    
                    for ii in range(len(drop_package1)):
                        new_solution = Neighborhood.findLocationForDropPackage(new_solution, i, drop_package1[ii])
                    if i != k:
                        for ii in range(len(drop_package2)):
                            new_solution = Neighborhood.findLocationForDropPackage(new_solution, k, drop_package2[ii])
                    
                    # print(new_solution)
                    # print(Function.fitness(new_solution)[0])
                    # print(Function.Check_if_feasible(new_solution))
                    # print(len(neighborhood))
                    # print("---------------------------") 
                    
                    pack_child = []
                    pack_child.append(new_solution)
                    a, b, c = Function.fitness(new_solution)                
                    pack_child.append([a, b, c])
                    pack_child.append([city1, city2])
                    pack_child.append(i)
                    pack_child.append(k)
                    neighborhood.append(pack_child)
                    
    return neighborhood


