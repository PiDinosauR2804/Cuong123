import random
import copy
import Data
import Function
import Neighborhood

alpha = 0.6

beta = - 0.2

def depot_or_after(package, truck_route):
    if Data.manhattan_distance(Data.city[0], Data.city[truck_route[1][0]]) + Data.unloading_time < Data.release_date[package] * 2:
        if Function.sum_weight(truck_route[1][1]) + Data.city_demand[package] <= Data.drone_capacity and Data.euclid_flight_matrix[0][truck_route[1][0]]*2 + Data.unloading_time <= Data.drone_limit_time:
            truck_route[1][1].append(package)
        else:
            truck_route[0][1].append(package)
    else:
        truck_route[0][1].append(package)
    return truck_route

def max_rd(truck_route, position):
    max_rd= Data.release_date[truck_route[1][0]]
    for i in range (1, position+1):
        if Data.release_date[truck_route[i][0]] > max_rd:
            max_rd = Data.release_date[truck_route[i][0]]

    return max_rd

def min_rd(truck_route, position):
    min_rd= Data.release_date[truck_route[1][0]]
    for i in range (1, position+1):
        if Data.release_date[truck_route[i][0]] < min_rd:
            min_rd = Data.release_date[truck_route[i][0]]

    return min_rd

def split_package(point):
    temp = []
    for j in range(Data.drone_capacity):
        temp.append(point[j])
    
    for i in range(len(temp)):
        #print(temp[i])
        #print(point)
        point.remove(temp[i])
    
    split = []
    split.append(temp)
    split.append(point)

    return split

def nearest_resupply(truck_route, position):
    index = position
    
    for i in range (position, 0, -1):
        if truck_route[i][1] != []:
            index = i
            break
    
    if index == position: return 0
    else: return index

def over_capacity(truck_route, position):
    minrd = 999
    package_to_go = truck_route[position][1][0]
    for i in range (len(truck_route[position][1])):
        if Function.sum_weight(truck_route[position][1]) - Data.city_demand[truck_route[position][1][i]] <= Data.drone_capacity:
            if Data.release_date[truck_route[position][1][i]] <= minrd:
                package_to_go = truck_route[position][1][i]
                minrd = truck_route[position][1][i]
    
    return package_to_go

def rearrange_package(package, truck_route, position):
    maxrd= max_rd(truck_route, position)
    minrd= min_rd(truck_route, position)
    nearest_position = nearest_resupply(truck_route, position)
    if Data.release_date[package]==0:
        truck_route[0][1].append(package)
    else:
        p = Function.max_release_date(truck_route[nearest_position][1])/Data.release_date[package]

        if  1 - alpha <= p <= 1 + alpha:
            if nearest_position != 0: 
                if Function.sum_weight(truck_route[nearest_position][1]) + Data.city_demand[package] <= Data.drone_capacity:
                    truck_route[nearest_position][1].append(package)
                else:
                    truck_route[nearest_position][1].append(package)
                    package_to_go = over_capacity(truck_route, nearest_position)
                    truck_route[nearest_position][1].remove(package_to_go)
                    if Function.sum_weight(truck_route[nearest_resupply(truck_route, nearest_position)][1]) + Data.city_demand[package_to_go] <= Data.drone_capacity:
                        truck_route[nearest_resupply(truck_route, nearest_position)][1].append(package_to_go)
                    else:
                        truck_route = depot_or_after(package_to_go, truck_route)
            else:
                if Function.sum_weight(truck_route[position][1]) + Data.city_demand[package] <= Data.drone_capacity and Data.euclid_flight_matrix[0][truck_route[position][0]]*2 + Data.unloading_time <= Data.drone_limit_time: 
                    truck_route[position][1].append(package)
                else:
                    truck_route[position-1][1].append(package)

        elif p < 1 - alpha:
            if Function.sum_weight(truck_route[position][1]) + Data.city_demand[package] <= Data.drone_capacity and Data.euclid_flight_matrix[0][truck_route[position][0]]*2 + Data.unloading_time <= Data.drone_limit_time: 
                truck_route[position][1].append(package)
            else:
                truck_route = depot_or_after(package, truck_route)

        else:
            if maxrd == minrd:
                if truck_route[0][1] != []: truck_route[0][1].append(package)
                else:
                    check = 0
                    for i in range (1, position):
                        if Function.sum_weight(truck_route[i][1]) + Data.city_demand[package] <= Data.drone_capacity and Data.euclid_flight_matrix[0][truck_route[i][0]]*2 + Data.unloading_time <= Data.drone_limit_time:
                            truck_route[i][1].append(package)
                            check +=1
                            break
                    if check == 0: truck_route = depot_or_after(package, truck_route)
                    
            else:
                ratio = (Data.release_date[package] - minrd)/(maxrd - minrd)
                if ratio == 1: newposition = position
                else: 
                    newposition = int(ratio/(1/(position+1)))
                if newposition == 0: truck_route = depot_or_after(package, truck_route)
                else:
                    if Function.sum_weight(truck_route[newposition][1]) + Data.city_demand[package] > Data.drone_capacity:
                        if Function.sum_weight(truck_route[newposition-1][1]) + Data.city_demand[package] <= Data.drone_capacity and Data.euclid_flight_matrix[0][truck_route[newposition-1][0]]*2 + Data.unloading_time <= Data.drone_limit_time:
                            truck_route[newposition-1][1].append(package)
                        else:
                            truck_route = depot_or_after(package, truck_route)
                    else:
                        if Data.euclid_flight_matrix[0][truck_route[newposition][0]]*2 + Data.unloading_time <= Data.drone_limit_time:           
                            truck_route[newposition][1].append(package)
                        else:
                            truck_route = depot_or_after(package, truck_route)
    return truck_route

def fix_drone_queue(newsolution):
    need_to_earse = []
    for h in range(len(newsolution[0])):
        for k in range(1, len(newsolution[0][h])):
            for l in range(len(newsolution[1])):
                for u in range(len(newsolution[1][l])):
                    if newsolution[1][l][u][0] == newsolution[0][h][k][0]:  # Cập nhật các điểm hàng được gộp, hoặc bị xóa
                        if newsolution[0][h][k][1] == []:
                            need_to_earse.append(l)
                        if newsolution[1][l][u][1] == newsolution[0][h][k][1]: continue
    need_to_earse=sorted(need_to_earse)
    if need_to_earse != []:
        for m in range (len(need_to_earse)):
            newsolution[1][need_to_earse[m]].clear()
    for i in range(len(need_to_earse)):
        newsolution[1].remove([])

    for h in range(len(newsolution[0])):
        for k in range(1, len(newsolution[0][h])):
            if newsolution[0][h][k][1] != []:
                check = 0
                for u in range(len(newsolution[1])):
                    flag = 0
                    for v in range(len(newsolution[1][u])):
                        if newsolution[0][h][k][0] == newsolution[1][u][v][0]:
                            if newsolution[1][u][v][1] == newsolution[0][h][k][1]: break
                            else:
                                #print(h, k, u)
                                newsolution[1][u][v][1] = copy.deepcopy(newsolution[0][h][k][1])
                                #print(newsolution[1][u][0][1], newsolution[0][h][k][1])
                        else: 
                             flag += 1
                    if flag != 0: check +=1
                if check == len(newsolution[1]): #Thêm mới vào drone_queue
                    min_pos=0
                    max_pos=len(newsolution[1]) - 1
                    
                    for c in range (len(newsolution[1])):
                        for m in range (len(newsolution[1][c])):
                            for d in range(len(newsolution[0][h])):
                                if newsolution[1][c][m][0] == newsolution[0][h][d][0]:
                                    if d < k: min_pos = c
                                    elif d > k: 
                                        max_pos = c
                                        break 
                    
                    index = -1
                    for e in range (min_pos + 1, max_pos - 1):
                        for q in range (len(newsolution[1][e])):
                            if Function.max_release_date(newsolution[1][e][q][1]) < Function.max_release_date(newsolution[0][h][k][1]):
                                index = e + 1

                    if index == -1: 
                        index = min_pos + 1
                    new = copy.deepcopy(newsolution[0][h][k])
                    newsolution[1].insert(index, [new])

    for i in range (len(newsolution[1])):
        for u in range(len(newsolution[1][i])):
            for j in range (i + 1, len(newsolution[1])):
                for v in range(len(newsolution[1][j])):
                    if Function.find_position(newsolution[1][i][u][0], newsolution[0])[0] == Function.find_position(newsolution[1][j][v][0], newsolution[0])[0] and Function.find_position(newsolution[1][i][u][0], newsolution[0])[1] > Function.find_position(newsolution[1][j][v][0], newsolution[0])[1]:
                        temp = newsolution[1][i][u]
                        newsolution[1][i][u] = newsolution[1][j][v]
                        newsolution[1][j][v] = temp

    removing = []
    for i in range(len(newsolution[1])):
        for h in range(len(newsolution[1][i])):
            for j in range (i+1, len(newsolution[1])):
                for k in range(len(newsolution[1][j])):
                    if newsolution[1][i][h][0] == newsolution[1][j][k][0]:
                        if len(newsolution[1][i][h][1]) > Data.drone_capacity or len(newsolution[1][j][k][1]) > Data.drone_capacity:
                            temp = split_package(newsolution[1][i][h][1])
                            newsolution[1][i][h][1] = temp[0]
                            newsolution[1][j][k][1] = temp[1]
                        else:
                            if newsolution[1][i][h][1] == newsolution[1][j][k][1]:
                                removing.append([j, k])

    for i in range(len(removing)):
        newsolution[1][removing[i][0]].remove(newsolution[1][removing[i][0]][removing[i][1]])

    update = []
    for i in range(len(newsolution[1])):
            if newsolution[1][i] == []:
                update.append(i)
    for i in range(len(update)):
        newsolution[1].remove([])
       
    return newsolution               
                            
def two_swap(solution, truck_time):
    neighbors = []
    used=[]
    for i in range(len(solution[0])):
        for j in range(1, len(solution[0][i])):
                for m in range(len(solution[0])):
                    for n in range(1, len(solution[0][m])):
                        if i==m and j==n : continue
                        else:
                            #print(i,", ",j,", ",m,", ",n)
                            
                            newsolution=copy.deepcopy(solution)
                            '''if i == 0 and j == 2 and m == 1 and n ==1:
                                print(newsolution[0][0])
                                print(newsolution[0][1])
                                print(newsolution[0][2])
                                print(newsolution[1])
                                print("--------------------------------")'''
                            city_change1=newsolution[0][i][j][0]
                            city_change2=newsolution[0][m][n][0]
                            if sorted([city_change1,city_change2]) in used: continue

                            package_needchange1=copy.deepcopy(newsolution[0][i][j][1])
                            package_needchange2=copy.deepcopy(newsolution[0][m][n][1])
                            
                            used.append(sorted([city_change1, city_change2]))
                            
                            #thay đổi vị trí 2 thành phố
                            newsolution[0][m][n][0]=city_change1
                            newsolution[0][i][j][0]=city_change2

                            
                            
                            #Xóa gói hàng ở city_change
                            newsolution[0][i][j][1].clear()
                            newsolution[0][m][n][1].clear()
                            
                            #Xóa gói hàng cần xóa 
                            for h1 in range (len(newsolution[0][i])):
                                if city_change1 in newsolution[0][i][h1][1]:
                                    newsolution[0][i][h1][1].remove(city_change1)
                                    break

                            for h2 in range (len(newsolution[0][m])):
                                if city_change2 in newsolution[0][m][h2][1]:
                                    newsolution[0][m][h2][1].remove(city_change2)
                                    break
                            '''if i == 0 and j == 2 and m == 1 and n ==1:
                                print(newsolution[0][0])
                                print(newsolution[0][1])
                                print(newsolution[0][2])
                                print(newsolution[1])
                                print("--------------------------------")'''
                            #Cập nhật vị trí giao hàng mới

                            if city_change1 in package_needchange1: package_needchange1.remove(city_change1)
                            if city_change2 in package_needchange1: package_needchange1.remove(city_change2)
                            if city_change2 in package_needchange2: package_needchange2.remove(city_change2)
                            if city_change1 in package_needchange2: package_needchange2.remove(city_change1)

    
                            
                            
                            #rearrange_package của city_change 1
                            if city_change1 in solution[0][i][0][1]: #Kiểm tra xem giao tại depot hay được resupply
                                if Data.release_date[city_change1] == 0:
                                    newsolution[0][m][0][1].append(city_change1)
                                else:
                                    p = Function.max_release_date(newsolution[0][m][0][1])/Data.release_date[city_change1]
                                    if p < 1 - beta:
                                        newsolution[0][m] = rearrange_package(city_change1, newsolution[0][m], n)
                                    else: newsolution[0][m][0][1].append(city_change1)
                            else:
                                newsolution[0][m] = rearrange_package(city_change1, newsolution[0][m], n)
                            
                            for k1 in range (len(package_needchange1)):
                                newsolution[0][i] = rearrange_package(package_needchange1[k1], newsolution[0][i], Function.find_position(package_needchange1[k1], newsolution[0])[1])
                            
                            #rearrange_package của city_change 2
                            if city_change2 in solution[0][m][0][1]: #Kiểm tra xem giao tại depot hay được resupply
                                if Data.release_date[city_change2]==0:
                                    newsolution[0][i][0][1].append(city_change2)
                                else:
                                    p = Function.max_release_date(newsolution[0][i][0][1])/Data.release_date[city_change2]
                                    if p < 1 - beta:
                                        newsolution[0][i] = rearrange_package(city_change2, newsolution[0][i], j)
                                    else: newsolution[0][i][0][1].append(city_change2)
                            else:
                                newsolution[0][i] = rearrange_package(city_change2, newsolution[0][i], j)
                            
                            for k2 in range (len(package_needchange2)):
                                newsolution[0][m] = rearrange_package(package_needchange2[k2], newsolution[0][m], Function.find_position(package_needchange2[k2], newsolution[0])[1])

                            for u in range (len(newsolution[1])):
                                if newsolution[1][u][0][0] == city_change1:
                                    newsolution[1].pop(u)
                                    break
                            for v in range (len(newsolution[1])):
                                if newsolution[1][v][0][0] == city_change2:
                                    newsolution[1].pop(v)
                                    break
                            
                            newsolution = fix_drone_queue(newsolution)

                            ''' print(newsolution[0][0])
                            print(newsolution[0][1])
                            print(newsolution[0][2])
                            print(newsolution[1])
                            print(city_change1,city_change2 , i, j, m, n)
                            print(newsolution)
                            print("----------------------------------------")'''
                            pack_child = []
                            pack_child.append(newsolution)
                            a, b = Function.fitness(newsolution)
                            pack_child.append([a, b])
                            pack_child.append([city_change1, city_change2])
                            pack_child.append(i)
                            pack_child.append(m)
                            neighbors.append(pack_child)


    return neighbors

def Neighborhood_move_1_1_ver2(solution):
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

def Neighborhood_two_opt(solution):
    neighborhood = []
    for i in range(len(solution[0])):
        for j in range(len(solution[0][i])):
            for k in range(j + 4, len(solution[0][i])):
                new_solution = copy.deepcopy(solution)
                
                pre_drop_package = []
                for ii in range(j+1, k):
                    pre_drop_package.append(new_solution[0][i][ii][0])
                    for iii in reversed(range(len(new_solution[0][i][ii][1]))):
                        pack = new_solution[0][i][ii][1][iii]
                        if pack not in pre_drop_package:
                            pre_drop_package.append(pack)
                        new_solution[0][i][ii][1].pop(iii)
                
                new_solution[0][i] = new_solution[0][i][:j+1] + new_solution[0][i][j+1:k][::-1] + new_solution[0][i][k:]
                
                drop_package = []
                
                for ii in range(len(new_solution[0][i])):
                    
                    city = new_solution[0][i][ii][0]
                    if city in pre_drop_package:
                        drop_package.append(city)
                        
                    for iii in reversed(range(len(new_solution[0][i][ii][1]))):
                        pack = new_solution[0][i][ii][1][iii]
                        if pack in pre_drop_package:
                            new_solution[0][i][ii][1].pop(iii)
                
                for ii in reversed(range(len(new_solution[1]))):
                    for iii in reversed(range(len(new_solution[1][ii]))):
                        for jj in reversed(range(len(new_solution[1][ii][iii][1]))):
                            pack = new_solution[1][ii][iii][1][jj]
                            if pack in pre_drop_package:
                                new_solution[1][ii][iii][1].pop(jj)
                        if new_solution[1][ii][iii][1] == []:
                            new_solution[1][ii].pop(iii)
                    if new_solution[1][ii] == []:
                        new_solution[1].pop(ii)
                
                for ii in range(len(drop_package)):
                    new_solution = Neighborhood.findLocationForDropPackage(new_solution, i, drop_package[ii])
                    
                # print(new_solution)
                # print(Function.fitness(new_solution)[0])
                # print(Function.Check_if_feasible(new_solution))
                # print(len(neighborhood))
                # print("---------------------------") 
                
                pack_child = []
                pack_child.append(new_solution)
                a, b, c = Function.fitness(new_solution)                
                pack_child.append([a, b, c])
                pack_child.append([solution[0][i][j][0], solution[0][i][k][0]])
                pack_child.append(i)
                pack_child.append(i)
                neighborhood.append(pack_child)
                
    return neighborhood

def Neighborhood_move_2_1(solution):
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
                    pre_drop_package = []
                    pre_drop_package.append(city_change1)
                    pre_drop_package.append(city_change2)
                    pre_drop_package.append(city_change3)
                    for ii in range(len(new_solution[0][i][j][1])):
                        city = new_solution[0][i][j][1][ii]
                        if city not in pre_drop_package:
                            pre_drop_package.append(city)
                    for ii in range(len(new_solution[0][i][j+1][1])):
                        city = new_solution[0][i][j+1][1][ii]
                        if city not in pre_drop_package:
                            pre_drop_package.append(city)
                    for ii in range(len(new_solution[0][k][l][1])):
                        city = new_solution[0][k][l][1][ii]
                        if city not in pre_drop_package:
                            pre_drop_package.append(city)
                    if i == k:
                        for ii in range(len(new_solution[0][i])):
                            for iii in reversed(range(len(new_solution[0][i][ii][1]))):
                                city = new_solution[0][i][ii][1][iii]
                                if city == city_change1 or city == city_change2 or city == city_change3:
                                    new_solution[0][i][ii][1].pop(iii)
                    else:
                        for ii in range(len(new_solution[0][i])):
                            for iii in reversed(range(len(new_solution[0][i][ii][1]))):
                                city = new_solution[0][i][ii][1][iii]
                                if city == city_change1 or city == city_change2:
                                    new_solution[0][i][ii][1].pop(iii)
                        for ii in range(len(new_solution[0][k])):
                            for iii in reversed(range(len(new_solution[0][k][ii][1]))):
                                city = new_solution[0][k][ii][1][iii]
                                if city == city_change3:
                                    new_solution[0][k][ii][1].pop(iii)
                                    
                    for ii in reversed(range(len(new_solution[1]))):
                        for jj in reversed(range(len(new_solution[1][ii]))):
                            city = new_solution[1][ii][jj][0]
                            if city == city_change1 or city == city_change2 or city == city_change3:
                                new_solution[1][ii].pop(jj)
                            else:
                                for kk in reversed(range(len(new_solution[1][ii][jj][1]))):
                                    pack = new_solution[1][ii][jj][1][kk]
                                    if pack in pre_drop_package:
                                        new_solution[1][ii][jj][1].pop(kk)
                                if new_solution[1][ii][jj][1] == []:
                                    new_solution[1][ii].pop(jj)
                        if new_solution[1][ii] == []:
                            new_solution[1].pop(ii) 
                            
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
                    
                    drop_package1 = []
                    drop_package2 = []
                    
                    for ii in range(len(new_solution[0][i])):
                        city = new_solution[0][i][ii][0]
                        if city in pre_drop_package:
                            drop_package1.append(city)
                        for jj in reversed(range(len(new_solution[0][i][ii][1]))):
                            pack = new_solution[0][i][ii][1][jj]
                            if pack in pre_drop_package:
                                new_solution[0][i][ii][1].pop(jj)
                    if i != k:
                        for ii in range(len(new_solution[0][k])):
                            city = new_solution[0][k][ii][0]
                            if city in pre_drop_package:
                                drop_package2.append(city)
                            for jj in reversed(range(len(new_solution[0][k][ii][1]))):
                                pack = new_solution[0][k][ii][1][jj]
                                if pack in pre_drop_package:
                                    new_solution[0][k][ii][1].pop(jj)
                    
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
                    pack_child.append([city_change1, city_change2, city_change3])
                    pack_child.append(i)
                    pack_child.append(k)
                    neighborhood.append(pack_child)
                    
    return neighborhood

def Neighborhood_two_opt_tue(solution):
    neighborhood = []
    for i in range(len(solution[0])):
        for j in range(1, len(solution[0][i]) - 1):
            for k in range(i + 1, len(solution[0])):
                for l in range(1, len(solution[0][k]) - 1):
                    new_solution = copy.deepcopy(solution)
                    pre_drop_package = []
                    drop_city = []
                    for ii in range(j + 1, len(new_solution[0][i])):
                        pre_drop_package.append(new_solution[0][i][ii][0])
                        drop_city.append(new_solution[0][i][ii][0])
                        for jj in range(len(new_solution[0][i][ii][1])):
                            city = new_solution[0][i][ii][1][jj]
                            if city not in pre_drop_package:
                                pre_drop_package.append(city)
                        new_solution[0][i][ii][1] = []
                                
                    for ii in range(l + 1, len(new_solution[0][k])):
                        drop_city.append(new_solution[0][k][ii][0])
                        pre_drop_package.append(new_solution[0][k][ii][0])
                        for jj in range(len(new_solution[0][k][ii][1])):
                            city = new_solution[0][k][ii][1][jj]
                            if city not in pre_drop_package:
                                pre_drop_package.append(city)
                        new_solution[0][k][ii][1] = []

                    for ii in range(j + 1):
                        for jj in reversed(range(len(new_solution[0][i][ii][1]))):
                            city = new_solution[0][i][ii][1][jj]
                            if city in pre_drop_package:
                                new_solution[0][i][ii][1].pop(jj)
                                
                    for ii in range(l + 1):
                        for jj in reversed(range(len(new_solution[0][k][ii][1]))):
                            city = new_solution[0][k][ii][1][jj]
                            if city in pre_drop_package:
                                new_solution[0][k][ii][1].pop(jj)
                    
                    for ii in reversed(range(len(new_solution[1]))):
                        for jj in reversed(range(len(new_solution[1][ii]))):
                            city = new_solution[1][ii][jj][0]
                            
                            if city in drop_city:
                                new_solution[1][ii].pop(jj)
                            else:
                                for kk in reversed(range(len(new_solution[1][ii][jj][1]))):
                                    pack = new_solution[1][ii][jj][1][kk]
                                    if pack in pre_drop_package:
                                        new_solution[1][ii][jj][1].pop(kk)
                                if new_solution[1][ii][jj][1] == []:
                                    new_solution[1][ii].pop(jj)
                        if new_solution[1][ii] == []:
                            new_solution[1].pop(ii)
                            
                    temp = new_solution[0][i][j+1:]
                    new_solution[0][i] = new_solution[0][i][:j+1] + new_solution[0][k][l+1:]
                    new_solution[0][k] = new_solution[0][k][:l+1] + temp
                    
                    drop_package1 = []
                    drop_package2 = []
                    for ii in range(len(new_solution[0][i])):
                        city = new_solution[0][i][ii][0]
                        if city in pre_drop_package:
                            drop_package1.append(city)
                            
                    for ii in range(len(new_solution[0][k])):
                        city = new_solution[0][k][ii][0]
                        if city in pre_drop_package:
                            drop_package2.append(city)
                    
                    for ii in range(len(drop_package1)):
                        new_solution = Neighborhood.findLocationForDropPackage(new_solution, i, drop_package1[ii])
                    for ii in range(len(drop_package2)):
                        new_solution = Neighborhood.findLocationForDropPackage(new_solution, k, drop_package2[ii])
                        
                    # print(new_solution[0][0])
                    # print(new_solution[0][1])
                    # print(new_solution[1])
                    # print(Function.fitness(new_solution)[0])
                    # print(Function.Check_if_feasible(new_solution))
                    # print(len(neighborhood))
                    # print("---------------------------") 
                    
                    pack_child = []
                    pack_child.append(new_solution)
                    a, b, c = Function.fitness(new_solution)                
                    pack_child.append([a, b, c])
                    pack_child.append([solution[0][i][j][0], solution[0][k][l][0]])
                    pack_child.append(i)
                    pack_child.append(k)
                    neighborhood.append(pack_child)
    return neighborhood
