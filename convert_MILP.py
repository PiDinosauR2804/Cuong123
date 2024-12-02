import Function
import Data
solution = [[], []]

def tranfer(arr, k):
    solution[0].append([])
    solution[0][k].append([0, []])
    for i in range(len(arr)):
        solution[0][k][0][1].append(arr[i])
    for i in range(1, len(arr) - 1):
        solution[0][k].append([arr[i], []])
    return solution

def add(solution, package, location):
    for i in range(len(solution[0])):
        if package in solution[0][i][0][1]:
            solution[0][i][0][1].remove(package)
    for j in range(len(solution[0])):
        for i in range(1, len(solution[0][j])):
            if solution[0][j][i][0] == location:
                solution[0][j][i][1].append(package)
    add = False
    for i in range(len(solution[1])):
        for j in range(len(solution[1][i])):
            if solution[1][i][j][0] == location:
                solution[1][i][j][1].append(package)
                add = True
    if not add:
        solution[1].append([[location, [package]]])
    return solution

a = tranfer([
            0,
            2,
            1,
            9,
            8,
            0
        ], 0)
a = tranfer([
            0,
            7,
            3,
            4,
            5,
            10,
            6,
            0
        ], 1)
# print(a)
# a = add(a, 6, 9)
# a = add(a, 9, 9)
# a = add(a, 14, 9)
# a = add(a, 7, 7)
# a = add(a, 10, 7)
# a = add(a, 15, 15)
# a = add(a, 12, 15)
# a = add(a, 8, 13)




a = add(a, 6, 6)

print(a)
print(Function.fitness(a))

