# -*- coding: utf-8 -*-
import math
import numpy as np

def length(point1, point2):
    return math.sqrt((point1[0] - point2[0]) ** 2 + (point1[1] - point2[1]) ** 2 + (point1[2] - point2[2]) ** 2)


def solve_it(points):
    # the inputs are:
    nodeCount=len(points)
    # greedy algorithm
    flag = [0] * nodeCount  # noted the node
    flag[0] = 1
    solution = [0] * nodeCount
    obj = 0
    i = 0
    index = 0
    #
    while i < nodeCount-1:
        dis_min = float("inf")
        for j in range(0, nodeCount):
            if flag[j] == 0 and j != index:
                dis = length(points[index], points[j])
                if dis < dis_min:
                    dis_min = dis
                    index = j
        print("the choosen point is:",index)
        solution[i+1] = index
        flag[index] = 1
        obj += dis_min
        i += 1
    obj += length(points[0], points[index])
    output_data = '%.2f' % obj + '\n'
    output_data += ' '.join(map(str, solution))

    return output_data

def main():
    # input: waypoints
    points_list=np.array([[2066, 2333, 0],[935, 1304, 0], [1270, 200, 0],[1389, 700, 0]])
    print("the points list is:",points_list)
    # output: waypoints list with order
    output_data=solve_it(points_list)
    print(output_data)


if __name__ == "__main__":
    main()


