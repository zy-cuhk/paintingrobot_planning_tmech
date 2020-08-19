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
                if index % 2 == 1:
                    if j==index-1:
                        dis=0
                else:
                    if j==index+1:
                        dis=0
                if dis < dis_min:
                    dis_min = dis
                    index1 = j
                # if index==2:
                #     print("dis is: ",j,dis,index)
        print("the choosen point is:",index1)
        solution[i+1] = index1
        flag[index1] = 1
        index=index1
        obj += dis_min
        i += 1

    obj += length(points[0], points[index])

    output_data = '%.2f' % obj + '\n'
    output_data += ' '.join(map(str, solution))

    return output_data

def main():
    # input: waypoints
    x_const=0.5
    y_init=0.5
    z_init=0.5
    z_interval=0.1
    points_num=23
    point_list=np.zeros((points_num,3))
    line_num=int(points_num/2)
    for i in range(line_num):
        point_list[2*i,0]=x_const
        point_list[2*i,1]=y_init
        point_list[2*i,2]=z_init-i*z_interval

        point_list[2*i+1,0]=x_const
        point_list[2*i+1,1]=-y_init
        point_list[2*i+1,2]=z_init-i*z_interval

    print("the points list is:",point_list)
    # output: waypoints list with order
    output_data=solve_it(point_list)
    print(output_data)


if __name__ == "__main__":
    main()


