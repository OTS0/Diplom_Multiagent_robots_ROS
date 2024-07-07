#!/usr/bin/env python3
import math  

def analyze_tasks(task_start,task_finish):
    count_tasks=len(task_start)
    sum_all=0
    min_time=math.inf
    max_time=0
    for i in range(count_tasks):
        time_goal=(task_start[i]-task_finish[i])/60
        sum_all=sum_all+time_goal
        if time_goal<min_time:
            min_time=time_goal
        if time_goal>max_time:
            max_time=time_goal
    middle=sum_all/count_tasks
    print("Cреднее время выполнения задачи, минимальное и максимальное время выполнения в минутах {0} {1} {2}".format(middle, min_time, max_time))
    print("количество выполненных задач {0}".format(count_tasks))

task_start=[201, 213, 214, 214, 217, 221, 243, 253, 266, 285, 343, 411, 413, 627, 674, 1229, 50, 53, 63, 68, 73, 81, 91, 95, 99, 115, 125, 129, 136, 138, 154, 155, 160, 165, 170, 195]
task_finish=[137, 17, 27, 116, 69, 51, 157, 203, 21, 23, 126, 64, 54, 101, 171, 92, 12, 19, 18, 24, 13, 14, 82, 19, 20, 24, 97, 16, 23, 20, 15, 21, 130, 16, 139, 155]
count_tasks=len(task_start)
print(len(task_start))
print(len(task_finish))
print("Время работы (мин):", 23*40/count_tasks)
analyze_tasks(task_start, task_finish)




