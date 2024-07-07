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

task_start=[108, 130, 106, 81, 70, 66, 65, 61, 157, 157, 178, 168, 164, 201, 204, 206, 210, 261, 258, 279, 307, 323, 337, 533, 861, 786, 744, 1049]
task_finish=[68, 19, 26, 25, 19, 24, 24, 18, 62, 23, 21, 21, 24, 107, 82, 130, 169, 27, 158, 211, 262, 126, 308, 423, 324, 371, 168, 280]
print(len(task_start))
print(len(task_finish))
print("Время работы (мин):", 26)
analyze_tasks(task_start, task_finish)




