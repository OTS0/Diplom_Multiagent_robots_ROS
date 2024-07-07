#!/usr/bin/env python3
import math  

def analyze_tasks(task_start,task_finish):
    count_tasks=len(task_start)
    sum_all=0
    min_time=math.inf
    max_time=0
    list_goal=list()
    print(len(task_start))
    print(len(task_finish))
    for i in range(count_tasks):
        time_goal=(task_start[i]-task_finish[i])/60
        list_goal.append(time_goal)
        sum_all=sum_all+time_goal
        if time_goal<min_time:
            min_time=time_goal
        if time_goal>max_time:
            max_time=time_goal
    # print(list_goal)
    middle=sum_all/count_tasks
    print("Cреднее время выполнения задачи, минимальное и максимальное время выполнения в минутах {0} {1} {2}".format(middle, min_time, max_time))
    print("количество выполненных задач {0}".format(count_tasks))

task_finish=[18, 109, 25, 187, 36, 74, 285, 21, 55]
task_start=[83, 185, 185, 198, 265, 283, 291, 280, 301]
count_tasks=len(task_start)
print(task_start[count_tasks-1]//60)
print("Время работы (мин):", task_start[count_tasks-1]//60*40/count_tasks)
analyze_tasks(task_start, task_finish)




