#!/usr/bin/env python3
import math  

def analyze_tasks(task_start,task_finish):
    count_tasks=16
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

task_start=[89, 97 ,110 ,117 ,122 ,147, 156, 157, 191, 274, 274, 264, 206, 319, 494, 652]
task_finish=[14, 17, 16, 18, 15, 124, 98, 19, 158, 20, 16, 19, 20, 191, 91, 549]
print("Время работы (мин):", 25)
analyze_tasks(task_start, task_finish)




