#! /usr/bin/python
import math
inf = -1
first = -1
second = -1
start = 0
last = 0
ranges=[inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, 1.3752853870391846, 1.3316771984100342, 1.2908449172973633, 1.2525348663330078, 1.216523289680481, 1.1826122999191284, 1.1506260633468628, 1.1204077005386353, 1.0918171405792236, 1.0647286176681519, 1.0390291213989258, 1.0146164894104004, 0.9913986325263977, 0.9692918658256531, 0.9482203125953674, 0.9281147122383118, 0.9089118838310242, 0.8905541896820068, 0.8729885816574097, 0.8561666011810303, 0.8400435447692871, 0.8245782256126404, 0.8097327351570129, 0.7954719662666321, 0.7817633748054504, 0.7685769200325012, 0.7558846473693848, 0.7436606287956238, 0.7318805456161499, 0.7205220460891724, 0.7095639705657959, 0.6989867091178894, 0.6887720227241516, 0.6789025664329529, 0.6693623065948486, 0.6601361036300659, 0.651209831237793, 0.6425700783729553, 0.6342043280601501, 0.6261007785797119, 0.6182482242584229, 0.6106362342834473, 0.6032549142837524, 0.5960947871208191, 0.5891470313072205, 0.5824033617973328, 0.5758557319641113, 0.5694966316223145, 0.5633189678192139, 0.5573159456253052, 0.5514811277389526, 0.5458084344863892, 0.5402920842170715, 0.5349264740943909, 0.5297064781188965, 0.5246269106864929, 0.5196831822395325, 0.5148706436157227, 0.5101850032806396, 0.5102222561836243, 0.5146505236625671, 0.5191962718963623, 0.5238633751869202, 0.5286558270454407, 0.5335778594017029, 0.5386338829994202, 0.543828547000885, 0.5491666793823242, 0.5546534061431885, 0.560293972492218, 0.566094160079956, 0.5720597505569458, 0.5781969428062439, 0.5845124125480652, 0.5910128951072693, 0.5977057218551636, 0.6045984625816345, 0.6116993427276611, 0.619016706943512, 0.6265596747398376, 0.6343377232551575, 0.6423609256744385, 0.6506399512290955, 0.6591861248016357, 0.6680114269256592, 0.6771286129951477, 0.6865512132644653, 0.6962935924530029, 0.7063711881637573, 0.7168001532554626, 0.7275981307029724, 0.7387835383415222, 0.7503764033317566, 0.7623980045318604, 0.7748711109161377, 0.7878202795982361, 0.8012717366218567, 0.8152537941932678, 0.8297969102859497, 0.8449339270591736, 0.8607003688812256, 0.8771345615386963, 0.8942781686782837, 0.9121761918067932, 0.9308777451515198, 0.9504362344741821, 0.9709098935127258, 0.9923624396324158, 1.0148634910583496, 1.038489818572998, 1.06332528591156, 1.0894629955291748, 1.117005467414856, 1.1460663080215454, 1.1767717599868774, 1.209262728691101, 1.2436963319778442, 1.280248999595642, 1.3191189765930176, 1.360530138015747, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf, inf]

count_inf = len(ranges)
print("Длина массива", count_inf)


# for i in range(0,len(ranges)):
#     if ranges[i]!=-1:
#         second = first
#         first = i
#         if second ==-1 and first !=-1:
#             start = i

#search min
min=10
index_min=-1
for i in range(0,len(ranges)):
    if ranges[i]!=-1 and ranges[i]<min :
        min=ranges[i]
        index_min=i
print("min and index: ", min, index_min)
print("angle", math.atan2(0.2,0.4))
print("angle in degrees ", math.atan2(0.2,0.4)*180/math.pi)
# print("middle, started:",start)
# i=start
# while (ranges[i]!=-1):
#     last = i
#     i=i+1
for i in range(0,len(ranges)):
    if ranges[i]==1.3914719820022583:
        print("start: ", i)
        break

# print("middle, finish:",last)

number_robot_str='1'
print('/robot'+number_robot_str+'/cmd_vel')
