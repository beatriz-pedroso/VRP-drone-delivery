import gurobipy as gp
from math import *
import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D


# import from excel file the nodes coordinates and the correspondent demand
nodes_coordinates = pd.read_excel('Nodes_Information.xlsx', keep_default_na=False)
X = nodes_coordinates['X'].tolist()
Y = nodes_coordinates['Y'].tolist()
q = nodes_coordinates['Demand'].tolist()
t_lower = nodes_coordinates['lower_time'].tolist()
t_upper = nodes_coordinates['upper_time'].tolist()


#Number of customers to be served + depot
N = len(X)
#Number of homogenous trucks with drones
V = 4

##########################################################################################
#constants definition
#Travel cost of trucks and drones per km and basis cost of using a truck equipped with a drone
C_T = 25
C_D = 1
#Load capacity of trucks and drones [kg]
Q_T = 500
Q_D = 3
#Average speed of trucks and drones [Km/h]
S_T = 60
S_D = 80
#Large number for either-or constraints
M = 100


#definition of the matrix that stores the distances and travel times between every node
d = [[0 for i in range(N)] for j in range(N)]
t_travel_t = [[0 for i in range(N)] for j in range(N)]
t_travel_d = [[0 for i in range(N)] for j in range(N)]


#distances and time computation
for i in range(N):
    d[i][i]=0
    for j in range(i+1,N):
        d_aux = sqrt(pow(X[i]-X[j], 2) + pow(Y[i]-Y[j],2))
        # Result with 2 decimals
        d[i][j] = round(d_aux,2)
        d[j][i] = d[i][j]

for i in range(N):
	for j in range(N):
		t_travel_t[i][j] = d[i][j] / S_T
		t_travel_d[i][j] = d[i][j] / S_D

##########################################################################################
# Start modelling optimization problem
model = gp.Model()

##########################################################################################
# VARIABLES
A = [(v, i, j) for v in range(V) for i in range(N) for j in range(N)]
B = [(v, i, j, k) for v in range(V) for i in range(N) for j in range(N) for k in range(N)]
C = [(v, i) for v in range(V) for i in range(N)]
D = [(v) for v in range(V)]
#x=1 if truck v goes from node i to node j
x = model.addVars(A, vtype=gp.GRB.BINARY)
#y=1 if drone v (equipped on truck v) is lauched on node i, serves customer j and is retrieved at node k
y = model.addVars(B, vtype=gp.GRB.BINARY)
#Arrival time of truck v at node i
t_t = model.addVars(C, vtype=gp.GRB.CONTINUOUS)
#Arrival time of drone v at node i
t_d = model.addVars(C, vtype=gp.GRB.CONTINUOUS)
#interval of time the truck/drone has to wait for the drone/truck
t_t_wait = model.addVars(C, vtype=gp.GRB.CONTINUOUS)
t_d_wait = model.addVars(C, vtype=gp.GRB.CONTINUOUS)
#time each vehicle takes to finish the route
t_total = model.addVars(D, vtype=gp.GRB.CONTINUOUS)
#maximum route time of all vehicles
t_total_max = model.addVar(lb=0, vtype=gp.GRB.CONTINUOUS)


#initialize time
for v in range(V):
	t_t[v,0]=0
	t_d[v,0]=0


model.update()


##########################################################################################
#OBJECTIVES

#The objectives were defned in hierarchical order

#minimize total time
obj1 = t_total_max
#minimize total delivery cost
obj2 = sum(x[v,i,j]*d[i][j]*C_T for v in range(V) for i in range(N) for j in range(N)) + sum(y[v,i,j,k]*(d[i][j]+d[j][k])*C_D for v in range(V) for i in range(N) for j in range(N) for k in range(N))
#minimize the total distance ( less energy consumption) - only trucks (drones electric)
obj3 = sum(d[i][j]*x[v,i,j] for v in range(V) for i in range(N) for j in range(N))

model.setObjectiveN(obj1, 0, 2)
model.setObjectiveN(obj2, 1, 1)
model.setObjectiveN(obj3, 2, 0)

model.ModelSense = gp.GRB.MINIMIZE


##########################################################################################
#CONSTRAINTS
#C1 (8) - all customers are served by trucks or drones
for j in range(1,N):
	model.addConstr(sum(x[v,i,j] for v in range(V) for i in range(N)) + sum(y[v,i,j,k] for v in range(V) for i in range(N) for k in range(N)) == 1, name='all_served') #C1

#C2 - trucks cannot go to the same node it departed from, the same for drones
for j in range(N): #C2
	model.addConstr(sum(x[v,j,j] for v in range(V)) == 0, name='x_different_node')
	model.addConstr(sum(y[v,j,j,k] for v in range(V) for k in range(N)) == 0, name='y_different_nodeiix')
	model.addConstr(sum(y[v,i,j,j] for v in range(V) for i in range(N)) == 0, name='y_different_nodexjj')
	model.addConstr(sum(y[v,i,j,i] for v in range(V) for i in range(N)) == 0, name='y_different_nodeixi')

#C3 - drones cannot be launched in the depot
for v in range(V):
	for j in range(N):
		for k in range(N):
			model.addConstr(y[v,0,j,k] == 0, name='y_not_lauched_depot')

#C4 - flow conservation - the # of trucks arriving at a node has to be the same as the # of trucks leaving
for v in range(V):
	for j in range(N):
		model.addConstr(sum(x[v,i,j] for i in range(N)) == sum(x[v, j, i] for i in range(N)), name='flow_conservation') #C4

#C5 (9) - each truck has to leave the depot
#C6 (10) - each truck has to arrive at the depot
#C7 (15) - ensures each truck does not exceed its load capacity during delivery
#C8 (12) / C9 (13) - ensures each drone is lauched or retrieved at most once at all customers and depot nodes

for v in range(V):
	model.addConstr(sum(x[v,0,j] for j in range(N)) == 1, name='truck_leave_depot') #C5
	model.addConstr(sum(x[v,i,0] for i in range(N)) == 1, name='truck_arrive_depot') #C6
	model.addConstr(sum(x[v,i,j]*q[j] for i in range(N) for j in range(N)) + sum(y[v,i,j,k]*q[j] for i in range(N) for j in range(N) for k in range(N)) <= Q_T, name='truck_load') #C7

	for i in range(N):
 		model.addConstr(sum(y[v,i,j,k] for j in range(N) for k in range(N)) <= 1, name='drone_lau_ret_max1_nodei1') #C8

for v in range(V):	
	for k in range(N):
		model.addConstr(sum(y[v,i,j,k] 
		for i in range(N) for j in range(N)) <= 1, name='drone_lau_ret_max1_nodei2') #C9


#C10 (16) - if the drone is launched at node i and retrieved at node k, the truck must pass through both nodes
#and it has to return to the node the truck is going to serve next
for v in range(V):	
	for i in range(N):		
		for j in range(N):
			for k in range(N):
				model.addConstr(y[v,i,j,k] <= x[v,i,k] + M * (1 - y[v,i,j,k]), name='drone_return_next_node_truck') #C10

#C11 (14) - ensures that each drone is not loaded beyond its load capacity during flight
for v in range(V):
	for j in range(N):
		model.addConstr(sum(y[v,i,j,k]*q[j] for i in range(N) for k in range(N)) <= Q_D, name='drone_load') #C11


# C12 - time windows
for v in range(V):
	for i in range(1,N):
		model.addConstr(t_t[v,i] <= t_upper[i], name='truck_upper_lim')
		model.addConstr(t_t[v,i] >= t_lower[i], name='truck_lower_lim')
		model.addConstr(t_d[v,i] <= t_upper[i], name='drone_upper_lim')
		model.addConstr(t_d[v,i] >= t_lower[i], name='drone_lower_lim')


#C13-C16 (18-21) - ensures the arrival times of each truck and its corresponding drone at the launch and retrieval nodes are synchronized
for v in range(V):
	for i in range(N):
		model.addConstr(t_d[v,i] >= t_t[v,i] - M * (1 - sum(y[v,i,j,k] for j in range(N) for k in range(N))), name='arrival_times_sinc1') #C13
		model.addConstr(t_d[v,i] <= t_t[v,i] + M * (1 - sum(y[v,i,j,k] for j in range(N) for k in range(N))), name='arrival_times_sinc2') #C14
		
	for k in range(N):
		model.addConstr(t_d[v,k] >= t_t[v,k] - M * (1 - sum(y[v,i,j,k] for i in range(N) for j in range(N))), name='arrival_times_sinc3') #C15
		model.addConstr(t_d[v,k] <= t_t[v,k] + M * (1 - sum(y[v,i,j,k] for i in range(N) for j in range(N))), name='arrival_times_sinc4') #C16


#C17-C19 (22-24) - ensure that the arrival times of the trucks and drones are reasonable during movement
#waiting times were added if the trucks need to wait for the dones before resuming the route
for v in range(V):
	for i in range(N):
		for j in range(1,N):
			model.addConstr(t_t[v,j] >= t_t[v,i] + t_t_wait[v,i] + t_travel_t[i][j] - M * (1 - x[v,i,j]), name='truck_arrival_time_node') #C17
			for k in range (N):
				model.addConstr(t_d[v,j] >= t_d[v,i] + t_d_wait[v,i] + t_travel_d[i][j] - M * (1 - y[v,i,j,k]), name='drone1_arrival_time_node') #C18

for v in range(V):
	for j in range(N):
		for k in range(N):
			for i in range (N):
				model.addConstr(t_d[v,k] >= t_d[v,j] + t_d_wait[v,j] +  t_travel_d[j][k] - M * (1 - (y[v,i,j,k])), name='drone2_arrival_time_node') #C19

#------------------
#compute total time
for v in range(V):
	for i in range(N):
		model.addConstr(t_total[v] <= t_t[v,i] + t_travel_t[i][0] + M*(1-x[v,i,0]))
		model.addConstr(t_total[v] >= t_t[v,i] + t_travel_t[i][0] - M*(1-x[v,i,0]))

for v in range(V):
	model.addConstr(t_total_max - t_total[v] >= 0)


#model.setParam('TimeLimit', 1200)

##########################################################################################
model.optimize()


#Printing the solution

print("Maximum time a vehicle takes to make a route")
print(t_total_max)

active_edges_t = [a for a in A if x[a].x > 0.99]
active_edges_d = [b for b in B if y[b].x > 0.99]


print("Route of the trucks")
for v, i, j in active_edges_t:
	print(f"\t\t  V{v} N{i}->N{j} :-- x[{v},{i},{j}] = {x[v,i,j]}")


print("Route of the drones")	
for v, i, j, k in active_edges_d:
	print(f"\t\t V{v} N{i}->N{j}->N{k} :-- y[{v},{i},{j},{k}] = {y[v,i,j,k]}")


print("Truck times")
for v in range(V):	
	print(f"Vehicle {v}")
	for i in range(N):
		print(f"\t\t N{i} truck t_t[{v},{i}] = {t_t[v,i]}")


print("Drone times")
for v in range(V):	
	print(f"Vehicle {v}")
	for i in range(N):
		print(f"\t\t N{i} drone t_d[{v},{i}]  = {t_d[v,i]}")

##########################################################################################
#Plot figure

plt.figure(figsize=(8,8))

# setting x and y axis range
plt.ylim(-50,50)
plt.xlim(-50,50)

plt.xlabel('x [Km]')
plt.ylabel('y [Km]')

for i in range(N):    
    if i == 0:
        plt.scatter(X[i], Y[i], c='grey', s=200)
        plt.text(X[i], Y[i], "depot", fontsize=12)
    else:
        plt.scatter(X[i], Y[i], c='cyan', s=200)
        plt.text(X[i], Y[i], str(i), fontsize=12)

count = 0 #to change the colors of the lines depending on the vehicle
for v, i, j in active_edges_t:
	if v == 0:
		plt.plot([X[i], X[j]], [Y[i], Y[j]], c="green")
	if v == 1:
		plt.plot([X[i], X[j]], [Y[i], Y[j]], c="magenta")
	if v == 2:	
		plt.plot([X[i], X[j]], [Y[i], Y[j]], c="red")
	if v == 3:	
		plt.plot([X[i], X[j]], [Y[i], Y[j]], c="orange")

count = 0 #to change the colors of the lines depending on the vehicle
for v, i, j, k in active_edges_d:
	if v == 0:
		plt.plot([X[i], X[j]], [Y[i], Y[j]], '--', c="green")
		plt.plot([X[j], X[k]], [Y[j], Y[k]], '--', c="green")
	if v == 1:
		plt.plot([X[i], X[j]], [Y[i], Y[j]], '--', c="magenta")
		plt.plot([X[j], X[k]], [Y[j], Y[k]], '--', c="magenta")
	if v == 2:
		plt.plot([X[i], X[j]], [Y[i], Y[j]], '--', c="red")
		plt.plot([X[j], X[k]], [Y[j], Y[k]], '--', c="red")
	if v == 3:
		plt.plot([X[i], X[j]], [Y[i], Y[j]], '--', c="orange")
		plt.plot([X[j], X[k]], [Y[j], Y[k]], '--', c="orange")

custom_lines = [Line2D([0], [0], color='green', lw=3, linestyle='-'),
				Line2D([0], [0], color='green', lw=3, linestyle='--'),
				Line2D([0], [0], color='magenta', lw=3, linestyle='-'),
				Line2D([0], [0], color='magenta', lw=3, linestyle='--'),
				Line2D([0], [0], color='red', lw=3, linestyle='-'),
				Line2D([0], [0], color='red', lw=3, linestyle='--'),
				Line2D([0], [0], color='orange', lw=3, linestyle='-'),
				Line2D([0], [0], color='orange', lw=3, linestyle='--')]				

plt.legend(custom_lines, ['Truck 0', 'Drone 0', 'Truck 1', 'Drone 1', 'Truck 2', 'Drone 2', 'Truck 3', 'Drone 3'])

plt.show()


