#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from move_base_msgs.msg import MoveBaseActionGoal
from geometry_msgs.msg import PoseWithCovarianceStamped
from geometry_msgs.msg import PoseStamped
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from nav_msgs.msg import Path
import pickle
import math
import random
from collections import defaultdict, deque # za Dijkstra
import numpy as np

# globalne spremenljivke
global t
global i_count
global frequency
global va_xy_z

t = 0.
i_count = 0
frequency = 50 #Hz

va_xy_z = np.array([0.,0.,0.,0.]) # začetni vektor pomika

robot_position_x = None
robot_position_y = None
robot_orientation_yaw = None

goal_x = None
goal_y = None

map_data = None
map_info = None

path = None # Ko se izračuna pot, je ta v obliki [(x0,y0),(x1,y1),(x2,y2),...]
current_path_point_id = None # ta določa trenutni segment poti (vrednost pomeni indeks izhodiščne točke segmenta)

distance_to_goal_thr = 0.3 # [m]

Kx = 0.8 
Ky = 1.5
Kth = 10 # Glejte zapiske predavanj, poglavje 3.3 (Vodenje po zvezni trajektoriji)

class Graph:
	# Vir: https://gist.github.com/econchick/4666413
	def __init__(self):
		self.nodes = set()
		self.edges = defaultdict(list)
		self.distances = {}

	def add_node(self, value):
		self.nodes.add(value)

	def add_edge(self, from_node, to_node, distance):
		self.edges[from_node].append(to_node)
		self.edges[to_node].append(from_node)
		self.distances[(from_node, to_node)] = distance

def euler_from_quaternion(x, y, z, w):
		"""
		Convert a quaternion into euler angles (roll, pitch, yaw)
		roll is rotation around x in radians (counterclockwise)
		pitch is rotation around y in radians (counterclockwise)
		yaw is rotation around z in radians (counterclockwise)
		"""
		t0 = +2.0 * (w * x + y * z)
		t1 = +1.0 - 2.0 * (x * x + y * y)
		roll_x = math.atan2(t0, t1)
		t2 = +2.0 * (w * y - z * x)
		t2 = +1.0 if t2 > +1.0 else t2
		t2 = -1.0 if t2 < -1.0 else t2
		pitch_y = math.asin(t2)
		t3 = +2.0 * (w * z + x * y)
		t4 = +1.0 - 2.0 * (y * y + z * z)
		yaw_z = math.atan2(t3, t4)
		return roll_x, pitch_y, yaw_z

def get_closest_node_id(location, nodes):
	min_distance = 1000000.0
	closest_node_id = None
	for i in range(len(nodes)):
		point1 = location
		point2 = nodes[i]
		dist = math.sqrt((point2[0]-point1[0])**2 + (point2[1]-point1[1])**2)
		if dist < min_distance:
			min_distance = dist
			closest_node_id = i
	return closest_node_id


def calculate_path():
	global path
	global current_path_point_id

	# Generiranje grafa
	map_width_px = map_info.width # število pikslov
	map_height_px = map_info.height # število pikslov
	
	resolution = 0.05 # v metrih (preberemo iz .yaml datoteke), v tej datoteki vidimo tudi, da je izhodišče mape (0.0, 0.0)

	map_width = map_width_px*resolution # v metrih
	map_height = map_height_px*resolution # v metrih

	###
	# Le enkrat generiran graf.
	###

	try: #Poskusimo odpreti v prejšnji iteraciji generiran graf.
		print("Trying to open pre-generated graph nodes, edges and distances")
		with open("/home/bizjan/ms_amr/src/advanced_move_base/maps/graph.pkl", "rb") as f:
			(nodes, edges, distances) = pickle.load(f)
	except: # Če datoteka graf.pkl ni prisotno jo generiramo z "get_graph()" funkcijo.
		print("File not found ...")
		nodes, edges, distances = get_graph(resolution, map_width_px, map_height_px, map_width, map_height)
		print("Writing generated graph nodes, edges and distances")
		with open("/home/bizjan/ms_amr/src/advanced_move_base/maps/graph.pkl", "wb") as f:
			pickle.dump((nodes, edges, distances), f)

	###
	###
	###

	# Iskanje poti (Dijkstra)
	print("Searching the shortest path")
	current_node_id = get_closest_node_id((robot_position_x, robot_position_y), nodes)
	destination_node_id = get_closest_node_id((goal_x, goal_y), nodes)
	graph = Graph() # Source: https://gist.github.com/econchick/4666413
	for i in range(len(nodes)):
		graph.add_node(i)
	for i in range(len(edges)):
		graph.add_edge(edges[i][0], edges[i][1], distances[i])
	try:
		path_nodes_ids = shortest_path(graph, current_node_id, destination_node_id)[1] # Source: https://gist.github.com/mdsrosa/c71339cb23bc51e711d8
	except:
		print("The path was not found!")
		print("(you can try again)")
		return 0
	path = [] # Ko se izračuna pot, je ta v obliki [(x0,y0),(x1,y1),(x2,y2),...]
	for node_id in path_nodes_ids:
		path.append(nodes[node_id])
	current_path_point_id = 0
	print("Path found")

def get_min_distance(point, points, stop_searching_dist=None):
	min_distance = 1000000.0
	for point2 in points:
		dist = math.sqrt((point2[0]-point[0])**2 + (point2[1]-point[1])**2)
		if dist < min_distance:
			min_distance = dist
			if stop_searching_dist != None:
				if min_distance < stop_searching_dist:
					return min_distance
	return min_distance

def get_graph(resolution, map_width_px, map_height_px, map_width, map_height):
	# Čas izvajanje te funkcije bi lahko zmanjšali npr. s tem, da se graf ustvari samo pri prvem iskanju poti, si ga zapomnemo
	# in potem tega uporabljamo naprej, ali pa npr. s paralelizacijo postopka ustvarjnja vozlišč, ali pa da poiščemo kakšno dr-
	# go metodo določanja vozlišč (verjetno že obstajajo kakšne boljše in hitrejše metode)

	# Ugotavljanje prostih in zasedenih celic
	all_cells = [] # koordinate središč vseh celic
	occupied_cells = set() # koordinate središč neprehodnih celic/celic kjer so ovire (set() podatkovna struktura je zato, da lahko hitreje preverjamo če je kakšna vrednost že v setu)
	for i in range(map_width_px):
		for j in range(map_height_px):
			x = i*resolution + resolution/2 # glejte http://users.umiacs.umd.edu/~fer/cmsc498F-828K/assignments/assignment_3.pdf
			y = j*resolution + resolution/2
			all_cells.append((x,y))
			index = i + map_width_px*j
			if map_data[index] == 100:
				occupied_cells.add((x,y))

	# Generiranje vozlišč
	print("Generating graph nodes")
	distance_threshold = 0.4 # v metrih (razdalja, koliko morajo biti vozlišča grafa minimalno oddaljena od najbližje ovire)
	method = "window_and_random" # "random" ali pa "window_and_random" (to druga verjetno ni standardna, izmislil sem si jo med programiranjem)
	nodes = [] # eno vozlišče je definirano z dvema koordinatama (x in y) takole: [x,y]
	min_distances = [] # razdalja do najbližje ovire (to si bomo zapomnili, da bomo opcijsko lahko dodatno uteževali povezave glede na oddaljenost od ovir)
	random_idxs = set() # da si zapomnimo, katere smo že izbrali
	if method == "random":
		nr_of_random_nodes = 2000 # rabimo za graf po metodi 4.1.3 Naključna karta cest
		while len(nodes) < nr_of_random_nodes:
			random_idx = random.randint(0,len(all_cells))
			min_distance = 0.0
			if random_idx not in random_idxs and all_cells[random_idx] not in occupied_cells:
				min_distance = get_min_distance(all_cells[random_idx], occupied_cells, stop_searching_dist=distance_threshold)
			if min_distance > distance_threshold:
				nodes.append(all_cells[random_idx])
				min_distances.append(min_distance)
				random_idxs.add(random_idx)
	if method == "window_and_random":
		window_size = 0.4 # v metrih
		max_nr_of_attempts = 100
		for i in range(int(map_width/window_size)):
			for j in range(int(map_height/window_size)):
				window_center_x = i*window_size + window_size/2
				window_center_y = j*window_size + window_size/2
				min_distance = get_min_distance((window_center_x,window_center_y), occupied_cells, stop_searching_dist=distance_threshold)
				if min_distance > distance_threshold:
					# Če lahko, potem damo vozlišče v sredino tega okna
					nodes.append((window_center_x,window_center_y))
					min_distances.append(min_distance)
				else:
					# Če pa ne moremo, potem pa vozlišče poskusimo postaviti na naključno lokacijo znotraj tega okna
					for k in range(max_nr_of_attempts):
						random_x = random.uniform(window_center_x-window_size/2, window_center_x+window_size/2)
						random_y = random.uniform(window_center_y-window_size/2, window_center_y+window_size/2)
						min_distance = get_min_distance((random_x,random_y), occupied_cells, stop_searching_dist=distance_threshold)
						if min_distance > distance_threshold:
							nodes.append((random_x,random_y))
							min_distances.append(min_distance)
							break

	# Generiranje povezav
	print("Generating graph edges")
	edge_threshold = 0.7
	additional_weighting_proximity_of_obstacles = True
	edges = [] # ena povezava v listi edges je definirana kot par id-jev izhodiščnega in ciljnega vozlišča, takole: [stard_node_id, end_node_id]
	distances = [] # dolžine povezav, ki pripadajo posameznim povezavam v listi edges (dolžino oz. utež povezave lahko dodatno utežimo če blizu povezave nahajajo ovire)
	for i in range(len(nodes)):
		for j in range(len(nodes)):
			point1 = nodes[i]
			point2 = nodes[j]
			dist = math.sqrt((point2[0]-point1[0])**2 + (point2[1]-point1[1])**2)
			if dist < edge_threshold:
				edges.append((i,j))
				if not additional_weighting_proximity_of_obstacles:
					distances.append(dist)
				else:
					min_distances_to_obstacles = [min_distances[i], min_distances[j]]
					min_distance = min(min_distances_to_obstacles)
					w2 = 0.0
					if min_distance < 1.0:
						w2 = 1.0 / min_distance**2
					distances.append(dist+w2)

	return nodes, edges, distances

def dijsktra(graph, initial):
	# Vir: https://gist.github.com/econchick/4666413
	visited = {initial: 0}
	path = {}

	nodes = set(graph.nodes)

	while nodes:
		min_node = None
		for node in nodes:
			if node in visited:
				if min_node is None:
					min_node = node
				elif visited[node] < visited[min_node]:
					min_node = node
		if min_node is None:
			break

		nodes.remove(min_node)
		current_weight = visited[min_node]

		for edge in graph.edges[min_node]:
			try:
				weight = current_weight + graph.distances[(min_node, edge)]
			except:
				continue
			if edge not in visited or weight < visited[edge]:
				visited[edge] = weight
				path[edge] = min_node

	return visited, path

def shortest_path(graph, origin, destination):
	# Vir: https://gist.github.com/mdsrosa/c71339cb23bc51e711d8
	visited, paths = dijsktra(graph, origin)
	full_path = deque()
	_destination = paths[destination]

	while _destination != origin:
		full_path.appendleft(_destination)
		_destination = paths[_destination]

	full_path.appendleft(origin)
	full_path.append(destination)

	return visited[destination], list(full_path)

###
# Vodenje po zvezni trajektoriji
###

def drive_along_the_path(cmd_vel_pub): # Vodenje po zvezni trajektoriji (predavanje poglavje 3.3)
	'''
	Na podlagi poglavja predavanj 3.3 izdelamo zvezno trajektorijo z uporabo polinoma petega reda.
	Program za input vzame pot ("path"), ki je definirana z točkami s koordinatama (xi,yi).
	- path: ((x0,y0),(x1,y1),(x2,y2), ...)
	- T: čas potreben za prehod enega segmenta (ene trajktorije)
	'''

	# definicija globalnih spremenljivk
	global goal_x
	global goal_y
	global path
	global current_path_point_id
	global t
	global i_count
	global va_xy_z
	
	# Ugotavljanje, ali smo že na cilju
	distance_to_goal = math.sqrt((goal_x-robot_position_x)**2 + (goal_y-robot_position_y)**2)
	if distance_to_goal < distance_to_goal_thr:
		goal_x = None
		goal_y = None
		path = None
		current_path_point_id = None
		print("Goal reached!")
		va_xy_z = np.array([0.,0.,0.,0.])
		i_count=0
		j_count=0
		return 0

	# Časovni potek posamezne trajektorije
	T=2 # čas od t=0 do t=T
	t+=dt # 
	j_count=i_count # Ko dosežemo t=T preidemo na nov segment in tačnemo ponovno slediti času t
	if (t>T*1.05):
		i_count=i_count+1 # prehod na segment i+1
		t=0.01 # ponastavitev časa

	# IZRAČUN TRAJEKTORIJE
	p = np.array([path[i_count][0], path[i_count][1]]) # vektor pozicij (xi, yi)
	p_next = np.array([path[i_count+1][0], path[i_count+1][1]])# vektor pozicij (xi+1, yi+1)

	# Robni pogoji trajektorije
	#začetna pozicija x0=x(t=0), y0=y(t=0) in končna pozicija xT=x(t=T) in yT=y(t=T)
	x0 = p[0] 
	y0 = p[1]
	xT = p_next[0]
	yT = p_next[1]
	# začetna (vx0, vy0) in končna (vxT, vyT) hitrost
	vx0 = va_xy_z[0]
	vxT = (xT-x0)/T #končno hitrost definiramo kot 2x povprečno glede na koordinate
	vy0 = va_xy_z[1]
	vyT = (yT-y0)/T
	# začetni (ax0, ay0) in končni (axT, ayT) pospešek
	ax0 = va_xy_z[2]
	axT = (vxT-vx0)/T
	ay0 =va_xy_z[3]
	ayT = (vyT-vy0)/T
	# Reševanje sistema enačb za koeficiente (en. 3.14)
	x_vec = np.array([x0,xT,vx0,vxT,ax0,axT]) # vektor pozicije, hitrosti in pospeška
	y_vec = np.array([y0,yT,vy0,vyT,ay0,ayT])
	T_list = np.array([[0,0,0,0,0,1],
                    	[T**5,T**4,T**3,T**2,T,1],
                    	[0,0,0,0,1,0],
                    	[5*T**4,4*T**3,3*T**2,2*T,1,0],
                    	[0,0,0,2,0,0],
                    	[20*T**3,12*T**2,6*T,2,0,0]])
	koef_x = np.linalg.inv(T_list).dot(x_vec)
	koef_y = np.linalg.inv(T_list).dot(y_vec)

	# Funkcije pozicije, hitrosti in pospeška (en. 3.13)
	xff = lambda t: np.dot(koef_x, np.array([t**5, t**4, t**3, t**2, t, 1]))  # pozicija
	yff = lambda t: np.dot(koef_y, np.array([t**5, t**4, t**3, t**2, t, 1]))
	vxff = lambda t: 5*koef_x[0]*t**4 + 4*koef_x[1]*t**3 + 3*koef_x[2]*t**2 + 2*koef_x[3]*t + koef_x[4] # hitrost
	vyff = lambda t: 5*koef_y[0]*t**4 + 4*koef_y[1]*t**3 + 3*koef_y[2]*t**2 + 2*koef_y[3]*t + koef_y[4]
	axff = lambda t: 20*koef_x[0]*t**3 + 12*koef_x[1]*t**2 + 6*koef_x[2]*t + 2*koef_x[3]  # pospešek
	ayff = lambda t: 20*koef_y[0]*t**3 + 12*koef_y[1]*t**2 + 6*koef_y[2]*t + 2*koef_y[3]

	# Ponastavitev začetnih vrednosti
	# ob koncu segmenta končne hitrosti in pospeški segmenta postanejo začetne naslednjega
	if i_count != j_count:
		va_xy_z[0] = vxff(T)
		va_xy_z[1] = vyff(T)
		va_xy_z[2] = axff(T)
		va_xy_z[3] = ayff(T)
		j_count=i_count


	# Izračun referenčne hitrosti, kota in kotne hitrosti iz trajektorije
	v_ff = np.sqrt(vxff(t)**2 + vyff(t)**2) #en. (3.15 - en. 3.16)
	theta_ff = np.arctan2(vyff(t), vxff(t))
	zg=vxff(t)*ayff(t)-vyff(t)*axff(t)
	sp=vxff(t)**2+vyff(t)**2
	omega_ff = np.divide(zg, sp, out=np.zeros_like(zg), where=sp!=0)

	# KRMILNI ZAKON
	# Napake
	qref=np.array([xff(t), yff(t), theta_ff]) #referenčni vektor (x,y,theta)
	q=np.array([robot_position_x, robot_position_y, robot_orientation_yaw]) #dejansji vektor (x,y,theta)
	e_matrika =np.array([(np.cos(robot_orientation_yaw), np.sin(robot_orientation_yaw), 0),
                     	(-np.sin(robot_orientation_yaw), np.cos(robot_orientation_yaw), 0),
                     	(0, 0, 1)])
	[ex, ey, eth] = np.dot(e_matrika, qref-q) # izračun napak

	# V določenih primerih je treba kot theta_ref popravit (do drugače težave pride, če je vrednost theta_ref blicu vrednosti PI oz. -PI)
	angle_difference_abs_values = [abs(eth),
								   abs(theta_ff+2*math.pi - robot_orientation_yaw),
								   abs(theta_ff-2*math.pi - robot_orientation_yaw)]
	angle_differences = [theta_ff - robot_orientation_yaw,
						 theta_ff+2*math.pi - robot_orientation_yaw,
						 theta_ff-2*math.pi - robot_orientation_yaw]
	eth = angle_differences[angle_difference_abs_values.index(min(angle_difference_abs_values)) ]

	# končni krmilni zakon # en. (3.21)
	v_x = v_ff*np.cos(eth) + Kx*ex # linearna hitrost
	omega = omega_ff +v_ff*(Ky*ey + Kth*np.sin(eth)) # nelinearna kotna hitrost

	# Če je na začetku poti robot zelo narobe zasukan, naj najprej samo rotira in poporavi orientacijo glede na prvi segment poti
	if np.absolute(i_count==0 and theta_ff - robot_orientation_yaw) > 0.2:
		v_x = 0.0
		#t = 0.01

	# POŠILJNJE UKAZ ZA HITROST
	twist_msg = Twist()
	twist_msg.linear.x = v_x
	twist_msg.angular.z = omega
	cmd_vel_pub.publish(twist_msg)


def publish_path(path_pub):
	if path != None:
		path_ = Path()
		path_.header.frame_id = "map"
		for position in path:
			pose_ = PoseStamped()
			pose_ .header.frame_id = "map"
			pose_.pose.position.x = position[0]
			pose_.pose.position.y = position[1]
			path_.poses.append(pose_)
		path_pub.publish(path_)

def stop(cmd_vel_pub):
	twist_msg = Twist()
	twist_msg.linear.x = 0.0
	twist_msg.angular.z = 0.0
	cmd_vel_pub.publish(twist_msg)

def pose_callback(msg):
	global robot_position_x
	global robot_position_y
	global robot_orientation_yaw
	robot_position_x = msg.pose.pose.position.x
	robot_position_y = msg.pose.pose.position.y
	robot_orientation_roll, robot_orientation_pitch, robot_orientation_yaw = euler_from_quaternion(msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w)

def goal_callback(msg):
	global goal_x
	global goal_y
	global path
	global current_path_point_id
	print("New goal received")
	path = None
	current_path_point_id = None
	goal_x = msg.goal.target_pose.pose.position.x
	goal_y = msg.goal.target_pose.pose.position.y
	calculate_path()

def map_callback(msg):
	global map_data
	global map_info
	map_data = msg.data
	map_info = msg.info

def advanced_move_base():
	global dt

	pose_sub = rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback)
	goal_sub = rospy.Subscriber('/advanced_move_base/goal', MoveBaseActionGoal, goal_callback)
	map_sub = rospy.Subscriber("/map", OccupancyGrid, map_callback)

	cmd_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
	path_pub = rospy.Publisher('/advanced_move_base/path', Path, queue_size=10)

	rospy.init_node('advanced_move_base')
	
	current_time = rospy.Time.now()
	last_time = rospy.Time.now()
	rate = rospy.Rate(frequency)

	path_prev = None # Zato da lahko objavimo pot, ko je ta izračunana

	
	while not rospy.is_shutdown():
		if path != None:
			# Če obstaja pot, naj robot pelje po poti
			drive_along_the_path(cmd_vel_pub)
			# Objava na novo izračunane poti
			if path_prev != path: publish_path(path_pub)
			path_prev = path
		else:
			# Če poti ni, potem naj robot miruje
			stop(cmd_vel_pub)

		# Štetje časa
		current_time = rospy.Time.now()
		dt = (current_time-last_time).to_sec() #korak 
		last_time = current_time

		rate.sleep()

if __name__ == '__main__':
	try:
		advanced_move_base()
	except rospy.ROSInterruptException:
		pass
