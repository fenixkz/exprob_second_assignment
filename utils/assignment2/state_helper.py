"""
.. module:: state_helper
  :platform: Unix
  :synopsis: Python module for the helpers used in :mod:`state_machine`
  
.. moduleauthor:: Ayan Mazhitov mazhitovayan@gmail.com

Python module that defines three clases of helpers to be used in :mod:`state_machine`: ProtegeHelper(), InterfaceHelper(), ActionClientHelper()

"""



# Import all the necessary libraries
import rospy 
import os
from armor_api.armor_client import ArmorClient
from assignment2 import architecture_name_mapper as anm
import re
import random
import math
import time 
from actionlib import SimpleActionClient
from actionlib_msgs.msg import GoalStatusArray
from threading import Lock
from std_msgs.msg import Bool
import actionlib
import actionlib.msg
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from move_base_msgs.msg import MoveBaseActionFeedback
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

class Room():
	def __init__(self, name, x, y, doors):
		self.name = name
		self.doors = doors
		self.x = x
		self.y = y



class ProtegeHelper():
	'''
	
	This class is used by :mod:`state_machine` to help to deal with `armor_py_api <https://github.com/EmaroLab/armor_py_api>`_. Also, loads the following parameters from the ROS param server
	
	*    config/ontology_name
	*    config/ontology_path
	
	
	Attributes
	----------
	client: :mod:`armor_api.armor_client.ArmorClient`
		An Armor client from `armor_py_api <https://github.com/EmaroLab/armor_py_api>`_ library to do queries and manipulation with the ontology file
	
	Methods
	----------
	get_doors_list()
		Class method to get the list of strings representing each door
	get_corridors_list()
		Class method to get the list of strings representing each corridor
	get_rooms_list()
		Class method to get the list of strings representing each room
	get_current_location()
		Class method to get the current location of the robot
	load_topological_map()
		Class method that loads the topologiacl map from the specified path into the buffer
	add_individual()
		Class method that adds each individual to the ontology
	build_map()
		Class method that builds the map
	decide_next_location()
		Class method that uses a certain policy to decide next location for the robot
	plan_to_recharge_station()
		Class method that returns the next location to be able to reach the charging station
	move_robot()
		Class method that does manipulations to the ontology to move the robot individual to the new location
	reason()
		Class method that invokes the ontology's reasoner
	canReach()
		Class method that gives the list of all locations that can be reached by the robot
	get_urgent_location()
		Class method that gives the list of urgent locations
	
	'''
	def __init__(self):
		'''
		
		Class constructor. Uses private attributes to ensure data encapsulation. 
		
		Parameters
		----------

		Returns
		-------
		None.

		'''
		# Initialize the Armor Client
		self.client = ArmorClient("example", "ontoRef")
		# Define a pair of individuals, each location with the door
		
		# Load the name of the ontology from the ROS param server
		self.__ontology_name = rospy.get_param(anm.PARAM_ONTOLOGY_NAME, "topological_map.owl")
		# Load the full path to the ontology from the ROS param server
		self.__path = rospy.get_param(anm.PARAM_ONTOLOGY_PATH, "~/catkin_ws/src/assignment2/topological_map")
		# Attribute to track the current location of the robot
		self.__current_location = ""
		self.__rooms = []
		self.__corridors = []
		self.__doors = []
		self.__coord_location_dict = {}
		# For logging purposes
		self.LOG_TAG = "Protege-Helper"

	def get_doors_list(self):
		'''
		
		Method that returns the list of string representing each door in the environment.

		Returns
		-------
		list
			List of str for each door in the environment.

		'''
		return self.__doors
		
	def get_corridors_list(self):
		'''
		
		Method that returns the list of string representing each corridor in the environment.

		Returns
		-------
		list
			List of str for each corridor in the environment.

		'''
		return self.__corridors
		
	def get_rooms_list(self):
		'''
		
		Method that returns the list of string representing each room in the environment.

		Returns
		-------
		list
			List of str for each room in the environment.

		'''
		return self.__rooms

	def get_current_location(self):
		'''
		
		Method that returns a string representing the current location of the robot.

		Returns
		-------
		str
			Current location of the robot.

		'''
		return self.__current_location

	def get_coord_dict(self):
		'''

		Method that return the dictionary of location and coordinates pair

		Returns
		-------
		dict
			Dictionary of (Location, coordinates) pairs.

		'''
		return self.__coord_location_dict

	def load_topological_map(self):
		'''
		
		Method that loads the ontology. The path and the ontology's name is specified as parameters in the param server. Also, mounts and sets the log to the terminal.

		Returns
		-------
		int
			Error code. 1 for success; 0 for the error.

		'''
		# Get the path
		path = os.path.join(self.__path, self.__ontology_name)
		if os.path.isfile(path): # If the path exists
			# Load the ontology
			self.client.utils.load_ref_from_file(path, 'http://bnc/exp-rob-lab/2022-23', True, "PELLET", True, False)
			self.client.utils.mount_on_ref()
			self.client.utils.set_log_to_terminal(True)
			return 1
		else: # Error
			rospy.logerr("Cannot find the ontology map file. Check the specified path.")
			return 0
		return 0
	
	def add_individuals(self, rooms):
		'''
		
		Method that adds all individuals specified by the class private attribute to the ontology. After adding, it also makes them disjoint

		Returns
		-------
		None.

		'''
		# For each pair of location and door add them to ontology
		for room in rooms:
			if "C" in room.name:
				self.__corridors.append(room.name)
			else:
				self.__rooms.append(room.name)
			for door in room.doors:
				self.client.manipulation.add_objectprop_to_ind("hasDoor", room.name, door)
				self.__doors.append(door)
		self.__doors = list(dict.fromkeys(self.__doors))
		# Make all individuals disjoint
		self.client.call('DISJOINT', 'IND', '', self.__rooms + self.__corridors + self.__doors)
	
	def build_map(self, rooms):
		'''
		
		Method that builds the topological map of the environment. First, it invokes :mod:`state_helper.ProtegeHelper.load_topological_map` and :mod:`state_helper.ProtegeHelper.add_individuals`.
		Also, adds the dataproperty 'visitedAt' to each location with the now timestamp. Finally, calls the :mod:`state_helper.ProtegeHelper.reason`.

		Returns
		-------
		int
			Error code. 1 for success. 0 for error.

		'''
		
		if self.load_topological_map():
			self.add_individuals(rooms)
			self.__coord_location_dict = {room.name: [room.x, room.y] for room in rooms}
			t = math.floor(rospy.Time.now().to_sec())
			for location in rooms:
				self.client.manipulation.add_dataprop_to_ind('visitedAt', location.name, "Long", str(t))
			self.reason()
			return 1
		else:
			rospy.logerr("Unexpected error. Map has not been built.")
			return 0
	
	def decide_next_location(self):
		'''
		
		Method that uses the surveillance policy to decide what should be the next location of the robot. First, it calls :mod:`state_helper.ProtegeHelper.get_urgent_location` 
		to know if there are urgent locations, and :mod:`state_helper.ProtegeHelper.canReach` to know the reachable locations. If no locations became urgent then returns a random reachable location.
		Else chooses random between the intersection of reachable and urgent locations.

		Returns
		-------
		str
			String representation of the next location.

		'''
		# Get the list of urgent locations
		urgent_locations = self.get_urgent_location()
		# Get the list of reachable locations
		reachable_locations = self.canReach()
		if urgent_locations:
			# Find what URGENT locations are within the range of the robot
			urgent_reachable_locations = [item for item in urgent_locations if item in reachable_locations]
			# Pick something random among these locations
			if urgent_reachable_locations:
				return random.choice(urgent_reachable_locations) 
		# If there are no yet urgent locations
		# Just pick something random among locations that can be reached
		return random.choice(reachable_locations) 
	
	def plan_to_recharge_station(self, charging_station):
		'''
		
		Method that checks if the robot is within the location containing the charging station. If not, then returns the closest location to it. 

		Parameters
		----------
		charging_station : str
			String representation of the location containing the charging station.

		Returns
		-------
		str
			String representation of the closest location to the charging station.
		
		Note
		----
		Works only when the charging location is 'E'
		
		'''
		if self.__current_location == charging_station:
			# If the robot is already in charging location
			return ""
		if self.__current_location in ['C1', 'C2']:
			# If the robot is in corridors C1 or C2, then return the charging location
			return charging_station
		if self.__current_location in ['R1', 'R2']:
			# Same logic
			return 'C1'
		if self.__current_location in ['R3', 'R4']:
			# Same logic
			return 'C2'
	
	def move_robot(self, location, time):
		'''
		
		Method that does manipulation to the ontology to replace the object property 'isIn' of Robot1 to new location in order to do the logic of moving the robot from one location to another

		Parameters
		----------
		location : str
			String representation of location where the robot wants to go.
		time : int
			Timestamp.

		Returns
		-------
		None.

		'''
		# Check whether the location given is a string or not
		if isinstance(location, str):
			if not self.__current_location: # For the beginning of the process the robot has no location
				try:
					# Add the object property 'isIn' of the robot with the new location
					self.client.manipulation.add_objectprop_to_ind('isIn', 'Robot1', location)
				except:
					rospy.logerr("Exception thrown try to add object property isIn to the robot")
			else:
				try:
					# Replace the object property 'isIn' of the robot with the new location
					self.client.manipulation.replace_objectprop_b2_ind('isIn', 'Robot1', location, self.__current_location)
				except:
					rospy.logerr("Exception is thrown try to replace the object property isIn to the robot")
			# Update the current location
			self.__current_location = location
			# Retrieve from the ontology the last timestamp for that location
			timestamp_last = self.client.query.dataprop_b2_ind("visitedAt", location)
			# The output from the query above is not exactly an int, so format it 
			timestamp_ = re.sub("[^0-9]", "", timestamp_last[0])
			try:
				# Replace the data property of that location with the new timestamp
				self.client.manipulation.replace_dataprop_b2_ind('visitedAt', location, "Long", str(time), timestamp_)
			except:
				msg = "Exception is thrown trying to replace data property visitedAt to location {0} with value {1}".format(location, time)
				rospy.logerr(msg)
			# Update the now data property of the Robot1
			robot_time_ = self.client.query.dataprop_b2_ind("now", 'Robot1')
			time_ = re.sub("[^0-9]", "", robot_time_[0])
			self.client.manipulation.replace_dataprop_b2_ind('now', 'Robot1', 'Long', str(time), time_)
			# Finally call the reasoner
			self.reason()
			
		else:
			rospy.logerr("The location must be of type string")
	
	def reason(self):
		'''
		
		Method that calls the reasoner to update the new changes in the ontology

		Returns
		-------
		None.

		'''
		self.client.utils.apply_buffered_changes()
		self.client.utils.sync_buffered_reasoner()
		
	def canReach(self):
		'''
		
		Method that gives the list of all locations that are within the range of the robot

		Returns
		-------
		list
			List of strings representing all reachable locations for the robot.

		'''
		return self.client.query.objectprop_b2_ind("canReach", 'Robot1')
	
	def get_urgent_location(self):
		'''
		
		Method that returns the list of all urgent locations.

		Returns
		-------
		list
			List of strings representing all urgent locations.

		'''
		return self.client.query.ind_b2_class('URGENT')
		
		
class InterfaceHelper:
	'''

	A helper class to make the interface easier. Responsible for quering Planner and Controller Action Client. Also, has method that checks the battery level through :mod:`robot_states` service: /state/battery_low. 

	Attributes
	----------
	mutex: `threading.Lock`
	A mutex that is used for synchronization
	charging_time: float
	Time that robot waits until the battery gets recharged. Retrieved from ROS param server by key `config/charging_time`
	waiting_time: float
	Time that robot waits in location for surveillance purposes. Retrieve from ROS param server by key `config/waiting_time`
	randomness: bool
	Indicates whether the battery is controlled manually or by random sense. Retrieved from ROS param server by key `test/random_sense_active`
	planner_client: :mod:`state_helper.ActionClientHelper`
	An action server for planning the trajectory to a point in a environment. 
	controller_client: :mod:`state_helper.ActionClientHelper`
	An action server for controlling the robot
	LOG_TAG: str
	Tag used for logging purposes

	Methods
	-------
	reset_battery()
	Class method to reset the state of the battery from flow to high
	get_coord_dict()
	Class method to get the dictionary of the (location, coordinates)
	get_charging_location()
	Class method to get the location where the charging station is set
	is_battery_low()
	Class method to know if the battery is low
	init_robot_pose(point)
	Class method to initialize the robot' position

	'''
	def __init__(self):

		self.mutex = Lock()
		# Retrieve the parameters that we need from the parameter server
		self.charging_time = rospy.get_param(anm.PARAM_BATTERY_TIME_CHARGE, 10.0)
		self.waiting_time = rospy.get_param(anm.PARAM_WAIT_IN_LOCATION, 30.0)
		self._charging_location = rospy.get_param(anm.PARAM_CHARGING_STATION, 'E')
		self.randomness = rospy.get_param(anm.PARAM_RANDOM_ACTIVE,  False)
		# Set the initial state involving the `self._battery_low`
		self.reset_battery()
		rospy.Subscriber(anm.TOPIC_BATTERY_LOW, Bool, self._battery_callback)
		# self.pub_j1 = rospy.Publisher("/robot/joint1_position_controller/command")
		# Define the clients for the the plan and control action servers.
		self.LOG_TAG = "Interface-Helper"
		self.move_base = SimpleActionClient('move_base', MoveBaseAction)
		self.current_goal = [0,0]
		self.pub_j1 = rospy.Publisher("/robot/joint1_position_controller/command", Float64, queue_size = 10)
	
	def inspect(self):
		msg = Float64()
		js = rospy.wait_for_message("/robot/joint_states", JointState)
		if js.position[0] < 3.14:
			msg.data = 6.28
		else:
			msg.data = 0.0	
		self.pub_j1.publish(msg)

	def cancel_goals(self):
		self.move_base.wait_for_server()
		self.move_base.cancel_all_goals()
		time.sleep(0.5)

	def plan_and_move(self, coord):
		# set the desired goal
		goal = MoveBaseGoal()
		goal.target_pose.header.frame_id = "map"
		goal.target_pose.header.stamp = rospy.Time.now()
		goal.target_pose.pose.position.x = coord[0]
		goal.target_pose.pose.position.y = coord[1]
		goal.target_pose.pose.orientation.w = 1
		self.current_goal[0] = coord[0]
		self.current_goal[1] = coord[1]
		self.move_base.wait_for_server()
		# Send a new `goal`, which is a message of type `MoveBaseGoal`
		self.move_base.send_goal(goal)

	def is_done(self):
		dist_th = 0.3
		# We are checking the distance to see if we reached the goal
		feedback = rospy.wait_for_message("move_base/feedback", MoveBaseActionFeedback)
		# Get the current x position of the robot
		x_now = feedback.feedback.base_position.pose.position.x
		# Get the current y position of the robot
		y_now = feedback.feedback.base_position.pose.position.y
		# Calculate the Euclidean distance
		dist = ((self.current_goal[0] - x_now)**2 + (self.current_goal[1] - y_now)**2)**0.5
		return (dist < dist_th) or (self.move_base.get_state() == 3)

	def reset_battery(self):
		'''

		Method that resets the state of the battery from LOW to HIGH. Also, publishes the new state to the battery topic

		Returns
		-------
		None.

		'''
		self.mutex.acquire()
		try:
			self._battery_low = False
		finally:
			self.mutex.release()
		publisher = rospy.Publisher(anm.TOPIC_BATTERY_LOW, Bool, queue_size=1, latch=True)
		publisher.publish(Bool(self._battery_low))

	def get_charging_location(self):
		'''

		Method that returns the location where the charging station is set

		Returns
		-------
		str
		String representation of the location.

		'''
		return self._charging_location
	# The subscriber to get messages published from the `robot-state` node into the `/state/battery_low/` topic.
	def _battery_callback(self, msg):
		# Acquire the mutex to assure the synchronization with the other subscribers and action clients (this assure data consistency).
		self.mutex.acquire()
		try:
			# Get the battery level and set the relative state variable encoded in this class.
			self._battery_low = msg.data

		finally:
			# Release the mutex to eventually unblock the other subscribers or action servers that are waiting.
			self.mutex.release()


	def is_battery_low(self):
		return self._battery_low

	# Update the current robot pose stored in the `robot-state` node.
	@staticmethod
	def init_robot_pose(point):
		'''

		Method that sets the initial position of the robot

		Parameters
		----------
		point : `warden_robot.msg.Point`
		x and y coordinates of the initial position of the robot.

		Returns
		-------
		None.

		'''
		# Eventually, wait for the server to be initialised.
		rospy.wait_for_service(anm.SERVER_SET_POSE)
		try:
			# Call the service and set the current robot position.
			service = rospy.ServiceProxy(anm.SERVER_SET_POSE, SetPose)
			service(point)  # None that the service `response` is not used.
			log_msg = f'Setting initial robot position ({point.x}, {point.y}) to the `{anm.SERVER_SET_POSE}` node.'
			rospy.loginfo(anm.tag_log(log_msg, "Interface-Helper"))
		except rospy.ServiceException as e:
			err_msg = f'Cannot set current robot position through `{anm.SERVER_SET_POSE}` server. Error: {e}'
			rospy.logerr(anm.tag_log(err_msg, "Interface-Helper"))

