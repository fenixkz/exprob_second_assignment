#!/usr/bin/env python
"""
.. module:: state_machine
  :platform: Unix
  :synopsis: Main python script that encapsulates the state machine and provides the implementation of the logic behind the states
  
.. moduleauthor:: Ayan Mazhitov mazhitovayan@gmail.com

This script is the main script that creates the state machine based on `SMACH <http://wiki.ros.org/smach>`_ ROS library.
The necessary connections to ontology via `armor_py_api <https://github.com/EmaroLab/armor_py_api>`_ is done via the helper class :mod:`state_helper.ProtegeHelper`
Interface helper (:mod:`state_helper.InterfaceHelper`) is used to do the logic described by the surveillance policy

"""

# Import of the necessary libraris 
import rospy
import smach_ros
from smach import StateMachine, State
import math

from assignment2 import architecture_name_mapper as anm
from assignment2.state_helper import ProtegeHelper, InterfaceHelper, Room
from std_msgs.msg import Int64, Int64MultiArray
from assignment2.srv import RoomInformation
from assignment2.msg import RoomConnection
# Define states of the state machine
STATE_BUILD_MAP = "BUILD_MAP" # Phase 1. State when the robot waits until the topological map is built

STATE_START_EXPLORING = "START_EXPLORING" # Phase 2. Higher level state for the normal behavior of the surveillance policy.
STATE_START_BEHAVIOR = "START_BEHAVIOR" # Phase 2. Lower level state of STATE_START_EXPLORING. Start the normal behavior.
STATE_PLAN_TO_LOCATION = 'PLAN_TO_GIVEN_LOCATION' # Phase 2. Lower level state of STATE_START_EXPLORING. Compute plan (via points) to a specified location
STATE_GO_TO_LOCATION = 'GO_TO_GIVEN_LOCATION' # Phase 2. Lower level state of STATE_START_EXPLORING. Follow the plan given by the previous state
STATE_WAIT_IN_LOCATION = "WAIT_IN_LOCATION" # Phase 2. Lower level state of STATE_START_EXPLORING. Wait in the location for the surveillance purposes.

STATE_RECHARGING_ROUTINE = "START_RECHARGING_ROUTINE" # Phase 2. Higher level state for the recharging routine. Compute and follow the plan to go to the location containing the charging station
STATE_PLAN_TO_CHARGING_LOCATION = "PLAN_TO_CHARGING_LOCATION" # Phase 2. Lower level state of START_RECHARGING_ROUTINE. Compute the plan to the charging station
STATE_GO_TO_CHARGING_LOCATION = "GO_TO_CHARGING_LOCATION" # Phase 2. Lower level state of START_RECHARGING_ROUTINE. Follow the plan given by the previous state
STATE_RECHARGE = "RECHARGE" # Phase 2. Lower level state of START_RECHARGING_ROUTINE. State where the robot recharges its battery

# Define transitions
TRANS_MAP_NOT_BUILT = "MAP_HAS_NOT_BUILT" # Map is not have yet been built
TRANS_MAP_BUILT = "MAP_BUILT" # Map has been built 
TRANS_REPEAT = "REPEAT" # Repeat the normal behavior routine
TRANS_BATTERY_LOW = "BATTERY_IS_LOW" # Battery is low
TRANS_BATTERY_FULL = "BATTERY_IS_FULL" # Battery is full
TRANS_PLAN_TO_LOCATION = "PLAN_TO_LOCATION" # Plan to location
TRANS_PLANNED_TO_LOCATION = "VIA_POINTS_HAVE_BEEN_COMPUTED" # Plan has been computed
TRANS_GO_TO_LOCATION = "GO_TO_LOCATION" # Follow the plan
TRANS_WENT_TO_LOCATION = "ROBOT_HAS_MOVED_TO_GIVEN_LOCATION" # Robot has successfully changed the location
TRANS_RECHARGING = "RECHARGING_THE_BATTERY" # Recharge the battery
TRANS_DONE_WAITING = "DONE_WAITING" # Done surveying the location

TRANS_IN_CHARGING_LOCATION = "ROBOT_IS_IN_CHARGING_LOCATION" # Robot is in location containing the charging station
TRANS_NOT_IN_CHARGING_LOCATION = "ROBOT_IS_NOT_IN_CHARGING_LOCATION"# Robot is not in location containing the charging station



LOOP_SLEEP_TIME = 0.1 # Loop sleep time. 10 Hz.

LOG_TAG = "STATE_MACHINE" # For logging purposes

room_srv = rospy.ServiceProxy('/room_info', RoomInformation)

# States
class BuildMap(State):
	'''
	
	This class represents the implementation of the **BUILD_MAP** state. It is in busy waiting mode when the robot scans the initial room for aruco markers.
	After that it receives a message that the inspection is done and retrieves the markerIDs to call the /marker_info service. In this state, the robot calls :mod:`state_helper.ProtegeHelper.build_map` function to build the 
	topological map of the environment, and then calls :mod:`state_helper.ProtegeHelper.move_robot` to move the robot to the starting location 'E'. 
	
	Attributes
	----------
	protege_helper: :mod:`state_helper.ProtegeHelper`
		Class attribute to deal with `armor_py_api <https://github.com/EmaroLab/armor_py_api>`_
	
	Methods
	----------
	execute(userdata)
		Class method that is called automatically when the state is active
	
	Notes
	----------
		Possible outputs:
			[TRANS_MAP_BUILT, TRANS_MAP_NOT_BUILT]
		Transitions:
			TRANS_MAP_BUILT -> STATE_START_EXPLORING
			
			TRANS_MAP_NOT_BUILT -> STATE_BUILD_MAP
		userdata:
			Input: 
				None
			
			Output: 
				None
			
	'''
	def __init__(self, protegeHelper, interfaceHelper):
		# Protege Helper class that helps with dealing with armor_py_api client
		self.helper = protegeHelper
		self._iHelper = interfaceHelper
		State.__init__(self, outcomes=[TRANS_MAP_BUILT, TRANS_MAP_NOT_BUILT])
	def execute(self, userdata):
		'''
		
		Does the building of the topological map using *protege_helper*. After the succesfull building moves the robot to location 'E' 
		
		Arguments
		----------
		userdata : dictionary
			dictionary of keys for trasmitting the data between the states.

		Returns
		-------
		str
			Possible outputs.
			
		'''
		rospy.loginfo(anm.tag_log("Waiting for the information gathering ...", LOG_TAG + " >>> " + STATE_BUILD_MAP))
		done = None
		while done is None:
			done = rospy.wait_for_message("/isDone", Int64)
		if done.data == 1:
			msg = rospy.wait_for_message("/markerIDs", Int64MultiArray)
			print(msg.data)
			markerIDs = msg.data
		# markerIDs = [11, 12, 13, 14, 142, 15, 16, 17]
		info = []
		for id in markerIDs:
			info.append(room_srv(id))
		rospy.loginfo(anm.tag_log("Building the map, please wait ...", LOG_TAG + " >>> " + STATE_BUILD_MAP))
		rooms = []
		for item in info:
			if item.room != "no room associated with this marker id":
				room = Room(item.room, item.x, item.y, [connection.through_door for connection in item.connections])
				rooms.append(room)
		if self.helper.build_map(rooms): # If success, then do the left routine
			# Logging
			rospy.loginfo(anm.tag_log("Great. Map has been built.", LOG_TAG + " >>> " + STATE_BUILD_MAP))
			log = "Okay. \n List of rooms = {0} \n List of corridors = {1} \n List of doors = {2}".format(self.helper.get_rooms_list(), self.helper.get_corridors_list(), self.helper.get_doors_list())
			rospy.loginfo(anm.tag_log(log, LOG_TAG + " >>> " + STATE_BUILD_MAP))
			# Move the robot to the initial position, which is room 'E'
			dict = self.helper.get_coord_dict()
			self._iHelper.plan_and_move(dict['E'])
			while(not self._iHelper.is_done()):
				rospy.sleep(LOOP_SLEEP_TIME)
				
			# Retrieve the time and move the robot to the starting location 'E'
			t0 = math.floor(rospy.Time.now().to_sec())
			self.helper.move_robot('E', t0)
			log = "The robot is now in location {0} at time {1} nsec".format('E', t0)
			rospy.loginfo(anm.tag_log(log, LOG_TAG + " >>> " + STATE_BUILD_MAP))
			# Return that the map has been built
			return TRANS_MAP_BUILT
		else: # If some error occured, try again
			rospy.sleep(10)
			rospy.logerr("Unexpected error when trying to build the map. Trying again..")
			return TRANS_MAP_NOT_BUILT

class StartBehavior(State):

	'''
	
	This class represents the implementation of the **START_BEHAVIOR** state. This state is the initial state of **START_EXPLORING** super state. During the activation of 
	current state, the robot calls the method :mod:`state_helper.ProtegeHelper.canReach` to retrieve of all location that it can reach. If the :mod:`state_helper.ProtegeHelper` cannot
	get the list of reachable location, raises an error. Else it calls :mod:`state_helper.ProtegeHelper.decide_next_location` to decide what should be the location that the robot should visit.
	
	Attributes
	----------
	protege_helper: :mod:`state_helper.ProtegeHelper`
		Class attribute to deal with `armor_py_api <https://github.com/EmaroLab/armor_py_api>`_
	interface_helper: :mod:`state_helper.InterfaceHelper`
		Class attribute to deal with the interface logic
	
	Methods
	----------
	execute(userdata)
		Class method which is called automatically when the state is active
	
	Notes
	----------
		Possible outputs:
			[TRANS_REPEAT, TRANS_PLAN_TO_LOCATION]
		Transitions:
			TRANS_REPEAT -> STATE_START_EXPLORING
			
			TRANS_GO_TO_LOCATION -> STATE_GO_TO_LOCATION
		userdata:
			Input: 
				None
			
			Output: 
				location(str)
					String representation of the next location
			
	'''
	def __init__(self, protegeHelper, interfaceHelper):
		State.__init__(self, outcomes=[TRANS_REPEAT, TRANS_GO_TO_LOCATION], output_keys = ['location'])
		self._pHelper = protegeHelper
		self._iHelper = interfaceHelper
	def execute(self, userdata):
		'''
		
		Decides which should be the next location where the robot needs to go and sends this location to the planning state		
		
		Arguments
		----------
		userdata : dictionary
			dictionary of keys for trasmitting the data between the states.

		Returns
		-------
		str
			Possible outputs.
			
		'''
		# Get from Protege the list of all reachable locations
		reachable_locations = self._pHelper.canReach()
		if reachable_locations: # Check if the returned list is not empty
			userdata.location = self._pHelper.decide_next_location() # Decide next location
			return TRANS_GO_TO_LOCATION
		else:
			rospy.sleep(3)
			rospy.logerr("Cannot obtain the list of reachable locations from armor_py_api")
			return TRANS_REPEAT
				
class GoToChargingStation(State):
	'''
	This class represents the implementation of the **GO_TO_CHARGING_STATION** state. When the previous state completes, it receives the transition
	and calls :mod:`state_helper.InterfaceHelper.plan_and_move` to follow the via points to the charging location. When the client finishes, it also calls :mod:`state_helper.ProtegeHelper.move_robot` to move the robot 
	in the ontology. After that, it checks whether the new location of the robot is the same as the charging one. If yes, then transits to **RECHARGE**.

	Attributes
	----------
	protege_helper: :mod:`state_helper.ProtegeHelper`
		Class attribute to deal with `armor_py_api <https://github.com/EmaroLab/armor_py_api>`_
	interface_helper: :mod:`state_helper.InterfaceHelper`
		Class attribute to deal with the interface logic

	Methods
	----------
	execute(userdata)
		Class method which is called automatically when the state is active

	Notes
	----------
		Possible outputs:
			[TRANS_BATTERY_FULL, TRANS_IN_CHARGING_LOCATION]
		Transitions:
			TRANS_BATTERY_FULL -> STATE_START_EXPLORING
			
			TRANS_IN_CHARGING_LOCATION -> STATE_RECHARGE
		userdata: 
			Output: 
				t0(long)
					Timestamp now
	'''
	
	def __init__(self, interface_helper, protege_helper):
		self._iHelper = interface_helper
		self._pHelper = protege_helper
		State.__init__(self, outcomes = [TRANS_BATTERY_FULL, TRANS_IN_CHARGING_LOCATION], input_keys = [], output_keys = ['t0'])
	
	def execute(self, userdata):
		'''
		
		Follows the plan given by the previous state and also moves the robot in the ontology to the next location. If the robot after that manipulation is in location containing the charging station, transits to the charging state
		
		Arguments
		----------
		userdata : dictionary
			dictionary of keys for trasmitting the data between the states.

		Returns
		-------
		str
			Possible outputs.
			
		'''
		# Retrieve the location containing the charging station
		charging_location = self._iHelper.get_charging_location()
		# Get dictionary of rooms and coordinate pairs
		dict = self._pHelper.get_coord_dict()
		# Send goal to the Controller service
		self._iHelper.plan_and_move(dict[charging_location])
		# Logging
		log = "Robot has low battery. Need to go to charging station. Therefore, following the plan to reach a room {0} ...".format(charging_location)
		rospy.loginfo(anm.tag_log(log, LOG_TAG + " >>> " + STATE_GO_TO_CHARGING_LOCATION))
		# Wait for the action server computation
		while not rospy.is_shutdown():
			# Acquire the mutex to assure data consistencies
			self._iHelper.mutex.acquire()
			try:
				if not self._iHelper.is_battery_low(): # If somehow the battery go recharged ( Not possible in the real life scenario, however, to make random sense work )
					rospy.loginfo(anm.tag_log("Miracle. Robot's battery got full...", LOG_TAG + " >>> " + STATE_GO_TO_CHARGING_LOCATION))
					# Cancel current goal
					self._iHelper.cancel_goals()
					return TRANS_BATTERY_FULL
				if self._iHelper.is_done(): # If the Controller service is done 
					# Move the robot in the ontology
					t = math.floor(rospy.Time.now().to_sec())
					self._pHelper.move_robot(charging_location, t)
					# Logging
					log = "The robot is now in location {0}".format(charging_location)
					rospy.loginfo(anm.tag_log(log, LOG_TAG + " >>> " + STATE_GO_TO_CHARGING_LOCATION))
					# If the robot is in location containing the charging station, then go to recharge state
					userdata.t0 = rospy.Time.now().to_sec()
					return TRANS_IN_CHARGING_LOCATION
					
			finally:
				# Release the mutex to unblock the `self._helper` subscription threads if they are waiting.
				self._iHelper.mutex.release()
			# Wait for a reasonably small amount of time to allow `self._helper` processing stimulus (eventually).
			rospy.sleep(LOOP_SLEEP_TIME)

class Recharge(State):
	
	'''
	
	This class represents the implementation of the **RECHARGE** state. This is the main recharging state. This state executes the simulation of the process of recharging the battery.
	There are two possible cases of the simulation. First, if the random sense is active, then this state just waits until the battery gets full again. Second, if the manual sense is active, then 
	it simulates the charging, by waiting the specified amount of time. Also, in order to better visualize, it prints to the screen the current battery charge. It is calculated by 
	calling :mod:`rospy.Time.now.to_sec` and subtracting *t0* (initial timestamp, when the battery got low, received as an input from whatever state that was active). Then, the obtained number is 
	divided by total charging time and multiplied by 100. 
	
	Attributes
	----------
	protege_helper: :mod:`state_helper.ProtegeHelper`
		Class attribute to deal with `armor_py_api <https://github.com/EmaroLab/armor_py_api>`_
	interface_helper: :mod:`state_helper.InterfaceHelper`
		Class attribute to deal with the interface logic
	
	Methods
	----------
	execute(userdata)
		Class method which is called automatically when the state is active
	non_blocking_wait(t0)
		Class method that does non blocking wait for some amount of time
	Notes
	----------
		Possible outputs:
			[TRANS_BATTERY_FULL, TRANS_BATTERY_LOW]
		Transitions:
			TRANS_BATTERY_FULL -> STATE_START_EXPLORING
			
			TRANS_BATTERY_LOW -> STATE_RECHARGE
		userdata:
			Input: 
				t0(long)
					Timestamp now
			Output: 
				None
					
	'''
	
	def __init__(self, interface_helper):
		State.__init__(self, outcomes=[TRANS_BATTERY_FULL, TRANS_BATTERY_LOW], input_keys = ['t0'])
		self._iHelper = interface_helper
		self.charging_time = self._iHelper.charging_time
	def execute(self, userdata):
		'''
		
		Simulates the recharging of the battery by waiting for the amount of time specified in the parameter server. Based on the timestamp given by the previous states when the battery got low and the charging time, calculates and prints the current battery's percent.
		Also, if the random sense is active, just waits until the battery's topic receives a message that battery is full.
		
		Arguments
		----------
		userdata : dictionary
			dictionary of keys for trasmitting the data between the states.

		Returns
		-------
		str
			Possible outputs.
			
		'''
		rospy.sleep(0.1)
		if not self._iHelper.randomness: # If the sense is manual, then simulate like the robot is charging; 
			if self.non_blocking_wait(userdata.t0): # Simulate charging the battery
				self._iHelper.reset_battery() # Reset the battery to say that is is not low anymore
		else: # If the sense is random, then just wait until the battery topic gets informed that the battery is full
			rospy.loginfo(anm.tag_log("Waiting until the battery gets full...", LOG_TAG + " >>> " + STATE_RECHARGE))
		if not self._iHelper.is_battery_low():
			return TRANS_BATTERY_FULL
		else:
			return TRANS_BATTERY_LOW
	def non_blocking_wait(self, t0):
		'''
		
		Function that simulates the wait without blocking the main program. Also, simulates the charging procedure

		Parameters
		----------
		t0 : long
			Timestamp when the battery got low.
		Returns
		----------
		flag : bool
			If the waiting period is over or not
			
		'''
		# Calculate the current percent of battery based on the time when it got low and the charging time
		percent = (rospy.Time.now().to_sec() - t0) / self.charging_time * 100
		# Logging
		log = "Battery is {0}%".format(percent)
		rospy.loginfo(anm.tag_log(log, LOG_TAG + " >>> " + STATE_RECHARGE))
		return t0 + self.charging_time < rospy.Time.now().to_sec()


class GoToLocation(State):
	
	'''
	
	This class represents the implementation of the **GO_TO_GIVEN_LOCATION** state.  The state calls the Controller Action Server (:mod:`state_helper.InterfaceHelper.moev_base`) to follow the desired trajectory to the derired location. 
	It also checks every period specified by *LOOP_RATE_SLEEP* the current state of the battery, if it got low, it terminates and transits to **START_CHARGING_ROTUINE** state.
	
	Attributes
	----------
	protege_helper: :mod:`state_helper.ProtegeHelper`
		Class attribute to deal with `armor_py_api <https://github.com/EmaroLab/armor_py_api>`_
	interface_helper: :mod:`state_helper.InterfaceHelper`
		Class attribute to deal with the interface logic
	
	Methods
	----------
	execute(userdata)
		Class method which is called automatically when the state is active
	
	Notes
	----------
		Possible outputs:
			[TRANS_WENT_TO_LOCATION, TRANS_RECHARGING]
		Transitions:
			TRANS_WENT_TO_LOCATION -> STATE_WAIT_IN_LOCATION
			
			TRANS_RECHARGING -> STATE_START_RECHARGING_ROUTINE
		userdata:
			Input: 
				room(str):
					String representation of the location where the robot needs to go
			Output: 
				None
					
	'''
	
	# Construct this class, i.e., initialise this state.
	def __init__(self, protege_helper, interface_helper):
		self._iHelper = interface_helper
		self._pHelper = protege_helper
		State.__init__(self, outcomes=[TRANS_WENT_TO_LOCATION, TRANS_RECHARGING], input_keys=['location'], output_keys=[])

	# Define the function performed each time a transition is such to enter in this state.
	def execute(self, userdata):
		'''
		
		Call Controller service to follow the plan given by the previous state. Also, checks the battery level in the meantime to ensure the transition to the charging routine if it got low.
			
		Arguments
		----------
		userdata : dictionary
			dictionary of keys for trasmitting the data between the states.

		Returns
		-------
		str
			Possible outputs.
			
		'''
		# Start the action server for moving the robot through the planned via-points.
		
		dict = self._pHelper.get_coord_dict()
		self._iHelper.plan_and_move(dict[userdata.location])
		log = "Following the plan to reach a room {0} ...".format(userdata.location)
		rospy.loginfo(anm.tag_log(log, LOG_TAG + " >>> " + STATE_GO_TO_LOCATION))
		while not rospy.is_shutdown():
			self._iHelper.mutex.acquire()
			try:
				# rospy.loginfo(self._iHelper.is_battery_low())
				if self._iHelper.is_battery_low():
					self._iHelper.cancel_goals()
					return TRANS_RECHARGING
				if self._iHelper.is_done():
					t = math.floor(rospy.Time.now().to_sec())
					self._pHelper.move_robot(userdata.location, t)
					log = "The robot is now in location {0}".format(userdata.location)
					rospy.loginfo(anm.tag_log(log, LOG_TAG + " >>> " + STATE_GO_TO_LOCATION))
					return TRANS_WENT_TO_LOCATION
			finally:
				self._iHelper.mutex.release()
			rospy.sleep(LOOP_SLEEP_TIME)

class WaitInLocation(State):
	
	'''
	
	This class represents the implementation of the **WAIT_IN_LOCATION** state. In this state, the algorithm rotates the base joint of the arm to do the inspection of the location, where the robot is located. 
	Also, every *LOOP_SLEEP_TIME* checks the battery state to transit to **START_CHARGING_ROUTINE** state in case if it is low.
	
	Attributes
	----------
	interface_helper: :mod:`state_helper.InterfaceHelper`
		Class attribute to deal with the interface logic
	
	Methods
	----------
	execute(userdata)
		Class method which is called automatically when the state is active
	
	Notes
	----------
		Possible outputs:
			[TRANS_DONE_WAITING, TRANS_RECHARGING]
		Transitions:
			TRANS_DONE_WAITING -> TRANS_REPEAT
			
			TRANS_RECHARGING -> STATE_START_RECHARGING_ROUTINE
		userdata:
			Input: 
				None
			Output: 
				t0(long):
					Timestamp now
					
	'''
	
	def __init__(self, interface_helper):
		State.__init__(self, outcomes=[TRANS_DONE_WAITING, TRANS_RECHARGING], output_keys = ['t0'])
		self._iHelper = interface_helper
	def execute(self, userdata):
		'''
		
		Executes the inspection and waiting in the location for the amount of time specified in the parameter server. Also, checks the battery level in the meantime to ensure the transition to the charging routine if it got low.
		
		Arguments
		----------
		userdata : dictionary
			dictionary of keys for trasmitting the data between the states.

		Returns
		-------
		str
			Possible outputs.
			
		'''
		# Logging
		rospy.loginfo(anm.tag_log("Performing inspection in the location", LOG_TAG + " >>> " + STATE_WAIT_IN_LOCATION))
		self._iHelper.inspect()
		
		# Simulate non-blocking wait
		i = 0
		while i < self._iHelper.waiting_time: # Amount of time is specified as a parameter
			rospy.sleep(LOOP_SLEEP_TIME) # 10 Hz, fast enough to check the battery meanwhile
			# Acquire the mutex to ensure data consistency
			self._iHelper.mutex.acquire() 
			try:
				# If the battery is low, then transit to recharging routine
				if self._iHelper.is_battery_low():  # Higher priority
					userdata.t0 = rospy.Time.now().to_sec()
					return TRANS_RECHARGING
			finally:
				# Release the mutex
				self._iHelper.mutex.release()
			i = i + LOOP_SLEEP_TIME
		rospy.loginfo(anm.tag_log("Done inspection", LOG_TAG + " >>> " + STATE_WAIT_IN_LOCATION))
		return TRANS_DONE_WAITING

def main():
	'''
	
	Main function. This function initializes the ROS node, State Machine and helper classes. Also, it sets the initial position of the robot. In addition, constructs the state machine with correct transitions.

	Returns
	-------
	None.

	'''
	rospy.init_node("state_machine", log_level=rospy.INFO)
	sm_main = StateMachine(outcomes=[])
	protegeHelper = ProtegeHelper()
	interfaceHelper = InterfaceHelper()
	robot_pose_param = rospy.get_param(anm.PARAM_INITIAL_POSE, [0, 0])
	
	# interfaceHelper.init_robot_pose(Point(x=robot_pose_param[0], y=robot_pose_param[1]))


	with sm_main:
		StateMachine.add(STATE_BUILD_MAP, BuildMap(protegeHelper, interfaceHelper), transitions={TRANS_MAP_NOT_BUILT: STATE_BUILD_MAP, TRANS_MAP_BUILT: STATE_START_EXPLORING})
		sm_behavior = StateMachine(outcomes=[TRANS_REPEAT, TRANS_RECHARGING])
		with sm_behavior:
			StateMachine.add(STATE_START_BEHAVIOR, StartBehavior(protegeHelper, interfaceHelper), transitions={TRANS_GO_TO_LOCATION: STATE_GO_TO_LOCATION, TRANS_REPEAT: TRANS_REPEAT})
			
			# StateMachine.add(STATE_PLAN_TO_LOCATION, PlanToLocation(interfaceHelper, protegeHelper), transitions= {TRANS_RECHARGING: TRANS_RECHARGING, TRANS_PLANNED_TO_LOCATION: STATE_GO_TO_LOCATION})
			
			StateMachine.add(STATE_GO_TO_LOCATION, GoToLocation(protegeHelper, interfaceHelper), transitions = {TRANS_RECHARGING: TRANS_RECHARGING, TRANS_WENT_TO_LOCATION: STATE_WAIT_IN_LOCATION})
			
			StateMachine.add(STATE_WAIT_IN_LOCATION, WaitInLocation(interfaceHelper), transitions = {TRANS_DONE_WAITING: TRANS_REPEAT, TRANS_RECHARGING: TRANS_RECHARGING})
		StateMachine.add(STATE_START_EXPLORING, sm_behavior, transitions={TRANS_REPEAT: STATE_START_EXPLORING, TRANS_RECHARGING: STATE_RECHARGING_ROUTINE})
		

		sm_recharge = StateMachine(outcomes = [TRANS_BATTERY_FULL])
		with sm_recharge:
			# StateMachine.add(STATE_PLAN_TO_CHARGING_LOCATION, PlanToChargingStation(interfaceHelper, protegeHelper), transitions={TRANS_BATTERY_FULL: TRANS_BATTERY_FULL, TRANS_IN_CHARGING_LOCATION: STATE_RECHARGE, TRANS_GO_TO_LOCATION: STATE_GO_TO_CHARGING_LOCATION})
			
			StateMachine.add(STATE_GO_TO_CHARGING_LOCATION, GoToChargingStation(interfaceHelper, protegeHelper), transitions={TRANS_BATTERY_FULL: TRANS_BATTERY_FULL, TRANS_IN_CHARGING_LOCATION: STATE_RECHARGE})
			
			StateMachine.add(STATE_RECHARGE, Recharge(interfaceHelper), transitions={TRANS_BATTERY_FULL: TRANS_BATTERY_FULL, TRANS_BATTERY_LOW: STATE_RECHARGE})
		
		StateMachine.add(STATE_RECHARGING_ROUTINE, sm_recharge, transitions={TRANS_BATTERY_FULL: STATE_START_EXPLORING})
		
	# Create and start the introspection server for visualizing the finite state machine.
	sis = smach_ros.IntrospectionServer('sm_introspection', sm_main, '/SM_ROOT')
	sis.start()

	# Execute the state machine. Note that the `outcome` value of the main Finite State Machine is not used.
	outcome = sm_main.execute()

	# Wait for ctrl-c to stop the application
	rospy.spin()
	sis.stop()
	
# The function that get executed at start time.
if __name__ == '__main__':
    main()  # Initialise and start the ROS node.
