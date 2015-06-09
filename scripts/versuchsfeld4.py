#!/usr/bin/env python

## ROS ##
import rospy
import tf
import numpy as np

## piTaSC ##
from pitasc_core.robots.robot_lbr4 import Robot_LBR4


from pitasc_core.chains.chain_simple_pose import Chain_SimplePose

from pitasc_core.kinematic_loop import KinematicLoop


from pitasc_core.solvers.solver_fmin_slsqp_v import Solver_FMIN_SLSQP
from pitasc_core.solvers.solver_prioritized_v import Solver_Prioritized
from pitasc_core.solvers.solver_joint_middle_v import Solver_Jointmiddle

from pitasc_core.controllers.controller_simple import Controller_P

from pitasc_core.task import Task
from pitasc_core.scene_simple import Scene_Simple


## ROS node ##
## -----------------------------------------------------------------##
def run():
	##---------ROS ##
	rospy.init_node('pitasc')

	#########################################################################################
	##---------Virtual Kinematic Chains ##
	# SYNTAX:        Chain_SimplePose(self, name, prefix, variable_type, from_frame, to_frame)
	# integrator=True  => dq's werden ueber sensor_msgs.msg/JointState gepublished
	# integrator=False => JointTrajectory wird zum roboter gesendet
	print 'Setting up the kinematic chains'
	robot = Robot_LBR4("lbr4", use_integrator=True)

	# Chain_SimplePose uses "Driver_Tflistener"
	target_to_eef  = Chain_SimplePose('target_to_eef', 'x1', 'feature', 'moving_tf', 'ee_link')
	base_to_target = Chain_SimplePose('base_to_target', 'o1', 'object', 'base_link', 'moving_tf')
	
	#########################################################################################
	##---------Kinematic Loops ##
	# SYNTAX: KinematicLoop(self, name, robot_chains, feature_chains):
	print 'Setting up the kinematic loops'
	loops = {}
	# SYNTAX: KinematicLoop(self, name, robot_chains, feature_chains):
	loops['l1'] = KinematicLoop('l1',
		robot_chains=[robot.chains[0]],
		feature_chains=[base_to_target.chains[0], target_to_eef.chains[0]]
	)
	#########################################################################################
	##---------Tasks ##
	# SYNTAX Task(self, symbols, desired, controllers = []):
	task = []
	task.append( Task( target_to_eef.chains[0].symbols[0:6] ,  [0,0,0, 0,0,0] , []) )
	#task.append( Task( target_to_eef.chains[0].symbols[0:6] ,  [0,0,0,0,0,0] , []) )
	#task.append( Task( robot.chains[0].symbols[0:7] ,  [0,0,0,0,0,0,0] , []) )

	#########################################################################################
	##---------Controller ##
	p_controller = Controller_P(0.3)

	##---------Solver ##
	#solver = Solver_Prioritized()
	#solver = Solver_Jointmiddle()
	solver = Solver_FMIN_SLSQP()
	
	## AUFGABE JOINT 1 BEGRENZEN, BEREICH 90 GRAD ##

	b1 = lambda x: np.array( (scene.measurements['joint_1'] + x[0]) - 1.26990367 )
	b2 = lambda x: np.array( 2.8407 - (scene.measurements['joint_1'] + x[0]) )

	def direct_kinematic(x):
		""" direct kinematic from base to end effector in dependency from current dq.
			x correspond to dq
		"""
		T1 = np.dot(scene.links[0].transform_offset,kin.TransformRotZ(scene.measurements['joint_1']+ x[0]))
		T2 = np.dot(scene.links[1].transform_offset,kin.TransformRotZ(scene.measurements['joint_2']+ x[1]))
		T3 = np.dot(scene.links[2].transform_offset,kin.TransformRotZ(scene.measurements['joint_3']+ x[2]))
		T4 = np.dot(scene.links[3].transform_offset,kin.TransformRotZ(scene.measurements['joint_4']+ x[3]))
		T5 = np.dot(scene.links[4].transform_offset,kin.TransformRotZ(scene.measurements['joint_5']+ x[4]))
		T6 = np.dot(scene.links[5].transform_offset,kin.TransformRotZ(scene.measurements['joint_6']+ x[5]))
		T7 = np.dot(scene.links[6].transform_offset,kin.TransformRotZ(scene.measurements['joint_7']+ x[6]))
		T8 = scene.links[7].transform
		T = T1.dot(T2).dot(T3).dot(T4).dot(T5).dot(T6).dot(T7).dot(T8)
		## return x,y,z of end effector
		return T[0,3],T[1,3],T[2,3]

	## example 2: keep end effector z value between 0.1 and 0.5 ##
	b14 = lambda x: direct_kinematic(x)[2] - 0.1
	b15 = lambda x: 0.5 - direct_kinematic(x)[2]

	## example 3: keep distance end effector to obstacle greater than 0.1 ##
	def b16(x):
		ee_x, ee_y, ee_z = direct_kinematic(x)
		distance_x = 0.3 - ee_x
		distance_y = -0.1 - ee_y
		distance_z = 0.2 - ee_z
		distance_norm = sqrt(distance_x**2+distance_y**2+distance_z**2)
		## return inequality
		## distance_norm > 0.1
		return distance_norm - 0.1

	## AUFGABE AMRKER FOLGEN und JOINT 1 in der MITTE halten ##

	#########################################################################################
	##---------Scene ##
	print 'Setting up the scene'
	scene = Scene_Simple()

	scene.default_controller = p_controller
	scene.solver = solver
	
	scene.add_drivers(target_to_eef.drivers)
	scene.add_drivers(base_to_target.drivers)
	scene.add_robot_drivers(robot.robot_drivers)

	scene.add_links(robot.links + target_to_eef.links + base_to_target.links)
	scene.add_chains(robot.chains + target_to_eef.chains + base_to_target.chains)
	scene.add_loops(loops)

	scene.build_symbols()
	scene.set_tasks(task)

	solver.add_inequality([b1,b2])
	robot.robot_drivers[0].set_observer(scene.update)



	# spin() simply keeps python from exiting until this node is stopped
	rospy.spin()
	#########################################################################################
	##---------ROS loop
	#while not rospy.is_shutdown():
	#	scene.update()


## Main
## -----------------------------------------------------------------##
if __name__ == '__main__':
	try:
		## If this file has been executed, run it ##
		run()
	except rospy.ROSInterruptException:
		pass

# eof