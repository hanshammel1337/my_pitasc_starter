#!/usr/bin/env python

## ROS ##
import rospy
import tf

## piTaSC ##
from pitasc_core.robots.robot_rml import Robot_RML
from pitasc_core.chains.chain_simple_pose import Chain_SimplePose
from pitasc_core.kinematic_loop import KinematicLoop


from pitasc_core.solvers.solver_prioritized import Solver_Prioritized
from pitasc_core.solvers.solver_fmin_slsqp import Solver_FMIN_SLSQP
#from pitasc_core.solvers.solver_joint_middle import Solver_Jointmiddle

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
	robot = Robot_RML("rml", use_integrator=True)

	# Chain_SimplePose uses "Driver_Tflistener"
	target_to_eef  = Chain_SimplePose('target_to_eef', 'x1', 'feature', 'marker1', 'right_link_ee')
	base_to_target = Chain_SimplePose('base_to_target', 'o1', 'object', 'rml_comau_base', 'marker1')
	
	#########################################################################################
	##---------Kinematic Loops ##
	# SYNTAX: KinematicLoop(self, name, robot_chains, feature_chains):
	print 'Setting up the kinematic loops'
	loops = {}
	# SYNTAX: KinematicLoop(self, name, robot_chains, feature_chains):
	loops['l1'] = KinematicLoop('l1',
		robot_chains=[robot.chains[0],robot.chains[1]],
		feature_chains=[base_to_target.chains[0], target_to_eef.chains[0]]
	)
	#########################################################################################
	##---------Tasks ##
	# SYNTAX Task(self, symbols, desired, controllers = []):
	task = []
	task.append( Task( target_to_eef.chains[0].symbols[0:3] ,  [0,0,0] , []) )

	#########################################################################################
	##---------Controller ##
	p_controller = Controller_P(0.001)

	##---------Solver ##
	#solver = Solver_FMIN_SLSQP()
	solver = Solver_Prioritized()
	#solver = Solver_Jointmiddle()

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