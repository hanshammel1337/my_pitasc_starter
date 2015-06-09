#!/usr/bin/env python

## Python includes ##
##
import sys
import numpy as np
import time
import operator
import math


## ROS includes ##
##
import rospy
import smach
import smach_ros


## npitask ##
##
from pitasc_smach.scene_smach import Scene_Smach
from pitasc_smach.state import State

from pitasc_core.robots.robot_rml import Robot_RML
from pitasc_core.chains.chain_simple_pose import Chain_SimplePose
from pitasc_core.kinematic_loop import KinematicLoop

from pitasc_core.solvers.solver_simple import *
from pitasc_core.solvers.solver_prioritized import Solver_Prioritized
from pitasc_core.solvers.solver_fmin_slsqp_comau import Solver_FMIN_SLSQP

from pitasc_core.controllers.controller_simple import Controller_P

from pitasc_core.task import Task


## STATES ##
##
class State_MoveTo(State):
    def __init__(self, state_id):
        State.__init__(self, state_id, None, scene,
            outcomes = ['done', 'preempted'],
        )

    def on_start(self, userdata):

        ## Task: Move to pose ##
        ##
        self.tasks = []

        #self.tasks.append(Task(eef_to_marker1.chains[0].symbols[0:6], [0,0,0,0,0,0], []))
        #self.tasks.append(Task(eef_to_marker2.chains[0].symbols[0:6], [0,0,0,0,0,0], []))

        self.tasks.append(Task(eef_to_marker1.chains[0].symbols[0:6] + eef_to_marker2.chains[0].symbols[0:6], [0,0,0,0,0,0]+[0,0,0,0,0,0], []))
        #self.tasks.append(Task(rob1_to_rob2.chains[0].symbols[0:6], [0,0,0,0,0,0], []))


## ROS node ##
##
def run():
    global scene, eef_to_marker1, eef_to_marker2, rob1_to_rob2

    ## ROS ##
    ##
    rospy.init_node('pitasc')


    ## Virtual Kinematic Chains ##
    ##
    print 'Setting up the kinematic chains'

    robot = Robot_RML("", use_integrator=True)

    chains = {}
    
    eef_to_marker1 = Chain_SimplePose('eef_to_marker1', 'x1', 'feature', 'marker1', 'right_link_ee')
    base_to_marker1 = Chain_SimplePose('base_to_marker1', 'o1', 'object', 'comau_rml_base', 'marker1')

    eef_to_marker2 = Chain_SimplePose('eef_to_marker2', 'x2', 'feature', 'marker2', 'left_link_ee')
    base_to_marker2 = Chain_SimplePose('base_to_marker2', 'o2', 'object', 'comau_rml_base', 'marker2')

    #rob1_to_rob2 = Chain_SimplePose('rob1_to_rob2', 'x3', 'feature', 'left_link_ee', 'right_link_ee')
    rob1_to_rob2 = Chain_SimplePose('rob1_to_rob2', 'x3', 'feature', 'right_link_ee', 'left_link_ee')


    ## Kinematic Loops ##
    ##
    print 'Setting up the kinematic loops'
    
    loops = {}

    loops['viaRightArm'] = KinematicLoop('viaRightArm',
        robot_chains=[robot.chains[0], robot.chains[1]],
        feature_chains=[base_to_marker1.chains[0], eef_to_marker1.chains[0]]
    )

    loops['viaLeftArm'] = KinematicLoop('viaLeftArm',
        robot_chains=[robot.chains[0], robot.chains[2]],
        feature_chains=[base_to_marker2.chains[0], eef_to_marker2.chains[0]]
    )

    loops['both'] = KinematicLoop('both',
        robot_chains=[robot.chains[2].get_inverse_chain(), robot.chains[1]],
        feature_chains=[rob1_to_rob2.chains[0].get_inverse_chain()]
    )
    

    ## Controllers ##
    ##
    p_controller = Controller_P(0.1)

    
    ## Solver ##
    ##
    #solver = Solver_Prioritized()
    solver = Solver_FMIN_SLSQP()
    
    
    ## Scene ##
    ##
    print 'Setting up the scene'
    scene = Scene_Smach()
    
    scene.default_controller = p_controller
    scene.solver = solver

    scene.add_drivers(eef_to_marker1.drivers)
    scene.add_drivers(base_to_marker1.drivers)
    scene.add_drivers(eef_to_marker2.drivers)
    scene.add_drivers(base_to_marker2.drivers)
    scene.add_drivers(rob1_to_rob2.drivers)
    scene.add_robot_drivers(robot.robot_drivers)

    scene.add_links(robot.links + rob1_to_rob2.links + eef_to_marker1.links + base_to_marker1.links + eef_to_marker2.links + base_to_marker2.links)

    scene.add_chains(robot.chains + rob1_to_rob2.chains + eef_to_marker1.chains + base_to_marker1.chains + eef_to_marker2.chains + base_to_marker2.chains)

    scene.add_loops(loops)
    
    robot.robot_drivers[0].set_observer(scene.update)


    ## SMACH state machine ##
    ##
    sm = smach.StateMachine(outcomes=['succeeded','aborted','preempted'])

    with sm:
        ## Add states to the container ##
        ##
        smach.StateMachine.add('State_MoveTo', State_MoveTo('State_MoveTo'),
           transitions={'done':'succeeded', 'preempted':'preempted'})


    sis = smach_ros.IntrospectionServer('server_name', sm, '/SM_ROOT')
    sis.start()


    ## Start ##
    ##
    print 'Starting the loop'
    outcome = sm.execute()
    rospy.spin()
    sis.stop()


    ## Quit ##
    ##
    print "Quitting"
    rospy.signal_shutdown("Ctrl+C")
    rospy.spin() # Start spinning again to quit statemachine



if __name__ == '__main__':
    try:
        ## If this file has been executed, run it ##
        ##
        run()
    except rospy.ROSInterruptException:
        pass

# eof
