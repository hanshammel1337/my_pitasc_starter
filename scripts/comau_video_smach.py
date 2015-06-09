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
from pitasc_core.scene_simple import Scene_Simple
from pitasc_smach.state import State

from pitasc_core.robots.robot_rml import Robot_RML
from pitasc_core.chains.chain_simple_pose import Chain_SimplePose
from pitasc_core.kinematic_loop import KinematicLoop

from pitasc_core.solvers.solver_simple import *
from pitasc_core.solvers.solver_prioritized import Solver_Prioritized

from pitasc_core.controllers.controller_simple import Controller_P

from pitasc_core.task import Task




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
    
    eef_to_marker1 = Chain_SimplePose('eef_to_marker1', 'x1', 'feature', 'marker2', 'right_link_ee')
    base_to_marker1 = Chain_SimplePose('base_to_marker1', 'o1', 'object', 'comau_rml_base', 'marker2')

    eef_to_marker2 = Chain_SimplePose('eef_to_marker2', 'x2', 'feature', 'marker1', 'left_link_ee')
    base_to_marker2 = Chain_SimplePose('base_to_marker2', 'o2', 'object', 'comau_rml_base', 'marker1')

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

    tasks = []
    tasks.append( Task(eef_to_marker1.chains[0].symbols[0:6] + eef_to_marker2.chains[0].symbols[0:6], [0,0,0,0,0,0]+[0,0,0,0,0,0], []))

    ## Controllers ##
    ##
    p_controller = Controller_P(0.1)

    
    ## Solver ##
    ##
    solver = Solver_Prioritized()
    
    
    ## Scene ##
    ##
    print 'Setting up the scene'
    scene = Scene_Simple()
    
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
    
    
    scene.build_symbols()
    scene.set_tasks(tasks)
    robot.robot_drivers[0].set_observer(scene.update)
    rospy.spin()



if __name__ == '__main__':
    try:
        ## If this file has been executed, run it ##
        ##
        run()
    except rospy.ROSInterruptException:
        pass

# eof
