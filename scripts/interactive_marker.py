#!/usr/bin/env python3

"""
Copyright (c) 2011, Willow Garage, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the Willow Garage, Inc. nor the names of its
      contributors may be used to endorse or promote products derived from
      this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE
LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
SUBSTITUTE GOODS OR SERVICES LOSS OF USE, DATA, OR PROFITS OR BUSINESS
INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
POSSIBILITY OF SUCH DAMAGE.
"""

import rospy
import copy

from interactive_markers.interactive_marker_server import *
from interactive_markers.menu_handler import *
from visualization_msgs.msg import *
from geometry_msgs.msg import Point, PoseStamped, Quaternion
from tf.broadcaster import TransformBroadcaster

from random import random
from math import sin
from IPython import embed

server = None
menu_handler = MenuHandler()
br = None
counter = 0

pub_pose=rospy.Publisher('move_abs',PoseStamped, queue_size=2)


def frameCallback( msg ):
    global counter, br
    time = rospy.Time.now()
    #br.sendTransform( (0, 0, sin(counter/140.0)*2.0), (0, 0, 0, 1.0), time, "world", "moving_frame" )

    #counter += 1

def processFeedback( feedback ):
    s = "Feedback from marker '" + feedback.marker_name
    s += "' / control '" + feedback.control_name + "'"
    p=PoseStamped()
    p.header.frame_id='world'
    p.header.stamp=rospy.Time.now()
    p.pose=feedback.pose
    sq_norm=(p.pose.orientation.w*p.pose.orientation.w+p.pose.orientation.x*p.pose.orientation.x+
        p.pose.orientation.y*p.pose.orientation.y+p.pose.orientation.z*p.pose.orientation.z)
    print(p.pose)

    if sq_norm>1.01  or sq_norm<0.99:
        print(sq_norm)
    else:
        pub_pose.publish(p)


    mp = ""
    if feedback.mouse_point_valid:
        mp = " at " + str(feedback.mouse_point.x)
        mp += ", " + str(feedback.mouse_point.y)
        mp += ", " + str(feedback.mouse_point.z)
        mp += " in frame " + feedback.header.frame_id

    # if feedback.event_type == InteractiveMarkerFeedback.BUTTON_CLICK:
    #     rospy.loginfo( s + ": button click" + mp + "." )
    # elif feedback.event_type == InteractiveMarkerFeedback.MENU_SELECT:
    #     rospy.loginfo( s + ": menu item " + str(feedback.menu_entry_id) + " clicked" + mp + "." )
    # elif feedback.event_type == InteractiveMarkerFeedback.POSE_UPDATE:
    #     rospy.loginfo( s + ": pose changed")    
# TODO
#          << "\nposition = "
#          << feedback.pose.position.x
#          << ", " << feedback.pose.position.y
#          << ", " << feedback.pose.position.z
#          << "\norientation = "
#          << feedback.pose.orientation.w
#          << ", " << feedback.pose.orientation.x
#          << ", " << feedback.pose.orientation.y
#          << ", " << feedback.pose.orientation.z
#          << "\nframe: " << feedback.header.frame_id
#          << " time: " << feedback.header.stamp.sec << "sec, "
#          << feedback.header.stamp.nsec << " nsec" )
    # elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_DOWN:
    #     rospy.loginfo( s + ": mouse down" + mp + "." )
    # elif feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP:
    #     rospy.loginfo( s + ": mouse up" + mp + "." )
    server.applyChanges()

def alignMarker( feedback ):
    pose = feedback.pose

    pose.position.x = round(pose.position.x-0.5)+0.5
    pose.position.y = round(pose.position.y-0.5)+0.5

    rospy.loginfo( feedback.marker_name + ": aligning position = " + str(feedback.pose.position.x) + "," + str(feedback.pose.position.y) + "," + str(feedback.pose.position.z) + " to " +
                                                                     str(pose.position.x) + "," + str(pose.position.y) + "," + str(pose.position.z) )

    server.setPose( feedback.marker_name, pose )
    server.applyChanges()

def rand( min_, max_ ):
    return min_ + random()*(max_-min_)

def makeBox( msg ):
    marker = Marker()

    marker.type = Marker.CUBE
    marker.scale.x = msg.scale * 0.45
    marker.scale.y = msg.scale * 0.45
    marker.scale.z = msg.scale * 0.45
    marker.color.r = 0.5
    marker.color.g = 0.5
    marker.color.b = 0.5
    marker.color.a = 1.0

    return marker

def makeBoxControl( msg ):
    control =  InteractiveMarkerControl()
    control.always_visible = True
    control.markers.append( makeBox(msg) )
    msg.controls.append( control )
    return control

def saveMarker( int_marker ):
  server.insert(int_marker, processFeedback)


#####################################################################
# Marker Creation

def make6DofMarker( fixed, interaction_mode, position, orientation, show_6dof = False):
    int_marker = InteractiveMarker()
    int_marker.header.frame_id = "world"
    int_marker.pose.position = position
    int_marker.pose.orientation = orientation
    int_marker.scale = 0.1

    int_marker.name = "simple_6dof"
    int_marker.description = "Simple 6-DOF Control"

    # insert a box
    makeBoxControl(int_marker)
    int_marker.controls[0].interaction_mode = interaction_mode

    if fixed:
        int_marker.name += "_fixed"
        int_marker.description += "\n(fixed orientation)"

    if interaction_mode != InteractiveMarkerControl.NONE:
        control_modes_dict = { 
                          InteractiveMarkerControl.MOVE_3D : "MOVE_3D",
                          InteractiveMarkerControl.ROTATE_3D : "ROTATE_3D",
                          InteractiveMarkerControl.MOVE_ROTATE_3D : "MOVE_ROTATE_3D" }
        int_marker.name += "_" + control_modes_dict[interaction_mode]
        int_marker.description = "3D Control"
        if show_6dof: 
          int_marker.description += " + 6-DOF controls"
        int_marker.description += "\n" + control_modes_dict[interaction_mode]
    
    if show_6dof: 
        # control = InteractiveMarkerControl()
        # control.orientation.w = 1
        # control.orientation.x = 1
        # control.orientation.y = 0
        # control.orientation.z = 0
        # control.name = "rotate_x"
        # control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        # if fixed:
        #     control.orientation_mode = InteractiveMarkerControl.FIXED
        # int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 1
        control.orientation.y = 0
        control.orientation.z = 0
        control.name = "move_x"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "rotate_z"
        control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 1
        control.orientation.z = 0
        control.name = "move_z"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

        # control = InteractiveMarkerControl()
        # control.orientation.w = 1
        # control.orientation.x = 0
        # control.orientation.y = 0
        # control.orientation.z = 1
        # control.name = "rotate_y"
        # control.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        # if fixed:
        #     control.orientation_mode = InteractiveMarkerControl.FIXED
        # int_marker.controls.append(control)

        control = InteractiveMarkerControl()
        control.orientation.w = 1
        control.orientation.x = 0
        control.orientation.y = 0
        control.orientation.z = 1
        control.name = "move_y"
        control.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        if fixed:
            control.orientation_mode = InteractiveMarkerControl.FIXED
        int_marker.controls.append(control)

    server.insert(int_marker, processFeedback)
    menu_handler.apply( server, int_marker.name )

if __name__=="__main__":
    rospy.init_node("basic_controls")
    br = TransformBroadcaster()
    
    # create a timer to update the published transforms
    rospy.Timer(rospy.Duration(0.01), frameCallback)

    server = InteractiveMarkerServer("basic_controls")

    menu_handler.insert( "First Entry", callback=processFeedback )
    menu_handler.insert( "Second Entry", callback=processFeedback )
    sub_menu_handle = menu_handler.insert( "Submenu" )
    menu_handler.insert( "First Entry", parent=sub_menu_handle, callback=processFeedback )
    menu_handler.insert( "Second Entry", parent=sub_menu_handle, callback=processFeedback )
  
    
    position = Point( 0, 0, 0.2)
    orientation = Quaternion(0,1,0,0)
    make6DofMarker( False, InteractiveMarkerControl.MOVE_ROTATE_3D, position,orientation, True )
   

    server.applyChanges()

    rospy.spin()
