#!/usr/bin/env python

import rospy
import random
import sys
from geometry_msgs.msg import PoseStamped, Pose, PoseArray, Point, Quaternion
from pips_msgs.msg import PathArray
from nav_msgs.msg import Path
from std_msgs.msg import Header, ColorRGBA
from cv_bridge import CvBridge, CvBridgeError
from pips.srv import GenerateDepthImage
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from copy import deepcopy

from gazebo_msgs.msg import ModelStates, ModelState
from gazebo_msgs.srv import SetModelState

#http://answers.ros.org/question/10330/whats-the-best-way-to-convert-a-ros-message-to-a-string-or-xml/
def ros2xml(msg, name, depth=0):
  xml = "";
  tabs = "\t"*depth

  if hasattr(msg, "_type"):
    type = msg._type
    xml = xml + tabs + "<" + name + " type=\"" + type + "\">\n"

    try:
      for slot in msg.__slots__:
        xml = xml + ros2xml(getattr(msg, slot), slot, depth=depth+1)
    except:
        xml = xml + tabs + str(msg)
    xml = xml + tabs + "</" + name + ">\n"
  else:
    xml = xml + tabs + "<" + name + ">" + str(msg) + "</" + name + ">\n"
  return xml
  

class Prototype():

  def statesCallback(self, data):
    ## Check if our model exists yet
    model_name = rospy.get_param('initial_pose/model_name')
    if(model_name in data.name):
      ppose = rospy.get_param('initial_pose')
      pposition = rospy.get_param('initial_pose/pose/position')
      porientation = rospy.get_param('initial_pose/pose/orientation')

      
      position = Point(pposition['x'],pposition['y'],pposition['z'])
      orientation = Quaternion(x=porientation['x'],y=porientation['y'], z=porientation['z'], w=porientation['w'])
      pose = Pose(position=position, orientation=orientation)
      
      state = ModelState(model_name=model_name, pose=pose)

      #print "State:\n", state
      
      response = self.modelStateService(state)
      #print response
      if(response.success):
        rospy.loginfo("Successfully set model starting pose")
        rospy.signal_shutdown("Successfully set model starting pose")

      ## Kill node



  def __init__(self):
    rospy.init_node('delay_set_model_pose')
    #rospy.on_shutdown(self.shutdown)
    
    rospy.loginfo("delay_set_model_pose started")
        
    self.serviceName = 'gazebo/set_model_state';
    self.topicName = 'gazebo/model_states';
    
    self.stateSub = rospy.Subscriber(self.topicName, ModelStates, self.statesCallback)
    
    rospy.loginfo("Waiting for service...")
    rospy.wait_for_service(self.serviceName)
    self.modelStateService = rospy.ServiceProxy(self.serviceName, SetModelState)
    rospy.loginfo("Service found...")

    rospy.spin()
    
  


if __name__ == '__main__':
  try:
    Prototype()
  except rospy.ROSInterruptException:
    rospy.loginfo("exception")
