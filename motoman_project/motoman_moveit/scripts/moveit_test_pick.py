#!/usr/bin/env python
import sys
import copy
import rospy
import moveit_commander
import moveit_msgs.msg
import geometry_msgs.msg
from math import pi, atan2, radians
from std_msgs.msg import String, Bool, Float64,Int8

from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose, Vector3
from moveit_msgs.msg import AttachedCollisionObject, CollisionObject
from tf.transformations import *
from moveit_commander.conversions import pose_to_list
import random
def gripper(state=False):
  msg = Bool()
  pub = rospy.Publisher('/twintool/gripper', Bool, queue_size=2)
  while pub.get_num_connections() < 1:
      pass

  if state:
    msg.data = True
    pub.publish(msg);pub.publish(msg);

  else:
    msg.data = False
    pub.publish(msg);pub.publish(msg);

def gripper_profile(profile): #0 all low, 1 grip, 2 blow air
  msg = Int8()
  pub = rospy.Publisher('/twintool/gripper_profile', Int8, queue_size=2)
  while pub.get_num_connections() < 1:
      pass

  msg.data = profile
  pub.publish(msg);pub.publish(msg)

def vacuum(state): #0 all low, 1 grip, 2 blow air
  msg = Int8()
  pub = rospy.Publisher('/twintool/vacuum', Int8, queue_size=2)
  while pub.get_num_connections() < 1:
      pass

  msg.data = state
  pub.publish(msg);pub.publish(msg)
 

def twintool_change_ee(state=False):  # true : gripper , false : vacuum
  msg = Float64()
  pub = rospy.Publisher('/twintool/joint1_controller/command', Float64, queue_size=2)
  while pub.get_num_connections() < 1:
      pass

  if state:
    msg.data = -3.1415
    pub.publish(msg);pub.publish(msg)

  else:
    msg.data = 0.0
    pub.publish(msg);pub.publish(msg)


def all_close(goal, actual, tolerance):
  """
  Convenience method for testing if a list of values are within a tolerance of their counterparts in another list
  @param: goal       A list of floats, a Pose or a PoseStamped
  @param: actual     A list of floats, a Pose or a PoseStamped
  @param: tolerance  A float
  @returns: bool
  """
  all_equal = True
  if type(goal) is list:
    for index in range(len(goal)):
      if abs(actual[index] - goal[index]) > tolerance:
        return False

  elif type(goal) is geometry_msgs.msg.PoseStamped:
    return all_close(goal.pose, actual.pose, tolerance)

  elif type(goal) is geometry_msgs.msg.Pose:
    return all_close(pose_to_list(goal), pose_to_list(actual), tolerance)

  return True


class MoveGroupPythonIntefaceTutorial(object):
  """MoveGroupPythonIntefaceTutorial"""

  def __init__(self):
    super(MoveGroupPythonIntefaceTutorial, self).__init__()

    # BEGIN_SUB_TUTORIAL setup
    ##
    # First initialize `moveit_commander`_ and a `rospy`_ node:
    moveit_commander.roscpp_initialize(sys.argv)
    rospy.init_node('sia5_bin', anonymous=True)

    # Instantiate a `RobotCommander`_ object. Provides information such as the robot's
    # kinematic model and the robot's current joint states
    robot = moveit_commander.RobotCommander()

    # Instantiate a `PlanningSceneInterface`_ object.  This provides a remote interface
    # for getting, setting, and updating the robot's internal understanding of the
    # surrounding world:
    scene = moveit_commander.PlanningSceneInterface()

    # Instantiate a `MoveGroupCommander`_ object.  This object is an interface
    # to a planning group (group of joints).  In this tutorial the group is the primary
    # arm joints in the Panda robot, so we set the group's name to "panda_arm".
    # If you are using a different robot, change this value to the name of your robot
    # arm planning group.
    # This interface can be used to plan and execute motions:
    group_name = "arm"
    move_group = moveit_commander.MoveGroupCommander(group_name)

    # Create a `DisplayTrajectory`_ ROS publisher which is used to display
    # trajectories in Rviz:
    display_trajectory_publisher = rospy.Publisher('/move_group/display_planned_path',
                                                   moveit_msgs.msg.DisplayTrajectory,
                                                   queue_size=20)

    # END_SUB_TUTORIAL

    # BEGIN_SUB_TUTORIAL basic_info
    ##
    # Getting Basic Information
    # ^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can get the name of the reference frame for this robot:
    planning_frame = move_group.get_planning_frame()
    print "============ Planning frame: %s" % planning_frame

    # We can also print the name of the end-effector link for this group:
    eef_link = move_group.get_end_effector_link()
    print "============ End effector link: %s" % eef_link

    # We can get a list of all the groups in the robot:
    group_names = robot.get_group_names()
    print "============ Available Planning Groups:", robot.get_group_names()

    # Sometimes for debugging it is useful to print the entire state of the
    # robot:
    print "============ Printing robot state"
    print robot.get_current_state()
    print ""
    # END_SUB_TUTORIAL

    # Misc variables
    self.box_name = ''
    self.robot = robot
    self.scene = scene
    self.move_group = move_group
    self.display_trajectory_publisher = display_trajectory_publisher
    self.planning_frame = planning_frame
    self.eef_link = eef_link
    self.group_names = group_names

  def go_to_joint_state(self):
    move_group = self.move_group
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = pi/2
    joint_goal[1] = 0
    joint_goal[2] = 0
    joint_goal[3] = 0
    joint_goal[4] = 0
    joint_goal[5] = 0
    joint_goal[6] = 0

    # The go command can be called with joint values, poses, or without any
    # parameters if you have already set the pose or joint target for the group
    move_group.go(joint_goal, wait=True)

    # Calling ``stop()`` ensures that there is no residual movement
    move_group.stop()

    # END_SUB_TUTORIAL

    # For testing:
    current_joints = move_group.get_current_joint_values()
    return all_close(joint_goal, current_joints, 0.01)

  def go_to_pose_goal(self):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    # BEGIN_SUB_TUTORIAL plan_to_pose
    ##
    # Planning to a Pose Goal
    # ^^^^^^^^^^^^^^^^^^^^^^^
    # We can plan a motion for this group to a desired pose for the
    # end-effector:
    pose_goal = geometry_msgs.msg.Pose()
    pose_goal.orientation.w = 1.0
    pose_goal.position.x = 0.4
    pose_goal.position.y = 0.1
    pose_goal.position.z = 0.4

    move_group.set_pose_target(pose_goal)

    # Now, we call the planner to compute the plan and execute it.
    plan = move_group.go(wait=True)
    # Calling `stop()` ensures that there is no residual movement
    move_group.stop()
    print move_group
    raw_input()
    # It is always good to clear your targets after planning with poses.
    # Note: there is no equivalent function for clear_joint_value_targets()
    move_group.clear_pose_targets()
    current_pose = self.move_group.get_current_pose().pose
    return all_close(pose_goal, current_pose, 0.01)

  def form_attach_collision_object_msg(self, link_name, object_name, size, pose):
    obj = AttachedCollisionObject()
    # The CollisionObject will be attached with a fixed joint to this link
    obj.link_name = link_name

    # This contains the actual shapes and poses for the CollisionObject
    # to be attached to the link
    col_obj = CollisionObject()
    col_obj.id = object_name
    col_obj.header.frame_id = link_name
    col_obj.header.stamp = rospy.Time.now()
    sp = SolidPrimitive()
    sp.type = sp.BOX
    sp.dimensions = [0.0]*3
    sp.dimensions[0] = size.x
    sp.dimensions[1] = size.y
    sp.dimensions[2] = size.z
    col_obj.primitives = [sp]
    col_obj.primitive_poses = [pose]

    # Adds the object to the planning scene. If the object previously existed, it is replaced.
    col_obj.operation = col_obj.ADD

    obj.object = col_obj

    # The weight of the attached object, if known
    obj.weight = 0.0
    return obj

  def form_remove_all_attached_msg(self, link_name):
    obj = AttachedCollisionObject()
    # The CollisionObject will be attached with a fixed joint to this link
    obj.link_name = link_name

    col_obj = CollisionObject()

    # If action is remove and no object.id is set, all objects
    # attached to the link indicated by link_name will be removed
    col_obj.operation = col_obj.REMOVE
    obj.object = col_obj

    return obj

  def home_pose(self):
    move_group = self.move_group
    print(move_group.get_current_pose())
    joint_goal = move_group.get_current_joint_values()
    joint_goal = [0, 0, 0, 0, 0, 0, 0]
    move_group.go(joint_goal, wait=True)
    move_group.stop()

  def home_pick_pose(self):
    move_group = self.move_group
    print(move_group.get_current_pose())
    joint_goal = move_group.get_current_joint_values()
    joint_goal[0] = 0
    joint_goal[1] = 0
    joint_goal[2] = 0
    joint_goal[3] = 0
    joint_goal[4] = 0
    joint_goal[5] = -pi/2
    joint_goal[6] = 0
    move_group.go(joint_goal, wait=True)
    move_group.stop()

  def move_test_pick(self,point_x,point_y,point_z,goalpoint_x,goalpoint_y,goalpoint_z,):
    print("move test pick: ", point_x,point_y,point_z,goalpoint_x,goalpoint_y,goalpoint_z,)
    
    THRES_DIS = 0.49
    G_ORENT = 80 # max about 100
    obj_distance = math.sqrt( (point_y**2)+(point_x**2) )
    if ( obj_distance > THRES_DIS) :
      q1 = quaternion_from_euler(0 , radians(80 - (obj_distance - THRES_DIS)*G_ORENT), atan2(point_y, point_x))
    else :
      q1 = quaternion_from_euler(0 ,radians(90), atan2(point_y, point_x))
      
    obj_distance = math.sqrt( (goalpoint_y**2)+(goalpoint_x**2) )
    if ( obj_distance > THRES_DIS) :
      q2 = quaternion_from_euler(0 ,radians(80 - (obj_distance - THRES_DIS)*G_ORENT), atan2(goalpoint_y, goalpoint_x))
    else :
      q2 = quaternion_from_euler(0 ,radians(90), atan2(goalpoint_y, goalpoint_x)) 

    move_group = self.move_group

    try:
        # self.home_pose()
        vacuum(0)
        twintool_change_ee(True)
        # gripper(True)
        gripper_profile(0)
        rospy.sleep(0.2)
        gripper(False)
        
        self.home_pick_pose()

        pose_goal = self.move_group.get_current_pose().pose
        pose_goal.position.x = point_x
        pose_goal.position.y = point_y
        pose_goal.position.z = point_z + 0.10
        pose_goal.orientation.x = q1[0]
        pose_goal.orientation.y = q1[1]
        pose_goal.orientation.z = q1[2]
        pose_goal.orientation.w = q1[3]
        move_group.set_pose_target(pose_goal)
        for i in range (4):
          plan = move_group.plan()

        # display_trajectory(plan)
        # rospy.sleep (1.0)
        move_group.execute(plan, wait=True)
        rospy.sleep(0.5)

        pose_goal = self.move_group.get_current_pose().pose
        pose_goal.position.x = point_x
        pose_goal.position.y = point_y
        pose_goal.position.z = point_z
        pose_goal.orientation.x = q1[0]
        pose_goal.orientation.y = q1[1]
        pose_goal.orientation.z = q1[2]
        pose_goal.orientation.w = q1[3]
        move_group.set_pose_target(pose_goal)
        for i in range (4):
          plan = move_group.plan()
        move_group.execute(plan, wait=True)
        # move_group.stop()

        print('wait enter')
        # raw_input()
        
        rospy.sleep(1)
        gripper(True)
        rospy.sleep(1.5)

        # post-pick
        pose_goal = self.move_group.get_current_pose().pose
        pose_goal.position.x = point_x
        pose_goal.position.y = point_y
        pose_goal.position.z = point_z + 0.10
        pose_goal.orientation.x = q1[0]
        pose_goal.orientation.y = q1[1]
        pose_goal.orientation.z = q1[2]
        pose_goal.orientation.w = q1[3]
        move_group.set_pose_target(pose_goal)
        for i in range (4):
          plan = move_group.plan()
        move_group.execute(plan, wait=True)
        # plan = move_group.go(wait=True)

        # joint_goal = move_group.get_current_joint_values()
        # joint_goal = [0.2428, 0.2472, -0.1123, 0.2638, -0.0286, -1.5894, -0.1344]
        # move_group.go(joint_goal, wait=True)
        # move_group.stop()

        # pre - place
        pose_goal = self.move_group.get_current_pose().pose
        pose_goal.position.x = goalpoint_x
        pose_goal.position.y = goalpoint_y
        pose_goal.position.z = goalpoint_z + 0.1
        pose_goal.orientation.x = q2[0]
        pose_goal.orientation.y = q2[1]
        pose_goal.orientation.z = q2[2]
        pose_goal.orientation.w = q2[3]
        move_group.set_pose_target(pose_goal)
        for i in range (4):
          plan = move_group.plan()
        move_group.execute(plan, wait=True)
        rospy.sleep(0.1)
        
         # place
        pose_goal = self.move_group.get_current_pose().pose
        pose_goal.position.x = goalpoint_x
        pose_goal.position.y = goalpoint_y
        pose_goal.position.z = goalpoint_z
        pose_goal.orientation.x = q2[0]
        pose_goal.orientation.y = q2[1]
        pose_goal.orientation.z = q2[2]
        pose_goal.orientation.w = q2[3]
        move_group.set_pose_target(pose_goal)
        for i in range (4):
          plan = move_group.plan()
        move_group.execute(plan, wait=True)

        rospy.sleep(0.5)
        gripper(False)
        rospy.sleep(1.5)

        # post-place
        pose_goal = self.move_group.get_current_pose().pose
        pose_goal.position.x = goalpoint_x
        pose_goal.position.y = goalpoint_y
        pose_goal.position.z = goalpoint_z + 0.1
        pose_goal.orientation.x = q2[0]
        pose_goal.orientation.y = q2[1]
        pose_goal.orientation.z = q2[2]
        pose_goal.orientation.w = q2[3]
        move_group.set_pose_target(pose_goal)
        for i in range (4):
          plan = move_group.plan()
        move_group.execute(plan, wait=True)

        move_group.stop()

    except KeyboardInterrupt:
      pass

  def move_vacuum_pick(self,point_x,point_y,point_z,goalpoint_x,goalpoint_y,goalpoint_z,): #pre-pick>vacuum>pre-place>place
    print ("move vacuum pick")
    move_group = self.move_group

    try:
        self.home_pose()
        # set vacuum config
        twintool_change_ee(False)
        # gripper_profile(7)
        # gripper(True)
        vacuum(0)
        self.home_pick_pose()
        
        pose_goal = self.move_group.get_current_pose().pose
        pose_goal.position.x = point_x
        pose_goal.position.y = point_y
        pose_goal.position.z = point_z + 0.06
        move_group.set_pose_target(pose_goal)
        for i in range (3) : 
          plan = move_group.go(wait=True)

        # pick
        pose_goal = self.move_group.get_current_pose().pose
        pose_goal.position.x = point_x
        pose_goal.position.y = point_y
        pose_goal.position.z = point_z
        move_group.set_pose_target(pose_goal)
        for i in range (3) : 
          plan = move_group.go(wait=True)
        # move_group.stop()

        rospy.sleep(1)
        vacuum(1)
        rospy.sleep(3)

        # post-pick
        pose_goal = self.move_group.get_current_pose().pose
        pose_goal.position.x = point_x
        pose_goal.position.y = point_y
        pose_goal.position.z = point_z + 0.06
        move_group.set_pose_target(pose_goal)
        for i in range (3) : 
          plan = move_group.go(wait=True)

        # pre-place
        pose_goal = self.move_group.get_current_pose().pose
        pose_goal.position.x = goalpoint_x - 0.01
        pose_goal.position.y = goalpoint_y
        pose_goal.position.z = goalpoint_z + 0.07
        move_group.set_pose_target(pose_goal)
        for i in range (3) : 
          plan = move_group.go(wait=True)

         # place
        pose_goal = self.move_group.get_current_pose().pose
        pose_goal.position.x = goalpoint_x
        pose_goal.position.y = goalpoint_y
        pose_goal.position.z = goalpoint_z 
        move_group.set_pose_target(pose_goal)
        for i in range (3) : 
          plan = move_group.go(wait=True)

        rospy.sleep(1)
        vacuum(2)
        vacuum(2)
        rospy.sleep(2.5)
        vacuum(0) 
        vacuum(0)
        rospy.sleep(1)

        # post-place
        pose_goal = self.move_group.get_current_pose().pose
        pose_goal.position.x = goalpoint_x - 0.01
        pose_goal.position.y = goalpoint_y
        pose_goal.position.z = goalpoint_z + 0.07
        move_group.set_pose_target(pose_goal)
        for i in range (3) : 
          plan = move_group.go(wait=True)

        self.home_pick_pose()

        # twintool_change_ee(True)
        # gripper(False)
        move_group.stop()
        current_pose = self.move_group.get_current_pose().pose
        all_close(pose_goal, current_pose, 0.01)

    except KeyboardInterrupt:
      pass

  def plan_cartesian_path(self, scale=1):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    # BEGIN_SUB_TUTORIAL plan_cartesian_path
    ##
    # Cartesian Paths
    # ^^^^^^^^^^^^^^^
    # You can plan a Cartesian path directly by specifying a list of waypoints
    # for the end-effector to go through. If executing  interactively in a
    # Python shell, set scale = 1.0.
    ##
    waypoints = []

    wpose = move_group.get_current_pose().pose
    wpose.position.z -= scale * 0.1  # First move up (z)
    wpose.position.y += scale * 0.2  # and sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.x += scale * 0.1  # Second move forward/backwards in (x)
    waypoints.append(copy.deepcopy(wpose))

    wpose.position.y -= scale * 0.1  # Third move sideways (y)
    waypoints.append(copy.deepcopy(wpose))

    # We want the Cartesian path to be interpolated at a resolution of 1 cm
    # which is why we will specify 0.01 as the eef_step in Cartesian
    # translation.  We will disable the jump threshold by setting it to 0.0,
    # ignoring the check for infeasible jumps in joint space, which is sufficient
    # for this tutorial.
    (plan, fraction) = move_group.compute_cartesian_path(
                                       waypoints,   # waypoints to follow
                                       0.01,        # eef_step
                                       0.0)         # jump_threshold

    # Note: We are just planning, not asking move_group to actually move the robot yet:
    return plan, fraction

    # END_SUB_TUTORIAL


  def display_trajectory(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    robot = self.robot
    display_trajectory_publisher = self.display_trajectory_publisher

    # BEGIN_SUB_TUTORIAL display_trajectory
    ##
    # Displaying a Trajectory
    # ^^^^^^^^^^^^^^^^^^^^^^^
    # You can ask RViz to visualize a plan (aka trajectory) for you. But the
    # group.plan() method does this automatically so this is not that useful
    # here (it just displays the same trajectory again):
    ##
    # A `DisplayTrajectory`_ msg has two primary fields, trajectory_start and trajectory.
    # We populate the trajectory_start with our current robot state to copy over
    # any AttachedCollisionObjects and add our plan to the trajectory.
    display_trajectory = moveit_msgs.msg.DisplayTrajectory()
    display_trajectory.trajectory_start = robot.get_current_state()
    display_trajectory.trajectory.append(plan)
    # Publish
    display_trajectory_publisher.publish(display_trajectory);

    # END_SUB_TUTORIAL


  def execute_plan(self, plan):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    move_group = self.move_group

    # BEGIN_SUB_TUTORIAL execute_plan
    ##
    # Executing a Plan
    # ^^^^^^^^^^^^^^^^
    # Use execute if you would like the robot to follow
    # the plan that has already been computed:
    move_group.execute(plan, wait=True)

    # **Note:** The robot's current joint state must be within some tolerance of the
    # first waypoint in the `RobotTrajectory`_ or ``execute()`` will fail
    # END_SUB_TUTORIAL


  def wait_for_state_update(self, box_is_known=False, box_is_attached=False, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    # BEGIN_SUB_TUTORIAL wait_for_scene_update
    ##
    # Ensuring Collision Updates Are Receieved
    # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    # If the Python node dies before publishing a collision object update message, the message
    # could get lost and the box will not appear. To ensure that the updates are
    # made, we wait until we see the changes reflected in the
    # ``get_attached_objects()`` and ``get_known_object_names()`` lists.
    # For the purpose of this tutorial, we call this function after adding,
    # removing, attaching or detaching an object in the planning scene. We then wait
    # until the updates have been made or ``timeout`` seconds have passed
    start = rospy.get_time()
    seconds = rospy.get_time()
    while (seconds - start < timeout) and not rospy.is_shutdown():
      # Test if the box is in attached objects
      attached_objects = scene.get_attached_objects([box_name])
      is_attached = len(attached_objects.keys()) > 0

      # Test if the box is in the scene.
      # Note that attaching the box will remove it from known_objects
      is_known = box_name in scene.get_known_object_names()

      # Test if we are in the expected state
      if (box_is_attached == is_attached) and (box_is_known == is_known):
        return True

      # Sleep so that we give other threads time on the processor
      rospy.sleep(0.1)
      seconds = rospy.get_time()

    # If we exited the while loop without returning then we timed out
    return False
    # END_SUB_TUTORIAL


  def add_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    # BEGIN_SUB_TUTORIAL add_box
    ##
    # Adding Objects to the Planning Scene
    # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    # First, we will create a box in the planning scene at the location of the left finger:
    box_pose = geometry_msgs.msg.PoseStamped()
    box_pose.header.frame_id = "panda_leftfinger"
    box_pose.pose.orientation.w = 1.0
    box_pose.pose.position.z = 0.07 # slightly above the end effector
    box_name = "box"
    scene.add_box(box_name, box_pose, size=(0.1, 0.1, 0.1))

    # END_SUB_TUTORIAL
    # Copy local variables back to class variables. In practice, you should use the class
    # variables directly unless you have a good reason not to.
    self.box_name=box_name
    return self.wait_for_state_update(box_is_known=True, timeout=timeout)


  def attach_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    robot = self.robot
    scene = self.scene
    eef_link = self.eef_link
    group_names = self.group_names

    # BEGIN_SUB_TUTORIAL attach_object
    ##
    # Attaching Objects to the Robot
    # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    # Next, we will attach the box to the Panda wrist. Manipulating objects requires the
    # robot be able to touch them without the planning scene reporting the contact as a
    # collision. By adding link names to the ``touch_links`` array, we are telling the
    # planning scene to ignore collisions between those links and the box. For the Panda
    # robot, we set ``grasping_group = 'hand'``. If you are using a different robot,
    # you should change this value to the name of your end effector group name.
    grasping_group = 'hand'
    touch_links = robot.get_link_names(group=grasping_group)
    scene.attach_box(eef_link, box_name, touch_links=touch_links)
    # END_SUB_TUTORIAL
  
    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=True, box_is_known=False, timeout=timeout)


  def detach_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene
    eef_link = self.eef_link

    # BEGIN_SUB_TUTORIAL detach_object
    ##
    # Detaching Objects from the Robot
    # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can also detach and remove the object from the planning scene:
    scene.remove_attached_object(eef_link, name=box_name)
    # END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_known=True, box_is_attached=False, timeout=timeout)


  def remove_box(self, timeout=4):
    # Copy class variables to local variables to make the web tutorials more clear.
    # In practice, you should use the class variables directly unless you have a good
    # reason not to.
    box_name = self.box_name
    scene = self.scene

    # BEGIN_SUB_TUTORIAL remove_object
    ##
    # Removing Objects from the Planning Scene
    # ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
    # We can remove the box from the world.
    scene.remove_world_object(box_name)

    # **Note:** The object must be detached before we can remove it from the world
    # END_SUB_TUTORIAL

    # We wait for the planning scene to update.
    return self.wait_for_state_update(box_is_attached=False, box_is_known=False, timeout=timeout)


def main():
  try:
    print ""
    print "----------------------------------------------------------"
    print "Welcome to Motoman Robot for Food Industrial"
    print "----------------------------------------------------------"
    print "Press Ctrl-D to exit at any time"
    print "============ Press `Enter` to begi commander ..."
    # raw_input()
    tutorial = MoveGroupPythonIntefaceTutorial()

    # tutorial.move_test_pick(0.38, -0.228586, 0.09 , 0.460, 0.15, 0.054 )
    # tutorial.home_pose()
    
    # rospy.sleep(2)
    for i in range(10) :
      tutorial.move_test_pick( 0.455 + random.uniform(-0.05, 0.05), 0.15 + random.uniform(-0.05, 0.05), 0.1, 0.50 + random.uniform(-0.05, 0.05), -0.28 + random.uniform(-0.05, 0.05), 0.08)
      # tutorial.home_pose()
      # tutorial.home_pick_pose()
      # tutorial.move_test_pick( 0.40, -0.228586, 0.06,  0.455, 0.15, 0.06)
      # tutorial.home_pick_pose()

    # for i in range(1) :
    #   tutorial.move_vacuum_pick(0.32517,0.26,0.04,0.460,0.14,0.04)
    #   tutorial.move_vacuum_pick(0.460,0.14,0.04,0.32517,0.26,0.04)
    

    print "============ Python tutorial demo complete!"
  except rospy.ROSInterruptException:
    return
  except KeyboardInterrupt:
    return

if __name__ == '__main__':
  main()
