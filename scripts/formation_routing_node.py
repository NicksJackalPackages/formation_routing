#!/usr/bin/env python
import rospy
import math
import tf2_ros
import tf.transformations
from nav_msgs.msg          import Odometry,Path
from geometry_msgs.msg     import Twist,PoseStamped
from formation_routing.msg import ZoneComplete
class formation_routing_node:
  state         = 0   #current state
  WAITING       = 1   #waiting for other robot to get into formation
  ABOUT_TO_MOVE = 2   #we're in formation, continue announcing for a bit
  MOVING        = 3   #complete waypoints within the zone
  FINISHED      = 4   #stop moving
  x    = []  # list of x positions
  y    = []  # list of y positions
  zone = []  # list of zone IDs
  wp = 0     # current waypoint index
  def __init__(self):
    self.params()
    self.subs()
    self.pubs()
    self.tf_buffer   = tf2_ros.Buffer()
    self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
    rospy.Timer(rospy.Duration(0.5), self.timerPublishPaths, oneshot=False)

  def params(self):
    self.robot_id       = rospy.get_param('~robot_id', 0)
    self.lin_vel        = rospy.get_param('~lin_vel', 0.5)
    self.ang_vel        = rospy.get_param('~ang_vel', 0.5)
    self.start_delay    = rospy.get_param('~start_delay', 3)
    self.wait_time      = rospy.get_param('~wait_time', 1)
    self.dist_req       = rospy.get_param('~dist_req', 1)
    self.map_frame      = rospy.get_param('~map_frame', "map")
    self.baselink_frame = rospy.get_param('~baselink_frame', "base_link")
    waypoints = rospy.get_param('~waypoints', -1)
    if len(waypoints) % 3 > 0:
      print("Incorrect waypoint parameter.")
      return
    print("Waypoints are: ")
    print(waypoints)
    self.x    = waypoints[0::3]
    self.y    = waypoints[1::3]
    self.zone = waypoints[2::3]
    self.wp = 0
    self.state = self.MOVING
    print(self.x)
    print(self.y)
    print(self.zone)

  def subs(self):
    rospy.Subscriber("/zone_complete", ZoneComplete, self.subscribeZoneCompletions)

  def pubs(self):
    self.pub_vel = rospy.Publisher('/jackal_velocity_controller/cmd_vel', Twist, queue_size=10)
    self.pub_zone = rospy.Publisher('/zone_complete', ZoneComplete, queue_size=1)
    self.pub_completed  = rospy.Publisher('completed_waypoints', Path, queue_size=1)
    self.pub_incomplete = rospy.Publisher('incomplete_waypoints', Path, queue_size=1)
    self.pub_current    = rospy.Publisher('current_waypoints', Path, queue_size=1)
    self.completed_waypoints  = Path()
    self.incomplete_waypoints = Path()
    self.current_waypoints    = Path()
    self.completed_waypoints.header.frame_id  = self.map_frame
    self.incomplete_waypoints.header.frame_id = self.map_frame
    self.current_waypoints.header.frame_id    = self.map_frame
    for i in range(0,len(self.x)):
      pose = PoseStamped()
      pose.header.frame_id = self.map_frame
      pose.header.stamp = rospy.get_rostime()
      pose.pose.position.x = self.x[i]
      pose.pose.position.y = self.y[i]
      self.incomplete_waypoints.poses.append(pose)

  ###### Waypoint display. ######

  def timerPublishPaths(self, event):
    self.publishCompletedWaypoints()
    self.publishIncompleteWaypoints()
    self.publishCurrentWaypoints()

  def publishCompletedWaypoints(self):
    self.completed_waypoints.header.seq = self.completed_waypoints.header.seq + 1
    self.completed_waypoints.header.stamp = rospy.get_rostime()
    poses = self.completed_waypoints.poses
    if self.wp > len(poses):
      pose = PoseStamped()
      pose.header.frame_id = self.map_frame
      pose.header.stamp = rospy.get_rostime()
      pose.pose.position.x = self.x[self.wp-1]
      pose.pose.position.y = self.y[self.wp-1]
      poses.append(pose)
      self.completed_waypoints.poses = poses
    self.pub_completed.publish(self.completed_waypoints)

  def publishIncompleteWaypoints(self):
    self.incomplete_waypoints.header.seq = self.incomplete_waypoints.header.seq + 1
    self.incomplete_waypoints.header.stamp = rospy.get_rostime()
    poses = self.incomplete_waypoints.poses
    if self.wp > len(self.x)-len(poses):
      poses.pop(0)
      self.incomplete_waypoints.poses = poses
    self.pub_incomplete.publish(self.incomplete_waypoints)

  def publishCurrentWaypoints(self):
    poses = []
    for i in range(self.wp-1, self.wp+1):
      if i < 0 or i >= len(self.x):
        continue
      pose = PoseStamped()
      pose.header.frame_id = self.map_frame
      pose.header.stamp = rospy.get_rostime()
      pose.pose.position.x = self.x[i]
      pose.pose.position.y = self.y[i]
      poses.append(pose)
    self.current_waypoints.poses = poses
    self.pub_current.publish(self.current_waypoints)

  ###### Waiting for formation ######

  def subscribeZoneCompletions(self, msg):
    print("Received a message:")
    print(self.robot_id)
    print(msg)
    if not self.state == self.WAITING or msg.robot_id == self.robot_id:
      return
    if msg.zone_id == self.zone[self.wp-1]:  #they've completed what we've completed
      self.state = self.ABOUT_TO_MOVE
      rospy.Timer(rospy.Duration(self.wait_time), self.timerStartMoving, oneshot=True)

  def timerStartMoving(self, event):
    self.state = self.MOVING

  ###### Helper functions ######

  def lookupXYYaw(self):
    try:
      trans = self.tf_buffer.lookup_transform(self.map_frame,self.baselink_frame, rospy.Time())
      quat = (trans.transform.rotation.x, trans.transform.rotation.y,
              trans.transform.rotation.z, trans.transform.rotation.w)
      eul = tf.transformations.euler_from_quaternion(quat)
      return (True, trans.transform.translation.x, trans.transform.translation.y, eul[2])
    except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
      return (False, 0, 0, 0)

  def wrapToPi(self, ang):
    pi = 3.1415926538
    while ang <= -pi:
      ang = ang + 2*pi
    while ang > pi:
      ang = ang - 2*pi
    return ang

  ###### Main functions ######
  def loop(self):
    if self.state == self.MOVING:
      # Get our current position.
      tup = self.lookupXYYaw()   #(True,x,y,yaw) or (False,0,0,0)
      if not tup[0]:
        return
      # Calculate the distance and angle change needed.
      dx = self.x[self.wp] - tup[1]
      dy = self.y[self.wp] - tup[2]
      dist = math.sqrt( dx*dx + dy*dy )
      dang = math.atan2( dy, dx ) - tup[3]
      dang = self.wrapToPi(dang)
      #print("My location: ")
      #print(tup)
      #print("Waypoint: ")
      #print( (self.x[self.wp], self.y[self.wp], (180/3.1415)*math.atan2( dy, dx)) )
      print("Target dist,ang: ")
      print( (dist, (180/3.1415)*dang) )
      # Calculate the command velocity.
      x_vel = self.lin_vel
      dang_abs = math.sqrt(dang*dang)
      if dang_abs > 0.5:
        yaw_vel = self.ang_vel
      else:
        yaw_vel = self.ang_vel*(dang_abs/0.5)
      if dang < 0:
        yaw_vel = -yaw_vel
      # Publish it.
      cmd_vel = Twist()
      cmd_vel.linear.x = x_vel
      cmd_vel.angular.z = yaw_vel
      self.pub_vel.publish(cmd_vel)
      # Check for waypoint completion.
      if dist <= self.dist_req:
        print("Waypoint completed")
        self.wp = self.wp + 1
        if self.wp == len(self.x):
          print("Finished")
          self.state = self.FINISHED
        elif self.zone[self.wp] == self.zone[self.wp-1]:
          print("Moving to next waypoint")
        else:
          print("Waiting for formation")
          self.state = self.WAITING

    elif self.state == self.WAITING:
      print("I'm waiting")
      # Publish an announcement.
      msg = ZoneComplete()
      msg.robot_id = self.robot_id
      msg.zone_id = self.zone[self.wp-1]
      self.pub_zone.publish(msg)
      # Sleep.
      rospy.sleep(0.2) #seconds

    elif self.state == self.ABOUT_TO_MOVE:
      print("I'm about to move")
      # Publish an announcement.
      msg = ZoneComplete()
      msg.robot_id = self.robot_id
      msg.zone_id = self.zone[self.wp-1]
      self.pub_zone.publish(msg)
      # Sleep.
      rospy.sleep(0.2) #seconds

    elif self.state == self.FINISHED:
      print("I'm finished")
      # Publish.

if __name__ == '__main__':
  rospy.init_node('formation_routing', anonymous=True)
  obj = formation_routing_node()
  rospy.sleep(obj.start_delay)
  r = rospy.Rate(30)  #Hz
  while not rospy.is_shutdown():
    obj.loop()
    r.sleep()
  #rospy.spin()
