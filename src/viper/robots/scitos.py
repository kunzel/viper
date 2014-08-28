import logging; logger = logging.getLogger("viper." + __name__)
import viper.core.robot
import viper.core.view
import rospy

class ScitosRobot(viper.core.robot.Robot):

    def __init__(self):
        view_generator = ScitosViewGenerator()
        view_controller = ScitosViewController()
        view_transition_model = ScitosTransitionModel()
        view_evaluator = ScitosViewEvaluator()
        view_action = ScitosViewAction()
        super(ScitosRobot, self).__init__(view_generator,
                                          view_controller,
                                          view_transition_model,
                                          view_evaluator,
                                          view_action)
        
##########################################################################

class ScitosView(viper.core.view.View):

    def __init__(self, robot_pose, ptu_state, ptu_pose):
        self._robot_pose = robot_pose
        self._ptu_state = ptu_state
        self._ptu_pose = ptu_pose
        self._frustum = None
        
    def get_robot_pose(self):
        return self._robot_pose

    def get_ptu_state(self):
        return self._ptu_state

    def get_ptu_pose(self):
        return self._ptu_pose

    def get_frustum(self):
        return self._frustum

    def set_frustum(self, frustum):
            self._frustum = frustum
    
##########################################################################

import random
from nav_goals_msgs.srv import NavGoals
from nav_goals_msgs.srv import WeightedNavGoals
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32
from viper.srv import PTU_FK

class ScitosViewGenerator(viper.core.robot.ViewGenerator):

    
    def __init__(self):
        self.inflation_radius = float(rospy.get_param('inflation_radius', '-0.5'))

        points = rospy.get_param('roi', '[]')

        roi = rospy.get_param('roi',[])
        points = []
        for point in roi:
            rospy.loginfo('Point: %s', point)
            points.append(Point32(float(point[0]),float(point[1]),0))

        polygon = Polygon(points) 
        self.roi = polygon
        
        self.views_at_pose = int(rospy.get_param('views_at_pose', '8'))
        self.min_pan = float(rospy.get_param('min_pan', '-2.09'))
        self.max_pan = float(rospy.get_param('max_pan', '2.09'))
        self.min_tilt = float(rospy.get_param('min_tilt', '-0.52'))
        self.max_tilt = float(rospy.get_param('max_tilt', '0.52'))
        self.velocity = float(rospy.get_param('velocity', '1.0'))
        rospy.loginfo("Wait for nav_goals")
        rospy.wait_for_service('nav_goals')
        rospy.loginfo("Done (nav_goals)")
        try:
            self.nav_goals = rospy.ServiceProxy('nav_goals', NavGoals)
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)
        try:
            self.ptu_fk = rospy.ServiceProxy('ptu_fk', PTU_FK)
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)
        self.views = []

    def generate_views(self):
        rospy.loginfo('Generate views')
        try:
            resp = self.nav_goals(1, self.inflation_radius, self.roi)
            
            if not resp.goals.poses:
                self.views = []
                return 
            for i in range(0,self.views_at_pose):
                pose = resp.goals.poses[0]
                pan = random.uniform(self.min_pan, self.max_pan)
                tilt = random.uniform(self.min_tilt, self.max_tilt)
                jointstate = JointState()
                jointstate.name = ['pan', 'tilt']
                jointstate.position = [pan, tilt]
                jointstate.velocity = [self.velocity, self.velocity]
                jointstate.effort = []
                resp_ptu_pose = self.ptu_fk(pan,tilt,pose)
                view = ScitosView(pose,jointstate,resp_ptu_pose.pose)
                self.views.append(view)
                
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)
        
        
    def generate(self):
        if not self.views:
            self.generate_views()
            if not self.views:
                return None
        rospy.loginfo('Generate view')
        return self.views.pop()
    
##########################################################################

from actionlib import *
from actionlib.msg import *
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
import threading
from scitos_ptu.msg import PtuGotoAction,PtuGotoGoal
import math
class ScitosViewController(viper.core.robot.ViewController):

    def __init__(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.ptu_cmd = rospy.Publisher('/ptu/cmd', JointState)
        rospy.Subscriber("/ptu/state", JointState, self.ptu_cb)
                    

    def ptu_cb(self,js):
        self.current_ptu_state = js


    def execute(self, view):
        rospy.loginfo("Execute view")

        try:
            rospy.loginfo("Wait for /ptu/state")
            msg = rospy.wait_for_message("/ptu/state", JointState, timeout=10.0)
            self.ptu_cb(msg)
        except rospy.ROSException, e:
            rospy.logwarn("Failed to get /ptu/state")

        
        self.robot_pose = view.get_robot_pose()

        self.mb_done = False
        mb_thread = threading.Thread(target = self.move_base)
        mb_thread.start()

        # 
        # ptu_client = actionlib.SimpleActionClient('SetPTUState', PtuGotoAction)
        # goal = PtuGotoGoal()
        # goal.pan = ptu_state.position[ptu_state.name.index('pan')] * 180/math.pi
        # goal.tilt = ptu_state.position[ptu_state.name.index('tilt')]  * 180/math.pi
        # goal.pan_vel = ptu_state.velocity[ptu_state.name.index('pan')] + 10
        # goal.tilt_vel = ptu_state.velocity[ptu_state.name.index('tilt')] + 10
        # ptu_client.send_goal(goal)
        # ptu_client.wait_for_result()

        joint_state = view.get_ptu_state()        
        # joint_state.header.frame_id = 'tessdaf'
        # joint_state.name = ['pan', 'tilt']
        # joint_state.position = [joint_state.position[joint_state.name.index('pan')],joint_state.position[joint_state.name.index('tilt')]]
        # joint_state.velocity = [joint_state.velocity[joint_state.name.index('pan')],joint_state.velocity[joint_state.name.index('tilt')]]
        # joint_state.effort = [float(1.0),float(1.0)]
        self.ptu_cmd.publish(joint_state)

        while not self.achieved(joint_state):
            rospy.loginfo("Wait for ptu")
            rospy.sleep(rospy.Duration(0.5))
        rospy.loginfo("Reached ptu goal")
        while self.mb_done == False: #self.client.get_state() == GoalStatus.ACTIVE:
            rospy.sleep(rospy.Duration(0.5))
            rospy.loginfo("Wait for move_base")
        rospy.loginfo("Reached move_base goal")
        self.client.cancel_goal()

    def achieved(self, joint_state):
        EPSILON = 0.05
        js_pan = joint_state.position[joint_state.name.index('pan')]
        current_js_pan = self.current_ptu_state.position[self.current_ptu_state.name.index('pan')]
        js_tilt = joint_state.position[joint_state.name.index('tilt')]
        current_js_tilt = self.current_ptu_state.position[self.current_ptu_state.name.index('tilt')]
        if (js_pan - current_js_pan) < EPSILON and (js_tilt - current_js_tilt) < EPSILON:
            return True
        return False

    def mb_done_cb(self,status,result):
        self.mb_done = True
    
    def move_base(self):
        self.client.wait_for_server(rospy.Duration(60))
        goal = MoveBaseGoal()
        goal.target_pose.header.frame_id = 'map'
        goal.target_pose.header.stamp = rospy.Time.now()
        goal.target_pose.pose = self.robot_pose
        self.client.send_goal(goal, done_cb=self.mb_done_cb)
        self.client.wait_for_result()


##########################################################################
    
class ScitosTransitionModel(viper.core.robot.ViewTransitionModel):

    def cost(view1, view2):
        """return path length of move_base planner"""
        pass

class LinearVTM(viper.core.robot.ViewTransitionModel):
        
    def cost(self, view1, view2):
        """return euclidean distance"""
        pass
    
class MotionPlanningVTM(viper.core.robot.ViewTransitionModel):

    def cost(view1, view2):
        """return path length of move_base planner"""
        pass

##########################################################################
import math
from viper.srv import ViewValue

    
class ScitosViewEvaluator(viper.core.robot.ViewEvaluator):

    def __init__(self):
        try:
            self.view_eval = rospy.ServiceProxy('view_eval', ViewValue)
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)

    def evaluate(self, view):
        resp = self.view_eval(view.get_ptu_pose())
        view.set_frustum(resp.frustum)
        return resp.value #math.fabs(view.get_robot_pose().position.x + view.get_robot_pose().position.y)

class TriangleViewEvaluator(viper.core.robot.ViewEvaluator):
    
    def __init__(self, triangle_conf, map2d):
        self._triangle_conf = triangle_conf
        self._map2d = map2d

    def evaluate(self, view):
        pass

class FrustumViewEvaluator(viper.core.robot.ViewEvaluator):

    def __init__(self, frustum_conf, map2d, map3d):
        self._frustum_conf = frustum_conf
        self._map2d = map2d
        self._map3d = map3d
    
    def evaluate(self, view):
        pass    
##########################################################################
from std_msgs.msg import String
import json

class ScitosViewAction(viper.core.robot.ViewAction):

    def __init__(self):
        pass
        #rospy.Subscriber("semcam", String, self.camera_cb)

    def camera_cb(self, data):
        obj_list = json.loads(data.data)
        if len(obj_list) == 0:
            rospy.loginfo("Nothing perceived")
        for obj_desc in obj_list:
            rospy.loginfo("Perceived: %s" % obj_desc.get('name'))

        for obj in obj_list:
            self.obj_list.append(obj)

    def execute(self):

        try:
            rospy.loginfo("Wait for /semcam")
            msg = rospy.wait_for_message("/semcam", String, timeout=10.0)
            rospy.loginfo("Received msg from /semcam")
            self.camera_cb(msg)
        except rospy.ROSException, e:
            rospy.logwarn("Failed to get /semcam")

        
            
