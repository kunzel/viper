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

    def __init__(self, ID, robot_pose, ptu_state, ptu_pose):
        self.ID = ID 
        self._robot_pose = robot_pose
        self._ptu_state = ptu_state
        self._ptu_pose = ptu_pose
        self._frustum = None
        self._keys = []
        self._values = []
        
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

    def get_keys(self):
        return self._keys
            
    def set_keys(self, keys):
        self._keys = keys

    def get_values(self):
        return self._values
        
    def set_values(self, values):
        self._values = values
    
##########################################################################

import random
from nav_goals_generator.srv import NavGoals
from nav_goals_generator.srv import WeightedNavGoals
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Polygon
from geometry_msgs.msg import Point32
from geometry_msgs.msg import Pose
from viper.srv import PTU_FK

class ScitosViewGenerator(viper.core.robot.ViewGenerator):

    
    def __init__(self):
        self.first_call = True
        self.id = -1

    def next_id(self):
        self.id += 1
        return str(self.id)

    def setup(self):
        
        self.inflation_radius = float(rospy.get_param('inflation_radius', '0.0'))

        #points = rospy.get_param('roi', '[]')

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
        self.min_tilt = float(rospy.get_param('min_tilt', '-0.22')) # self.min_tilt = float(rospy.get_param('min_tilt', '-0.22'))
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
        rospy.loginfo('Generate views (%s views at pose)' % self.views_at_pose)
        try:
            resp = self.nav_goals(1, self.inflation_radius, self.roi)
            
            if not resp.goals.poses:
                print "DOES THIS HAPPEN???????????"
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
                jointstate.effort = [float(1.0), float(1.0)]
                resp_ptu_pose = self.ptu_fk(pan,tilt,pose)
                p = Pose()
                view = ScitosView(self.next_id(), pose,jointstate, resp_ptu_pose.pose)
                self.views.append(view)
                
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)
        
        
    def generate(self):
        if self.first_call:
            self.setup()
            self.first_call = False
            
        if not self.views:
            self.generate_views()
            if not self.views:
                print "SOMETHING BAD HAPPEND!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
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
        self.first_call = True


    def setup(self):
        self.client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.ptu_client = actionlib.SimpleActionClient('SetPTUState', PtuGotoAction)
        rospy.loginfo("Wait for PTU action server")
        self.ptu_client.wait_for_server(rospy.Duration(60))
        
        self.ptu_cmd = rospy.Publisher('/ptu/cmd', JointState)
        rospy.Subscriber("/ptu/state", JointState, self.ptu_cb)

    def ptu_cb(self,js):
        self.current_ptu_state = js


    def execute(self, view):
        rospy.loginfo("Execute view")
        if self.first_call:
            self.setup()
            self.first_call = False

        #try:
        #    rospy.loginfo("Wait for /ptu/state")
        #    msg = rospy.wait_for_message("/ptu/state", JointState, timeout=10.0)
        #    self.ptu_cb(msg)
        #except rospy.ROSException, e:
        #    rospy.logwarn("Failed to get /ptu/state")

        
        self.robot_pose = view.get_robot_pose()

        self.mb_done = False
        mb_thread = threading.Thread(target = self.move_base)
        mb_thread.start()


        ptu_state = view.get_ptu_state()

        goal = PtuGotoGoal()
        goal.pan = ptu_state.position[ptu_state.name.index('pan')] * 180/math.pi
        goal.tilt = ptu_state.position[ptu_state.name.index('tilt')]  * 180/math.pi
        goal.pan_vel = 100 #ptu_state.velocity[ptu_state.name.index('pan')] * 100
        goal.tilt_vel =100 #ptu_state.velocity[ptu_state.name.index('tilt')] * 100
        self.ptu_client.send_goal(goal)
        self.ptu_client.wait_for_result()

        # joint_state = view.get_ptu_state()        
        # joint_state.header.frame_id = 'tessdaf'
        # joint_state.name = ['pan', 'tilt']
        # joint_state.position = [joint_state.position[joint_state.name.index('pan')],joint_state.position[joint_state.name.index('tilt')]]
        # joint_state.velocity = [joint_state.velocity[joint_state.name.index('pan')],joint_state.velocity[joint_state.name.index('tilt')]]
        # joint_state.effort = [float(1.0),float(1.0)]
        # self.ptu_cmd.publish(joint_state)

        #while not self.achieved(joint_state):
        #    rospy.loginfo("Wait for ptu")
        #    rospy.sleep(rospy.Duration(0.5))
        rospy.loginfo("Reached ptu goal")
        while self.mb_done == False: #self.client.get_state() == GoalStatus.ACTIVE:
            rospy.sleep(rospy.Duration(0.5))
            rospy.loginfo("Wait for move_base")
        rospy.loginfo("Reached move_base goal")
        self.client.cancel_goal()

        # rospy.loginfo("Execute view")
        # if self.first_call:
        #     self.setup()
        #     self.first_call = False

        # #try:
        # #    rospy.loginfo("Wait for /ptu/state")
        # #    msg = rospy.wait_for_message("/ptu/state", JointState, timeout=10.0)
        # #    self.ptu_cb(msg)
        # #except rospy.ROSException, e:
        # #    rospy.logwarn("Failed to get /ptu/state")

        
        # self.robot_pose = view.get_robot_pose()

        # self.mb_done = False
        # mb_thread = threading.Thread(target = self.move_base)
        # mb_thread.start()


        # ptu_state = view.get_ptu_state()

        # goal = PtuGotoGoal()
        # goal.pan = ptu_state.position[ptu_state.name.index('pan')] * 180/math.pi
        # goal.tilt = ptu_state.position[ptu_state.name.index('tilt')]  * 180/math.pi
        # goal.pan_vel = ptu_state.velocity[ptu_state.name.index('pan')] * 100
        # goal.tilt_vel = ptu_state.velocity[ptu_state.name.index('tilt')] * 100
        # self.ptu_client.send_goal(goal)
        # self.ptu_client.wait_for_result()

        # # joint_state = view.get_ptu_state()        
        # # joint_state.header.frame_id = 'tessdaf'
        # # joint_state.name = ['pan', 'tilt']
        # # joint_state.position = [joint_state.position[joint_state.name.index('pan')],joint_state.position[joint_state.name.index('tilt')]]
        # # joint_state.velocity = [joint_state.velocity[joint_state.name.index('pan')],joint_state.velocity[joint_state.name.index('tilt')]]
        # # joint_state.effort = [float(1.0),float(1.0)]
        # # self.ptu_cmd.publish(joint_state)

        # #while not self.achieved(joint_state):
        # #    rospy.loginfo("Wait for ptu")
        # #    rospy.sleep(rospy.Duration(0.5))
        # rospy.loginfo("Reached ptu goal")
        # while self.mb_done == False: #self.client.get_state() == GoalStatus.ACTIVE:
        #     rospy.sleep(rospy.Duration(0.5))
        #     rospy.loginfo("Wait for move_base")
        # rospy.loginfo("Reached move_base goal")
        # self.client.cancel_goal()

    def achieved(self, joint_state):
        EPSILON = 0.1
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
import math
from nav_msgs.srv import GetPlan
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped
from tf.transformations import euler_from_quaternion, quaternion_inverse, quaternion_multiply

class ScitosTransitionModel(viper.core.robot.ViewTransitionModel):

    def __init__(self):
        self.first_call = True
        self.nav_lin_vel = 0.15
        self.nav_ang_vel = 1.0
        self.ptu_ang_vel = 1.0

    def setup(self):
        self.nav_cost_dict = dict()
        self.nav_cost_list = list()
        try:
            self.make_plan = rospy.ServiceProxy('move_base/make_plan', GetPlan)
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)

        
    def cost(self, view1, view2):
        
        if self.first_call:
            self.setup()
            self.first_call = False
        p1 = view1.get_robot_pose()
        p2 = view2.get_robot_pose()
        nav_cost = self.nav_cost(p1,p2)
                
        s1 = view1.get_ptu_state()
        s2 = view2.get_ptu_state()
        ptu_cost = self.ptu_cost(s1,s2)

        cost = max(nav_cost, ptu_cost) + 2.0 # (perception costs)
    
        return cost

    def nav_cost(self, p1, p2):
        ps1 = PoseStamped()
        ps1.header.frame_id = 'map'
        ps1.pose = p1

        ps2 = PoseStamped()
        ps2.header.frame_id = 'map'
        ps2.pose = p2

        if (ps1,ps2) in self.nav_cost_list:
            #rospy.loginfo("Retrieve nav cost from cache (p1,p2)")
            return self.nav_cost_dict[self.nav_cost_list.index((ps1,ps2))]

        # if (ps2,ps1) in self.nav_cost_list:
        #     #rospy.loginfo("Retrieve nav cost from cache (p2,p1)")
        #     return self.nav_cost_dict[self.nav_cost_list.index((ps2,ps1))]

        #rospy.loginfo("Calc nav cost from plan: %s %s" % (p1,p2))
        res = self.make_plan(ps1, ps2, 0.1)

        cost = 0.0
        old_ang = 0.0
        for i in range(1,len(res.plan.poses)):
            p1 = res.plan.poses[i-1].pose
            p2 = res.plan.poses[i].pose
            # lin diff 
            lin_diff = math.sqrt( math.pow(p1.position.x - p2.position.x, 2) +
                                  math.pow(p1.position.y - p2.position.y, 2))
            lin_cost = lin_diff / self.nav_lin_vel
            

            # Version 1 --- not working
            # comput yaw angles and calc difference
            # q1 = [p1.orientation.x, p1.orientation.y, p1.orientation.z, p1.orientation.w]
            # q2 = [p2.orientation.x, p2.orientation.y, p2.orientation.z, p2.orientation.w]
            # qdiff = quaternion_multiply(q2,quaternion_inverse(q1))            
            # ang_diff = euler_from_quaternion(qdiff,axes='sxyz')
            # ang_cost = abs(ang_diff[2]) / self.nav_ang_vel

            # Version 2 --- not working
            # q1 = [p1.orientation.x, p1.orientation.y, p1.orientation.z, p1.orientation.w]
            # q2 = [p2.orientation.x, p2.orientation.y, p2.orientation.z, p2.orientation.w]
            # euler1 = euler_from_quaternion(q1,axes='sxyz')
            # euler2 = euler_from_quaternion(q2,axes='sxyz')
            # euler_diff = abs(euler1[2] - euler2[2])            
            # ang_cost = euler_diff / self.nav_ang_vel

            # CORRECT???
            # dx = p2.position.x - p1.position.x
            # dy = p2.position.y - p1.position.y
            # ang = math.atan2(dy,dx)
            # if ang < 0:
            #     ang = ang + 2*math.pi
            # if i == 1: 
            #     ang_diff = 0.0
            # else:
            #     ang_diff = abs(old_ang - ang) 

            # old_ang = ang
            # ang_cost = ang_diff / self.nav_ang_vel    

            # ONLY TAKE LIN COST 
            cost += lin_cost #max(lin_cost, ang_cost)

        self.nav_cost_list.append((ps1,ps2))
        self.nav_cost_dict[self.nav_cost_list.index((ps1,ps2))] = cost
        return cost 
        
    def ptu_cost(self, s1, s2):

        pan_diff = abs(s1.position[s1.name.index('pan')] - s2.position[s2.name.index('pan')])
        pan_cost = pan_diff / self.ptu_ang_vel
        
        tilt_diff = abs(s1.position[s1.name.index('tilt')] - s2.position[s2.name.index('tilt')])
        tilt_cost = tilt_diff / self.ptu_ang_vel
        
        cost = max(pan_cost, tilt_cost)
        
        return cost
        

##########################################################################
import math
from viper.srv import ViewValue, ViewValueRequest

    
class ScitosViewEvaluator(viper.core.robot.ViewEvaluator):

    def __init__(self):
        self.first_call = True
        
    def setup(self):
        try:
            self.view_eval = rospy.ServiceProxy('view_eval', ViewValue)
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)
    
    def evaluate(self, view, octomap):
        if self.first_call:
            self.setup()
            self.first_call = False


        req = ViewValueRequest()
        req.pose = view.get_ptu_pose()
        req.octomap = octomap
        try:
            resp = self.view_eval(req)
            view.set_keys(resp.keys)
            view.set_values(resp.values)
            view.set_frustum(resp.frustum)
            return resp.value #math.fabs(view.get_robot_pose().position.x + view.get_robot_pose().position.y
        except rospy.ServiceException, e:
            rospy.logerr("Service call failed: %s" % e)
        return 0


##########################################################################
from std_msgs.msg import String
import json

class ScitosViewAction(viper.core.robot.ViewAction):

    def __init__(self):
        pass
        #self.obj_list = list()
        #rospy.Subscriber("semcam", String, self.callback)
        #self.obj_list = []
        #self.active = False
        #self.first_call = False

    def camera_cb(self, data):
        obj_list = json.loads(data.data)
        if len(obj_list) == 0:
            rospy.loginfo("Nothing perceived")
        for obj_desc in obj_list:
            rospy.loginfo("Perceived: %s" % obj_desc.get('name'))
        return obj_list

    
    # def callback(self,data):
    #     if self.active == True and self.first_call == True:
    #         self.first_call = False
    #         obj_list = json.loads(data.data)
    #         if len(obj_list) == 0:
    #             rospy.loginfo("Nothing perceived")
    #         for obj_desc in obj_list:
    #             rospy.loginfo("Perceived: %s" % obj_desc.get('name'))

    #         for obj in obj_list:
    #             self.obj_list.append(obj)
    
    def execute(self):
        try:
            rospy.loginfo("Wait for /semcam")
            msg = rospy.wait_for_message("/semcam", String, timeout=10.0)
            rospy.loginfo("Received msg from /semcam")
            return self.camera_cb(msg)
        except rospy.ROSException, e:
            rospy.logwarn("Failed to get /semcam")


        # self.obj_list = []
        # self.active = True
        # self.first_call = True
        # while self.first_call == True:
        #     rospy.sleep(rospy.Duration(0.5))
        #     rospy.loginfo("Wait for semcam")
        # self.active = False
        # return self.obj_list

        
