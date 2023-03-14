""" Ur5Generic module

Here it is implemented a class that allows to easily load and setup a UR5 robot within a Gazebo
simulation.

The code is mainly based on the ur5_generic.py script in the locosim framework. Still, it has been
adapated in order to met the requirement of distributed estimation and control.
"""
import numpy as np
import rospy as ros 
import pinocchio as pin
import rospkg
import tf
import time
import robot_configuration as conf



from base_controllers.utils.math_tools import *
from base_controllers.base_controller_fixed import BaseControllerFixed
from base_controllers.components.controller_manager import ControllerManager


from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger, TriggerRequest
from geometry_msgs.msg import WrenchStamped
from sensor_msgs import point_cloud2
from sensor_msgs.msg import PointCloud2
from controller_manager_msgs.srv import SwitchController, SwitchControllerRequest
from controller_manager_msgs.srv import LoadController, LoadControllerRequest


class Ur5Generic(BaseControllerFixed):

    def __init__(self, index: int = 0):
        """ Ur5Generic class constructor
    
        The robot is initialized with the homing flag enabled and position control (torque control 
        is not allowed on UR5 robots). By default the gripper is disabled.

        Params
        ------
        index: int
            Index of the UR5 that needs to be spawned; defaults to 0
        """
        robot_name = "ur5"
        print("Spawing robot", robot_name)
        # conf.robot_params[robot_name] = conf.robot_params["ur5"]
        super().__init__(robot_name = robot_name, external_conf = conf)
        self.controller_name = "ur5"
        self.topic_prefix = "/" + self.robot_name + "/"
        self.real_robot = False
        self.homing_flag = True 
        self.use_torque_control = 0
        self.gripper = False

        self.controller_manager = ControllerManager(conf.robot_params[self.robot_name])
        self.world_name = "empty.world"

        # self.base_offset = np.array([
        #     conf[self.robot_name].spawn_x,
        #     conf[self.robot_name].spawn_y,
        #     conf[self.robot_name].spawn_z
        #     ])
        # self.joint_names = conf[self.robot_name].joint_names


    def loadModelAndPublishers(self, xacro_path: str):
        """ Loads the urdf model and initialize all topics/services

        Here the "loadModelAndPublishers" method is called from the "BaseControllerFixed" parent 
        class that loads the xacro file and initialize the model.
        Then:
            - we subscribe to the topic where wrench at tool tips are published;
            - we setup 2 publishers in order to load a controller and commanding commanding the 
              desired position;
            - we setup 2 services client connections that allows to switch controller and ??? TODO
        Publishers for the controller manager are also initialized here; depending on the robot 
        status (real/simulated) different control schemes are set and made available.

        Params 
        ------
        xacro_path: str
            Full path of the UR5 xacro file containing informations for the simulation.
        """

        super().loadModelAndPublishers(xacro_path)

        self.sub_ftsensor = ros.Subscriber(
                self.topic_prefix + "wrench",
                WrenchStamped,
                callback = self._receive_ftsensor,
                queue_size = 1,
                tcp_nodelay = True
                )
        self.switch_controller_srv = ros.ServiceProxy(
                self.topic_prefix + "controller_manager/switch_controller",
                SwitchController 
                )
        self.load_controller_srv = ros.ServiceProxy(
                self.topic_prefix + "controller_manager/load_controller",
                LoadController
                )
        self.pub_reduced_des_jstate = ros.Publisher(
                self.topic_prefix + "joint_group_pos_controller/command",
                Float64MultiArray,
                queue_size = 10
                )
        self.zero_sensor = ros.ServiceProxy(
                self.topic_prefix + "ur_hardware_interface/zero_ftsensor",
                Trigger
                )
        self.controller_manager.initPublishers(self.robot_name)
        
        if self.real_robot:
            self.available_controllers = [
                    "joint_group_pos_controller",
                    "pos_joint_traj_controller"
                    ]
        else:
            self.available_controllers = [
                    "joint_group_pos_controller",
                    "scaled_pos_joint_traj_controller"
                    ]

        self.active_controller = self.available_controllers[0]
        self.broadcaster = tf.TransformBroadcaster()
        self.utils = Utils()
        self.utils.putIntoGlobalParamServer("gripper_sim", self.gripper)

        # For ZED2 camera
        self.sub_pointcloud = ros.Subscriber(
                self.topic_prefix + "zen_node/point_cloud/cloud_registered",
                PointCloud2,
                callback = self._receive_pointcloud,
                queue_size = 1
                )


    def _receive_ftsensor(self, msg: WrenchStamped):
        """ Callback of the ftsensor topic """
        contactForceTool0       = np.zeros(3)
        contactMomentTool0      = np.zeros(3)
        contactForceTool0[0]    = msg.wrench.force.x
        contactForceTool0[1]    = msg.wrench.force.y
        contactForceTool0[2]    = msg.wrench.force.z
        contactMomentTool0[0]   = msg.wrench.force.x
        contactMomentTool0[1]   = msg.wrench.force.y
        contactMomentTool0[2]   = msg.wrench.force.z
        self.contactForceW      = self.w_R_tool0.dot(contactForceTool0)
        self.contactMomentW     = self.w_R_tool0.dot(contactMomentTool0)


    def deregister_node(self):
        ros.loginfo("Deregistering nodes")
        self.ros_pub.deregister_node()
        if not self.real_robot:
            os.system("rosnode kill " + self.topic_prefix + "ros_impedance_controller")
            os.system("rosnode kill /gzserver /gzclient")


    def updateKinematicsDynamics(self):
        """ Updates the kinematics and the dynamics of the manipulator

        Computes mass matrix, bias and gravity forces.
        Determines the coordinate of the end effector in the cartesian ground reference frame as 
        well as it's rotation matrix and the Jacobian.
        """
        self.robot.computeAllTerms(self.q, self.qd)
        self.M = self.robot.mass(self.q)                                                                               
        self.h = self.robot.nle(self.q, self.qd)
        self.g = self.robot.gravity(self.q)
        frame_name = conf.robot_params[self.robot_name]["ee_frame"]
        self.x_ee = self.robot.framePlacement(
                self.q, 
                self.robot.model.getFrameId(frame_name)
                ).translation
        self.w_R_tool0 = self.robot.framePlacement(
                self.q,
                self.robot.model.getFrameId(frame_name)
                ).rotation
        
        self.J6 = self.robot.frameJacobian(
                self.q,
                self.robot.model.getFrameId(frame_name),
                False,
                pin.ReferenceFrame.LOCAL_WORLD_ALIGNED
                )
        self.J = self.J6[:3, :]
        self.broadcaster.sendTransform(
                self.base_offset, 
                (0.0, 0.0, 0.0, 1.0), 
                ros.Time.now(),
                "/base_link",
                "/world"
                )


    def startupProcedure(self):
        if self.real_robot:
            self.zero_sensor()

        self.u.putIntoGlobalParamServer(
                "real_robot",
                self.real_robot
                )
        ros.loginfo("Startup finished, starting controller")


    def switch_controller(self, target_controller):
        print("List of available controllers:")
        for controller in self.available_controllers:
            print(" >", controller)
        ros.loginfo("Controller manager: loading", target_controller)

        other_controllers = (self.available_controllers)
        other_controllers.remove(target_controller)

        srv = LoadControllerRequest()
        srv.name = target_controller
        self.load_controller_srv(srv)
        
        srv = SwitchControllerRequest()
        srv.stop_controllers  = other_controllers
        srv.start_controllers = [target_controller]
        srv.strictness = SwitchControllerRequest.BEST_EFFORT
        self.switch_controller_srv(srv)
        self.active_controller = target_controller


    def homing_procedure(self, dt, v_des, q_home, rate):
        """ Performs an homing procedure to restore initial condition of the robot.

        The homing configuration is the one specified in the configuration file.
        """
        self.broadcaster.sendTransform(
                self.base_offset, 
                (0.0, 0.0, 0.0, 1.0), 
                ros.Time.now(), 
                "/base_link",
                "/world"
                )
        v_ref = 0.0

        while True:
            e = q_home - self.q_des
            e_norm = np.linalg.norm(e)

            if e_norm != 0.0:
                v_ref += 0.005 * (v_des - v_ref)
                self.q_des += dt * v_ref * e / e_norm
                self.controller_manager.sendReference(self.q_des)

            rate.sleep()
            if e_norm < 0.001:
                self.homing_flag = False 
                ros.loginfo("Homing procedure accomplished")
                # TODO: gripper reset
                break


    def _receive_pointcloud(self, msg: PointCloud2):
        print("TO BE IMPLEMENTED")
        pass



def talker(p: Ur5Generic):
    """ Main loop function for executing the ROS program.

    It loads the robot based on the .xacro files and the additional arguments given.
    It then performs a startup procedure and, if needed, a homing.
    By default controller should be "joint_group_pos_controller".

    The program here continues by providing a sinusoidal reference to each joint based on the
    default joint value.


    Params
    ------
    p: UrGeneric
        Object instantiated by the script.
    """
    p.start()
    if p.real_robot:
        p.startRealRobot()
    else:
        additional_args = [
                "gripper := " + str(p.gripper),
                "soft_gripper := " + str(conf.robot_params[p.robot_name]["soft_gripper"])
                ] 
        p.startSimulator(
                world_name = p.world_name,
                use_torque_control = p.use_torque_control,
                additional_args = additional_args
                )
        xacro_path = rospkg.RosPack().get_path("ur_description") + "/urdf/ur5.urdf.xacro"
        p.loadModelAndPublishers(xacro_path)
        p.initVars()
        p.startupProcedure()

        time.sleep(8.0)

        rate = ros.Rate(1 / conf.robot_params[p.robot_name]["dt"])
        p.q_des_q0 = conf.robot_params[p.robot_name]["q_0"]
        p.q_des = np.copy(p.q_des_q0)

        # if not p.use_torque_control:
        #     p.switch_controller("joint_group_pos_controller")

        if p.homing_flag:
            if p.real_robot:
                v_des = 0.2
            else:
                v_des = 0.6

            p.homing_procedure(
                    conf.robot_params[p.robot_name]["dt"],
                    v_des,
                    conf.robot_params[p.robot_name]["q_0"],
                    rate
                    )

        gripper_on = 0

        p.time = 0
        while not ros.is_shutdown():
            p.updateKinematicsDynamics()

            p.q_des = p.q_des_q0 + 0.1 * np.sin(2*np.pi*0.5*p.time)
            p.qd_des = 0.1*2*np.pi*0.5*p.time * np.cos(2*np.pi*0.5*p.time) * np.ones(p.robot.na)
            p.controller_manager.sendReference(p.q_des, p.qd_des, p.h)
            
            p.logData()
            p.ros_pub.publishVisual()

            rate.sleep()
            p.time = np.round(p.time + np.array([conf.robot_params[p.robot_name]["dt"]]), 3)


if __name__ == "__main__":
    p = Ur5Generic(0)

    try:
        talker(p)
    except(ros.ROSInitException, ros.service.ServiceException):
        ros.signal_shutdown("killed")
        p.deregister_node()
