#!/usr/bin/env python

import yaml
import rospy
import threading

from pathlib import Path
from enum import Enum, auto
from geometry_msgs.msg import Pose, PoseStamped, TwistStamped, Twist
from screwed_msgs.srv import *
from screwed_msgs.msg import *
from moveit_msgs.msg import Constraints, JointConstraint, OrientationConstraint
from iiwa_msgs.srv import ConfigureControlMode, ConfigureControlModeRequest, ConfigureControlModeResponse
from iiwa_msgs.msg import CartesianWrench
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import Float32

from yaml import load
try:
    from yaml import CLoader as Loader
except ImportError:
    from yaml import Loader


class State(CurrentState):
        INIT = 0
        STANDBY = 1
        ACQUIRE = 2
        APPROACH = 3
        EXECUTE = 4
        FAILED = 5
        NAVIGATION = 6
        ARRIVED = 7

class StateMachine:

    def __init__(self, ros_logs=True, debugs=True):
        
        self.ros_logs = ros_logs
        self.debugs = debugs

        self.debug_out("Initializing State Machine...")

        # State variables
        self.state = None
        self.prev_state = None
        
        # Toggles
        self.frozen = False
        self.manual_control = True 

        # Flags
        self.step_once = False
        self.home = False
        self.acquire = False
        self.approach = False
        self.navigate = False
        self.arrived = False
        self.reset = False
        self.impedance = False
        self.retract = False

        self.configs = self.load_configurations()

        self.task_list = []
        self.task_sel = None

        self.finish_time = 0
        self.retract_time = 0
        self.force_feedback = 0

        # ROS service servers
        self.input_srv = rospy.Service('/state_machine/inputs', UserInput, self.input_callback)

        # ROS service clients
        self.plan_trajectory_srv = rospy.ServiceProxy('/plan_trajectory', PlanTrajectory)
        self.trigger_execute_srv = rospy.ServiceProxy('/execute_trajectories', TriggerExecute)
        self.get_tasks_srv = rospy.ServiceProxy('/state_machine/get_tasks', GetTasks)
        self.force_control_srv = rospy.ServiceProxy('/force_control', DesiredForce)
        self.control_mode_srv = rospy.ServiceProxy('/iiwa/configuration/ConfigureControlMode', ConfigureControlMode)

        # ROS publishers
        self.state_pub = rospy.Publisher('/state_machine/state', CurrentState, queue_size=10)
        self.vel_cmd_pub = rospy.Publisher('/iiwa/command/CartesianVelocity', TwistStamped, queue_size=10)
        self.rb_vel_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.lift_pos_pub = rospy.Publisher('/ewellix_tlt/actions/position', Float32, queue_size=10)
        self.eef_pub = rospy.Publisher('/screwed_stack/command/end_effector', EndEffector, queue_size=10)

        # ROS subscribers
        self.force_sub = rospy.Subscriber('/iiwa/state/CartesianWrench', CartesianWrench, self.force_callback)
        self.rb_vel_sub = rospy.Subscriber('/move_base/cmd_vel', Twist, self.rb_vel_callback)
        self.nav_status_sub = rospy.Subscriber('/move_base/status', GoalStatusArray, self.nav_status_callback)

        self._vel_thread = threading.Thread(target=self._vel_loop)
        self._vel_thread.daemon = True
        self._vel_thread.start()

        # self.set_control_mode(impedance=False) 
        self.state = self.on_enter_init()

    def run_step(self):
        """
        Run a single step of the state machine.
        """

        self.publish_state()

        if self.reset:
            self.execute_plan(count=-1)
            self.prev_state = self.state
            self.state = State.INIT
            self.reset = False
            self.on_enter_init()


        if self.step_once:
            self.step_once = False
            self.debug_out("Running one step...")
        else:
            if self.frozen:
                return

        # Handle state transitions
        if self.state == State.INIT:
            if self.check_standby():
                self.prev_state = self.state
                self.state = self.on_enter_standby()

        elif self.state == State.STANDBY:
            if self.check_acquire():
                self.prev_state = self.state
                self.state = self.on_enter_acquire()
            elif self.check_approach():
                self.prev_state = self.state
                self.state = self.on_enter_approach()
            elif self.check_navigation():
                self.prev_state = self.state
                self.state = self.on_enter_navigation()

        elif self.state == State.ACQUIRE:
            if self.check_standby():
                rospy.sleep(6.5)
                self.prev_state = self.state
                self.state = self.on_enter_standby()
            self.get_tasks()

        elif self.state == State.NAVIGATION:
            if self.check_arrived():
                self.prev_state = self.state
                self.state = self.on_enter_arrived()
        
        elif self.state == State.ARRIVED:
            if self.check_standby():
                self.prev_state = self.state
                self.state = self.on_enter_standby()
            elif self.check_navigation():
                self.prev_state = self.state
                self.state = self.on_enter_navigation()

        elif self.state == State.APPROACH:
            if self.check_standby():
                self.prev_state = self.state
                self.set_lift_position(0.0)
                self.state = self.on_enter_standby()
            elif self.check_execute():
                self.prev_state = self.state
                self.state = self.on_enter_execute()
            
        elif self.state == State.EXECUTE:
            if self.check_approach():
                self.prev_state = self.state
                self.state = self.on_enter_approach()
                return 
            if not self.retract:
                self.execute_eef(100.0)
            

        elif self.state == State.FAILED:
            pass # TODO: Handle failure state


    def _vel_loop(self):
        rate = rospy.Rate(1000)
        while not rospy.is_shutdown():
            if self.state == State.EXECUTE and self.retract:
                vel_msg = TwistStamped()
                vel_msg.twist.linear.z = 0.08
                self.vel_cmd_pub.publish(vel_msg)
            if self.state == State.APPROACH and self.impedance and not self.frozen:
                vel_msg = TwistStamped()
                vel_msg.twist.linear.z = -0.02
                self.vel_cmd_pub.publish(vel_msg)
            rate.sleep()

    def info_out(self, msg: str):
        if self.ros_logs:
            rospy.loginfo(msg)
        else:
            print("[INFO]: " + str(msg))

    def debug_out(self, msg: str):
        if self.ros_logs:
            rospy.logdebug(msg)
        else:
            if self.debugs:
                print("[DEBUG]: " + str(msg))

    def warn_out(self, msg: str):
        if self.ros_logs:
            rospy.logwarn(msg)
        else:
            print("[WARN]: " + str(msg))

    def load_configurations(self):
        """
        Load configurations from a YAML file.
        :return: A dictionary containing the loaded configurations.
        """
        this_dir = Path.joinpath(Path(__file__).parent, "../config/saved_configurations.yaml")
        self.debug_out(f"Loading saved configurations from {this_dir}")
        stream = open(this_dir, 'r')
        configs = yaml.load(stream, Loader=Loader)
        self.debug_out(f"Sucessfully loaded configurations: " + str([state.upper() for state in configs]))
        return configs
    
    def is_service_available(self, service_name: str) -> bool:
        """
        Check if a ROS service is available.
        :param service_name: The name of the service to check.
        :return: True if the service is available, False otherwise.
        """
        try:
            self.debug_out(f"Waiting for service {service_name}...")
            rospy.wait_for_service(service_name, timeout=5)
            return True
        except rospy.ROSException as e:
            self.debug_out(f"Service {service_name} not available: {e}")
            return False

    def set_control_mode(self, impedance: bool):
        """
        Set the control mode of the robot.
        :param mode: True for Cartesian control, False for joint control.
        :return: True if the control mode was set successfully, False otherwise.
        """
        if not self.is_service_available('/iiwa/configuration/ConfigureControlMode'):
            self.warn_out("Service /iiwa/configuration/ConfigureControlMode not available.")
            return False
        
        req = ConfigureControlModeRequest()
        if impedance:
            req.control_mode = 1
            req.joint_impedance.joint_stiffness.a1 = 250.0
            req.joint_impedance.joint_stiffness.a2 = 250.0
            req.joint_impedance.joint_stiffness.a3 = 300.0
            req.joint_impedance.joint_stiffness.a4 = 250.0
            req.joint_impedance.joint_stiffness.a5 = 300.0
            req.joint_impedance.joint_stiffness.a6 = 300.0
            req.joint_impedance.joint_stiffness.a7 = 300.0
            req.joint_impedance.joint_damping.a1 = 0.95
            req.joint_impedance.joint_damping.a2 = 0.75
            req.joint_impedance.joint_damping.a3 = 0.95
            req.joint_impedance.joint_damping.a4 = 0.75
            req.joint_impedance.joint_damping.a5 = 0.95
            req.joint_impedance.joint_damping.a6 = 0.75
            req.joint_impedance.joint_damping.a7 = 1.00
            self.info_out("Setting control mode to Joint impedance.")
        else:
            req.control_mode = 0
            self.info_out("Setting control mode to Joint.")

        try:
            resp = self.control_mode_srv(req)
            if resp.success:
                self.info_out("Control mode set successfully.")
                self.impedance = impedance
                return True
            else:
                self.warn_out("Failed to set control mode.")
                return False
        except rospy.ServiceException as e:
            self.debug_out(f"Service call failed: {e}")
            return False
    
    def set_lift_position(self, position: float):
        """
        Set the position of the lift.
        :param position: The target position for the lift.
        :return: True if the position was set successfully, False otherwise.
        """
        if position > 0.5:
            return
        msg = Float32()
        msg.data = position
        self.lift_pos_pub.publish(msg)
        self.info_out(f"Setting lift position to {position} m.")

    def add_joint_constraint(self, joint_name, above_tol, below_tol) -> JointConstraint: 
        """
        Create a joint constraint for the robot.
        :param joint_name: The name of the joint to constrain.
        :param above_tol: The tolerance above the joint position.
        :param below_tol: The tolerance below the joint position.
        :return: A JointConstraint object.
        """
        jc = JointConstraint(
            joint_name=joint_name,  # Replace with actual joint name
            position=0.0,  # Replace with actual position
            tolerance_above=above_tol,
            tolerance_below=below_tol,
            weight=1.0
        )
        return jc

    def add_plan(self, 
                 use_cartesian: bool,
                 constraints: Constraints=None,
                 max_vel_scale: float=0.1,
                 max_acc_scale: float=0.1, 
                 cartesian_pose: Pose=None, 
                 joint_pose: tuple=None) -> bool:
        """
        Plan and execute a trajectory to a given pose.
        :param use_cartesian: Whether to use cartesian or joint space for planning. 
        :param use_constraints: Whether to use constraints for planning.
        :param constraints: The constraints to use for planning.
        :param max_vel_scale: The maximum velocity scaling factor for the trajectory.
        :param max_acc_scale: The maximum acceleration scaling factor for the trajectory.
        :param cartesian_pose: The target cartesian pose to plan to.
        :param joint_pose: The target joint pose to plan to, as a tuple of (joint_names, joint_positions).
        :return: True if the trajectory was planned successfully, False otherwise.
        """
        if not self.is_service_available('/plan_trajectory'):
            self.debug_out("Service /plan_trajectory not available.")
            return False

        self.debug_out("Planning trajectory...")
        req = PlanTrajectoryRequest()
        req.use_constraints = True if not constraints is None else False
        req.constraints = constraints if not constraints is None else Constraints()
        req.max_velocity_scaling_factor = max_vel_scale
        req.max_acceleration_scaling_factor = max_acc_scale

        # Check if we are using cartesian or joint space
        if use_cartesian:
            if cartesian_pose is None:
                self.warn_out("No cartesian pose provided.")
                return False
            req.use_cartesian = True
            req.target_pose = cartesian_pose
            self.debug_out(f"Cartesian pose: {cartesian_pose}")
            
        else:
            if joint_pose is None:
                self.warn_out("No joint pose provided.")
                return False
            req.use_cartesian = False
            req.joint_names = joint_pose[0]
            req.joint_positions = joint_pose[1]
            self.debug_out(f"Joint names: {req.joint_names}")

        # Send plan trajectory request
        try:
            resp = self.plan_trajectory_srv(req)

            if resp.success:
                self.info_out("Trajectory planned successfully. Added to queue.")

            else:
                self.warn_out("Failed to plan trajectory.")
                return False

        except rospy.ROSException as e:
            self.debug_out(f"Service not available: {e}")
            return False
        except rospy.ServiceException as e:
            self.debug_out(f"Service call failed: {e}")
            return False
        
    def force_callback(self, msg: CartesianWrench):
        if self.task_sel is None or self.task_sel < 0 or self.task_sel >= len(self.task_list):
            return

        self.force_feedback = msg.wrench.force.z
        self.torque_feedback = msg.wrench.torque.z

        if self.state == State.APPROACH and self.force_feedback < -1.0:
            self.task_list[self.task_sel]['status'] = Task.RUNNING
            self.retract = False
            return

        if self.state == State.EXECUTE and self.torque_feedback < -1.20 and not self.retract:
            self.execute_force(0.0, timeout=0.1)
            self.retract = True
            self.retract_time = rospy.Time.now().to_sec() + 2.0
            self.execute_eef(0.0, run=False)
            self.info_out("Task completed successfully. Retracting...")
            return

        if self.retract and rospy.Time.now().to_sec() > self.retract_time:
            self.retract = False
            self.info_out("Retract time exceeded. Stopping retraction.")
            self.task_list[self.task_sel]['status'] = Task.COMPLETE
            return
        
    def rb_vel_callback(self, msg: TwistStamped):
        if self.state == State.NAVIGATION and not self.frozen:
            self.rb_vel_pub.publish(msg)

    def nav_status_callback(self, msg: GoalStatusArray):
        if self.state != State.NAVIGATION:
            return
        
        if len(msg.status_list) > 0:
            status = msg.status_list[0].status
            if status == 3:
                self.info_out("Navigation goal reached.")
                self.arrived = True
                if not self.manual_control:
                    self.acquire = True 
            

    def execute_plan(self, count: int=1) -> bool:
        """
        Execute the planned trajectory.
        :param count: The number of items in trajectory queue to execute.
        :return: True if the trajectory was executed successfully, False otherwise.
        """
        if not self.is_service_available('/execute_trajectories'):
            self.debug_out("Service /execute_trajectories not available.")
            return False
        
        exec = TriggerExecuteRequest()
        exec.count = count

        try:
            resp = self.trigger_execute_srv(exec)
            if resp.success:
                self.info_out("Successfully moved to target pose.")
                return True
            else:
                self.warn_out("Failed to move to target pose.")
                return False
        except rospy.ServiceException as e:
            self.debug_out(f"Service call failed: {e}")
            return False
        
    def go_to_config(self, config_name: str):
        """
        Move the robot to a specific configuration defined in YAML.
        :param config_name: The name of the configuration to move to.
        :return: True if the robot moved successfully, False otherwise. 
        """

        if config_name not in self.configs:
            self.warn_out(f"Configuration {config_name} not found in loaded configurations.")
            return False
        
        if not self.is_service_available('/plan_trajectory'):
            self.debug_out("Service /plan_trajectory not available.")
            return False
        self.info_out(f"Moving to configuration {config_name}...")
        self.add_plan(use_cartesian=False, 
                      joint_pose=(self.configs[config_name]['joint_pose']['names'], 
                                  self.configs[config_name]['joint_pose']['position']))

        self.execute_plan(count=1)

        return True
        
    def go_to_approach(self):
        """
        Move the robot to the approach pose of the selected task.
        :return: True if the robot moved successfully, False otherwise.
        """
        if self.task_sel is None:
            self.warn_out("No task selected.")
            return True
        
        if self.task_list[self.task_sel]['status'] != Task.PENDING:
            self.debug_out("Task already executed or in progress.")
            return False
        
        task_id = self.task_list[self.task_sel]['task_id']
        task_pose = self.task_list[self.task_sel]['goal_pose']
        task_offset = self.task_list[self.task_sel]['approach_offset']

        task_pose.position.z += task_offset # Hover above the target pose

        self.debug_out(f"Moving to approach pose of task {task_id}...")

        self.add_plan(use_cartesian=True, 
                      cartesian_pose=task_pose,
                      max_vel_scale=0.06,
                      joint_pose=None)
        
        self.execute_plan(count=1)

        if self.set_control_mode(impedance=True):
            return True
        else:
            return False

    def execute_force(self, desired_force, axis: int=3, timeout: float=0.0):
        
        req = DesiredForceRequest()
        req.axis = axis
        req.desired_force = desired_force
        req.kP = 0.00058
        req.kI = 0.0001
        req.kD = 0.000032
        req.timeout = timeout

        try:
            self.set_control_mode(impedance=True)
            resp = self.force_control_srv(req)

            if resp.success:
                self.info_out("Force planned successfully. Executing...")
                self.finish_time = rospy.Time.now().to_sec() + timeout
                return True # TODO: Implement a smarter way to change execution state

            else:
                self.warn_out("Failed to plan desired force.")
                return False

        except rospy.ROSException as e:
            self.debug_out(f"Service not available: {e}")
            return False
        except rospy.ServiceException as e:
            self.debug_out(f"Service call failed: {e}")
            return False
        
    def execute_eef(self, pwm, run=True):
        eef_msg = EndEffector()
        eef_msg.run = run
        eef_msg.pwm = pwm
        if self.frozen:
            eef_msg.run = False
        self.eef_pub.publish(eef_msg)
    

    def get_next_task(self):
        """
        Move to the next task in the task list.# Offset for approach
        :return: True if the robot moved successfully, False otherwise.
        """

        count = 0
        for task in self.task_list:
            if task['status'] == Task.PENDING:
                self.task_sel = count
                self.debug_out(f"Selected task {self.task_sel} for execution.")
                return True
            count += 1
        self.debug_out("No tasks pending.")
        self.task_sel = None
        return False
    
    def input_callback(self, req: UserInputRequest) -> UserInputResponse:
        """
        Callback function for the input service.
        :param req: The request object containing the input data.
        :return: A response object indicating success or failure.
        """
    
        # Handle Toggles
        if req.freeze_toggle:
            if self.frozen:
                self.frozen = False
                self.debug_out("Unfreezing state machine...")
            else:
                self.frozen = True
                self.debug_out("Freezing state machine...")

        if req.manual_toggle:
            if self.manual_control:
                self.manual_control = False
                self.debug_out("Disabling manual control...")
            else:
                self.manual_control = True
                self.debug_out("Enabling manual control...")


        # Handle Flags
        if req.reset:
            self.reset = True

        if req.step_once:
            self.step_once = True

        if req.acquire:
            self.acquire = True

        if req.approach:
            self.approach = True

        if req.navigate:
            self.navigate = True
        

        return UserInputResponse(success=True)
    
    def does_task_exist(self, task_id: int) -> bool:
        """
        Check if a task with the given ID exists in the task list.
        :param task_id: The ID of the task to check.
        :return: True if the task exists, False otherwise.
        """
        for task in self.task_list:
            if task['task_id'] == task_id:
                return True
        return False

    def get_tasks(self):

        if self.is_service_available('/state_machine/get_tasks'):
            self.debug_out("Waiting for tasks...")
            try:
                resp = self.get_tasks_srv(GetTasksRequest())
                if resp.success:
                    count = 0
                    for task in resp.tasks:
                        if self.does_task_exist(task.task_id):
                            return

                        item = {}
                        item['task_id'] = task.task_id
                        item['goal_pose'] = task.goal_pose # Note that this is a Pose message
                        item['approach_offset'] = task.approach_offset
                        item['status'] = task.status
                        item['frame'] = task.fixed_frame
                        # TODO: Implement more params as needed
                        self.task_list.append(item)
                        count += 1

                    self.info_out(f"Received and appended {count} new tasks. Total tasks: {len(self.task_list)}")
                else:
                    self.warn_out("Failed to get tasks.")
            except rospy.ServiceException as e:
                self.debug_out(f"Service call failed: {e}")

    def publish_state(self):

        msg = CurrentState()
        msg.header.stamp = rospy.Time.now()

        if self.state == State.INIT:
            msg.current_state = CurrentState.HOME 
        elif self.state == State.STANDBY:
            msg.current_state = CurrentState.STANDBY
        elif self.state == State.ACQUIRE:
            msg.current_state = CurrentState.ACQUIRE
        elif self.state == State.APPROACH:
            msg.current_state = CurrentState.APPROACH
        elif self.state == State.EXECUTE:
            msg.current_state = CurrentState.INSERTION 
        elif self.state == State.NAVIGATION:
            msg.current_state = CurrentState.NAVIGATING
        elif self.state == State.ARRIVED:
            msg.current_state = CurrentState.ARRIVED
        elif self.state == State.FAILED:
            msg.current_state = CurrentState.FAILED

        # Optionally fill the task_list if you have tasks to report
        tasks = []
        for task in self.task_list:
            task_msg = Task()
            task_msg.task_id = task['task_id']
            task_msg.goal_pose = task['goal_pose']
            task_msg.approach_offset = task['approach_offset']
            task_msg.status = task['status']
            task_msg.fixed_frame = task['frame']
            tasks.append(task_msg)
        msg.task_list = tasks

        msg.is_frozen = self.frozen
        msg.is_manual = self.manual_control

        self.state_pub.publish(msg)

    # Define checks
    def check_standby(self) -> bool:
        if self.home and self.state == State.INIT:
            return True
        elif self.state == State.ACQUIRE:
            return True
        elif self.state == State.APPROACH and self.task_sel is None:
            return True
        elif self.state == State.ARRIVED:
            return True
        else:
            return False

    def check_acquire(self) -> bool:
        if self.state == State.STANDBY and self.acquire:
            self.acquire = False
            return True
        else:
            return False

    def check_approach(self) -> bool:
        if self.state == State.STANDBY and self.approach and len(self.task_list) > 0:
            self.approach = False
            return True
        elif self.state == State.EXECUTE and self.task_list[self.task_sel]['status'] == Task.COMPLETE:
            return True
        elif self.state == State.EXECUTE and self.task_list[self.task_sel]['status'] == Task.FAILED:
            return True
        else:
            return False
        
    def check_navigation(self) -> bool:
        if self.state == State.STANDBY and self.navigate:
            self.navigate = False
            return True
        else:
            return False
        
    def check_arrived(self) -> bool:
        if self.state == State.NAVIGATION and self.arrived:
            self.arrived = False
            return True
        else:
            return False

    def check_execute(self) -> bool:
        if self.state == State.APPROACH and self.task_list[self.task_sel]['status'] == Task.RUNNING:
            return True
        else:
            return False
        
    def check_finish(self):
        if rospy.Time.now().to_sec() > self.finish_time:
            return True
        else:
            return False


    # Define actions when a state is entered
    def on_enter_init(self):
        self.info_out("Sending HOME request...")
        self.home = self.go_to_config('home')
        self.set_control_mode(impedance=False)

        return State.INIT

    def on_enter_standby(self):
        # self.set_control_mode(impedance=False)
        # self.set_lift_position(0.0)
        if self.go_to_config('standby'):
            self.home = False
        else:
            return State.FAILED

        return State.STANDBY

    def on_enter_acquire(self):
        self.set_lift_position(0.35)
        self.task_list = [] # Clear task list (not sure if I should or not)
        if not self.manual_control:
            self.approach = True

        return State.ACQUIRE
    
    def on_enter_navigation(self):
        self.set_lift_position(0.0)

        return State.NAVIGATION
    
    def on_enter_arrived(self):

        return State.ARRIVED

    def on_enter_approach(self):
        self.set_control_mode(impedance=False)
        self.get_next_task()
        
        if self.go_to_approach():
            return State.APPROACH
        else:
            return State.FAILED

    def on_enter_execute(self):
        if self.execute_force(-35):
            self.debug_out("Executing task...")
            return State.EXECUTE
        else:
            return State.FAILED
    

if __name__ == "__main__":
    rospy.init_node('state_machine_node', anonymous=True)

    state_machine = StateMachine()
    rate = rospy.Rate(15)  # 10 Hz

    while not rospy.is_shutdown():
        state_machine.run_step()
        rate.sleep()