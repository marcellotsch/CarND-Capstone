from pid import PID
import rospy
from yaw_controller import YawController
from lowpass import LowPassFilter
from std_msgs.msg import Float32


GAS_DENSITY = 2.858
ONE_MPH = 0.44704


class Controller(object):
    def __init__(self, *args, **kwargs):
    	print("twist controller init")
    	self.old_time = 0.0

    	self.yaw_control  = YawController(wheel_base=3.0, steer_ratio=2.67, min_speed=0.0, max_lat_accel=3.0, max_steer_angle=8.0)    

    	self.error_filter  = LowPassFilter(0.2, 0.1)
        self.steering_filter = LowPassFilter(0.7, 0.1)

        self.steering_PID = PID(1.5, 0.0, 0.01)


    def control(self, current_velocity, vehicle_dbw_enabled, twist_cmd):
        # TODO: Change the arg, kwarg list to suit your needs
        # Return throttle, brake, steer

        # Current and goal velocity and twist values
        v_goal_lin  = current_velocity.linear.x
        v_goal_ang = current_velocity.angular.z
        v_cur_lin  = twist_cmd.linear.x
        v_cur_ang = twist_cmd.angular.z

        if (vehicle_dbw_enabled):
        	print("dbw enabled")
        	if (self.old_time == 0.0):
        		# Set current timestamp in first cycle
        		#self.old_time = rospy.get_time()
        		self.get_delta_t()
        		return 0.5, 0., 0.

        	else:
        	    steer_goal = self.yaw_control.get_steering(v_goal_lin, v_goal_ang, v_goal_lin)
        	    steer_cur = self.yaw_control.get_steering(v_cur_lin, v_cur_ang, v_cur_lin)

        	    steer_error = self.error_filter.filt(steer_goal - steer_cur)
        	    steering = self.steering_filter.filt(self.steering_PID.step(steer_error, self.get_delta_t()))

        	    #steer_error = steer_goal - steer_cur
        	    #steering = self.steering_PID.step(steer_error, self.get_delta_t())
        	    print("return steering")
        	    return 0.5, 0., steering

        else:
        	print("dbw not enabled")
        	self.steering_PID.reset()
        	return 0., 0., 0.


    def get_delta_t(self):
        time = rospy.get_time()
        dt = time - self.old_time
        self.old_time = time
        return dt
