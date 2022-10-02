#!/usr/bin/env python3

"""
2D Controller Class to be used for the CARLA waypoint follower demo.
"""

import control.cutils as cutils
import numpy as np



class Controller2D(object):
    def __init__(self, waypoints):
        self.vars                   = cutils.CUtils()
        self._lookahead_distance    = 2.0
        self._current_x             = 0
        self._current_y             = 0
        self._current_yaw           = 0
        self._current_speed         = 0
        self._desired_speed         = 0
        self._current_frame         = 0
        self._current_timestamp     = 0
        self._start_control_loop    = False
        self._set_throttle          = 0
        self._set_brake             = 0
        self._set_steer             = 0
        self._waypoints             = waypoints
        self._conv_rad_to_steer     = 180.0 / 70.0 / np.pi
        self._pi                    = np.pi
        self._2pi                   = 2.0 * np.pi
        

    def update_values(self, x, y, yaw, speed, timestamp, frame):
        self._current_x         = x
        self._current_y         = y
        self._current_yaw       = yaw
        self._current_speed     = speed
        self._current_timestamp = timestamp
        self._current_frame     = frame
        if self._current_frame:
            self._start_control_loop = True

    def get_lookahead_index(self, lookahead_distance):
        min_idx       = 0
        min_dist      = float("inf")
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                    self._waypoints[i][0] - self._current_x,
                    self._waypoints[i][1] - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i

        total_dist = min_dist
        lookahead_idx = min_idx
        for i in range(min_idx + 1, len(self._waypoints)):
            if total_dist >= lookahead_distance:
                break
            total_dist += np.linalg.norm(np.array([
                    self._waypoints[i][0] - self._waypoints[i-1][0],
                    self._waypoints[i][1] - self._waypoints[i-1][1]]))
            lookahead_idx = i
        return lookahead_idx

    def update_desired_speed(self):
        min_idx       = 0
        min_dist      = float("inf")
        desired_speed = 0
        for i in range(len(self._waypoints)):
            dist = np.linalg.norm(np.array([
                    self._waypoints[i][0] - self._current_x,
                    self._waypoints[i][1] - self._current_y]))
            if dist < min_dist:
                min_dist = dist
                min_idx = i
        if min_idx < len(self._waypoints)-1:
            desired_speed = self._waypoints[min_idx][2]
        else:
            desired_speed = self._waypoints[-1][2]
        self._desired_speed = desired_speed

    def update_waypoints(self, new_waypoints):
        self._waypoints = new_waypoints

    def get_commands(self):
        return self._set_throttle, self._set_steer, self._set_brake

    def set_throttle(self, input_throttle):
        # Clamp the throttle command to valid bounds
        throttle           = np.fmax(np.fmin(input_throttle, 1.0), 0.0)
        self._set_throttle = throttle

    def set_steer(self, input_steer_in_rad):
        # Covnert radians to [-1, 1]
        input_steer = self._conv_rad_to_steer * input_steer_in_rad

        # Clamp the steering command to valid bounds
        steer           = np.fmax(np.fmin(input_steer, 1.0), -1.0)
        self._set_steer = steer

    def set_brake(self, input_brake):
        # Clamp the steering command to valid bounds
        brake           = np.fmax(np.fmin(input_brake, 1.0), 0.0)
        self._set_brake = brake
    def normalize_angle(self,angle):
        """
        Normalize an angle to [-pi, pi].
        :param angle: (float)
        :return: (float) Angle in radian in [-pi, pi]
        """
        if angle > np.pi:
            angle -= 2.0 * np.pi

        if angle < -np.pi:
            angle += 2.0 * np.pi

        return angle

    def update_controls(self):
        ######################################################
        # RETRIEVE SIMULATOR FEEDBACK
        ######################################################
        x               = self._current_x
        y               = self._current_y
        yaw             = self._current_yaw
        v               = self._current_speed
        self.update_desired_speed()
        v_desired       = self._desired_speed
        t               = self._current_timestamp
        waypoints       = self._waypoints
        throttle_output = 0
        steer_output    = 0
        brake_output    = 0

        self.vars.create_var('kp', 0.50)
        self.vars.create_var('ki', 0.10)
        self.vars.create_var('integrator_min', 0.0)
        self.vars.create_var('integrator_max', 2.50)
        self.vars.create_var('kd', 0.13)
        self.vars.create_var('kp_heading', 8.00)
        self.vars.create_var('k_speed_crosstrack', 0.00)
        self.vars.create_var('cross_track_deadband', 0.01)
        self.vars.create_var('x_prev', 0.0)
        self.vars.create_var('y_prev', 0.0)
        self.vars.create_var('yaw_prev', 0.0)
        self.vars.create_var('v_prev', 0.0)
        self.vars.create_var('t_prev', 0.0)
        self.vars.create_var('v_error', 0.0)
        self.vars.create_var('v_error_prev', 0.0)
        self.vars.create_var('v_error_integral', 0.0)
        
        # Skip the first frame to store previous values properly
        if self._start_control_loop:
            
            self.vars.v_error           = v_desired - v
            self.vars.v_error_integral += self.vars.v_error * \
                                          (0.000000001)
            v_error_rate_of_change      = (self.vars.v_error - self.vars.v_error_prev) /\
                                          (0.000000001)

            # cap the integrator sum to a min/max
            self.vars.v_error_integral = \
                    np.fmax(np.fmin(self.vars.v_error_integral, 
                                    self.vars.integrator_max), 
                            self.vars.integrator_min)

            throttle_output = self.vars.kp * self.vars.v_error +\
                              self.vars.ki * self.vars.v_error_integral +\
                              self.vars.kd * v_error_rate_of_change


            x0 = waypoints[0][0]
            x1 = waypoints[50][0]
            y0 = waypoints[0][1]
            y1 = waypoints[50][1]
        

            
            k_e = 1
            k_v = 0.1
            
            yaw_path = np.arctan2(y1 - y0,x1 - x0)
            yaw_diff = yaw - yaw_path

            yaw_diff = self.normalize_angle(yaw_diff)

            # Cross track error
            center_axle_current = np.array([x, y])
            crosstrack_error = np.min(np.sum((center_axle_current - np.array(waypoints)[:, :2]) ** 2, axis=1))

            yaw_cross_track = np.arctan2(y - y0, x - x0)
            yaw_diff_of_path_cross_track = self.normalize_angle(-yaw_path + yaw_cross_track)
            crosstrack_error = abs(crosstrack_error) if yaw_diff_of_path_cross_track > 0 else -abs(crosstrack_error)

            yaw_diff_crosstrack = np.arctan(k_e * crosstrack_error / (k_v + v)) 

            expected_steering_angle = max(-1.22, min(1.22, self.normalize_angle(yaw_diff + yaw_diff_crosstrack)))
            
            steer = expected_steering_angle

            #print(yaw_diff, yaw_diff_crosstrack)
            #print(steer, + np.arctan2(k*e,(v+ks)), psi)
            #print(theta, yaw)
            #print(len(waypoints))
            #print(np.arctan2(k*e,v))
            if (steer>1.22):
                steer = 1.22
            elif (steer<-1.22):
                steer = -1.22


            # Change the steer output with the lateral controller. 
            steer_output = steer

            if v < 0.5:
                throttle_output = 0.5
                
            ######################################################
            # SET CONTROLS OUTPUT
            ######################################################
            self.set_throttle(throttle_output)  # in percent (0 to 1)
            self.set_steer(steer_output)        # in rad (-1.22 to 1.22)
            self.set_brake(brake_output)        # in percent (0 to 1)

        self.vars.x_prev       = x
        self.vars.y_prev       = y
        self.vars.yaw_prev     = yaw
        self.vars.v_prev       = v
        self.vars.v_error_prev = self.vars.v_error
        self.vars.t_prev       = t
        
