import rclpy
from rclpy.node import Node

from rclpy.exceptions import ParameterNotDeclaredException
from rcl_interfaces.msg import Parameter
from rcl_interfaces.msg import ParameterType
from rcl_interfaces.msg import ParameterDescriptor

from geometry_msgs.msg import Twist
from geometry_msgs.msg import Vector3
from std_msgs.msg import Float64
from robocon_interfaces.msg import PixyVector
from time import sleep
from datetime import datetime
from collections import deque
import numpy as np

class LineFollow(Node):

    def _init_(self):
        super()._init_('track_follow')

        
        self.start_delay =1.0
        self.camera_vector_topic = "/cupcar0/PixyVector"
        self.linear_velocity = 2.0
        self.angular_velocity = -1.0
        self.single_line_steer_scale = 1.0
        self.cur_error=0
        self.prev_error=0
        self.m1=0
        self.m=0
        self.i=0
        self.q1=deque()

        # Time to wait before running
        self.get_logger().info('Waiting to start for {:s}'.format(str(self.start_delay)))
        sleep(self.start_delay)
        self.get_logger().info('Started')

        self.start_time = datetime.now().timestamp()
        self.restart_time = True

        # Subscribers
        self.pixy_subscriber = self.create_subscription(
            PixyVector,
            self.camera_vector_topic,
            self.listener_callback,
            10)

        # Publishers
        self.cmd_vel_publisher = self.create_publisher(Twist, '/cupcar0/cmd_vel', 10)

        self.speed_vector = Vector3()
        self.steer_vector = Vector3()
        self.cmd_vel = Twist()

        # Timer setup
        # timer_period = 0.5 #seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        # self.i = 0
    # def timer_callback(self):
    #     #TODO

    def listener_callback(self, msg):
        #TODO
        if(msg.m0_x1 == 0):
            msg.m0_x1, msg.m0_x0 = msg.m0_x0, msg.m0_x1
            msg.m0_y1, msg.m0_y0 = msg.m0_y0, msg.m0_y1
            
        current_time = datetime.now().timestamp()
        frame_width = 100
        frame_height = 65
        window_center = (frame_width / 2)
        x = 0
        y = 0
        steer = 0
        speed = 1.1
        
        a0 = 0
        b0 = 0
        a1 = 0
        b1 = 0
        #m = 52.00
        Kp = 0.5
        Kd = 1.4
        k = -0.8
        ki = 0.009
        
        
        a0 = (msg.m0_x0 + msg.m1_x0)/2
        b0 = (msg.m0_y0 + msg.m1_y0)/2
        a1 = (msg.m0_x1 + msg.m1_x1)/2
        b1 = (msg.m0_y1 + msg.m1_y1)/2
        if a0 != a1 :
            self.m = (b0 - b1)/(a0 - a1)
        
        self.cur_error = (a1 - window_center)/50
        
        
        if  self.i<10:
            self.q1.append(self.cur_error)
            self.i += 1
        else:
            self.q1.popleft()
            self.q1.append(self.cur_error)
            self.i += 1
        
        s = 0
        s=np.sum(self.q1)
        
        
        pid = Kp * self.cur_error + Kd * (self.cur_error - self.prev_error) + ki * s
        
        self.prev_error = self.cur_error
        
        if self.cur_error in [0,10]:
        	speed=speed+0.2
        else:
        	speed=1.1
        
        if msg.m0_x1!=0 and msg.m1_x1!=0:
            steer = k * pid
        elif msg.m0_x1!=0 and msg.m1_x1==0:
            if msg.m0_x1 in range(50,100):
                steer= 1.5 * k * pid
            else:
                steer = 1.5 * k *pid

        self.speed_vector.x = float(speed*(1-np.abs(0.05*steer)))
        self.steer_vector.z = float(steer)

        self.cmd_vel.linear = self.speed_vector
        self.cmd_vel.angular = self.steer_vector

        self.cmd_vel_publisher.publish(self.cmd_vel)

def main(args=None):
    rclpy.init(args=args)

    line_follow = LineFollow()

    rclpy.spin(line_follow)

    line_follow.destroy_node()
    rclpy.shutdown()

if __name__ == '_main_':
    main()