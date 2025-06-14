import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty
import math
import numpy as np
from gazebo_msgs.srv import SetEntityState
from gazebo_msgs.msg import EntityState
from time import sleep
import random

class ControlRobot(Node):
    def __init__(self):
        super().__init__('control_robot')
        self.subscription2 = self.create_subscription(LaserScan, '/scan', self.listener_callback2, 10)
        
        self.publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription1 = self.create_subscription(Odometry,'wheel/odometry',self.listener_callback1,10)
        
        
        # Service client for reset simulation
        self.cli = self.create_client(Empty, '/reset_simulation')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting again...')
        self.req = Empty.Request()
        #set location
        self.send_reset_sim()
        # Initialize variables
        x_target = 3
        y_target = 1
        self.end_point = np.array((x_target,y_target))  # Target coordinates
        self.error = np.array([self.end_point[0], self.end_point[1]])
        self.sensor = np.ones(11) * 20  # Initialize sensor with a default large distance
        self.fitness = 99999
        #self.delta_time = 0
        self.iii = 0  # Counter to trigger reset
        self.flat = True
	# Lưu tích phân của sai số x, y và theta
        self.integral_error = np.zeros(2)
        self.prev_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.current_time = 0
	# trong so
        self.W = np.random.randn(6,2,16)
        self.V = np.random.randn(6,2,2)
        self.individual = 0
        #ga
        self.pop_size = 5
        self.min_max = [-2,2]
        self.npar = 2
        self.ngen = None
        self.mutation_rate = 0.01
        self.fitness_ga = []
        self.a = 0.2
	
    def listener_callback1(self, msg):
    	# setup location
        # Get the robot's current position and orientation

        rx = msg.pose.pose.position.x
        ry = msg.pose.pose.position.y
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w

        # Calculate yaw (theta) from quaternion
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Update error (difference between current position and target)
        self.error = np.array([(rx-self.end_point[0])**2,(ry-self.end_point[1])**2])
        
        # Neural network inputs: sensor data and current error
        nn_input = np.hstack((self.sensor, rx,ry,yaw,self.end_point[0],self.end_point[1]))
               
        # Neural network feedforward
        net_h = self.W[self.individual] @ nn_input
        yh = (1-np.exp(-net_h))/(1+np.exp(-net_h))        
        net_out = self.V[self.individual] @ yh
        
        # Output of the neural network controlling the robot
        linear_vel = 2.0 - self.a
        angular_vel = 0.2371939534944676 - 0.01
        self.a += 0.2
        self.get_logger().info(f'linear_vel: {linear_vel}, angular_vel: {angular_vel}  error:{self.error}')
        # Publish velocity command
        msg = Twist()
        msg.linear.x = linear_vel
        msg.linear.y = 0.1
        msg.angular.z = angular_vel
        self.publisher.publish(msg)
        # Log current position and orientation
        self.get_logger().info(f'Position: ({rx:.2f}, {ry:.2f}), Yaw: {yaw:.2f}')
        #self.get_logger().info(f'self.prev_time: {self.prev_time}')
        self.iii += 1
        self.get_logger().info(f'so {self.iii}')
        #Accumulation of integral of error
        

        # Increment counter to reset simulation after a certain number of iterations
        if self.iii >= 200:
        	pass
	    
    def listener_callback2(self, msg):
        # Update sensor data from the Lidar readings
        for i in range(len(msg.ranges)):
        	if not math.isinf(msg.ranges[i]):  # Kiểm tra nếu không phải là vô hạn
            		self.sensor[i] = msg.ranges[i]
        self.get_logger().info(f'Lidar data: {self.sensor}')
        
    def punish(self, point=100):
        self.fitness += point
        self.get_logger().info(f" fitness: {self.fitness}")
        self.fitness_ga.append(self.fitness)
        self.send_reset_sim()
        self.individual += 1
        sleep(0.5)
        self.iii = 0
        self.integral_error = np.zeros(2)
        self.delta_time = 0
        self.get_logger().info(f" ca the thu {self.individual}")
        
    def send_reset_sim(self):
        # Reset the simulation after some iterations
        self.cli.call_async(self.req)
        self.get_logger().info('Simulation reset triggered')
        
    def compute_fitness(self):
        # Hàm mục tiêu: Tổng bình phương của tích phân sai số
        fitness = np.sum((self.integral_error))
        return fitness
        
def main(args=None):
    rclpy.init(args=args)
    control_robot = ControlRobot()
    rclpy.spin(control_robot)
    control_robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

