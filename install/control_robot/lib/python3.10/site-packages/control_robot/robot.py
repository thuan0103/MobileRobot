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
import os
import ast

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
        sleep(1)
        # Initialize variables
        self.x_target = -2
        self.y_target = -5
        self.end_point = np.array((self.x_target,self.y_target))  # Target coordinates
        self.error = np.array([self.end_point[0], self.end_point[1]])
        self.sensor = np.ones(11) * 20  # Initialize sensor with a default large distance
        self.fitness = 0
        #self.delta_time = 0
        self.iii = 0  # Counter to trigger reset
        self.flat = True
	# Lưu tích phân của sai số x, y và theta
        self.integral_error = np.zeros(2)
        self.prev_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.current_time = 0
        self.check = 52
        self.distance = 1
	# trong so
        #with open('W.txt', 'r') as file:
            #data = file.read()
        #self.W = np.array(ast.literal_eval(data))
        self.W = np.random.randn(6,16,16)
        
        #with open('W1.txt', 'r') as file:
            #data = file.read()
        #self.W1 = np.array(ast.literal_eval(data))
        self.W1 = np.random.randn(6,16,16)
        
        #with open('V.txt', 'r') as file:
            #data = file.read()
        #self.V = np.array(ast.literal_eval(data))
        self.V = np.random.randn(6,2,16)
        self.individual = 0
        #ga
        self.pop_size = 6
        self.min_max = [-2,2]
        self.npar = 2
        self.ngen = None
        self.mutation_rate = 0.01
        self.fitness_ga = []
        self.count = 0
        self.best_individual = 0
        # dao ham
        self.prev_error_x_squared = 0
        self.prev_error_y_squared = 0

        self.prev_derivative_x = 99999
        self.prev_derivative_y = 99999
        self.generation = 0
        self.get_logger().info(f"the he thu: {self.generation}")
        self.position = []
        self.save = []
        
    def listener_callback1(self, msg):
        
    	# setup location
        self.flat = True
        # Get the robot's current position and orientation

        rx = msg.pose.pose.position.x
        ry = msg.pose.pose.position.y
        x = msg.pose.pose.orientation.x
        y = msg.pose.pose.orientation.y
        z = msg.pose.pose.orientation.z
        w = msg.pose.pose.orientation.w
        self.position.append((rx, ry))
        # Calculate yaw (theta) from quaternion
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        # Update error (difference between current position and target)
        self.error = np.array([(rx-self.end_point[0])**2,(ry-self.end_point[1])**2])
        distance_to_goal = math.sqrt(self.error[0] + self.error[1])
        # Neural network inputs: sensor data and current error
        nn_input = np.hstack((self.sensor, rx,ry,yaw,self.end_point[0],self.end_point[1]))
               
        # Neural network feedforward
        net_h = self.W[self.individual] @ nn_input
        yh = (1-np.exp(-net_h))/(1+np.exp(-net_h))
        
        net_h1 = self.W1[self.individual] @ yh
        yh1 = (1-np.exp(-net_h1))/(1+np.exp(-net_h1)) 
                
        net_out = self.V[self.individual] @ yh1
        
        # Output of the neural network controlling the robot
        linear_vel = round(float(net_out[0]),1) #0.25
        angular_vel = round(float(net_out[1])*0.1,1) #0.02

        #self.get_logger().info(f'linear_vel: {linear_vel}, angular_vel: {angular_vel}  error:{self.error}')
        # Publish velocity command
        msg = Twist()
        msg.linear.x = linear_vel
        self.publisher.publish(msg)
        msg.angular.z = angular_vel
        self.publisher.publish(msg)
        self.iii += 1
        #Accumulation of integral of error
        if self.iii % 40 == 0:          
            self.current_time = self.get_clock().now().seconds_nanoseconds()[0]           
            delta_time = self.current_time - self.prev_time
            self.integral_error += self.error * delta_time
            fitness = np.sum((self.error*delta_time))
            self.fitness += fitness
            self.distance = math.sqrt((self.x_target - rx)**2 + (self.y_target - ry)**2)
            self.check -= 1
            if self.distance <= 9:
                self.fitness -= 200
            #dao ham
            error_x_squared = self.error[0]
            error_y_squared = self.error[1]
        
            derivative_error_x_squared = (error_x_squared - self.prev_error_x_squared) / delta_time
            derivative_error_y_squared = (error_y_squared - self.prev_error_y_squared) / delta_time
        
        
            self.prev_error_x_squared = error_x_squared
            self.prev_error_y_squared = error_y_squared
            
            
            if derivative_error_x_squared > self.prev_derivative_x or derivative_error_y_squared > self.prev_derivative_y:
                pass   
                #self.fitness += 500
                #self.get_logger().info(f'Phạt vì đạo hàm, Fitness: {self.fitness}')
       
            self.prev_derivative_x = derivative_error_x_squared
            self.prev_derivative_y = derivative_error_y_squared

            
            self.prev_time = self.current_time
            self.get_logger().info(f'Fitness: {self.fitness} || Position: ({rx:.2f}, {ry:.2f})|| linear_vel: {linear_vel}, angular_vel: {angular_vel}')
        # Increment counter to reset simulation after a certain number of iterations
               
        if self.iii >= 1200:
            self.save.append(self.position[-1])
            self.punish(0)
                    
	    
    def listener_callback2(self, msg):
        min_distance_threshold = 1
        for i in range(len(msg.ranges)):
            if not math.isinf(msg.ranges[i]):  # Kiểm tra nếu không phải là vô hạn
                self.sensor[i] = msg.ranges[i]
                if msg.ranges[i] < min_distance_threshold:
                    self.get_logger().info('Obstacle detected! Resetting position...')
                    self.save.append(self.position[-1])
                    self.punish(10000*self.distance)
                    self.check = 52
                    break                   
        #self.get_logger().info(f'Lidar data: {msg.ranges}')

    def send_reset_sim(self):
        # Reset the simulation after some iterations
        self.cli.call_async(self.req)
        self.get_logger().info('Simulation reset triggered')
        
    def compute_fitness(self):
        # Hàm mục tiêu: Tổng bình phương của tích phân sai số
        fitness = np.sum((self.integral_error))
        return fitness
        
    def punish(self, point=1000):
        self.fitness += point
        self.get_logger().info(f" fitness: {self.fitness}")
        self.fitness_ga.append(self.fitness)
        self.send_reset_sim()
        self.individual += 1
        sleep(1.5)
        self.iii = 0
        self.integral_error = np.zeros(2)
        self.delta_time = 0
        self.fitness = 0
        self.get_logger().info(f" ca the thu {self.individual}")
        if self.individual >= 6:
            self.generation += 1
            self.count += 1
            self.get_logger().info(f"bat dau qua trinh tien hoa lan {self.count}")
            self.save_weight(self.generation, self.W, self.W1, self.V)
            self.W = self.ga_callback(self.W,i = 'w')
            self.W1 = self.ga_callback(self.W1,i = 'w1')
            self.V = self.ga_callback(self.V, i= 'v')
            self.individual = 0
            self.fitness_ga.clear()
            
    def save_weight(self,generation,W,W1,V):
        with open("weights_log.txt","a") as file:
            file.write(f"the he thu: {generation}:\n")
            for i in range(self.pop_size):
                file.write(f"Ca the {i} \n")
                rx,ry = self.save[i]
                fitness = self.fitness_ga[i]
                file.write(f'Position: ({rx:.2f}, {ry:.2f})\n')
                file.write(f'fitness: {fitness}\n')
                file.write(f"W: \n" + np.array2string(W[i], separator=', ') + "\n")
                file.write(f"W1: \n" + np.array2string(W1[i], separator=', ') + "\n")
                file.write(f"V: \n" + np.array2string(V[i], separator=', ') + "\n")
            file.write("\n")
# call ga
    def ga_callback(self,pop,i = 'w'):
        if i == 'w':
            self.ngene = 16
            self.npar = 16
        elif i == 'w1':
            self.ngene = 16
            self.npar = 16
        else:
            self.ngene = 16
            self.npar = 2
        	
        en = self.encode(pop)
        s = self.selection(en,c=0.3)
        c = self.crossover(s)
        m = self.mutation(c)
        de = self.decode(m)
        return de
#ga
    def encode(self,pop):
        en = []
        for i in range(len(pop)):
            p = pop[i]
            value = np.zeros((self.npar, self.ngene))
            for j in range(self.npar):
                for k in range(len(p[j])):
                    norm_value = (p[j][k] - self.min_max[0]) / (self.min_max[1] - self.min_max[0]) * (10**self.ngene-5)
                    value[j,k] = norm_value/(10**(self.ngene-1)*9)
            en.append(value)
        return np.array(en)

    def decode(self, pop):
        de = []
        for c in range(len(pop)):
            p = pop[c]
            value = np.zeros((self.npar, self.ngene))
            for i in range(self.npar):
                for j in range(self.ngene):
                    decode_value = p[i,j]*(10**(self.ngene-1)*9)
                    a = (decode_value/(10**self.ngene-5)) * (self.min_max[1] - self.min_max[0]) + self.min_max[0]
                    value[i,j] = a
            de.append(value)
        return np.array(de)

    def selection(self,pop, c):
        sort_index = np.argsort(self.fitness_ga)
        p = pop[sort_index,:,:]
        n = len(self.fitness_ga)
        pk = [1]
        for _ in range(n-1):
            next_element = pk[-1]*c
            pk.append(next_element)
        pk = pk/np.sum(pk)
        p1=np.random.choice(range(int(n)), size=int(n), p=pk)
        p = p[p1,:,:]
        return p
    
    def crossover(self,p):
        save = []
        k = int(len(p)/2)
        numbers = list(range(0,k*2))
        random_numbers = random.sample(numbers,k*2)
        pairs = [(random_numbers[i], random_numbers[i+1]) for i in range(0,6,2)]
        for a in pairs:
            try:
                point = np.random.randint(1, self.ngene-1)
            except:
                point = np.random.randint(1, self.ngene)
            p1 = p[a[0],:,:]
            p2 = p[a[1],:,:]
            c1 = np.hstack((p1[:, :point], p2[:, point:]))
            c2 = np.hstack((p2[:, :point], p1[:, point:]))
            save.append(c1)
            save.append(c2)
        save = np.array(save)
        best_individual = p[np.argmin(self.fitness_ga), :, :]
        save = np.vstack((save[:self.pop_size - 1], [best_individual]))
        self.best_individual = best_individual
        return save

    def mutation(self,p):
        pp = p
        for k in range(self.pop_size):
            if np.array_equal(pp[k], self.best_individual):
                continue
            for i in range(self.npar):
                if np.random.rand() <= self.mutation_rate:
                    self.get_logger().info(f"co dot bien")
                    indices_to_change = np.random.choice(self.ngene, size=1, replace=False)
                    for idx in indices_to_change:
                        pp[k,i,idx] = np.random.randint(0,10)
        return pp
    
def main(args=None):
    rclpy.init(args=args)
    control_robot = ControlRobot()
    rclpy.spin(control_robot)
    control_robot.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

