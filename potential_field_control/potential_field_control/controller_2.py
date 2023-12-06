import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import math
import sympy
import numpy as np



class Control_Robot(Node):

    def __init__(self):
        super().__init__('robot_control_node')
        
        self.declare_parameter('xd_yd_kv_kw_arr', rclpy.Parameter.Type.DOUBLE_ARRAY)

        param = self.get_parameter('xd_yd_kv_kw_arr')

        cond_arr = param.value

        print("\n\n")
        print("Goal (x,y) : (",cond_arr[0],",",cond_arr[1],')\n') 

        print("Kv = ",cond_arr[2])
        print("Kw = ",cond_arr[3])
        print("\n") 

        self.tolerance = 0.1

        self.odom_sub = self.create_subscription(Odometry,'/odom',self.odom_callback, 20)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 20)

        self.vel_pub_timer = self.create_timer(0.01, self.vel_pub_callback)

        self.vel_msg = Twist()
        
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.qz = 0.0
        self.qw = 0.0

        kv = cond_arr[2]
        kw = cond_arr[3]

        center = [[0.0,0.0]]
        radius = [[10.8]]

        kappa = 3.5
        self.qd = [cond_arr[0],cond_arr[1]]

        x, y, qz, qw= sympy.symbols('x y qz qw')
        
        beta_sphere_0 = radius[0][0]**2 - (sympy.Pow((x-center[0][0]),2) + sympy.Pow((y-center[0][1]),2))
        beta_prod = beta_sphere_0

        gamma = sympy.Pow(x-self.qd[0],2) + sympy.Pow(y-self.qd[1],2)

        den = sympy.Pow(sympy.Pow(gamma,kappa) + beta_prod,1/kappa)
        nav = gamma/den

        dndx_sym = sympy.diff(nav,'x')
        dndy_sym = sympy.diff(nav,'y')      

        grad = [dndx_sym,dndy_sym]

        nu = sympy.sqrt(dndx_sym**2 + dndy_sym**2)

        v = kv*nu

        rho = sympy.sqrt(x**2+y**2)
        alpha = sympy.atan2(y,x)

        w = 2*kw*(nu/rho)*qz*qw*sympy.cos(alpha) - kw*(nu/rho)*(qw**2 - qz**2)*sympy.sin(alpha)
       

        self.v_func = sympy.lambdify([x,y],v,"numpy")
        self.w_func = sympy.lambdify([x,y,qz,qw],w,"numpy")
        self.first_msg = False 



    def odom_callback(self, msg):
       self.x_pos = round(msg.pose.pose.position.x,6)
       self.y_pos = round(msg.pose.pose.position.y,6)
       self.qz = msg.pose.pose.orientation.z
       self.qw = msg.pose.pose.orientation.w
       
       self.first_msg = True
       

       
    def vel_pub_callback(self):

        dist_norm = math.sqrt((self.x_pos-self.qd[0])**2 + (self.y_pos-self.qd[1])**2)

        if self.first_msg and dist_norm >self.tolerance:
            
            v_in = self.v_func(self.x_pos,self.y_pos)
            w_in = self.w_func(self.x_pos,self.y_pos,self.qz,self.qw)
            
        
            #print("vin",v_in,"win",w_in)
            print(dist_norm)

            if(not math.isnan(w_in)):
                self.vel_msg.linear.x = float(v_in)
                self.vel_msg.angular.z = float(w_in)
                self.vel_pub.publish(self.vel_msg) 
        



def main(args=None):
    rclpy.init(args=args)

    robot_control = Control_Robot()

    rclpy.spin(robot_control)

    robot_control.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
