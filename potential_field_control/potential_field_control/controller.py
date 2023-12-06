import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

import math
import sympy
import numpy as np
from sympy import Quaternion


class Control_Robot(Node):

    def __init__(self):
        super().__init__('robot_control_node')
        
        self.declare_parameter('xd_yd_k1_k2_arr', rclpy.Parameter.Type.DOUBLE_ARRAY)

        param = self.get_parameter('xd_yd_k1_k2_arr')

        cond_arr = param.value

        print("\n\n")
        print("Goal (x,y) : (",cond_arr[0],",",cond_arr[1],')\n') 

        print("K1 = ",cond_arr[2])
        print("K2 = ",cond_arr[3])
        print("\n") 

        self.odom_sub = self.create_subscription(Odometry,'/odom',self.odom_callback, 20)
        self.vel_pub = self.create_publisher(Twist, '/cmd_vel', 20)

        self.vel_pub_timer = self.create_timer(0.01, self.vel_pub_callback)

        self.vel_msg = Twist()
        
        self.x_pos = 0.0
        self.y_pos = 0.0
        self.theta = 0.0

        k1 = cond_arr[2]
        k2 = cond_arr[3]

        center = [[0,0]]
        radius = [[7]]

        kappa = 5.0
        qd = [cond_arr[0],cond_arr[1]]


        x, y, t= sympy.symbols('x y t')
        
        beta_sphere_0 = radius[0][0]**2 - (sympy.Pow((x-center[0][0]),2) + sympy.Pow((y-center[0][1]),2))
        beta_prod = beta_sphere_0

        gamma = sympy.Pow(x-qd[0],2) + sympy.Pow(y-qd[1],2)

        den = sympy.Pow(sympy.Pow(gamma,kappa) + beta_prod,1/kappa)
        nav = gamma/den

        a = 0.5
        
        sxy = sympy.exp(np.divide(-a,sympy.Pow(1-nav,2)) + a)

        z = sxy*sympy.atan2(y,x)+(sympy.pi)*(y/sympy.sqrt(y**2))*(1-sxy)

        rot_list = [[-sympy.cos(z),sympy.sin(z)],[-sympy.sin(z),-sympy.cos(z)]]

        M = sympy.Matrix(rot_list)

        dndx_sym = sympy.diff(nav,'x')
        dndy_sym = sympy.diff(nav,'y')

        grad_norm = sympy.sqrt(dndx_sym**2+dndy_sym**2)
        grad = [dndx_sym,dndy_sym]

        delta_nav = np.divide(grad,grad_norm)

        fxy = np.matmul(M,delta_nav)

        norm_q = sympy.sqrt(sympy.Pow((x-qd[0]),2)+sympy.Pow((y-qd[1]),2))
        norm_fxy = sympy.sqrt(fxy[0]**2+fxy[1]**2)

        
        v = k1*sympy.tanh(norm_q)


        do_fy = sympy.diff(fxy[1],'y')*sympy.sin(t) + sympy.diff(fxy[1],'x')*sympy.cos(t)
        do_fx = sympy.diff(fxy[0],'y')*sympy.sin(t) + sympy.diff(fxy[0],'x')*sympy.cos(t)

        w = -k2*(t-sympy.atan2(fxy[1],fxy[0])) + (v/norm_fxy)*(fxy[0]*do_fy - fxy[1]*do_fx)

        self.v_func = sympy.lambdify([x,y],v,"numpy")
        self.w_func = sympy.lambdify([x,y,t],w,"numpy") 

        self.first_msg = False



    def odom_callback(self, msg):
       self.x_pos = round(msg.pose.pose.position.x,6)
       self.y_pos = round(msg.pose.pose.position.y,6)
       RPY = Quaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w).to_euler('xyz')
       self.theta = float(round(RPY[0].evalf(),6))
       self.first_msg = True
       

       
    def vel_pub_callback(self):
        if self.first_msg:
            v_in = self.v_func(self.x_pos,self.y_pos)
            w_in = self.w_func(self.x_pos,self.y_pos,self.theta)
            
        
            
            #print(self.theta)
            print("vin",v_in,"win",w_in)
            
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
