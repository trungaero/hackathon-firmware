# LEGO type:standard slot:3 autostart

import time                  # Need to import Time module to use Timer or get current time
from hub import BT_VCP      # Import USB_VCP module to use Serial Communication on LEGO Car
from hub import motion       # Import Motion module to get current motion status on LEGO Car
from hub import led          # Import LED module to control LED on LEGO Car
from hub import button       # Import Button module to use Button on LEGO Car
from hub import Image        # Import Image module to use built-in image
from hub import display      # Import Display module to control LEGO Car Display Screen
from hub import port         # Import Port module to set / get Port from LEGO Car

"""/**************************************************************************************************
CONSTANT DEFINITION
****************************************************************************************************/"""
E_OK = 0  # No error status
E_NOT_OK = 1  # Error
BASE_CYCLE_TIME = 1  # Base cycle time for function to be run
OS_20ms = 0  # Define name to memorize easier
OS_200ms = 1  # Define name to memorize easier
OS_1000ms = 2  # Define name to memorize easier
Emergency_Fault_Debounce = 3  # Debouncing time before triggering Emergency Stop
Emergency_Heal_Debounce = 3  # Debouncing time before clearing Emergency Stop

"""/**************************************************************************************************
GLOBAL DEFINITION
****************************************************************************************************/"""
Distance_to_Object = 0     # Distance from Sonar sensor to object (unit : cm)
LEGO_Speed = 0     # LEGO Car speed (unit : cm/s)
Motor_Power = 0     # Percentage of the rated speed for motor. Positive means clockwise, negative means counterclockwise (unit : %)
Motor_Cur_Angle = 0     # Current angle of motor (unit : deg)
Motor_Tar_Angle = 0     # Target angle for motor (unit : deg)
Elapsed_time = 0     # Elapsed time since start timer (unit : ms)
Cur_Yaw = 0     # Current Yaw
Cur_Pitch = 0     # Current Pitch
Cur_Roll = 0     # Current Roll
Emergency_Stop = False  # Emergency Stop Flag
Emergency_Fault_Counter = 0     # Debouncing counter
Emergency_Heal_Counter = 0     # Debouncing counter
RIGHT_MOTOR = ""
LEFT_MOTOR = ""
LIFT_MOTOR = ""
ARM_MOTOR = ""
Arm_Position = 0
Arm_Speed = 0
Mode = "manual"
Default_Speed = 60
Default_Turn_Speed = 10


import math
RAD2DEG = 180 / math.pi
DEG2RAD = 1 / RAD2DEG



"""/**************************************************************************************************
INIT HARDWARE
****************************************************************************************************/"""
def init_sensor(inport):
    global SENSOR
    SENSOR = inport
    SENSOR.mode(0)
    return SENSOR


def init_com_port(inport):
    global COMPort
    COMPort = inport
    COMPort.mode(1)  # Set full duplex mode
    time.sleep(0.1)
    COMPort.baud(115200)  # Set baud rate
    return COMPort


def init_left_motor(inport):
    global LEFT_MOTOR
    LEFT_MOTOR = inport
    LEFT_MOTOR.default(stop=0, deceleration=100)
    time.sleep(0.2)
    return LEFT_MOTOR


def init_right_motor(inport):
    global RIGHT_MOTOR
    RIGHT_MOTOR = inport
    RIGHT_MOTOR.default(stop=0, deceleration=100)
    time.sleep(0.2)
    return RIGHT_MOTOR


def init_lift_motor(inport):
    global LIFT_MOTOR
    LIFT_MOTOR = inport
    LIFT_MOTOR.default(stop=2, deceleration=100)
    LIFT_MOTOR.preset(0)
    abs_pos = LIFT_MOTOR.get()[2]
    LIFT_MOTOR.run_to_position(-abs_pos)
    time.sleep(0.5)
    LIFT_MOTOR.preset(0)
    LIFT_MOTOR.run_to_position(180, speed=20)
    return LIFT_MOTOR


def init_arm_motor(inport):
    global ARM_MOTOR
    ARM_MOTOR = inport
    ARM_MOTOR.default(stop=2, deceleration=100)
    ARM_MOTOR.preset(0)
    abs_pos = ARM_MOTOR.get()[2]
    ARM_MOTOR.run_to_position(-abs_pos)
    time.sleep(0.5)
    ARM_MOTOR.preset(0)
    return ARM_MOTOR


def move_robot(data):
    global Default_Speed
    global Default_Turn_Speed

    if data == "n1":
        move_forward(Default_Speed)
    elif data == "n2":  # Move backward
        move_backward(Default_Speed)
    elif data == "n3":  # Turn left
        turn_left(Default_Turn_Speed)
    elif data == "n4":  # Turn right
        turn_right(Default_Turn_Speed)


def move_arm(data):
    global Arm_Position
    global Arm_Speed

    # caculate degress
    arm_pos = int(data[1:])

    if arm_pos > 0:
        Arm_Position = arm_pos
        Arm_Speed = 20
    elif arm_pos <= 0:
        Arm_Position = abs(arm_pos)
        Arm_Speed = -20


def move_lift_position(data):
    max_position = 180
    min_position = 0

    value = data[1:]
    value = int(value)

    # map the int to a value between 90-160
    mapped_value = min_position + ((value / 100) * (max_position-min_position))
    LIFT_MOTOR.run_to_position(mapped_value, speed=20)


def move_foot(data):
    if data == "ff":
        control_lift_motor_clockwise()
    elif data == "fb":
        control_lift_motor_counter_clockwise()
    else:
        move_lift_position(data)


def set_speed_robot(data):
    global Default_Speed
    global Default_Turn_Speed

    value = data[1:]

    # convert value to an int
    value = int(value)

    # set default_speed
    if (value >= 0) and (value <= 100):
        Default_Speed = value


def move_forward(speed):
    LEFT_MOTOR.run_at_speed(-speed)
    RIGHT_MOTOR.run_at_speed(speed)


def move_backward(speed):
    RIGHT_MOTOR.run_at_speed(-speed)
    LEFT_MOTOR.run_at_speed(speed)


def turn_left(speed):
    RIGHT_MOTOR.run_at_speed(speed)
    LEFT_MOTOR.run_at_speed(speed)


def turn_right(speed):
    RIGHT_MOTOR.run_at_speed(-speed)
    LEFT_MOTOR.run_at_speed(-speed)


def control_lift_motor_clockwise():
    LIFT_MOTOR.run_at_speed(20)


def control_lift_motor_counter_clockwise():
    LIFT_MOTOR.run_at_speed(-20)


def display_image(image):
    display.show(image)


def change_led_color(led_color):
    led(led_color)


def check_pressed_button(button):
    if button == "Center":
        return button.center.is_pressed()
    if button == "Left":
        return button.left.is_pressed()
    if button == "Right":
        return button.right.is_pressed()



def init_lego_car() -> int:
    try:
        global COMPort, LEFT_MOTOR, RIGHT_MOTOR, LIFT_MOTOR, ARM_MOTOR, SONAR_SENSOR
        # Use Virtual Com Port for Serial communication
        COMPort = BT_VCP(0)
        # Wait 200ms for connection to be connected
        time.sleep(0.02)
        # Use Port A for Right motor
        RIGHT_MOTOR = init_right_motor(port.A.motor)
        # Wait 200ms to initialize Motor
        time.sleep(0.02)
        # Use Port B for Left motor
        LEFT_MOTOR = init_left_motor(port.B.motor)
        # Wait 200ms to initialize Motor
        time.sleep(0.02)

        # Use Port C for foot motor
        LIFT_MOTOR = init_lift_motor(port.C.motor)
        # Wait 200ms to initialize Motor
        time.sleep(0.02)
        # Use Port D for hand motor
        ARM_MOTOR = init_arm_motor(port.D.motor)
        # Wait 200ms to initialize Motor
        time.sleep(0.02)

        # Use Port E for Sonar sensor
        SONAR_SENSOR = init_sensor(port.E.device)

        # Reset gyro
        motion.yaw_pitch_roll(0)
        # Display Happy face since there is no problem :)
        display_image(Image.HAPPY)
        # Wait 1 second for everything to be completed
        time.sleep(0.5)
        # Set LED color to Green
        change_led_color(2)
        return E_OK
    except:
        # Error happen :( Display Sad face
        display_image(Image.SAD)
        # Set LED color to Red
        change_led_color(9)
        return E_NOT_OK



class Navigator:
    def __init__(self, robot, x0=0, y0=0, the0=0):
        """
        robot: Robot
        x0: float
        y0: float
        the0: float
        """
        self.x = x0  # in cm
        self.y = y0  # in cm   
        self.the = the0 # in deg
        self.robot = robot
        self.prev_encoder = None

    def reset(self, x=0, y=0, the=0):
        self.x = x
        self.y = y
        self.the = the

    def __diff_encoder(self, prev_phi, phi, sign=1):
        """
            prev_phi: last position in degree
            phi: cur position in degree
            speed: speed in deg/s 

            return: change in angle
        """
        dphi = phi - prev_phi
        return sign * dphi

    def update(self):
        """
            dr: change in right wheel angle
            dl: change in left wheel angle
        """
        cur_encoder = self.robot.get_encoder()

        if self.prev_encoder is None:
            self.prev_encoder = cur_encoder
            return
            
        dl = self.__diff_encoder(self.prev_encoder[0], cur_encoder[0])
        dr = self.__diff_encoder(self.prev_encoder[1], cur_encoder[1])
        dthe = self.robot.R_WHEEL / self.robot.L_CHASSIS * (dl - dr)
        self.the += dthe

        if self.the > 180:
            self.the -= 360
        elif self.the < -180:
            self.the += 360
        
        # compute change in x, y
        dx = (self.robot.R_WHEEL/2) * math.sin(self.the * DEG2RAD) * (dr + dl) * DEG2RAD
        dy = (self.robot.R_WHEEL/2) * math.cos(self.the * DEG2RAD) * (dr + dl) * DEG2RAD

        # update x, y
        self.x += dx * 100
        self.y += dy * 100

        self.prev_encoder = cur_encoder

        # debug
        # print((self.x, self.y, self.the))


class GoToGoal:
    def __init__(self):
        self.type = 'gotogoal'
        # gains
        self.Kp = 2
        self.Ki = 0.01
        self.Kd = 0.03

        # memory
        self.E_k = 0
        self.e_k_1 = 0
        self.the_1 = 0

    def execute(self, nav, input, dt):
        """
        nav: Navigator
        input:  (x, y, v)
        dt: float

        Return
            (v, w): v: desired speed in cm/s, w: desired rate in rad/s
        """
        x_g = input[0]
        y_g = input[1]
        v = input[2]

        u_x = x_g - nav.x
        u_y = y_g - nav.y
        theta_g = math.atan2(u_x, u_y) * RAD2DEG

        # heading error
        e_k = (theta_g - nav.the) * DEG2RAD
        e_k = math.atan2(math.sin(e_k), math.cos(e_k))

        e_the = (nav.the - self.the_1) * DEG2RAD
        e_the = math.atan2(math.sin(e_the), math.cos(e_the))

        # calculate pid
        e_P = e_k
        e_I = self.E_k + e_k * dt
        e_D = e_the/dt

        w = self.Kp*e_P + self.Ki*e_I - self.Kd*e_D
        # save errors
        self.E_k = e_I
        self.e_k_1 = e_k
        self.the_1 = nav.the

        return (v, w)
    
    def reset(self):
        self.E_k = 0
        self.e_k_1 = 0
        self.the_1 = 0


class GoToHeading(GoToGoal):
    def __init__(self):
        super().__init__()
        self.type = 'gotoheading'
        self.Kp = 2
        self.Ki = 0.05
        self.Kd = 0.05

    def execute(self, nav, input, dt):
        _, w = super().execute(nav, input, dt)
        return (0, w)


class Stop:
    def __init__(self):
        self.type = 'stop'

    def execute(self, nav, input, dt):
        return (0, 0)
    

class Robot:
    R_WHEEL = 5.6 / 2 / 100
    L_CHASSIS = 0.095

    def __init__(self):
        self.motor_left = None
        self.motor_right = None

    def attach_motors(self, motor_left, motor_right):
        self.motor_left = motor_left
        self.motor_right = motor_right

    def set_wheel_speeds(self, vel_l, vel_r):
        self.motor_left.run_at_speed(-vel_l * 5)
        self.motor_right.run_at_speed(vel_r * 5)

    def uni_to_diff(self, v, w):
        """ 
        v: cm/s
        w: rad/s

        Return
            vel_r: left wheel speed in rad/s
            vel_l: right wheel speed in rad/s
        """
        # to cm
        L = self.L_CHASSIS * 100
        R = self.R_WHEEL * 100   

        vel_l = (2*v + w*L) / (2*R)
        vel_r = (2*v - w*L) / (2*R)
        return (vel_l, vel_r)

    def diff_to_uni(self, vel_l, vel_r):
        """
        vel_l: left wheel speed in rad/s
        vel_r: right wheel speed in rad/s

        Return
            v: cm/s
            w: rad/s
        """
        # to cm
        L = self.L_CHASSIS * 100
        R = self.R_WHEEL * 100    

        v = R/2*(vel_l + vel_r)
        w = R/L*(vel_l - vel_r)

        return (v,w)

    def get_encoder(self):
        return (-self.motor_left.get()[1], self.motor_right.get()[1])

    def reset_encoder(self):
        self.motor_left.preset(0)
        self.motor_right.preset(0)


class Supervisor:
    def __init__(self, navigator, robot):
        self.controllers = {
            'gotogoal': GoToGoal(), 
            'stop': Stop(), 
            'gotoheading': GoToHeading(),
        }
        self.nav = navigator
        self.robot = robot

        # targets
        # desired speed in percent
        self.v = 20     # cm/s
        self.x_g = None  # cm
        self.y_g = None  # cm
        self.d_stop = 2 # cm
        self.the_stop = 2
        self.targets = []
        self.init()
        
    def init(self):
        self.x_g = self.nav.x
        self.y_g = self.nav.y
        self.targets = []
        self.start()
        
    def add_target(self, x_g, y_g):
        self.targets.append((x_g, y_g))

    def has_target(self):
        return len(self.targets) > 0

    def fetch_target(self):
        self.x_g, self.y_g = self.targets.pop(0)

    def execute(self, dt):
        if self.at_heading():
            self.current_controller.reset()
            self.current_controller = self.controllers['gotogoal']
        elif self.at_goal():
            if self.has_target():
                self.fetch_target()
                self.current_controller.reset()
                self.current_controller = self.controllers['gotoheading']
            else:
                self.current_controller = self.controllers['stop']

        input = (self.x_g, self.y_g, self.v)   
        output = self.current_controller.execute(self.nav, input, dt)
        vel_l, vel_r = self.ensure_w(output)
        self.robot.set_wheel_speeds(vel_l, vel_r)

    def stop(self):
        self.current_controller = self.controllers['stop']

    def start(self):
        self.current_controller = self.controllers['gotoheading']

    def at_goal(self):
        return norm([self.nav.x, self.nav.y], [self.x_g, self.y_g]) < self.d_stop
    
    def at_heading(self):
        return abs(math.atan2(self.x_g-self.nav.x, self.y_g-self.nav.y) * RAD2DEG - self.nav.the) < self.the_stop

    def ensure_w(self, output):
        return self.robot.uni_to_diff(*output)


def lin_interp(x1, y1, x2, y2, x):
    return y1 + (y2 - y1) * ((x - x1) / (x2 - x1))


def norm(p1, p2):
    return math.sqrt((p2[1]-p1[1])**2 + (p2[0]-p1[0])**2)


def com_check():
    global Mode
    # # Read keyboard input from USB serial
    data = COMPort.read()
    if data:
        try:
            data = str(data)
            data = data[2:-1]
            COMPort.write(data)
            if Mode == 'manual': 
                if data[0] == "n":
                    move_robot(data)
                elif data[0] == "a":
                    move_arm(data)
                elif data[0] == "f":
                    move_foot(data)
                elif data[0] == "s":
                    set_speed_robot(data)
                elif data[0] == "d":  # Display numbers on LEGO Car Display Screen
                    # DisplayScreen(data)
                    pass
                else:
                    RIGHT_MOTOR.float()
                    LEFT_MOTOR.float()

            if data[0] == "m":
                if data[1] == '0':
                    Mode = 'manual'
                elif data[1] == '1':
                    Mode = 'auto'
        except:
            pass


def Activate(OS_Type):
    # If OS type is 20ms, then activate function named MainFnc_20ms
    if OS_Type == OS_20ms:
        MainFnc_20ms()
    # If OS type is 200ms, then activate function named MainFnc_200ms
    if OS_Type == OS_200ms:
        MainFnc_200ms()
    # If OS type is 1000ms, then activate function named MainFnc_100ms
    if OS_Type == OS_1000ms:
        MainFnc_1000ms()
    # You can customize this function to use more OS type if required


def MainFnc_20ms():
    global Mode
    MyNavigator.update()
    if Mode == 'auto':
        MySupervisor.execute(0.02)
    com_check()
    COMPort.write("nav:%.1f,%.1f,%.1f" % (MyNavigator.x, MyNavigator.y, MyNavigator.the)) 
    

def MainFnc_200ms():
    pass
    
    
def MainFnc_1000ms():
    pass


MyRobot = Robot()
MyNavigator = Navigator(MyRobot)
MySupervisor = Supervisor(MyNavigator, MyRobot)
MySupervisor.add_target(20,0)
MySupervisor.add_target(20,20)
MySupervisor.add_target(40,20)
MySupervisor.add_target(40,40)
MySupervisor.add_target(60,60)
MySupervisor.add_target(60,0)
MySupervisor.add_target(0,0)


def main():
    '''--------------------------
            LOCAL VARIABLE
    --------------------------'''
    OS_Counter = 0 # Store current OS Count. Count up every BASE_CYCLE_TIME
    '''------------------------'''

    # Initialize LEGO Car
    # If error happens, exit program
    if (init_lego_car() == E_NOT_OK):
        exit()
    MyRobot.attach_motors(LEFT_MOTOR, RIGHT_MOTOR)
    MyRobot.reset_encoder()
    time.sleep(0.5)

    # Run program until Center Button is pressed
    while not button.center.is_pressed():
        # Count up OS_Counter by 1
        OS_Counter = OS_Counter + 1
        # Depend on current count, we activate corresponding task
        # Activate OS_20ms every BASE_CYCLE_TIME (20ms)
        if OS_Counter % BASE_CYCLE_TIME == 0:
            Activate(OS_20ms)
        # Activate OS_200ms every BASE_CYCLE_TIME * 10 (200ms)
        if OS_Counter % (BASE_CYCLE_TIME*10) == 0:
            Activate(OS_200ms)
        # Activate OS_1000ms every BASE_CYCLE_TIME * 50 (1000ms)
        if OS_Counter % (BASE_CYCLE_TIME*50) == 0:
            Activate(OS_1000ms)
        # If required, you can add more OS type here to call specific function every X ms
        # Reset OS_Counter every 50 cycles to prevent data overflow
        if OS_Counter >= 50:
            OS_Counter = 0
        # Sleep for 20ms
        time.sleep(0.02)


main()  # Runs main function
