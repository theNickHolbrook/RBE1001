#region VEXcode Generated Robot Configuration
from vex import *
import urandom
import math

# Brain should be defined by default
brain=Brain()

# Robot configuration code
controller_1 = Controller(PRIMARY)
motor_1 = Motor(Ports.PORT1, GearSetting.RATIO_18_1, True)
motor_2 = Motor(Ports.PORT2, GearSetting.RATIO_18_1, False)
line_tracker_h = Line(brain.three_wire_port.h)
line_tracker_g = Line(brain.three_wire_port.g)
# AI Vision Color Descriptions
ai_vision_7__G = Colordesc(1, 48, 142, 89, 10, 0.2)
ai_vision_7__P = Colordesc(2, 154, 110, 187, 10, 0.2)
ai_vision_7__O = Colordesc(3, 204, 69, 84, 10, 0.2)
# AI Vision Code Descriptions
ai_vision_7 = AiVision(Ports.PORT7, ai_vision_7__G, ai_vision_7__P, ai_vision_7__O)
motor_10 = Motor(Ports.PORT10, GearSetting.RATIO_18_1, False)
motor_8 = Motor(Ports.PORT8, GearSetting.RATIO_18_1, False)
motor_9 = Motor(Ports.PORT9, GearSetting.RATIO_18_1, True)
bumper_f = Bumper(brain.three_wire_port.f)
inertial_12 = Inertial(Ports.PORT12)


# wait for rotation sensor to fully initialize
wait(30, MSEC)


# Make random actually random
def initializeRandomSeed():
    wait(100, MSEC)
    random = brain.battery.voltage(MV) + brain.battery.current(CurrentUnits.AMP) * 100 + brain.timer.system_high_res()
    urandom.seed(int(random))
      
# Set random seed 
initializeRandomSeed()


def play_vexcode_sound(sound_name):
    # Helper to make playing sounds from the V5 in VEXcode easier and
    # keeps the code cleaner by making it clear what is happening.
    print("VEXPlaySound:" + sound_name)
    wait(5, MSEC)

# add a small delay to make sure we don't print in the middle of the REPL header
wait(200, MSEC)
# clear the console to make sure we don't have the REPL in the console
print("\033[2J")

#endregion VEXcode Generated Robot Configuration

# ------------------------------------------
# 
# 	Project:      VEXcode Project
#	Author:       VEX
#	Created:
#	Description:  VEXcode V5 Python Project
# 
# ------------------------------------------

# Library imports
from vex import *
import math

# Begin project code
    
class drive:
    
    def __init__(self, motorA, motorB, line_tracker_left, line_tracker_right, odometry):
        self.motorA = motorA
        self.motorB = motorB

        self.line_tracker_left = line_tracker_left
        self.line_tracker_right = line_tracker_right

        self.o = odometry
        
        self.pos_tollerance = 0.05
        self.rot_tollerance = 0.1
        self.rot_while_driving_tollerance = math.pi/12

        self.threshold = 50

        self.rot_kP = 100
        self.dist_kP = 100

        self.min_drive_speed = 20
        self.min_rot_speed = 20

    
    def basic_drive(self, r, l):
        self.motorA.spin(FORWARD, l, PERCENT)
        self.motorB.spin(FORWARD, r, PERCENT)

    def _withinTolleranceDistance(self, actual, goal, tollerance):
        return (actual > goal - tollerance) and (actual < goal + tollerance)
    
    def _withinTolleranceAngular(self, actual, goal, tollerance):
        r_E = goal - actual
        return (abs(r_E) <= tollerance) or (abs(r_E) >= (2 * math.pi) - tollerance)
    
    #turns towards object such as tree or bucket, assumes object has tag in first index
    def turn_to_object(self, pos, obj):
        x, y, rot = pos
        v = [obj[0] - x, obj[1] - y]
        # theta = math.atan2(v[1], v[0]) + math.pi
        theta = math.atan2(v[1], v[0])
        if theta < 0:
            theta += (2 * math.pi)
        r_E = theta - rot
        # r_E2 = rot - theta
        # if abs(r_E) > abs(r_E2): r_E = r_E2
        if r_E > math.pi:
            r_E -= 2 * math.pi
        elif r_E < -1 * math.pi:
            r_E += 2 * math.pi
        turning_speed = self.rot_kP * r_E

        brain.screen.set_cursor(3,2)
        brain.screen.print("Goal angle ", theta)
        brain.screen.set_cursor(4,2)
        brain.screen.print("Turning Speed ", turning_speed)
        if self._withinTolleranceAngular(rot, theta, self.rot_tollerance):
            self.basic_drive(0, 0)
            return True
        else:
            self.basic_drive(turning_speed, -1 * turning_speed)
            return False

    def trajectory_drive(self, pos, waypoint):#pos = [x, y, rot], waypoint = [x_goal, y_goal, on line]
        brain.screen.set_cursor(3,2)
        brain.screen.print("waypoint of", waypoint)
        
        #get heading
        # goal_x, goal_y, online = waypoint
        #for some reason the waypoint will sometimes have four indecies which is not supposed to ever happen and messing the code up
        goal_x = waypoint[0]
        goal_y = waypoint[1]
        online = waypoint[2]
        pos_x, pos_y, pos_rot = pos
        #fix
        goal_heading = math.atan2(goal_y - pos_y, goal_x - pos_x)
        if goal_heading < 0:
            goal_heading += 2 * math.pi
        if goal_heading > 2*math.pi:
            goal_heading -= 2 * math.pi


        brain.screen.set_cursor(4,2)
        brain.screen.print("Goal heading ", goal_heading)

        d_E = math.sqrt(math.pow(goal_x - pos_x, 2) + math.pow(goal_y - pos_y, 2))
        drive_speed = self.dist_kP * (d_E)

        if abs(drive_speed) < self.min_drive_speed:
            if drive_speed < 0:
                drive_speed = self.min_drive_speed * -1
            else:
                drive_speed = self.min_drive_speed

        r_E = goal_heading - pos_rot
        
        if r_E > math.pi:
            r_E -= (2 * math.pi)
        elif r_E < -1 * math.pi:
            r_E += (2 * math.pi)
        brain.screen.set_cursor(11,2)
        brain.screen.print("r_E: ", r_E)
        turn_speed = self.rot_kP * (r_E)
        #Fix heading
        if not self._withinTolleranceAngular(pos_rot, goal_heading, self.rot_while_driving_tollerance):
            # drive_speed *= 0.2
            drive_speed = 0
            if abs(turn_speed) < self.min_rot_speed:
                if turn_speed < 0:
                    turn_speed = self.min_rot_speed * -1
                else:
                    turn_speed = self.min_rot_speed
            brain.screen.set_cursor(6,2)
            brain.screen.print("Outside angle tollerance")
            online = False
        else:
            brain.screen.set_cursor(6,2)
            brain.screen.print("Within angle tollerance")

        if online and False:#add back once line stuff is ready
            # Get sensor values
            # if abs(goal_heading - (math.pi/2)) < math.pi/12:
            #     self.o.set_x = goal_x
            # elif abs(goal_heading - (3 * math.pi/2)) < math.pi/12:
            #     self.o.set_y = goal_y
            self.o.set_rot(goal_heading)
            left_value = self.line_tracker_left.reflectivity()
            right_value = self.line_tracker_right.reflectivity()
    
            # Robot logic control
            if left_value > self.threshold and not right_value > self.threshold:
                # left_drive_smart.spin(REVERSE, speed, PERCENT) 
                # right_drive_smart.spin(REVERSE, slow_speed, PERCENT) 
                turn_speed = 20
                brain.screen.set_cursor(9,2)
                brain.screen.print("Left value caught")
            elif right_value > self.threshold and not left_value > self.threshold:
                # left_drive_smart.spin(REVERSE, slow_speed, PERCENT) 
                # right_drive_smart.spin(REVERSE, speed, PERCENT)
                turn_speed = -20
                brain.screen.set_cursor(9,2)
                brain.screen.print("Right value caught")
            else:
                brain.screen.set_cursor(9,2)
                brain.screen.print("No value caught")
                turn_speed = 0
            
            brain.screen.set_cursor(8,2)
            brain.screen.print("On line")

        else:
            brain.screen.set_cursor(8,2)
            brain.screen.print("Off line")
        
        brain.screen.set_cursor(5,2)
        brain.screen.print("Turning speed ", turn_speed)
        brain.screen.set_cursor(7,2)
        brain.screen.print("Drive Speed: ", drive_speed, " d_E: ", d_E)

        if self._withinTolleranceDistance(pos_x, goal_x, self.pos_tollerance) and self._withinTolleranceDistance(pos_y, goal_y, self.pos_tollerance):
            self.motorA.spin(FORWARD, 0, PERCENT)
            self.motorB.spin(FORWARD, 0, PERCENT)
            return True
        else:
            self.motorA.spin(FORWARD, drive_speed - turn_speed, PERCENT)
            self.motorB.spin(FORWARD, drive_speed + turn_speed, PERCENT)
            return False

    
class odometry:
    
    def __init__(self, x=0.32, y=0.3, rot=math.pi/2, imu=inertial_12):
    # def __init__(self, x=0.25, y=3.495, rot=3*math.pi/2):
        self.t = brain.timer.time(SECONDS)
        self.s_t = self.t
        self.imu = inertial_12
        self.imu.set_heading(rot * (180 / math.pi), DEGREES)
        self.x = x
        self.y = y
        self.rot = rot

        self.past_A = 0
        self.past_B = 0

        #both guesses in meters
        self.wheel_radius = 0.055
        self.wheel_base =  0.31
        self.gear_ratio = 4.64
    
    def set_rot(self, rot):
        self.rot = rot
    def set_x(self, x):
        self.x = x
    def set_y(self, y):
        self.y = y

    def track(self, motorA, motorB):
        A = motorA.position(DEGREES) * (math.pi / 180) / self.gear_ratio
        B = motorB.position(DEGREES) * (math.pi / 180) / self.gear_ratio
        d_A = (A - self.past_A) * self.wheel_radius
        d_B = (B - self.past_B) * self.wheel_radius

        self.past_A = A
        self.past_B = B

        d_r = (d_A + d_B) / 2
        d_theta = (d_A - d_B) / self.wheel_base

        self.x = self.x + d_r * math.cos(self.rot + d_theta/2)
        self.y = self.y + d_r * math.sin(self.rot + d_theta/2)
        self.rot = self.rot - d_theta
        if self.rot > 2 * math.pi:
            self.rot -= 2 * math.pi
        if self.rot < 0:
            self.rot += 2 * math.pi
        

    def getX(self):
        return self.x
    def getY(self):
        return self.y
    def getRot(self):
        return self.rot



class c2:
    def __init__(self, d, claw1, claw2, elevator, bumper):
        self.d = d
        self.claw1 = claw1
        self.claw2 = claw2
        self.elevator = elevator
        self.elevator.set_position(0, TURNS)
        self.max_elevator_height = 11.4
        self.t = brain.timer
        self.goal_pos = [0, 0, False]
        self.bumper = bumper
        self.rise_complete = False
        self.target_reached = False

        self.max_y = 200
        self.min_width = 100

        self.dist_ratio = 1800

        self.center_height = 160

        self.fruit_height = 7.5

        self.deposit_time = 1
        self.focal_x = 207.272727273

    def eject(self):
        self.claw1.spin(REVERSE, 50, PERCENT)
        self.claw2.spin(REVERSE, 50, PERCENT)
    def suck(self):
        self.claw1.spin(FORWARD, 50, PERCENT)
        self.claw2.spin(FORWARD, 50, PERCENT)
    def stopit(self):
        self.claw1.spin(FORWARD, 0, PERCENT)
        self.claw2.spin(FORWARD, 0, PERCENT)

    def approach(self, pos):
        if self.target_reached:
            return True
        elif self.d.trajectory_drive(pos, self.goal_pos):
            self.target_reached = True
            return True
        return False
    
    def  move_elevator(self, direction):
        if direction == 'U' and self.elevator.position(TURNS) < self.max_elevator_height:
            self.elevator.spin(REVERSE, 40, PERCENT)
        elif direction == 'D' and not bumper_f.pressing():
            self.elevator.spin(FORWARD, 30, PERCENT)
        else:
            self.elevator.spin(REVERSE, 0, PERCENT)



    def rise(self, obj):
        if obj == None:
            return True
        brain.screen.set_cursor(2,2)
        brain.screen.print(obj.centerY)
        if (obj.centerY < self.center_height) and not self.rise_complete:
            self.move_elevator('U')
            return False
        else:
            if not self.rise_complete:
                self.rise_complete = True
                self.move_elevator('S')
            return True
    
    def lower(self):
        if self.bumper.pressing():
            self.move_elevator('S')
            return True
        self.move_elevator('D')
        return False

    def best_obj(self, objs):
        obj = objs[0]
        max_height = 0
        if not obj.exists:
            brain.screen.clear_screen()
            brain.screen.set_cursor(1,1)
            brain.screen.print("CANT FIND OBJ")
            return None
        for i in objs:
            if i.height > max_height and i.centerY < self.max_y and i.width > self.min_width:
                max_height = i.height
                obj = i
        return obj

    
    def determine_pos(self, pos, obj):
        robot_x, robot_y, theta = pos

        # distance from size
        dist = (self.dist_ratio / obj.height) / 100

        # horizontal angle offset from center
        # angle_offset = ((obj.centerX - 160) / 160) * 0.532#(0.61)   # 70 deg FOV
        x_dist = ((obj.centerX - 160) * dist) / self.focal_x
        y_dist = math.sqrt(dist**2 - x_dist**2)
        angle_offset = math.asin(x_dist / dist)
        brain.screen.set_cursor(5,2)
        brain.screen.print("X dist: ", x_dist)
        
        goal_x = robot_x + x_dist
        goal_y = robot_y + y_dist
        brain.screen.set_cursor(9,2)
        brain.screen.print("Goal x: ", goal_x)
        brain.screen.set_cursor(10,2)
        brain.screen.print("Goal y: ", goal_y)
        return [goal_x, goal_y]


    def setup_c(self, pos, objs):
        if not objs[0].exists:
            # self.d.basic_drive(30, -30)
            brain.screen.set_cursor(1,2)
            brain.screen.print("OBJ NOT FOUND")
            return False
        brain.screen.print("OBJ FOUND")
        self.d.basic_drive(0,0)
        obj = self.best_obj(objs)
        self.goal_pos = self.determine_pos(pos, obj)
        self.goal_pos.append(False)
        self.rise_complete = False
        self.target_reached = False
        return True
    def setup_d(self):
        self.d.basic_drive(0,0)
        self.rise_complete = False
        self.target_reached = False
        self.t.clear()
        return True

    def collect(self, pos, snapshot):
        brain.screen.set_cursor(3,2)
        obj = self.best_obj(snapshot)
        if self.rise(obj):
            brain.screen.print("Rise")
            self.suck()
            if self.approach(pos):
                brain.screen.print("Approach")
                self.stopit()
                if self.lower():
                    brain.screen.print("Lower")
                    return True
        return False

    
    def deposit(self):
        if self.t.time(SECONDS) < self.deposit_time:
            self.eject()
            return False
        self.stopit()
        return True
  
class field:
    
    def __init__(self):
        
        self.ms = [
            [0.32, 0.30],#bottom left

            [0.32, 1.22],#left bottom,
            [0.32, 2.05], #left middle
            [0.32, 2.87],#left top

            [0.32, 3.43],#top left

            [0.81, 3.495],#top left
            [1.70, 3.495],#top right

            [2.2, 3.43],#top right

            [2.2, 2.85],#right top
            [2.2, 2.06], #right middle
            [2.2, 1.22],#right bottom

            [2.2, 0.325], #bottom right

            [0.99, 0.3],
            [1.37, 0.3],
            [1.73, 0.3]
        ]

        self.trees = [
            # adjustment
            ['P', [0.80, 1.22], [1.66, 1.22]],
            ['O', [0.80, 2.05], [1.66, 2.06]],
            ['G', [0.80, 2.87], [1.66, 2.85]]
        ]

        #first is the default bucket position, then the next three with same key assignment as trees, but no E
        self.buckets = [
            ['G', [99, 12]],#Leftmost bucket
            ['P', [137, 12]], 
            ['O', [173, 12]] #rightmost bucket
            ]

    #finds trajectory along line to object
    def _find(self, x, y, goal):
        obj, type = goal#G -> green, P -> Purple, O -> orange

        #finding ideal goal
        if obj == 'T':#tree
            ideal = ['N']
            for i in self.trees:
                if i[0] == type:#add condition to select perfered tree of the two
                    ideal = i
            if ideal[0] == 'N':
                for i in self.trees:
                    if i[0] == 'N':
                        ideal = i
                        break
            d1 = math.sqrt(math.pow(ideal[1][0] - x, 2) + math.pow(ideal[1][1] - y, 2))
            d2 = math.sqrt(math.pow(ideal[2][0] - x, 2) + math.pow(ideal[2][1] - y, 2))
            #add check for if empty
            if d2 > d1:
                ideal = ideal[1]
            else:
                ideal = ideal[2]
       
        elif obj == 'B':
            ideal = ['N']
            for i in self.buckets:
                if i[0] == type:
                    ideal = i
            if ideal[0] == 'N':
                for i in self.buckets:
                    if i[0] == 'N':
                        ideal = i
                        break
            ideal = ideal [1]
        
        

        #TODO use self.m and account for edges so we dont constantly stop
        index_of_ideal = 0
        a = 0
        min_dist = 100
        closest_to_ideal = []
        for i in self.ms:
            dist = math.sqrt(math.pow(i[0] - ideal[0], 2) + math.pow(i[1] - ideal[1], 2))
            if dist < min_dist:
                min_dist = dist
                closest_to_ideal = i
                index_of_ideal = a
            a += 1
        
        index_of_robot = 0
        a = 0
        min_dist = 100
        closest_to_robot = []
        for i in self.ms:
            dist = math.sqrt(math.pow(i[0] - x, 2) + math.pow(i[1] - y, 2))
            if dist < min_dist:
                min_dist = dist
                closest_to_robot = i
                index_of_robot = a
            a += 1
        

        path_a = []
        path_b = []
        size = len(self.ms)

        path_a.append(self.ms[index_of_robot].copy())
        path_a[0].append(False)
        path_b.append(self.ms[index_of_robot].copy())
        path_b[0].append(False)

        if index_of_robot > index_of_ideal:
            for i in range(index_of_robot - 1, index_of_ideal - 1, -1):
                a = self.ms[i].copy()
                a.append(True)
                path_a.append(a)
            for i in range(index_of_robot + 1, size):
                a = self.ms[i].copy()
                a.append(True)
                path_b.append(a)
            for i in range(0, index_of_ideal + 1):
                path_b.append(self.ms[i])
        else:
            for i in range(index_of_robot + 1, index_of_ideal + 1):
                a = self.ms[i].copy()
                a.append(True)
                path_a.append(a)
            for i in range(index_of_robot - 1, -1, -1):
                a = self.ms[i].copy()
                a.append(True)
                path_b.append(a)
            for i in range(size - 1, index_of_ideal - 1, -1):
                a = self.ms[i].copy()
                a.append(True)
                path_b.append(a)
        
        path = []

        if len(path_a) < len(path_b):
            path = path_a
        else:
            path = path_b

        return path, ideal
        


    def _find_line(self, x, y):#get onto line
        min_dist = 100
        path = []
        for i in self.points:
            dist = math.sqrt(math.pow((i[0] - x), 2) + math.pow((i[1] - y), 2))
            if dist < min_dist:
                min_dist = dist
                path = i
        for i in self.edges:
            dist = math.sqrt(math.pow((i[0] - x), 2) + math.pow((i[1] - y), 2))
            if dist < min_dist:
                min_dist = dist
                path = i
        return path
    
    def getTrajectory(self, x, y, goal):
        task, type = goal
        return self._find(x, y, goal)
    
class stateMachine:
    
    def __init__(self, controller, motorA, motorB, line_tracker_left, line_tracker_right, elevator, claw1, claw2, bumper, cam):
        self.pos = odometry()
        self.d = drive(motorA, motorB, line_tracker_left, line_tracker_right, self.pos)
        self.f = field()
        self.c = claw(self.d)
        self.c2 = c2(self.d, claw1, claw2, elevator, bumper)
        self.cam = cam

        self.motorA = motorA
        self.motorB = motorB


        self.controller = controller

        self.time = brain.timer
        self.time.clear()

        # self.cargo = ['N', 'T', 'G'] # [task, arguments], to start navigate to a tree of unassinged color
        # self.cargo = ['T']
        # self.cargo = ['A']
        self.cargo = ['B', 'G', True]

        #TODO add queue for colors to cycle through them

    def loop(self):
        while True:
            self.pos.track(self.motorA, self.motorB)
            brain.screen.set_cursor(1,2)
            brain.screen.print(self.cargo[0])
            state = self.cargo[0]
            if state == 'T':# teleop
                self.cargo = self.tele_op(self.cargo)
            elif state == 'N':# navigation
                self.cargo = self.navigation(self.cargo)
            elif state == 'C':# collecting
                self.cargo = self.collecting(self.cargo)
            elif state == 'D':# depositing
                self.cargo = self.depositing(self.cargo)
            elif state == 'A':
                self.cargo = self.a(self.cargo)
            elif state == 'B':
                self.cargo = self.b(self.cargo)
            elif state == 'E':# end
                brain.program_stop()
                break
    
    def tele_op(self, cargo):
        r = self.controller.axis2.position()
        l = self.controller.axis3.position()

        brain.screen.clear_screen()
        brain.screen.set_cursor(2,2)
        brain.screen.print("X", self.pos.getX())
        brain.screen.print("Y", self.pos.getY())
        brain.screen.print("R", self.pos.getRot() * 180/math.pi)

        self.controller.screen.clear_screen()
        self.controller.screen.set_cursor(2,2)
        self.controller.screen.print("X", self.pos.getX())
        self.controller.screen.print("Y", self.pos.getY())
        self.controller.screen.print("R", (self.pos.getRot() * 180/math.pi))

        self.d.basic_drive(l, r)
        return cargo
    
    def navigation(self, cargo):#cargo = [state, object to navigate to, color]
        brain.screen.set_cursor(2,2)
        brain.screen.print("Position ", [self.pos.getX(), self.pos.getY(), self.pos.getRot()])
        if len(cargo) < 4:
            junk, obj, type = cargo
            trajectory, end_goal = self.f.getTrajectory(self.pos.getX(), self.pos.getY(), [obj, type])
            cargo.append(0)
            cargo.append(trajectory)
            cargo.append(end_goal)
        if cargo[3] > (len(cargo[4]) - 1):
            if self.d.turn_to_object([self.pos.getX(), self.pos.getY(), self.pos.getRot()], cargo[5]):
                brain.screen.clear_screen()
                brain.screen.set_cursor(2,2)
                brain.screen.print("FINISHED")
                self.d.basic_drive(0, 0)
                if cargo[1] == 'T': state = 'C'
                else: state = 'D'
                cargo = [state, cargo[1], True]
                return cargo
            else:
                return cargo
        elif self.d.trajectory_drive([self.pos.getX(), self.pos.getY(), self.pos.getRot()], cargo[4][cargo[3]]):
            cargo[3] += 1
        return cargo
    
    def collecting(self, cargo):
        state, type, initial = cargo
        if type == 'G' or True:
            snapshot = self.cam.take_snapshot(ai_vision_7__G)
        elif type == 'O':
            snapshot = self.cam.take_snapshot(ai_vision_7__O)
        # elif type == 'P':
        else:
            snapshot = self.cam.take_snapshot(ai_vision_7__P)
        if initial:
            if self.c2.setup_c([self.pos.getX(), self.pos.getY(), self.pos.getRot()], snapshot):
                cargo = [state, type, False]
            else:
                return cargo
        if self.c2.collect([self.pos.getX(), self.pos.getY(), self.pos.getRot()], snapshot):
            cargo = ['N', 'B', self.cycle(type)]
        return cargo
        
    
    def depositing(self, cargo):
        state, type, initial = cargo
        if initial:
            self.c2.setup_d()
            cargo = [state, type, False]
        if self.c2.deposit():
            type = self.cycle(type)
            cargo = ['N', 'T', type]
        return cargo

    def cycle(self, type):
        if type == 'P':
            return 'G'
        elif type == 'O':
            return 'P'
        else:
            return 'O'

    def a(self, cargo):
        
        brain.screen.clear_screen()
        a = 2
        t, j = self.f.getTrajectory(self.pos.getX(), self.pos.getY(), ['T', 'O'])
        brain.screen.set_cursor(1,2)
        brain.screen.print("O: ", j)
        for i in t:
            brain.screen.set_cursor(a, 2)
            brain.screen.print("# ", a, " ", i)
            a += 1
        b = len(t) - 1
        x = t[b][0]
        y = t[b][1]
        t, j = self.f.getTrajectory(x, y, ['B', 'G'])
        brain.screen.set_cursor(a, 2)
        brain.screen.print("G: ", j)
        a += 1
        for i in t:
            brain.screen.set_cursor(a, 2)
            brain.screen.print("# ", a, " ", i)
            a += 1
        b = len(t) - 1
        x = t[b][0]
        y = t[b][1]
        
        wait(1000, SECONDS)
        brain.program_stop()
       
        return cargo
    
    def b(self, cargo):
        state, type, initial = cargo
        if type == 'G' or True:
            snapshot = self.cam.take_snapshot(ai_vision_7__G)
        elif type == 'O':
            snapshot = self.cam.take_snapshot(ai_vision_7__O)
        elif type == 'P':
            snapshot = self.cam.take_snapshot(ai_vision_7__P)
        if initial:
            if self.c2.setup_c([self.pos.getX(), self.pos.getY(), self.pos.getRot()], snapshot):
                cargo = [state, type, False]
            else:
                return cargo
        if self.c2.collect([self.pos.getX(), self.pos.getY(), self.pos.getRot()], snapshot):
            cargo = ['E']
        return cargo
        

robot = stateMachine(controller_1, motor_1, motor_2, line_tracker_h, line_tracker_g, motor_10, motor_8, motor_9, bumper_f, ai_vision_7)

robot.loop()