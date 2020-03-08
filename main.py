import maestro
import ikpy
import numpy as np
import time
import serial
import sys
from math import pi
#import detectFruit
from ikpy import plot_utils
from ikpy.chain import Chain
from ikpy.link import OriginLink, URDFLink
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D



class FruitHarvester():
    def __init__(self):
        self.servo = maestro.Controller()
        self.rate = 1/1000
        # self.fruit_harvest_chain = Chain.from_urdf_file("my_robot.urdf")
        self.fruit_harvest_chain = Chain(name='fruit_harvester', links=[
            URDFLink(
                name = "55kg",
                translation_vector=[-0.1843, 0.1202, -0.6482],
                orientation=[0,0,0],
                rotation=[0,0,1],
            ),
            URDFLink(
              name="40kg",
              translation_vector=[0, -0.01807, 0.0678],   #place holder values TODO
              orientation=[0, 0, 0],
              rotation=[1, 0, 0],
            ),
            URDFLink(
              name="25kg",
              translation_vector=[0, 0.00861, 0.435],    #place holder values TODO
              orientation=[0, 0, 0],
              rotation=[-1, 0, 0],
             ),
            URDFLink(
                name = "gripper",
                translation_vector=[0, 0.350, -0.06268],
                orientation=[0,0,0],
                rotation=[0,0,0],
            )
        ])
        self.start_pos = [-0.1843, 0.1202-0.01807+0.00861+0.350, -0.6482+0.0678+0.435-0.06268]
        #print self.fruit_harvest_chain

        try:
            print("TODO: add corrections")
            self.run()
        except (KeyboardInterrupt):
            print("\nSystem Stopped.")

    def run(self):
        FLAG_TARGET_REACHED = 0
        old_target = np.zeros(4)
        while(True):
            if (FLAG_TARGET_REACHED):
                hand = Hand()
                hand.move(1)
                time.sleep(1)
                hand.move(0)
                print "Fruit Grabbed"
                self.wait_for_update(target_frame)
            print("Running...")
            #  TODO: import fruit detection code to get target positions
            print('Enter target (one element at a time): ')
            for i in range(3):
                if i == 0:
                    x = input("x = ")
                elif i == 1:
                    y = input("y = ")
                elif i == 2:
                    z = input("z = ")
            target_vector = np.array([x, y, z])
            print target_vector
            target_frame = [[1, 0, 0,(self.start_pos[0]+x)],
                            [0, 1, 0,(self.start_pos[1]+y)],
                            [0, 0, 1, (self.start_pos[2]+z)],
                            [0, 0, 0, 1]]
            angles = self.fruit_harvest_chain.inverse_kinematics(target_frame)
            print("The angles of each joints are : ", angles)
            ax = matplotlib.pyplot.figure().add_subplot(111, projection='3d', navigate=1)
            self.fruit_harvest_chain.plot(angles, ax)
            matplotlib.pyplot.show()
            self.servo_target(angles)
            if (np.array_equal(old_target,target_frame)):
                FLAG_TARGET_REACHED = 1
            old_target = target_frame # Saving old target to ensure target is reached.
            time.sleep(2)

    def wait_for_update(self, target):
        target_frame = target
        print("Reached target, waiting for update.")
        while(np.array_equal(target, target_frame)):
            for i in range(1000):
                #target_frame = target_frame[:3,3] = getPosition()  #TODO:
                time.sleep(self.rate)

    def servo_target(self, angles):
        for i in range(len(angles)):
            j = len(angles) - i
            angle = angles[j]
            angle = angle * 180 / pi
            PW = int(1500 + angle * 10)
            PW = PW * 4
            print("Moving joint:", i, ". Pulse width = ", PW)
            if j == 0:
                if abs(angle) > 60:
                    self.out_of_range
                else:
                    self.servo.setTarget(0,PW)
            elif j == 1:
                if angle > 0 or angle < -75:
                    self.out_of_range
                else:
                    PW = 4 * int(2090 + angle * 10)
                    self.servo.setTarget(1,PW)
                    self.servo.setTarget(2,PW)
            elif j == 2:
                if abs(angle) > 100:
                    self.out_of_range
                else:
                    self.servo.setTarget(3,PW)
                    self.servo.setTarget(4,PW)

    def move_to_home(self):
        target_frame = [[1, 0, 0,(self.start_pos[0])],
                        [0, 1, 0,(self.start_pos[1])],
                        [0, 0, 1, (self.start_pos[2])],
                        [0, 0, 0, 1]]
        angles = self.fruit_harvest_chain.inverse_kinematics(target_frame)
        self.servo_target(angles)
        print('Arrived at home position')

    def out_of_range(self):
        print('Out of Range, cannot move to desired point.')



class Hand():
    def __init__(self):
        arduino_connected = False
        self.ser = serial.Serial('/dev/ttyUSB0',9600)
        self.rate = 1/10
        t = 0
        while not arduino_connected:
            t = t+1
            serin = self.ser.read()
            if serin == '1':
                print "Connected"
                arduino_connected = True
                self.ser.write("0")
                time.sleep(self.rate)
            if t % 50 == 0:
                print("Connection timed out. No arduino found.")
                sys.exit()
            elif t % 10 == 0:
                print("Arduino not connected...\nAttempting to connect... \n")
            # time.sleep(self.rate)

    def move(self, OPEN):
        ## OPEN is bool
        ## OPEN = 1, opens hand.
        ## OPEN = 0, closed hand.
        #print self.ser.inWaiting()
        # while not (self.ser.inWaiting() > 0):
        #     self.ser.read()
        #     pass
        #self.ser.read()
        if(self.ser.read() == '1'):
            if(OPEN == 1):
                print "Opening hand"
                self.ser.write("1")
            else:
                print "Closing hand"
                self.ser.write("0")




if __name__ == '__main__':
    try:
        whatever = FruitHarvester()
    except (KeyboardInterrupt, SystemExit):
        hand.ser.close()
        pass
