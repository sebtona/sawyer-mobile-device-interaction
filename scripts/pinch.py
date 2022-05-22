#!/usr/bin/env python

import rospy
import intera_interface

class Waypoints(object):
    def __init__(self, speed, accuracy):
        # Create intera_interface limb instance
        self._limb = intera_interface.Limb('right')

        # Parameters which will describe joint position moves
        self._speed = speed
        self._accuracy = accuracy

        # Waypoints
        self._waypoints = list()

        # Verify robot is enabled
        print("Getting robot state... ")
        self._rs = intera_interface.RobotEnable()
        self._init_state = self._rs.state().enabled
        print("Enabling robot... ")
        self._rs.enable()

    def add_waypoint(self, waypoint):
        """
        Stores joint position waypoints
        """

        self._waypoints.append(waypoint)

    def go(self):
        rospy.sleep(1.0)

        rospy.loginfo("Waypoint Trajectory Started")
        print("  Press Ctrl-C to stop...")

        # Set joint position speed ratio for execution
        self._limb.set_joint_position_speed(self._speed)

        for waypoint in self._waypoints:
            if rospy.is_shutdown():
                break
            i = self._waypoints.index(waypoint)
            if i == 0:
                self._limb.move_to_joint_positions(waypoint, timeout=20.0, threshold=self._accuracy)
                gripper = intera_interface.Gripper("right_gripper")
                gripper.open() # use gripper.close() if pinching in opposite direction
                rospy.sleep(3.0)
            elif i == 1:
                self._limb.move_to_joint_positions(waypoint, timeout=20.0, threshold=self._accuracy)
                if self._limb.endpoint_effort()['force'].z > 5.0:
                    self._rs.disable()
                    break
                rospy.sleep(1.0)
                gripper.close()
                rospy.sleep(1.0)
                #if self._limb.endpoint_effort()['force'].z < 3.0:
                #    break
            elif i == 2:
                self._limb.move_to_joint_positions(waypoint, timeout=20.0, threshold=self._accuracy)
            else: # move to neutral to be safe
                self._limb.move_to_joint_positions({'right_j6': 3.311806640625, 'right_j5': 0.566732421875, 'right_j4': 0.0016279296875, 'right_j3': 2.177560546875, 'right_j2': -0.0017841796875, 'right_j1': -1.1785966796875, 'right_j0': -0.002052734375}, timeout=20.0, threshold=self._accuracy)
                break
            print(self._limb.joint_angles())

        # Set joint position speed back to default
        self._limb.set_joint_position_speed(0.3)

    def clean_shutdown(self):
        print("\nExiting...")
        return True


def main():

    print(intera_interface.settings.JOINT_ANGLE_TOLERANCE)

    waypoints = Waypoints(0.1, intera_interface.settings.JOINT_ANGLE_TOLERANCE)

    # Register clean shutdown
    rospy.on_shutdown(waypoints.clean_shutdown)

    # Begin program
    # neutral: waypoints.add_waypoint({'right_j6': 3.311806640625, 'right_j5': 0.566732421875, 'right_j4': 0.0016279296875, 'right_j3': 2.177560546875, 'right_j2': -0.0017841796875, 'right_j1': -1.1785966796875, 'right_j0': -0.002052734375})
    waypoints.add_waypoint({'right_j6': 1.9472236328125, 'right_j5': 0.5868603515625, 'right_j4': 0.5702646484375, 'right_j3': 1.544044921875, 'right_j2': -0.2835673828125, 'right_j1': -0.4257314453125, 'right_j0': -0.7799189453125})
    waypoints.add_waypoint({'right_j6': 1.9398125, 'right_j5': 0.5491083984375, 'right_j4': 0.6011611328125, 'right_j3': 1.5392802734375, 'right_j2': -0.2889189453125, 'right_j1': -0.38977734375, 'right_j0': -0.7677568359375})
    waypoints.add_waypoint({'right_j6': 1.9472236328125, 'right_j5': 0.5868603515625, 'right_j4': 0.5702646484375, 'right_j3': 1.544044921875, 'right_j2': -0.2835673828125, 'right_j1': -0.4257314453125, 'right_j0': -0.7799189453125})
    waypoints.go()

if __name__ == '__main__':
    print("Initializing node... ")
    rospy.init_node('tap_block')
    try:
        if not rospy.is_shutdown():
    	    main()
    except rospy.ROSInterruptException:
        pass
