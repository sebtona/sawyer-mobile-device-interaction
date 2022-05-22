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
                rospy.sleep(3.0)
            elif i == 1:
                self._limb.move_to_joint_positions(waypoint, timeout=20.0, threshold=self._accuracy)
                if self._limb.endpoint_effort()['force'].z > 5.0:
                    self._rs.disable()
                    break
                rospy.sleep(1.0)
            elif i == 2:
                self._limb.move_to_joint_positions(waypoint, timeout=20.0, threshold=self._accuracy)
                rospy.sleep(1.0)
            elif i == 3:
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
    waypoints.add_waypoint({'right_j6': 2.867515625, 'right_j5': 1.035748046875, 'right_j4': 1.75679296875, 'right_j3': 1.492583984375, 'right_j2': -1.0360302734375, 'right_j1': 0.191751953125, 'right_j0': -0.3628310546875})
    waypoints.add_waypoint({'right_j6': 2.867515625, 'right_j5': 1.260927734375, 'right_j4': 1.9660283203125, 'right_j3': 1.46136328125, 'right_j2': -1.287865234375, 'right_j1': 0.4449140625, 'right_j0': -0.3598466796875})
    waypoints.add_waypoint({'right_j6': 2.867515625, 'right_j5': 1.291306640625, 'right_j4': 1.9631357421875, 'right_j3': 1.35075390625, 'right_j2': -1.350271484375, 'right_j1': 0.4649423828125, 'right_j0': -0.4805908203125})
    waypoints.add_waypoint({'right_j6': 3.0073154296875, 'right_j5': 1.29936328125, 'right_j4': 1.7981728515625, 'right_j3': 1.2729453125, 'right_j2': -1.37785546875, 'right_j1': 0.4200546875, 'right_j0': -0.5109931640625})
    waypoints.go()

if __name__ == '__main__':
    print("Initializing node... ")
    rospy.init_node('tap_block')
    try:
        if not rospy.is_shutdown():
    	    main()
    except rospy.ROSInterruptException:
        pass
