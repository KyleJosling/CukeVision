#!/usr/bin/env python
import rospy
import roslaunch

def controlFSM():

    rospy.init_node('controlFSM')

    uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
    roslaunch.configure_logging(uuid)

    # Launch rosserial node
    rosserialNode = roslaunch.core.Node('rosserial_python','serial_node.py', args='/dev/ttyACM0')
    rosserialLaunch = roslaunch.scriptapi.ROSLaunch()
    rosserialLaunch.start()
    process = rosserialLaunch.launch(rosserialNode)

    # Launch stepper control node
    stepperControlNode = roslaunch.core.Node('cuke_vision','stepperControlNode')
    stepperLaunch = roslaunch.scriptapi.ROSLaunch()
    stepperLaunch.start()
    process = stepperLaunch.launch(stepperControlNode)

    # Launch realsense nodes
    rsLaunch = roslaunch.parent.ROSLaunchParent(uuid, ['/home/kylejosling/cuke_ws/src/realsense-ros/realsense2_camera/launch/rs_camera.launch'])
    rsLaunch.start()
     
    rsLaunch.spin()
    



if __name__ == '__main__':
    try:
        controlFSM()
    except rospy.ROSInterruptException:
        pass
