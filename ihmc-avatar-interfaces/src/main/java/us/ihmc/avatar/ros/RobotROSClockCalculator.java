package us.ihmc.avatar.ros;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.utilities.ros.RosMainNode;

public interface RobotROSClockCalculator
{
   /**
    * Overrides this method for calculators that requires to subscribe to a ROS topic.
    * 
    * @param rosMainNode
    */
   default void setROSMainNode(RosMainNode rosMainNode)
   {
   }

   /**
    * This method is called every time a new {@code RobotConfigurationData} has been received from the
    * controller.
    * 
    * @param robotConfigurationData
    */
   default void receivedRobotConfigurationData(RobotConfigurationData robotConfigurationData)
   {
   }

   /**
    * Computes the ROS time for a given {@code robotConfigurationData}.
    * 
    * @param wallTime UTC robot time.
    * @param monotonicTime the monotonic time as published by the robot.
    * @return the time to use with ROS.
    */
   long computeROSTime(long wallTime, long monotonicTime);

   /**
    * Computes the wall-time from the robot perspective for a given ROS time.
    * <p>
    * The default implementation returns {@code rosTime} as wall-time and ROS time should be equivalent.
    * </p>
    * 
    * @param rosTime the ROS time.
    * @return the UTC robot time.
    */
   default long computeRobotWallTime(long rosTime)
   {
      return rosTime;
   }

   /**
    * Computes the monotonic time from the robot perspective for a given ROS time.
    * 
    * @param rosTime the ROS time.
    * @return the robot monotonic time.
    */
   long computeRobotMonotonicTime(long rosTime);
}
