package us.ihmc.communication.ros2log;

import controller_msgs.msg.dds.RobotConfigurationData;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.HumanoidControllerAPI;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;

import java.time.Clock;
import java.util.concurrent.atomic.AtomicLong;
import java.util.function.LongSupplier;

public enum ROS2LogTimeSource
{
   /* Record/Replay is timestamped by system time */
   SYSTEM,
   /* Record/Replay is timestamped by simulation time via RobotConfigurationData#getSyncTimestamp */
   SIMULATION;

   public LongSupplier createTimestampProvider(String robotName, ROS2Node ros2Node)
   {
      if (this == ROS2LogTimeSource.SYSTEM)
      {
         return Clock.systemUTC()::millis;
      }
      else
      {
         AtomicLong timestamp = new AtomicLong(-1);

         ROS2Topic<?> controllerOutputTopic = HumanoidControllerAPI.getOutputTopic(robotName);
         ROS2Topic<RobotConfigurationData> robotConfigurationData = controllerOutputTopic.withTypeName(RobotConfigurationData.class);
         ros2Node.createSubscription(robotConfigurationData, s -> timestamp.set(Conversions.nanosecondsToMilliseconds(s.takeNextData().getSyncTimestamp())));

         return timestamp::get;
      }
   }

}
