package us.ihmc.avatar.handControl;

import com.google.common.base.CaseFormat;

import us.ihmc.communication.ROS2Tools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.RealtimeRos2Node;

public abstract class HandControlThread implements Runnable
{
   protected final RealtimeRos2Node realtimeRos2Node;

   public HandControlThread(RobotSide robotSide)
   {
      String nodeName = robotSide.getLowerCaseName() + "_" + CaseFormat.UPPER_CAMEL.to(CaseFormat.LOWER_UNDERSCORE, getClass().getSimpleName());
      realtimeRos2Node = ROS2Tools.createRealtimeRos2Node(PubSubImplementation.FAST_RTPS, nodeName);
   }
}
