package us.ihmc.communication.ros2;

import std_msgs.msg.dds.Empty;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Throttler;

/**
 * Use this class to indicate to other things on the network that something is active.
 */
public class ROS2Heartbeat
{
   /**
    * About twice a second seems like a reasonable value. Let's only introduce it as a parameter
    * if it's really meaningful. Things will respond to other things being on or off within
    * half a second and there could 1000s of these without overloading the network.
    */
   public static final double STATUS_FREQUENCY = 2.5;
   public static final double HEARTBEAT_PERIOD = UnitConversions.hertzToSeconds(STATUS_FREQUENCY);
   private ROS2PublishSubscribeAPI ros2;
   private IHMCROS2Publisher<Empty> heartbeatPublisher;
   private final Empty emptyMessage = new Empty();
   private final ROS2Topic<Empty> heartbeatTopic;
   private volatile boolean alive = false;
   private final Throttler throttler = new Throttler();

   public ROS2Heartbeat(ROS2PublishSubscribeAPI ros2, ROS2Topic<Empty> heartbeatTopic)
   {
      this.ros2 = ros2;
      this.heartbeatTopic = heartbeatTopic;

   }
   public ROS2Heartbeat(ROS2NodeInterface ros2Node, ROS2Topic<Empty> heartbeatTopic)
   {
      this.heartbeatTopic = heartbeatTopic;
      heartbeatPublisher = ROS2Tools.createPublisher(ros2Node, heartbeatTopic);
   }

   public void setAlive(boolean alive)
   {
      if (alive)
      {
         if (!this.alive)
            ThreadTools.startAsDaemon(() -> ExceptionTools.handle(this::publishThread, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE),
                                      "Heartbeat-" + heartbeatTopic.getName());
         this.alive = true;
      }
      else
      {
         this.alive = false;
      }
   }

   private void publishThread()
   {
      while (alive)
      {
         throttler.waitAndRun(HEARTBEAT_PERIOD);
         if (ros2 != null)
            ros2.publish(heartbeatTopic);
         else
            heartbeatPublisher.publish(emptyMessage);
      }
   }

   public void destroy()
   {
      if (heartbeatPublisher != null)
         heartbeatPublisher.destroy();
   }
}
