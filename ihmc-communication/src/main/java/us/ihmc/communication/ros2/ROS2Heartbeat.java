package us.ihmc.communication.ros2;

import std_msgs.msg.dds.Empty;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;
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
   public static final double HEARTBEAT_PERIOD = UnitConversions.hertzToSeconds(2.5);
   private final ROS2PublishSubscribeAPI ros2;
   private final ROS2Topic<Empty> heartbeatTopic;
   private volatile boolean alive = false;
   private final Throttler throttler = new Throttler();

   public ROS2Heartbeat(ROS2PublishSubscribeAPI ros2, ROS2Topic<Empty> heartbeatTopic)
   {
      this.ros2 = ros2;
      this.heartbeatTopic = heartbeatTopic;
   }

   public void setAlive(boolean alive)
   {
      if (alive)
      {
         if (!this.alive)
            ThreadTools.startAsDaemon(() -> ExceptionTools.handle(this::publishThread, DefaultExceptionHandler.MESSAGE_AND_STACKTRACE),
                                      "Heartbeat-" + heartbeatTopic.getName());
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
         ros2.publish(heartbeatTopic);
      }
   }
}
