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
 * To elaborate, this class sends a periodic broadcast that is just an Empty message
 * with no data. This class intended to be very mutlipurpose. Use this class with 
 * {@link ROS2HeartbeatMonitor} which subscribes to this heartbeat and returns a boolean
 * value for whether this heartbeat is active or not.
 * Furthermore, the use of this class can allow for dynamic events where dropped messages
 * do not affect state so severely. For example, instead one might send a status message
 * when something is "enabled" and a message when something is "disabled". There are many
 * legitimate reasons why those messages might not be received, as in restarting or
 * starting an application later which would come online not knowing the current
 * status. To solve this ever present issue, a periodic status is required. If publishing
 * a periodic status, one way to do that is to do what this class does and just have 
 * the act of publishing in itself be a signifier of being enabled.
 * One might think that you could instead publish a periodic status that contains a
 * boolean for alive, and always publishing it, but when the process containing this
 * heartbeat is stopped, that breaks down, and the monitor would have to handle the case
 * of it not existing anyway.
 * This class is inspired by the common term in computing:
 * https://en.wikipedia.org/wiki/Heartbeat_(computing)
 * 
 * @author Duncan Calvert
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

   /**
    * Set the "alive" status, which is like being "enabled" or "active".
    */
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
         if (ros2 != null)
            ros2.publish(heartbeatTopic);
         else
            heartbeatPublisher.publish(emptyMessage);
         throttler.waitAndRun(HEARTBEAT_PERIOD);
      }
   }

   public void destroy()
   {
      setAlive(false);
      if (heartbeatPublisher != null)
         heartbeatPublisher.destroy();
   }
}
