package us.ihmc.communication.ros2;

import std_msgs.Empty;
import us.ihmc.commons.UnitConversions;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2Topic;

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
   private static final Empty EMPTY_MESSAGE = new Empty();

   private final ROS2Publisher<Empty> heartbeatPublisher;
   private final RepeatingTaskThread publishThread;

   public ROS2Heartbeat(ROS2Node ros2Node, ROS2Topic<Empty> heartbeatTopic)
   {
      heartbeatPublisher = ros2Node.createPublisher(heartbeatTopic);
      publishThread = new RepeatingTaskThread("Heartbeat-" + heartbeatTopic.getName(), () -> heartbeatPublisher.publish(EMPTY_MESSAGE));
      publishThread.setFrequencyLimit(STATUS_FREQUENCY);
   }

   /**
    * Set the "alive" status, which is like being "enabled" or "active".
    */
   public void setAlive(boolean alive)
   {
      if (alive)
         publishThread.startRepeating();
      else
         publishThread.stopRepeating();
   }

   public void destroy()
   {
      publishThread.blockingKill(); // Important to use blockingKill() - we don't want the publisher to get removed before this finishes
      // TODO: Need ros2Node reference to call destroyPublisher
      // ros2Node.destroyPublisher(heartbeatPublisher);
   }
}
