package us.ihmc.avatar.ros;

import java.util.concurrent.TimeUnit;

import controller_msgs.msg.dds.HighLevelStateChangeStatusMessage;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;
import us.ihmc.utilities.ros.RosMainNode;

/**
 *
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class PeriodicRosHighLevelStatePublisher implements PacketConsumer<HighLevelStateChangeStatusMessage>, Runnable
{
   private final RosHighLevelStatePublisher statePublisher = new RosHighLevelStatePublisher(false);
   private final RosMainNode rosMainNode;

   private HighLevelControllerName currentState = HighLevelControllerName.DO_NOTHING_BEHAVIOR;
   private final PeriodicNonRealtimeThreadScheduler scheduler = new PeriodicNonRealtimeThreadScheduler(getClass().getName());

   public PeriodicRosHighLevelStatePublisher(RosMainNode rosMainNode, String rosNameSpace)
   {
      this.rosMainNode = rosMainNode;
      initialize(rosNameSpace);
   }

   private void initialize(String rosNameSpace)
   {
      rosMainNode.attachPublisher(rosNameSpace + "/output/high_level_state", statePublisher);

      scheduler.schedule(this, 1, TimeUnit.MILLISECONDS);
   }

   @Override
   public void receivedPacket(HighLevelStateChangeStatusMessage packet)
   {
      if (packet.getEndHighLevelControllerName() != currentState.toByte())
      {
         currentState = HighLevelControllerName.fromByte(packet.getEndHighLevelControllerName());
      }
   }

   @Override
   public void run()
   {
      if (Thread.interrupted())
      {
         scheduler.shutdown();
      }
      else if (rosMainNode.isStarted())
      {
         statePublisher.publish(currentState);
      }
   }
}
