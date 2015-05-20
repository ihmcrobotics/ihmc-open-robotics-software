package us.ihmc.darpaRoboticsChallenge.ros;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.HighLevelStateChangePacket;
import us.ihmc.communication.packets.dataobjects.HighLevelState;
import us.ihmc.util.PeriodicNonRealtimeThreadScheduler;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.msgToPacket.converter.ROSMessageConverter;
import us.ihmc.utilities.ros.publisher.RosUInt8Publisher;

import java.util.concurrent.TimeUnit;

/**
 *
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class RosHighLevelStatePublisher implements PacketConsumer<HighLevelStateChangePacket>, Runnable
{
   private final RosUInt8Publisher stateBytePublisher = new RosUInt8Publisher(false);
   private final RosMainNode rosMainNode;

   private HighLevelState currentState = HighLevelState.DO_NOTHING_BEHAVIOR;
   private final PeriodicNonRealtimeThreadScheduler scheduler = new PeriodicNonRealtimeThreadScheduler(getClass().getName());

   public RosHighLevelStatePublisher(RosMainNode rosMainNode, String rosNameSpace)
   {
      this.rosMainNode = rosMainNode;
      initialize(rosNameSpace);
   }

   private void initialize(String rosNameSpace)
   {
      rosMainNode.attachPublisher(rosNameSpace + "/output/high_level_state", stateBytePublisher);

      scheduler.schedule(this, 1, TimeUnit.MILLISECONDS);
   }

   @Override
   public void receivedPacket(HighLevelStateChangePacket packet)
   {
      if (packet.getEndState() != currentState)
      {
         currentState = packet.getEndState();
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
         stateBytePublisher.publish(ROSMessageConverter.convertEnumToByte(currentState));
      }
   }
}
