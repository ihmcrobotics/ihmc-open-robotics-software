package us.ihmc.humanoidBehaviors.behaviors.primitives;

import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.CommunicationBridgeInterface;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataClearCommand;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataClearCommand.DepthDataTree;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;

public class ClearLidarBehavior extends AbstractBehavior
{
   private final BooleanYoVariable packetHasBeenSent = new BooleanYoVariable("packetHasBeenSent" + behaviorName, registry);
   private DepthDataClearCommand clearLidarPacket;

   public ClearLidarBehavior(CommunicationBridgeInterface outgoingCommunicationBridge)
   {
      super(outgoingCommunicationBridge);

   }

   @Override
   public void doControl()
   {
      clearLidarPacket = new DepthDataClearCommand(DepthDataTree.DECAY_POINT_CLOUD);

      //      clearLidarPacket.setDestination(PacketDestination.NETWORK_PROCESSOR);

      if (!packetHasBeenSent.getBooleanValue() && (clearLidarPacket != null))
      {
         sendPacketToNetworkProcessor();
      }
   }

   private void sendPacketToNetworkProcessor()
   {
      if (!isPaused.getBooleanValue() && !isAborted.getBooleanValue())
      {
         sendPacket(clearLidarPacket);
         packetHasBeenSent.set(true);
      }
   }

   @Override
   public void onBehaviorEntered()
   {
      packetHasBeenSent.set(false);
   }

   @Override
   public void onBehaviorExited()
   {
      packetHasBeenSent.set(false);
   }

   @Override
   public boolean isDone()
   {
      return packetHasBeenSent.getBooleanValue() && !isPaused.getBooleanValue();
   }


   public boolean hasInputBeenSet()
   {
      if (clearLidarPacket != null)
         return true;
      else
         return false;
   }

   @Override
   public void onBehaviorAborted()
   {
   }

   @Override
   public void onBehaviorPaused()
   {
   }

   @Override
   public void onBehaviorResumed()
   {
   }
}
