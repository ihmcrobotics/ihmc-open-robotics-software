package us.ihmc.humanoidBehaviors.behaviors.primitives;

import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidBehaviors.communication.OutgoingCommunicationBridgeInterface;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataStateCommand;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataStateCommand.LidarState;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;

public class EnableLidarBehavior extends AbstractBehavior
{
   private final BooleanYoVariable packetHasBeenSent = new BooleanYoVariable("packetHasBeenSent" + behaviorName, registry);
   private DepthDataStateCommand enableLidarPacket;
   private LidarState lidarState;

   public EnableLidarBehavior(OutgoingCommunicationBridgeInterface outgoingCommunicationBridge)
   {
      super(outgoingCommunicationBridge);

   }

   @Override
   public void doControl()
   {
      if (hasInputBeenSet())
      {
         enableLidarPacket = new DepthDataStateCommand(lidarState);

         if (!packetHasBeenSent.getBooleanValue() && (enableLidarPacket != null))
         {
            sendPacketToNetworkProcessor();
         }
      }
      else
      {
         setLidarState(LidarState.ENABLE);
      }
   }

   public void setLidarState(LidarState lidarState)
   {
      this.lidarState = lidarState;
   }

   private void sendPacketToNetworkProcessor()
   {
      if (!isPaused.getBooleanValue() && !isAborted.getBooleanValue())
      {
         sendPacketToNetworkProcessor(enableLidarPacket);
         packetHasBeenSent.set(true);
      }
   }

   @Override
   public void initialize()
   {
      packetHasBeenSent.set(false);

      isPaused.set(false);
      isAborted.set(false);
   }

   @Override
   public void doPostBehaviorCleanup()
   {
      packetHasBeenSent.set(false);

      isPaused.set(false);
      isAborted.set(false);
   }

   @Override
   public boolean isDone()
   {
      return packetHasBeenSent.getBooleanValue() && !isPaused.getBooleanValue();
   }

   @Override
   public boolean hasInputBeenSet()
   {
      if (lidarState != null)
         return true;
      else
         return false;
   }
}
