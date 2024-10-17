package us.ihmc.humanoidBehaviors.behaviors.primitives;

import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.yoVariables.variable.YoBoolean;

/**
 * @deprecated The lidar stuff is broken. What about implement a centralized LidarBufferBehavior that records lidar in an internal buffer.
 */
@Deprecated
public class ClearLidarBehavior extends AbstractBehavior
{
   private final YoBoolean packetHasBeenSent = new YoBoolean("packetHasBeenSent" + behaviorName, registry);
//   private DepthDataClearCommand clearLidarPacket;

   public ClearLidarBehavior(String robotName, ROS2Node ros2Node)
   {
      super(robotName, ros2Node);

   }

   @Override
   public void doControl()
   {
//      clearLidarPacket = new DepthDataClearCommand(DepthDataTree.DECAY_POINT_CLOUD);
//
//      //      clearLidarPacket.setDestination(PacketDestination.NETWORK_PROCESSOR);
//
//      if (!packetHasBeenSent.getBooleanValue() && (clearLidarPacket != null))
//      {
//         sendPacketToNetworkProcessor();
//      }
   }

   private void sendPacketToNetworkProcessor()
   {
      if (!isPaused.getBooleanValue() && !isAborted.getBooleanValue())
      {
//         sendPacket(clearLidarPacket);
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
//      if (clearLidarPacket != null)
//         return true;
//      else
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
