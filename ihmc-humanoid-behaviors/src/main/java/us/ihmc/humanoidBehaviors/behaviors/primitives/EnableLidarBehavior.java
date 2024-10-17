package us.ihmc.humanoidBehaviors.behaviors.primitives;

import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.yoVariables.variable.YoBoolean;

/**
 * @deprecated Broken. See {@link ClearLidarBehavior}.
 */
@Deprecated
public class EnableLidarBehavior extends AbstractBehavior
{
   private final YoBoolean packetHasBeenSent = new YoBoolean("packetHasBeenSent" + behaviorName, registry);
//   private DepthDataStateCommand enableLidarPacket;
//   private LidarState lidarState;

   public EnableLidarBehavior(String robotName, ROS2Node ros2Node)
   {
      super(robotName, ros2Node);

   }

   @Override
   public void doControl()
   {
      
//         enableLidarPacket = new DepthDataStateCommand(lidarState);
//         
//         if (!packetHasBeenSent.getBooleanValue() && (enableLidarPacket != null))
//         {
//            sendPacketToNetworkProcessor();
//         }
      
   }

//   public void setLidarState(LidarState lidarState)
//   {
//      this.lidarState = lidarState;
//   }

   private void sendPacketToNetworkProcessor()
   {
      if (!isPaused.getBooleanValue() && !isAborted.getBooleanValue())
      {
         System.out.println("EnableLidarBehavior: sending enable packet");
//         sendPacket(enableLidarPacket);
         packetHasBeenSent.set(true);
      }
   }

   @Override
   public void onBehaviorEntered()
   {
      packetHasBeenSent.set(false);

      isPaused.set(false);
      isAborted.set(false);
   }

   @Override
   public void onBehaviorExited()
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
