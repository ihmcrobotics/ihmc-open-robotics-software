package us.ihmc.humanoidBehaviors.behaviors.primitives;

import us.ihmc.humanoidBehaviors.behaviors.AbstractBehavior;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DepthDataFilterParameters;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.yoVariables.variable.YoBoolean;

/**
 * @deprecated This is not working anymore. See {@link ClearLidarBehavior}.
 *
 */
public class SetLidarParametersBehavior extends AbstractBehavior
{
   private final YoBoolean packetHasBeenSent = new YoBoolean("packetHasBeenSent" + behaviorName, registry);
   private DepthDataFilterParameters lidarParamPacket;

   public SetLidarParametersBehavior(String robotName, Ros2Node ros2Node)
   {
      super(robotName, ros2Node);

   }

   public void setInput(DepthDataFilterParameters clearLidarPacket)
   {
      this.lidarParamPacket = clearLidarPacket;
   }

   @Override
   public void doControl()
   {
      if (!packetHasBeenSent.getBooleanValue() && (lidarParamPacket != null))
      {
         sendPacketToNetworkProcessor();
      }
   }

   private void sendPacketToNetworkProcessor()
   {
      if (!isPaused.getBooleanValue() && !isAborted.getBooleanValue())
      {
         //         sendPacket(lidarParamPacket);
         packetHasBeenSent.set(true);
      }
   }

   @Override
   public void onBehaviorEntered()
   {
      packetHasBeenSent.set(false);
      lidarParamPacket = null;
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

   public boolean hasInputBeenSet()
   {
      if (lidarParamPacket != null)
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
