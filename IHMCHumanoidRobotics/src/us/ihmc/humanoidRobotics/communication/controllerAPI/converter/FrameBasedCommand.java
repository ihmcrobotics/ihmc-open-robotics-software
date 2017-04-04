package us.ihmc.humanoidRobotics.communication.controllerAPI.converter;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public interface FrameBasedCommand<M extends Packet<M>>
{
   public void set(ReferenceFrame dataFrame, ReferenceFrame trajectoryFrame, M message);

   default public void getTransformFromBodyToControlFrame(RigidBodyTransform transformToPack)
   {
      transformToPack.setToNaN();
   }

   default public boolean useCustomControlFrame()
   {
      return false;
   }
}
