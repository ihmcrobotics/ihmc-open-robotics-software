package us.ihmc.humanoidRobotics.communication.controllerAPI.converter;

import us.ihmc.communication.packets.Packet;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public interface FrameBasedCommand<M extends Packet<M>>
{
   public void set(ReferenceFrameHashCodeResolver resolver, M message);

   default public void packControlFramePose(RigidBodyTransform transformToPack)
   {
      transformToPack.setToNaN();
   }

   default public boolean useCustomControlFrame()
   {
      return false;
   }
}
