package us.ihmc.humanoidRobotics.communication.controllerAPI.converter;

import us.ihmc.communication.packets.Packet;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public interface FrameBasedCommand<M extends Packet<M>>
{
   public void set(ReferenceFrameHashCodeResolver resolver, M message);
}
