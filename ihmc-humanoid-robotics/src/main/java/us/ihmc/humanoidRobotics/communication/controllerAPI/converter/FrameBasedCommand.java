package us.ihmc.humanoidRobotics.communication.controllerAPI.converter;

import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public interface FrameBasedCommand<M extends Settable<M>>
{
   public void set(ReferenceFrameHashCodeResolver resolver, M message);
}
