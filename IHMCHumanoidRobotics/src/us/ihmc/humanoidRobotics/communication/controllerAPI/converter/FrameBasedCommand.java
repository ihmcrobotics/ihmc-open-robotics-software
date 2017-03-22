package us.ihmc.humanoidRobotics.communication.controllerAPI.converter;

import us.ihmc.communication.packets.Packet;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public interface FrameBasedCommand<M extends Packet<M>>
{
   void set(ReferenceFrame dataFrame, ReferenceFrame trajectoryFrame, M message);
}
