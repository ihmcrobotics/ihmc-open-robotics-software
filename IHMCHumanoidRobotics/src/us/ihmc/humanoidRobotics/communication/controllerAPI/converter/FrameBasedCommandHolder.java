package us.ihmc.humanoidRobotics.communication.controllerAPI.converter;

import us.ihmc.communication.controllerAPI.CommandConversionInterface;
import us.ihmc.communication.packets.Packet;

/**
 * Commands implementing this interface contain frame information that require a {@link #ReferenceFrameHashCodeResolver}
 * for conversion.
 */
public interface FrameBasedCommandHolder <M extends Packet<M>>
{
   public void set(CommandConversionInterface commandConverter, M message);
}
