package us.ihmc.communication.controllerAPI;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.Packet;

public interface CommandConversionInterface
{
   <C extends Command<?, M>, M extends Packet<M>> boolean isConvertible(C command, M message);
   <C extends Command<?, M>, M extends Packet<M>> void process(C command, M message);
}
