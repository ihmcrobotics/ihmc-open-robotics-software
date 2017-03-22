package us.ihmc.communication.controllerAPI;

import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.communication.packets.Packet;

/**
 * Helps converts packets to commands
 */
public interface CommandConversionInterface
{
   /**
    * Can the command converter handle a packet, generally just check the type of the message (incoming packet) and the command
    * @param command the command type that is going to be updated
    * @param message the message that was received
    * @return whether it can convert it
    */
   <C extends Command<?, M>, M extends Packet<M>> boolean isConvertible(C command, M message);
   
   
   /**
    * actually update the command based on the message
    * @param command the command to update
    * @param message the message with the source data
    */
   <C extends Command<?, M>, M extends Packet<M>> void process(C command, M message);
}
