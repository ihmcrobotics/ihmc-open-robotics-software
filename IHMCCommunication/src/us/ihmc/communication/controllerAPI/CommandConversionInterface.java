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

   /**
    * Checks whether the given command is a {@link #FrameBasedCommandHolder} in that case the default setter from message
    * of the command should not be used since it requires a {@link #ReferenceFrameHashCodeResolver} to resolve the reference
    * frames contained in the message. The method {@link #processFrameBasedCommandHolder(Command, Packet)} should be used
    * for conversion instead.
    * 
    * @param command to check
    * @return whether the command is an instance of {@link #FrameBasedCommandHolder}
    */
   <C extends Command<?, M>, M extends Packet<M>> boolean isFrameBasedCommandHolder(C command);

   /**
    * This will call the setter from message for given command with a {@link #ReferenceFrameHashCodeResolver}. Use this
    * if the command is an instance of {@link #FrameBasedCommandHolder}. This can be checked with the method
    * {@link CommandConversionInterface#isFrameBasedCommandHolder(Command)}.
    * 
    * @param command to set from message (modified)
    * @param message to set command from
    */
   <C extends Command<?, M>, M extends Packet<M>> void processFrameBasedCommandHolder(C command, M message);
}
