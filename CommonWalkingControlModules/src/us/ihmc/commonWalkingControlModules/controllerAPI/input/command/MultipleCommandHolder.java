package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import java.util.List;

import us.ihmc.communication.packets.Packet;

/**
 * Extension of {@link Command} with the possibility of wrapping a set of {@link Command}.
 * @author Sylvain
 *
 * @param <C> Type of the final implementation of this MultipleCommandHolder. It is used for the copy method {@link #set(MultipleCommandHolder)}.
 * @param <M> Type of the corresponding {@link Packet} that this MultipleCommandHolder can copy.
 */
public interface MultipleCommandHolder<C extends MultipleCommandHolder<C, M>, M extends Packet<M>> extends Command<C, M>
{
   /**
    * @return the list of commands held by this MultipleCommandHolder.
    */
   public abstract List<Command<?, ?>> getControllerCommands();
}
