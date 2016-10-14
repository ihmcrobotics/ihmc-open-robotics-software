package us.ihmc.communication.controllerAPI.command;

import us.ihmc.communication.packets.Packet;

/**
 * Extension of {@link Command} with the possibility of combining this CompilableCommand with another one through the method {@link #compile(CompilableCommand)}.
 * @author Sylvain
 *
 * @param <C> Type of the final implementation of this CompilableCommand. It is used for the copy method {@link #set(CompilableCommand)}.
 * @param <M> Type of the corresponding {@link Packet} that this CompilableCommand can copy.
 */
public interface CompilableCommand<C extends CompilableCommand<C, M>, M extends Packet<M>> extends Command<C, M>
{
   /**
    * Complete the information held in this with the information held in the other command.
    * @param other Command with which this will be completed.
    */
   public abstract void compile(C other);
}
