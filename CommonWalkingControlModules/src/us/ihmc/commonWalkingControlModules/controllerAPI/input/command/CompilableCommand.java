package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import us.ihmc.communication.packets.Packet;

public interface CompilableCommand<T extends CompilableCommand<T, M>, M extends Packet<M>> extends Command<T, M>
{
   public abstract void compile(T other);
}
