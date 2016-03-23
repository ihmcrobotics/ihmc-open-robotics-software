package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import us.ihmc.communication.packets.Packet;

public interface ControllerCommand<T extends ControllerCommand<T, M>, M extends Packet<M>>
{
   public abstract void clear();
   public abstract void set(T other);
   public abstract void set(M message);
   public abstract Class<M> getMessageClass();
   public abstract boolean isCommandValid();
}
