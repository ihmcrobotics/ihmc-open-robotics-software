package us.ihmc.commonWalkingControlModules.controllerAPI.input.command;

import us.ihmc.communication.packets.Packet;

public interface ControllerMessage<T extends ControllerMessage<T, M>, M extends Packet<M>>
{
   public abstract void set(T other);
   public abstract void set(M message);
   public abstract void clear();
   public abstract Class<M> getMessageClass();
}
