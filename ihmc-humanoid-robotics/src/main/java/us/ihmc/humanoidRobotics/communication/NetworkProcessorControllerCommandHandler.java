package us.ihmc.humanoidRobotics.communication;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.Packet;

public interface NetworkProcessorControllerCommandHandler extends PacketConsumer<Packet>
{
   public void sendObjectToFieldComputer(Packet<?> object);
   public <T extends Packet<?>> void attachListener(Class<T> clazz, PacketConsumer<T> object);
}
