package us.ihmc.humanoidRobotics.communication;

// TODO: PacketConsumer and Packet don't exist in jros2 - this interface needs refactoring
// import us.ihmc.communication.net.PacketConsumer;
// import us.ihmc.communication.packets.Packet;

public interface NetworkProcessorControllerCommandHandler // extends PacketConsumer<Packet>
{
   // TODO: Refactor to use ROS2 messages instead of Packet
   // public void sendObjectToFieldComputer(Packet<?> object);
   // public <T extends Packet<?>> void attachListener(Class<T> clazz, PacketConsumer<T> object);
}
