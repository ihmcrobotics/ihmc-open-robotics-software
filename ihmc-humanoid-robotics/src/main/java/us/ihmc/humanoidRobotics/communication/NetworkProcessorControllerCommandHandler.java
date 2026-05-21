package us.ihmc.humanoidRobotics.communication;

// TODO: PacketConsumer and Packet don't exist in jros2 - this interface needs refactoring
// import us.ihmc.communication.net.PacketConsumer;
// import us.ihmc.communication.packets.Packet;

public interface NetworkProcessorControllerCommandHandler // extends PacketConsumer<Packet>
{
   // TODO: Refactor to use ROS2 messages instead of Packet
   // public void sendObjectToFieldComputer(ROS2Message<?> object);
   // public <T extends ROS2Message<?>> void attachListener(Class<T> clazz, PacketConsumer<T> object);
}
