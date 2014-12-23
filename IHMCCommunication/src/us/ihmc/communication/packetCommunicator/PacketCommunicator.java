package us.ihmc.communication.packetCommunicator;

import java.io.IOException;

import us.ihmc.communication.net.NetStateListener;
import us.ihmc.communication.packets.Packet;

public interface PacketCommunicator extends PacketConsumer<Packet>
{
   public <T extends Packet> void attachListener(Class<T> clazz, PacketConsumer<T> listener);
   public <T extends Packet> void detachListener(Class<T> clazz, PacketConsumer<T> listener);
   
   public void removeGlobalListener();
   public void setGlobalListener(PacketConsumer<Packet> externalConsumer);
   
   public void send(Packet p);
   
   public void attachStateListener(NetStateListener stateListener);
   public void detatchStateListener(NetStateListener stateListener);
   
   public boolean isConnected();
   public void close();
   public void connect() throws IOException;
   
   public int getId();
   public String getName();
}
