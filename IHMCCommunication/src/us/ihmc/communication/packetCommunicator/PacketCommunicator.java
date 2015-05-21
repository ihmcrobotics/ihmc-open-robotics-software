package us.ihmc.communication.packetCommunicator;

import java.io.IOException;
import java.util.HashMap;
import java.util.List;

import us.ihmc.communication.net.GlobalObjectConsumer;
import us.ihmc.communication.net.KryoObjectClient;
import us.ihmc.communication.net.KryoObjectServer;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.net.NetStateListener;
import us.ihmc.communication.net.NetworkedObjectCommunicator;
import us.ihmc.communication.net.ObjectConsumer;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.net.TcpNetStateListener;
import us.ihmc.communication.net.local.IntraprocessObjectCommunicator;
import us.ihmc.communication.packetCommunicator.interfaces.GlobalPacketConsumer;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.util.NetworkPorts;

public class PacketCommunicator
{
   public static final int BUFFER_SIZE = 2097152 * 20;
   
   private final NetworkedObjectCommunicator communicator;
   private final HashMap<Class<?>, HashMap<PacketConsumer<?>, ObjectConsumer<?>>> consumers = new HashMap<>();
   private final HashMap<GlobalPacketConsumer, GlobalObjectConsumer> globalConsumers = new HashMap<>();

   private final List<Class<?>> registeredClasses;
   
   private final String description;
   
   public static PacketCommunicator createTCPPacketCommunicatorClient(String host, NetworkPorts port, NetClassList netClassList)
   {
      return createTCPPacketCommunicatorClient(host, port, netClassList, true);
   }

   public static PacketCommunicator createTCPPacketCommunicatorClient(String host, NetworkPorts port, NetClassList netClassList, boolean reconnectAutomatically)
   {
      KryoObjectClient objectCommunicator = new KryoObjectClient(KryoObjectClient.getByName(host), port.getPort(), netClassList, BUFFER_SIZE, BUFFER_SIZE);
      objectCommunicator.setReconnectAutomatically(reconnectAutomatically);
      return new PacketCommunicator("TCPClient[host=" + host + ",port=" + port + "]", objectCommunicator, netClassList.getPacketClassList());
   }

   public static PacketCommunicator createTCPPacketCommunicatorServer(NetworkPorts port, NetClassList netClassList)
   {
      return createTCPPacketCommunicatorServer(port, BUFFER_SIZE, BUFFER_SIZE, netClassList);
   }

   public static PacketCommunicator createTCPPacketCommunicatorServer(NetworkPorts port, int writeBufferSize, int receiveBufferSize, NetClassList netClassList, int maximumObjectSize)
   {
      KryoObjectServer server = new KryoObjectServer(port.getPort(), netClassList, writeBufferSize, receiveBufferSize);
      server.setMaximumObjectSize(maximumObjectSize);

      return new PacketCommunicator("TCPServer[port=" + port + "]", server, netClassList.getPacketClassList());
   }

   public static PacketCommunicator createTCPPacketCommunicatorServer(NetworkPorts port, int writeBufferSize, int receiveBufferSize, NetClassList netClassList)
   {
      return new PacketCommunicator("TCPServer[port=" + port + "]", new KryoObjectServer(port.getPort(), netClassList, writeBufferSize, receiveBufferSize), netClassList.getPacketClassList());
   }

   public static PacketCommunicator createIntraprocessPacketCommunicator(NetworkPorts port, NetClassList netClassList)
   {
      return new PacketCommunicator("IntraProcess[port=" + port + "]", new IntraprocessObjectCommunicator(port.getPort(), netClassList), netClassList.getPacketClassList());
   }
   
   public static PacketCommunicator createCustomPacketCommunicator(NetworkedObjectCommunicator objectCommunicator, NetClassList netClassList)
   {
      return new PacketCommunicator("Custom[class=" + objectCommunicator.getClass().getSimpleName() + "]", objectCommunicator, netClassList.getPacketClassList());
   }

   private PacketCommunicator(String description, NetworkedObjectCommunicator communicator, List<Class<?>> registeredClasses)
   {
      this.description = description;
      this.communicator = communicator;
      this.registeredClasses = registeredClasses;
   }

   public void attachStateListener(NetStateListener stateListener)
   {
      communicator.attachStateListener(stateListener);
   }

   public void attachStateListener(TcpNetStateListener stateListener)
   {
      communicator.attachStateListener(stateListener);
   }

   public <T extends Packet<?>> void attachListener(Class<T> clazz, PacketConsumer<T> listener)
   {
      PacketObjectConsumer<T> objectConsumer = new PacketObjectConsumer<>(listener);
      HashMap<PacketConsumer<?>, ObjectConsumer<?>> clazzConsumers = consumers.get(clazz);
      if(clazzConsumers == null)
      {
         clazzConsumers = new HashMap<>();
         consumers.put(clazz, clazzConsumers);
      }
      
      clazzConsumers.put(listener, objectConsumer);
      communicator.attachListener(clazz, objectConsumer);
   }

   @SuppressWarnings({ "unchecked", "rawtypes" })
   public <T extends Packet> void detachListener(Class<T> clazz, PacketConsumer<T> listener)
   {
      HashMap<PacketConsumer<?>, ObjectConsumer<?>> clazzConsumers = consumers.get(clazz);
      if(clazzConsumers == null)
      {
         return;
      }
      ObjectConsumer consumer = clazzConsumers.get(listener);
      if(consumer != null)
      {
         communicator.detachListener(clazz, consumer);
         consumers.remove(listener);
      }
   }

   public void attachGlobalListener(GlobalPacketConsumer listener)
   {
      GlobalPacketObjectConsumer consumer = new GlobalPacketObjectConsumer(listener);
      globalConsumers.put(listener, consumer);
      communicator.attachGlobalListener(consumer);
   }

   public void detachGlobalListener(GlobalPacketConsumer listener)
   {
      GlobalObjectConsumer consumer = globalConsumers.get(listener);
      if(consumer != null)
      {
         communicator.detachGlobalListener(consumer);
         globalConsumers.remove(listener);
      }
   }

   public boolean isConnected()
   {
      return communicator.isConnected();
   }
   
   public void closeConnection()
   {
      communicator.closeConnection();
   }

   public void close()
   {
      communicator.close();
   }

   public void connect() throws IOException
   {
      communicator.connect();
   }
   
   /**
    * @param packet Send a packet to connected receivers. Does not call listeners
    * @return
    */
   public int send(Packet<?> packet)
   {
      return communicator.send(packet);
   }
   
   private static class GlobalPacketObjectConsumer implements GlobalObjectConsumer
   {
      private final GlobalPacketConsumer globalPacketConsumer;
      private GlobalPacketObjectConsumer(GlobalPacketConsumer globalPacketConsumer)
      {
         this.globalPacketConsumer = globalPacketConsumer;
      }

      @Override
      public void consumeObject(Object object)
      {
         if(object instanceof Packet)
         {
            globalPacketConsumer.receivedPacket((Packet<?>) object);
         }
      }
   }

   private static class PacketObjectConsumer<T extends Packet<?>> implements ObjectConsumer<T>
   {
      private final PacketConsumer<T> packetConsumer;

      private PacketObjectConsumer(PacketConsumer<T> packetConsumer)
      {
         this.packetConsumer = packetConsumer;
      }

      @Override
      public void consumeObject(T object)
      {
         packetConsumer.receivedPacket(object);
      }
   }

   public List<Class<?>> getRegisteredClasses()
   {
      return registeredClasses;
   }
   
   @Override
   public String toString()
   {
      return description;
   }

}
