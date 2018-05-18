package us.ihmc.communication.packetCommunicator;

import java.io.IOException;
import java.util.HashMap;

import us.ihmc.commons.PrintTools;
import us.ihmc.communication.CommunicationOptions;
import us.ihmc.communication.interfaces.Connectable;
import us.ihmc.communication.net.ConnectionStateListener;
import us.ihmc.communication.net.GlobalObjectConsumer;
import us.ihmc.communication.net.KryoObjectClient;
import us.ihmc.communication.net.KryoObjectServer;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.net.NetClassList.PacketTrimmer;
import us.ihmc.communication.net.NetworkedObjectCommunicator;
import us.ihmc.communication.net.ObjectConsumer;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.net.ROS2ObjectCommunicator;
import us.ihmc.communication.net.TcpNetStateListener;
import us.ihmc.communication.net.local.IntraprocessObjectCommunicator;
import us.ihmc.communication.packetCommunicator.interfaces.GlobalPacketConsumer;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;

public class PacketCommunicator implements Connectable
{
   public static final int BUFFER_SIZE = 2097152 * 20;

   private final NetworkedObjectCommunicator communicator;
   private final HashMap<Class<?>, HashMap<PacketConsumer<?>, ObjectConsumer<?>>> consumers = new HashMap<>();
   private final HashMap<GlobalPacketConsumer, GlobalObjectConsumer> globalConsumers = new HashMap<>();

   private final NetClassList netClassList;

   private final String description;

   private static boolean freezeCommunication = false;

   public static PacketCommunicator createTCPPacketCommunicatorClient(String host, NetworkPorts port, NetClassList netClassList)
   {
      return createTCPPacketCommunicatorClient(host, port, netClassList, true);
   }

   public static void freezeCommunication(boolean freeze)
   {
      freezeCommunication = freeze;
   }

   public static PacketCommunicator createTCPPacketCommunicatorClient(String host, NetworkPorts port, NetClassList netClassList, boolean reconnectAutomatically)
   {
      NetworkedObjectCommunicator objectCommunicator;
      if (CommunicationOptions.USE_ROS2)
      {

         PrintTools.info(PacketCommunicator.class, "Creating ROS2 Fast-RTPS topic: " + port.getName(), true);
         objectCommunicator = new ROS2ObjectCommunicator(PubSubImplementation.FAST_RTPS, port, netClassList);
      }
      else
      {
         PrintTools.info(PacketCommunicator.class, "Creating Kryo TCP client on port: " + port.getName(), true);
         objectCommunicator = new KryoObjectClient(KryoObjectClient.getByName(host), port.getPort(), netClassList, BUFFER_SIZE, BUFFER_SIZE);
         ((KryoObjectClient) objectCommunicator).setReconnectAutomatically(reconnectAutomatically);
      }
      return new PacketCommunicator("TCPClient[host=" + host + ",port=" + port + "]", objectCommunicator, netClassList);
   }

   public static PacketCommunicator createTCPPacketCommunicatorServer(NetworkPorts port, NetClassList netClassList)
   {
      return createTCPPacketCommunicatorServer(port, BUFFER_SIZE, BUFFER_SIZE, netClassList);
   }

   public static PacketCommunicator createTCPPacketCommunicatorServer(NetworkPorts port, int writeBufferSize, int receiveBufferSize, NetClassList netClassList,
                                                                      int maximumObjectSize)
   {
      NetworkedObjectCommunicator objectCommunicator;
      if (CommunicationOptions.USE_ROS2)
      {
         PrintTools.info(PacketCommunicator.class, "Creating ROS2 Fast-RTPS topic: " + port.getName(), true);
         objectCommunicator = new ROS2ObjectCommunicator(PubSubImplementation.FAST_RTPS, port, netClassList);
      }
      else
      {
         PrintTools.info(PacketCommunicator.class, "Creating Kryo TCP server on port: " + port.getName(), true);
         objectCommunicator = new KryoObjectServer(port.getPort(), netClassList, writeBufferSize, receiveBufferSize);
         ((KryoObjectServer) objectCommunicator).setMaximumObjectSize(maximumObjectSize);
      }
      return new PacketCommunicator("TCPServer[port=" + port + "]", objectCommunicator, netClassList);
   }

   public static PacketCommunicator createTCPPacketCommunicatorServer(NetworkPorts port, int writeBufferSize, int receiveBufferSize, NetClassList netClassList)
   {
      return createTCPPacketCommunicatorServer(port, writeBufferSize, receiveBufferSize, netClassList, 0); //infinite
   }

   public static PacketCommunicator createIntraprocessPacketCommunicator(NetworkPorts port, NetClassList netClassList)
   {
      NetworkedObjectCommunicator objectCommunicator;
      if (CommunicationOptions.USE_ROS2)
      {
         PrintTools.info(PacketCommunicator.class, "Creating ROS2 intraprocess on port: " + port.getName(), true);
         objectCommunicator = new ROS2ObjectCommunicator(PubSubImplementation.INTRAPROCESS, port, netClassList);
      }
      else
      {
         PrintTools.info(PacketCommunicator.class, "Creating Kryo intraprocess on port: " + port.getName(), true);
         objectCommunicator = new IntraprocessObjectCommunicator(port.getPort(), netClassList);
      }
      return new PacketCommunicator("IntraProcess[port=" + port + "]", objectCommunicator, netClassList);
   }

   public static PacketCommunicator createCustomPacketCommunicator(NetworkedObjectCommunicator objectCommunicator, NetClassList netClassList)
   {
      PrintTools.info(PacketCommunicator.class, "Creating custom", true);
      return new PacketCommunicator("Custom[class=" + objectCommunicator.getClass().getSimpleName() + "]", objectCommunicator, netClassList);
   }

   private PacketCommunicator(String description, NetworkedObjectCommunicator communicator, NetClassList netClassList)
   {
      this.description = description;
      this.communicator = communicator;
      this.netClassList = netClassList;
   }

   public void attachStateListener(ConnectionStateListener stateListener)
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
      if (clazzConsumers == null)
      {
         clazzConsumers = new HashMap<>();
         consumers.put(clazz, clazzConsumers);
      }

      clazzConsumers.put(listener, objectConsumer);
      communicator.attachListener(clazz, objectConsumer);
   }

   @SuppressWarnings({"unchecked", "rawtypes"})
   public <T extends Packet> void detachListener(Class<T> clazz, PacketConsumer<T> listener)
   {
      HashMap<PacketConsumer<?>, ObjectConsumer<?>> clazzConsumers = consumers.get(clazz);
      if (clazzConsumers == null)
      {
         return;
      }
      ObjectConsumer consumer = clazzConsumers.get(listener);
      if (consumer != null)
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
      if (consumer != null)
      {
         communicator.detachGlobalListener(consumer);
         globalConsumers.remove(listener);
      }
   }

   @Override
   public boolean isConnected()
   {
      return communicator.isConnected();
   }

   public void closeConnection()
   {
      communicator.closeConnection();
   }

   public void disconnect()
   {
      try
      {
         communicator.disconnect();
      }
      catch (IOException e)
      {
         // handle silently for now
      }
   }

   public void connect() throws IOException
   {
      communicator.connect();
   }

   /**
    * @param packet Send a packet to connected receivers. Does not call listeners
    * @return
    */
   @SuppressWarnings({"unchecked", "rawtypes"})
   public int send(Packet<?> packet)
   {
      if (!freezeCommunication)
      {
         PacketTrimmer packetTrimmer = netClassList.getPacketTrimmer(packet.getClass());
         if (packetTrimmer != null)
            return communicator.send(packetTrimmer.trim(packet));
         else
            return communicator.send(packet);
      }
      else
      {
         return -1;
      }
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
         if (object instanceof Packet)
         {
            if (!freezeCommunication)
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
         if (!freezeCommunication)
            packetConsumer.receivedPacket(object);
      }
   }

   @Override
   public String toString()
   {
      return description;
   }

}
