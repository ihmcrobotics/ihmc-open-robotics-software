package us.ihmc.communication.packetCommunicator;

import java.io.IOException;
import java.util.HashMap;

import us.ihmc.communication.net.GlobalObjectConsumer;
import us.ihmc.communication.net.KryoObjectClient;
import us.ihmc.communication.net.KryoObjectServer;
import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.net.NetStateListener;
import us.ihmc.communication.net.ObjectCommunicator;
import us.ihmc.communication.net.ObjectConsumer;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.net.local.InterprocessObjectCommunicator;
import us.ihmc.communication.packetCommunicator.interfaces.GlobalPacketConsumer;
import us.ihmc.communication.packets.Packet;

public class PacketCommunicatorMock
{

   private final ObjectCommunicator communicator;
   private final HashMap<Class<?>, HashMap<PacketConsumer<?>, ObjectConsumer<?>>> consumers = new HashMap<>();
   private final HashMap<GlobalPacketConsumer, GlobalObjectConsumer> globalConsumers = new HashMap<>();

   public static PacketCommunicatorMock createTCPPacketCommunicatorClient(String host, int port, NetClassList netClassList)
   {
      return new PacketCommunicatorMock(new KryoObjectClient(host, port, netClassList));
   }

   public static PacketCommunicatorMock createTCPPacketCommunicatorServer(int port, NetClassList netClassList)
   {
      return new PacketCommunicatorMock(new KryoObjectServer(port, netClassList));
   }

   public static PacketCommunicatorMock createInterprocessPacketCommunicator(int port, NetClassList netClassList)
   {
      return new PacketCommunicatorMock(new InterprocessObjectCommunicator(port, netClassList));
   }

   private PacketCommunicatorMock(ObjectCommunicator communicator)
   {
      this.communicator = communicator;
   }

   public void attachStateListener(NetStateListener stateListener)
   {
      communicator.attachStateListener(stateListener);
   }

   public <T extends Packet> void attachListener(Class<T> clazz, PacketConsumer<T> listener)
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
      GlobalPacketObjectConsumer consumer = new GlobalPacketObjectConsumer(listener);
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

   public void close()
   {
      communicator.close();
   }

   public void connect() throws IOException
   {
      communicator.connect();
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

      @Override
      public void consumeObject(Object object, boolean consumeGlobal)
      {
         throw new RuntimeException();
      }
      
   }

   private static class PacketObjectConsumer<T extends Packet> implements ObjectConsumer<T>
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

}
