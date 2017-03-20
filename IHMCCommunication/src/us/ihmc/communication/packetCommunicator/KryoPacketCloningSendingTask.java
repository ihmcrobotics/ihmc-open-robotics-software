package us.ihmc.communication.packetCommunicator;

import java.util.ArrayList;
import java.util.concurrent.Callable;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.atomic.AtomicBoolean;

import com.esotericsoftware.kryo.Kryo;

import us.ihmc.communication.net.NetClassList;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.Packet;

public class KryoPacketCloningSendingTask implements Callable<Void>
{
   private PacketConsumer consumer;
   private final ConcurrentLinkedQueue<Packet> packetQueue = new ConcurrentLinkedQueue<Packet>();
   private Packet packetToSend;
   private final AtomicBoolean isRunning = new AtomicBoolean();
   private final Kryo kryo = new Kryo();
   private final ArrayList<Class<?>> packetList = new ArrayList<Class<?>>();
   private final boolean clonePackets = true;

   
   //should just populate the class list using the passed in classes instead of the whole classlist
   public KryoPacketCloningSendingTask(NetClassList classList, PacketConsumer consumer)
   {
      classList.getPacketClassList(packetList);
      for (Class<?> clazz : packetList)
      {
         kryo.register(clazz);
      }
      for (Class<?> type : classList.getPacketFieldList())
      {
         kryo.register(type);
      }
      this.consumer = consumer;
   }

   public void setConsumer(PacketConsumer consumer)
   {
      this.consumer = consumer;
   }

   public PacketConsumer getConsumer()
   {
      return this.consumer;
   }

   public void submitPacket(Packet packet)
   {
      packetQueue.add(packet);
   }

   @Override
   public Void call() throws Exception
   {
      isRunning.set(true);
      while ((packetToSend = packetQueue.poll()) != null)
      {
         if (packetList.contains(packetToSend.getClass()) && clonePackets )
         {
            Packet<?> clonedPacket = kryo.copy(packetToSend);
            consumer.receivedPacket(clonedPacket);
         }
         else
         {
            consumer.receivedPacket(packetToSend);
         }
      }
      isRunning.set(false);
      return null;
   }

   public boolean isRunning()
   {
      return isRunning.get();
   }
}
