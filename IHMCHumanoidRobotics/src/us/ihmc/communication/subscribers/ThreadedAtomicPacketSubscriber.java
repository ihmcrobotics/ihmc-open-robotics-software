package us.ihmc.communication.subscribers;

import java.util.ArrayList;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.Packet;

public class ThreadedAtomicPacketSubscriber<T extends Packet<?>> implements PacketConsumer<T>, Runnable
{
   private AtomicReference<T> lastPacket = new AtomicReference<T>();
   private final Object syncObject = new Object();
   private ArrayList<PacketConsumer<T>> listOfListeners = new ArrayList<PacketConsumer<T>>();

   public ThreadedAtomicPacketSubscriber()
   {
      Thread t = new Thread(this);
      t.start();
   }

   public void registerListener(PacketConsumer<T> listener)
   {
      listOfListeners.add(listener);
   }

   @Override
   public void run()
   {
      while (true)
      {
         synchronized (syncObject)
         {
            try
            {
               syncObject.wait();
            }
            catch (InterruptedException e)
            {
            }
            
            T packet = lastPacket.getAndSet(null);
            if (packet != null)
            {
               for (PacketConsumer<T> listener : listOfListeners)
               {
                  listener.receivedPacket(packet);
               }
            }
         }
      }
   }

   @Override
   public void receivedPacket(T packet)
   {
      synchronized (syncObject)
      {
         lastPacket.set(packet);
         syncObject.notifyAll();
      }
   }

}
