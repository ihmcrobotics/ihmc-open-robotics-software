package us.ihmc.humanoidRobotics.communication.subscribers;

import java.util.ArrayList;
import java.util.concurrent.ConcurrentLinkedQueue;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.ThreadFactory;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DrillDetectionPacket;
import us.ihmc.tools.thread.ThreadTools;

public class DrillDetectionStatusSubscriber implements PacketConsumer<DrillDetectionPacket>
{
   private ConcurrentLinkedQueue<DrillDetectionPacket> incomingDetectedDrillPackets = new ConcurrentLinkedQueue<>();

   private ArrayList<DrillDetectionStatusListener> listOfListeners = new ArrayList<DrillDetectionStatusListener>();

   private final ThreadFactory threadFactory = ThreadTools.getNamedThreadFactory(getClass().getName());
   private final ExecutorService executorService = Executors.newSingleThreadExecutor(threadFactory);

   public DrillDetectionStatusSubscriber()
   {
   }

   public void registerListener(DrillDetectionStatusListener listener)
   {
      listOfListeners.add(listener);
      executorService.execute(createCallListenersTask());
   }

   @Override
   public void receivedPacket(DrillDetectionPacket packet)
   {
      incomingDetectedDrillPackets.add(packet);
      executorService.execute(createCallListenersTask());
      System.out.println("received drill packet");
   }

   private Runnable createCallListenersTask()
   {
      return new Runnable()
      {
         @Override
         public void run()
         {
            try
            {
               callListeners();
            }
            catch (Exception e)
            {
               e.printStackTrace();
            }
         }
      };
   }
   
   private void callListeners()
   {
      DrillDetectionPacket newDrillPacket = incomingDetectedDrillPackets.poll();
      if (newDrillPacket != null)
      {
         for (DrillDetectionStatusListener listener : listOfListeners)
         {
            listener.updateStatusPacket(newDrillPacket);
         }
      }
   }

   public void shutdown()
   {
      executorService.shutdownNow();
   }
}
