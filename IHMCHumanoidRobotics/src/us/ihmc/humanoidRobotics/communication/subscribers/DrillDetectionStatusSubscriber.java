package us.ihmc.humanoidRobotics.communication.subscribers;

import java.util.ArrayList;
import java.util.concurrent.ConcurrentLinkedQueue;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.sensing.DrillDetectionPacket;
import us.ihmc.tools.thread.ThreadTools;

public class DrillDetectionStatusSubscriber implements PacketConsumer<DrillDetectionPacket>, Runnable
{
   private ConcurrentLinkedQueue<DrillDetectionPacket> incomingDetectedDrillPackets = new ConcurrentLinkedQueue<>();

   private ArrayList<DrillDetectionStatusListener> listOfListeners = new ArrayList<DrillDetectionStatusListener>();

   public DrillDetectionStatusSubscriber()
   {
      Thread t = new Thread(this);
      t.start();
   }

   public void registerListener(DrillDetectionStatusListener listener)
   {
      listOfListeners.add(listener);
   }

   @Override
   public void receivedPacket(DrillDetectionPacket packet)
   {
      incomingDetectedDrillPackets.add(packet);
   }

   @Override
   public void run()
   {
      while (true)
      {
         DrillDetectionPacket newDrillPacket = incomingDetectedDrillPackets.poll();
         if (newDrillPacket != null)
         {

            for (DrillDetectionStatusListener listener : listOfListeners)
            {
               listener.updateStatusPacket(newDrillPacket);
            }
         }

         ThreadTools.sleep(1);
      }
   }

}
