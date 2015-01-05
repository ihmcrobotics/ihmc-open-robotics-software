package us.ihmc.humanoidBehaviors.communication;

import java.util.concurrent.ConcurrentLinkedQueue;

import us.ihmc.communication.net.PacketCommunicator;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.Packet;
import us.ihmc.utilities.ThreadTools;

public class NonBlockingGlobalObjectConsumerRelay implements PacketConsumer
{
   private static final boolean DEBUG = false;

   private final ConcurrentLinkedQueue<Packet> queuedData = new ConcurrentLinkedQueue<Packet>(); 
   private final PacketCommunicator communicatorToForwardFrom;
   private final PacketCommunicator communicatorToForwardTo;
   
   public NonBlockingGlobalObjectConsumerRelay(PacketCommunicator communicatorToForwardFrom, PacketCommunicator communicatorToForwardTo)
   {
      this.communicatorToForwardFrom = communicatorToForwardFrom;
      this.communicatorToForwardTo = communicatorToForwardTo;
      startProducingData();
   }
   
   public void enableForwarding()
   {
      communicatorToForwardFrom.attacthGlobalListener(this);
      System.out.println("enabled forwarder from " + communicatorToForwardFrom.getClass().getSimpleName());
   }
   
   public void disableForwarding()
   {
      communicatorToForwardFrom.detachGlobalListener(this);
      System.out.println("disabled forwarder" + communicatorToForwardFrom.getClass().getSimpleName());
   }

   @Override
   public void receivedPacket(Packet packet)
   {
      queuedData.add(packet);
   }

   public void startProducingData()
   {
      Runnable runnable = new Runnable()
      {

         public void run()
         {
            while (true)
            {
               Packet dataObject;
               while((dataObject = queuedData.poll()) != null)
               {
                  if (DEBUG)
                  {
                     if(!dataObject.getClass().getSimpleName().equals("RobotConfigurationData") && !dataObject.getClass().getSimpleName().equals("RobotPoseData"))
                        System.out.println(dataObject.getClass().getSimpleName());
                  }
                  communicatorToForwardTo.receivedPacket(dataObject);
               }
               
               ThreadTools.sleep(100);

            }
         }
      };
      ThreadTools.startAsDaemon(runnable, "NonBlockingGlobalObjectConsumerRelay for " + communicatorToForwardFrom.getClass().getSimpleName());
   }
}
