package us.ihmc.humanoidBehaviors.communication;

import java.util.HashMap;

import us.ihmc.communication.net.PacketCommunicator;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.Packet;

public class BehaviorPacketPassThroughManager implements PacketConsumer<Packet>
{

   private final PacketCommunicator toCommunicator;
   private final HashMap<Class, Boolean> classPassthroughMap;
   private boolean passThroughActive;

   public BehaviorPacketPassThroughManager(PacketCommunicator fromCommunicator, PacketCommunicator toCommunicator, Class[] packetsToPass)
   {
      this.toCommunicator = toCommunicator;

      this.classPassthroughMap = new HashMap<Class, Boolean>();

      for (Class clz : packetsToPass)
      {
         classPassthroughMap.put(clz, true);
      }

      passThroughActive = true;
      fromCommunicator.attacthGlobalListener(this);
   }
   
   public void setPassThrough(Class clzz, boolean activatePassthrough)
   {
      classPassthroughMap.put(clzz, activatePassthrough);
   }

   @Override
   public void receivedPacket(Packet packet)
   {
      if (passThroughActive && classPassthroughMap.containsKey(packet.getClass()) && classPassthroughMap.get(packet.getClass()))
      {
         toCommunicator.send(packet);
      }
   }

   public void setPassthroughActive(boolean passThroughActive)
   {
      this.passThroughActive = passThroughActive;
   }
}
