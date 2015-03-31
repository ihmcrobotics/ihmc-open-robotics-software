package us.ihmc.humanoidBehaviors.communication;

import java.util.HashMap;

import us.ihmc.communication.packetCommunicator.PacketCommunicatorMock;
import us.ihmc.communication.packetCommunicator.interfaces.GlobalPacketConsumer;
import us.ihmc.communication.packets.Packet;

public class BehaviorPacketPassThroughManager implements GlobalPacketConsumer
{

   private final PacketCommunicatorMock toCommunicator;
   private final HashMap<Class, Boolean> classPassthroughMap;
   private boolean passThroughActive;

   public BehaviorPacketPassThroughManager(PacketCommunicatorMock fromCommunicator, PacketCommunicatorMock toCommunicator, Class[] packetsToPass)
   {
      this.toCommunicator = toCommunicator;

      this.classPassthroughMap = new HashMap<Class, Boolean>();

      for (Class clz : packetsToPass)
      {
         classPassthroughMap.put(clz, true);
      }

      passThroughActive = true;
      fromCommunicator.attachGlobalListener(this);
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
