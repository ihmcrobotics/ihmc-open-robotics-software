package us.ihmc.communication;

import gnu.trove.iterator.TIntIntIterator;
import gnu.trove.map.hash.TIntIntHashMap;
import gnu.trove.map.hash.TIntObjectHashMap;
import us.ihmc.communication.net.PacketCommunicator;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packets.Packet;

public class NetworkProcessor
{
   private final TIntObjectHashMap<PacketCommunicator> communicators = new TIntObjectHashMap<PacketCommunicator>();
   private final TIntObjectHashMap<PacketConsumer<Packet>> consumers = new TIntObjectHashMap<PacketConsumer<Packet>>();
   private final TIntIntHashMap redirects = new TIntIntHashMap();

   public NetworkProcessor()
   {
   }

   public void attachPacketCommunicator(PacketCommunicator packetCommunicator, int id)
   {
      PacketConsumer<Packet> packetConsumer = new PacketConsumer<Packet>()
      {
         @Override
         public void receivedPacket(Packet packet)
         {
            int destination = packet.getDestination();
            if (redirects.containsKey(destination))
            {
               destination = getRedirectDestination(destination);
               packet.setDestination(destination);
            }
            PacketCommunicator destinationCommunicator = communicators.get(destination);
            if (destinationCommunicator != null && destinationCommunicator.isConnected())
            {
               destinationCommunicator.receivedPacket(packet);
            }
         }
      };

      packetCommunicator.attacthGlobalListener(packetConsumer);

      consumers.put(id, packetConsumer);
      communicators.put(id, packetCommunicator);
   }
   
   /**
    * possible infinite recursion for funs.
    */
   private int getRedirectDestination(int destination)
   {
      if (redirects.containsKey(destination))
      {
         destination = getRedirectDestination(redirects.get(destination));
      }
      return destination;
   }

   public void detatchObjectCommunicator(int id)
   {
      PacketCommunicator communicator = communicators.remove(id);
      PacketConsumer<Packet> consumer = consumers.remove(id);
      
      communicator.detachGlobalListener(consumer);
      
      redirects.remove(id);

      if (redirects.containsValue(id))
      {
         for (TIntIntIterator iterator = redirects.iterator(); iterator.hasNext();)
         {
            iterator.advance();
            if (iterator.value() == id)
            {
               iterator.remove();
            }
         }
      }
   }

   public void setPacketRedirects(int redirectFrom, int redirectTo)
   {
      redirects.put(redirectFrom, redirectTo);
   }

   public void removePacketRedirect(int redirectFrom)
   {
      redirects.remove(redirectFrom);
   }
}
