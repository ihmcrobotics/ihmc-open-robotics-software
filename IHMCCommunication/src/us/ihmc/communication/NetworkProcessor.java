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
   private final int BROADCAST = 0;

   public NetworkProcessor()
   {
   }

   public void attachPacketCommunicator(final PacketCommunicator packetCommunicator)
   {
      PacketConsumer<Packet> packetConsumer = new PacketConsumer<Packet>()
      {
         PacketCommunicator communicator = packetCommunicator;
         
         @Override
         public void receivedPacket(Packet packet)
         {
            processPacket(communicator,packet);
         }
      };

      packetCommunicator.attacthGlobalListener(packetConsumer);
      
      int id = packetCommunicator.getId();
      consumers.put(id, packetConsumer);
      communicators.put(id, packetCommunicator);
   }
   
   /**
    * will send to the destination of or if a redirect is set, it will
    * forward to the redirect.
    * If a the redirect happens to be set to the senders id it will assume 
    * the sender new about the redirect and send to the original destination,  
    * ignoring the redirect
    * @param source the source communicator that sent the packet
    * @param packet
    */
   private void processPacket(PacketCommunicator source, Packet packet)
   {
      int destination = packet.getDestination();
      if (redirects.containsKey(destination))
      {
         destination = getRedirectDestination(destination);
         if(destination != source.getId())
         {
            packet.setDestination(destination);
         }
      }
      
      PacketCommunicator destinationCommunicator = communicators.get(destination);
      if (destinationCommunicator != null && destinationCommunicator.isConnected())
      {
         destinationCommunicator.receivedPacket(packet);
      }
      
      if(destination == BROADCAST)
      {
         broadcastPacket(source, packet);
      }
   }
   
   /**
    * sends the packet to every communicator once, except the sender or any
    * communicators with redirects
   **/
   private void broadcastPacket(PacketCommunicator source, Packet packet)
   {
      int[] ids = communicators.keys();
      for(int i = 0; i < ids.length; i++)
      {
         int destinationId = ids[i];
         if(destinationId != source.getId() && !redirects.containsKey(destinationId))
         {
            PacketCommunicator destinationCommunicator = communicators.get(destinationId);
            if (destinationCommunicator != null && destinationCommunicator.isConnected())
            {
               destinationCommunicator.receivedPacket(packet);
            }
         }
      }
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
