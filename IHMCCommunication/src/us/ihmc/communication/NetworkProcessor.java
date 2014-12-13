package us.ihmc.communication;

import gnu.trove.iterator.TIntIntIterator;
import gnu.trove.map.hash.TIntIntHashMap;
import gnu.trove.map.hash.TIntObjectHashMap;
import us.ihmc.communication.net.GlobalObjectConsumer;
import us.ihmc.communication.net.ObjectCommunicator;
import us.ihmc.communication.net.ObjectConsumer;
import us.ihmc.communication.packets.Packet;

public class NetworkProcessor
{
   private final TIntObjectHashMap<ObjectCommunicator> communicators = new TIntObjectHashMap<ObjectCommunicator>();
   private final TIntObjectHashMap<ObjectConsumer<Packet>> consumers = new TIntObjectHashMap<ObjectConsumer<Packet>>();
   private final TIntIntHashMap redirects = new TIntIntHashMap();

   public NetworkProcessor()
   {

   }

   public void attachObjectCommunicator(ObjectCommunicator objectCommunicator, int id)
   {
      GlobalObjectConsumer packetConsumer = new GlobalObjectConsumer()
      {
         @Override
         public void consumeObject(Object packet)
         {
//            int destination = packet.getDestination();
//            if (redirects.containsKey(destination))
//            {
//               destination = redirects.get(destination);
//            }
//
//            communicators.get(destination).consumeObject(packet);
         }

         @Override
         public void consumeObject(Object packet, boolean consumeGlobal)
         {
            consumeObject(packet);
         }
      };
      
//      objectCommunicator.attachListener(Packet.class, packetConsumer);
//      consumers.put(id, packetConsumer);
      communicators.put(id, objectCommunicator);
   }

   public void detatchObjectCommunicator(int id)
   {
      communicators.remove(id);
      consumers.remove(id);
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
