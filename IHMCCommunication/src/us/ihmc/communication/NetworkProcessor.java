package us.ihmc.communication;

import gnu.trove.iterator.TIntIntIterator;
import gnu.trove.map.hash.TIntIntHashMap;
import gnu.trove.map.hash.TIntObjectHashMap;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.packets.Packet;

public class NetworkProcessor
{
   private boolean DEBUG = false;
   private int sourceCommunicatorIdToDebug = Integer.MIN_VALUE; //set to Integer.MIN_VALUE to debug all sources
   private int destinationCommunicatorIdToDebug = Integer.MIN_VALUE;//set to Integer.MIN_VALUE to debug all destinations
   private Class packetTypeToDebug = null;//set to null to debug all packets
   
   private final int BROADCAST = 0;
   private final TIntObjectHashMap<PacketCommunicator> communicators = new TIntObjectHashMap<PacketCommunicator>();
   private final TIntObjectHashMap<PacketConsumer<Packet>> consumers = new TIntObjectHashMap<PacketConsumer<Packet>>();
   private final TIntIntHashMap redirects = new TIntIntHashMap();

   public NetworkProcessor()
   {
   }

   public void attachPacketCommunicator(final PacketCommunicator packetCommunicator)
   {
      if(packetCommunicator.getId() == BROADCAST)
      {
         throw new IllegalArgumentException("packetCommunicator cannot have an id of zero hommie, it's reserved for broadcast!!");
      }
      
      if(DEBUG)
      {
         System.out.println(getClass().getSimpleName() + " Attaching " + packetCommunicator.getName() + ":" + packetCommunicator.getId() + " to the network processor");
      }
      PacketConsumer<Packet> packetConsumer = new PacketConsumer<Packet>()
      {
         PacketCommunicator communicator = packetCommunicator;
         
         @Override
         public void receivedPacket(Packet packet)
         {
            if(shouldPrintDebugStatement(communicator.getId(), packet.getDestination(), packet.getClass()))
            {
                  System.out.println(getClass().getSimpleName() + " NP received " + packet.getClass().getSimpleName() + " heading for " + packet.getDestination() + " from " + communicator.getName());
            }
            processPacket(communicator,packet);
         }
      };

      packetCommunicator.attacthGlobalSendListener(packetConsumer);
      
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
         if(shouldPrintDebugStatement(source.getId(), destination, packet.getClass()))
         {
            System.out.println("Sending " + packet.getClass().getSimpleName() + " from " + source.getName() + " to " + destinationCommunicator.getName());
         }
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
               if(shouldPrintDebugStatement(source.getId(), destinationCommunicator.getId(), packet.getClass()))
               {
                  System.out.println("Sending " + packet.getClass().getSimpleName() + " from " + source.getName() + " to " + destinationCommunicator.getName());
               }
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
      
      communicator.detachGlobalSendListener(consumer);
      
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
   
   private boolean shouldPrintDebugStatement(int sourceCommunicatorId, int destinationCommunicatorId, Class packetType)
   {
      if(!DEBUG)
      {
         return false;
      }
      
      if(sourceCommunicatorIdToDebug != Integer.MIN_VALUE && sourceCommunicatorIdToDebug != sourceCommunicatorId)
      {
         return false;
      }
      
      if(destinationCommunicatorIdToDebug != Integer.MIN_VALUE && destinationCommunicatorIdToDebug != destinationCommunicatorId)
      {
         return false;
      }
      
      if(packetTypeToDebug != null && packetTypeToDebug != packetType)
      {
         return false;
      }
      
      return true;
   }
   
   
   //Put these here so they aty not final, I can change these debug variables at runtime
   public void setDEBUG(boolean debug)
   {
      DEBUG = debug;
   }

   public void setSourceCommunicatorIdToDebug(int sourceCommunicatorIdToDebug)
   {
      this.sourceCommunicatorIdToDebug = sourceCommunicatorIdToDebug;
   }

   public void setDestinationCommunicatorIdToDebug(int destinationCommunicatorIdToDebug)
   {
      this.destinationCommunicatorIdToDebug = destinationCommunicatorIdToDebug;
   }
   
   public void setPacketTypeToDebug(Class packetTypeToDebug)
   {
      this.packetTypeToDebug = packetTypeToDebug;
   }
}
