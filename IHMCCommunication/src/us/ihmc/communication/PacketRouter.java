package us.ihmc.communication;

import gnu.trove.iterator.TIntIntIterator;
import gnu.trove.map.hash.TIntIntHashMap;
import gnu.trove.map.hash.TIntObjectHashMap;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.KryoPacketClientEndPointCommunicator;
import us.ihmc.communication.packetCommunicator.KryoPacketServer;
import us.ihmc.communication.packetCommunicator.interfaces.PacketCommunicator;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.packets.wholebody.WholeBodyTrajectoryDevelopmentPacket;
import us.ihmc.communication.packets.wholebody.WholeBodyTrajectoryPacket;

public class PacketRouter
{
   private boolean DEBUG = false;
   private int sourceCommunicatorIdToDebug = Integer.MIN_VALUE; //set to Integer.MIN_VALUE to debug all sources
   private int destinationCommunicatorIdToDebug = Integer.MIN_VALUE;//set to Integer.MIN_VALUE to debug all destinations
   private Class[] packetTypesToDebug =null;//set to null to debug all packets
   
   private final int BROADCAST = 0;
   private enum RouteReceieveType {ON_SEND, ON_RECEIVE};
   private final TIntObjectHashMap<RouteReceieveType> communicatorRouteTypes = new TIntObjectHashMap<RouteReceieveType>();
   private final TIntObjectHashMap<PacketCommunicator> communicators = new TIntObjectHashMap<PacketCommunicator>();
   private final TIntObjectHashMap<PacketConsumer<Packet>> consumers = new TIntObjectHashMap<PacketConsumer<Packet>>();
   private final TIntIntHashMap redirects = new TIntIntHashMap();
   
   public PacketRouter()
   {
      if(DEBUG)
      {
         System.out.println("Creating Packet Router");
      }
   }

   public void attachPacketCommunicator(final PacketCommunicator packetCommunicator)
   {
      checkCommunicatorId(packetCommunicator);
      RouteReceieveType routeType = getCommunicatorRouteType(packetCommunicator);
      int id = packetCommunicator.getId();
      
      PacketConsumer<Packet> packetRoutingAction = new packetRoutingAction(packetCommunicator);
      
      if(routeType == RouteReceieveType.ON_RECEIVE)
      {
         packetCommunicator.attacthGlobalReceiveListener(packetRoutingAction);
      } 
      else 
      {
         packetCommunicator.attacthGlobalSendListener(packetRoutingAction);
      }
      
      consumers.put(id, packetRoutingAction);
      communicators.put(id, packetCommunicator);
      communicatorRouteTypes.put(id, routeType);
      
      if(DEBUG)
      {
         System.out.println(getClass().getSimpleName() + " Attached " + packetCommunicator.getName() + ":" + packetCommunicator.getId() + " to the network processor");
      }
   }

   private RouteReceieveType getCommunicatorRouteType(final PacketCommunicator packetCommunicator)
   {
      RouteReceieveType routeType = RouteReceieveType.ON_SEND;
      if(packetCommunicator.getClass() == KryoPacketClientEndPointCommunicator.class || packetCommunicator.getClass() == KryoPacketServer.class)
      {
         routeType = RouteReceieveType.ON_RECEIVE;
      }
      return routeType;
   }
   
   private void checkCommunicatorId(final PacketCommunicator packetCommunicator)
   {
      int communicatorId = packetCommunicator.getId();
      
      if(communicatorId == BROADCAST)
      {
         throw new IllegalArgumentException("packetCommunicator cannot have an id of zero, it's reserved for broadcast!!");
      }
      if(communicators.containsKey(communicatorId))
      {
         throw new IllegalArgumentException("Tried to register " + packetCommunicator.getName() + " with id of " + communicatorId + " but " + communicators.get(communicatorId).getName() + " already registered with that id!");
      }
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
   private void processPacketRouting(PacketCommunicator source, Packet packet)
   {
      if(shouldPrintDebugStatement(source.getId(), packet.getDestination(), packet.getClass()))
      {
         System.out.println(getClass().getSimpleName() + " NP received " + packet.getClass().getSimpleName() + " heading for " + packet.getDestination() + " from " + source.getName());
      }
      
      int destination = getPacketDestination(source, packet);
      
      PacketCommunicator destinationCommunicator = communicators.get(destination);
      if (destinationCommunicator != null && destinationCommunicator.isConnected())
      {
         if(shouldPrintDebugStatement(source.getId(), destination, packet.getClass()))
         {
            System.out.println("Sending " + packet.getClass().getSimpleName() + " from " + source.getName() + " to " + destinationCommunicator.getName());
         }
         
         forwardPacket(packet, destinationCommunicator);
      }
      
      if(destination == BROADCAST)
      {
         broadcastPacket(source, packet);
      }
   }

   private void forwardPacket(Packet packet, PacketCommunicator destinationCommunicator)
   {
      //send and recv swapped on forward
      if(communicatorRouteTypes.get(destinationCommunicator.getId()) == RouteReceieveType.ON_SEND)
      {
         destinationCommunicator.receivedPacket(packet);
      } 
      else 
      {
         destinationCommunicator.send(packet);
      }
   }
   
   private int getPacketDestination(PacketCommunicator source, Packet packet)
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
      return destination;
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
               forwardPacket(packet, destinationCommunicator);
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
      
      if(packetTypesToDebug != null )
      {
         for (int i=0; i< packetTypesToDebug.length; i++)
         {
            if(packetTypesToDebug[i] == packetType) {
               return true;
            }
         }
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
      this.packetTypesToDebug = new Class[]{ packetTypeToDebug };
   }
   
   public void setPacketTypesToDebug(Class[] packetTypesToDebug)
   {
      this.packetTypesToDebug = packetTypesToDebug;
   }

   
   private class packetRoutingAction implements PacketConsumer<Packet>
   {
      PacketCommunicator communicator;
      public packetRoutingAction(PacketCommunicator packetCommunicator)
      {
         this.communicator = packetCommunicator;
      }
      
      @Override
      public void receivedPacket(Packet packet)
      {
         processPacketRouting(communicator, packet);
      }
   }
}
