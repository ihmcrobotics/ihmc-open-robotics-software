package us.ihmc.communication;

import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packetCommunicator.interfaces.GlobalPacketConsumer;
import us.ihmc.communication.packets.Packet;
import us.ihmc.tools.io.printing.PrintTools;

import java.util.EnumMap;
import java.util.HashMap;

public class PacketRouter<T extends Enum<T>> 
{
   private boolean DEBUG = false;

   private final T[] destinationConstants;
   private int sourceCommunicatorIdToDebug = Integer.MIN_VALUE; //set to Integer.MIN_VALUE to debug all sources
   private int destinationCommunicatorIdToDebug = Integer.MIN_VALUE; //set to Integer.MIN_VALUE to debug all destinations
   private String[] packetTypesToDebug = null; //set to null to debug all packets
   
   private final int BROADCAST = 0;

   private final EnumMap<T, PacketCommunicator> communicators;
   private final HashMap<PacketCommunicator, T> communicatorDestinations;

   private final EnumMap<T, GlobalPacketConsumer> consumers;
   private final EnumMap<T, T> redirects;
   
   public PacketRouter(Class<T> destinationType)
   {
      destinationConstants = destinationType.getEnumConstants();
      communicators = new EnumMap<>(destinationType);
      consumers = new EnumMap<>(destinationType);
      communicatorDestinations = new HashMap<>();
      redirects = new EnumMap<>(destinationType);
      
      if (DEBUG)
         PrintTools.debug(this, "Creating Packet Router");
   }

   public void attachPacketCommunicator(T destination, final PacketCommunicator packetCommunicator)
   {
      checkCommunicatorId(destination);

      // check if this communicator already points to the other destination
      GlobalPacketConsumer packetRoutingAction = null;
      T otherDestination = communicatorDestinations.get(packetCommunicator);

      if (otherDestination != null)
      {
         packetRoutingAction = consumers.get(otherDestination);
         // this listener is already attached to that communicator
      }
      else
      {
         packetRoutingAction = new PacketRoutingAction(packetCommunicator);
         packetCommunicator.attachGlobalListener(packetRoutingAction);
      }
      
      consumers.put(destination, packetRoutingAction);
      communicators.put(destination, packetCommunicator);
      communicatorDestinations.put(packetCommunicator, destination);
      
      if (DEBUG)
         PrintTools.debug(this, "Attached " + destination + " to the network processor");
   }
   
   private void checkCommunicatorId(final T destination)
   {
      if(isBroadcast(destination))
      {
         throw new IllegalArgumentException("packetCommunicator cannot have an id of zero, it's reserved for broadcast!!");
      }
      if(communicators.containsKey(destination))
      {
         throw new IllegalArgumentException("Tried to register " + destination + " but already registerd communicator with that id!");
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
   private void processPacketRouting(PacketCommunicator source, Packet<?> packet)
   {
      if (shouldPrintDebugStatement(source, packet.getDestination(), packet.getClass()))
      {
         PrintTools.debug(this, "NP received " + packet.getClass().getSimpleName() + " heading for " + destinationConstants[packet.destination] + " from " + communicatorDestinations.get(source));
      }
      
      T destination = getPacketDestination(source, packet);
      
      PacketCommunicator destinationCommunicator = communicators.get(destination);
      if(isBroadcast(destination))
      {
         broadcastPacket(source, packet);
      }
      else if (destinationCommunicator != null && destinationCommunicator.isConnected())
      {
         if(shouldPrintDebugStatement(source, destination.ordinal(), packet.getClass()))
         {
            PrintTools.debug(this, "Sending " + packet.getClass().getSimpleName() + " from " + communicatorDestinations.get(source) + " to " + destination);
         }
         
         forwardPacket(packet, destinationCommunicator);
      }
      
   }

   private boolean isBroadcast(T destination)
   {
      return destination.ordinal() == BROADCAST;
   }

   private void forwardPacket(Packet<?> packet, PacketCommunicator destinationCommunicator)
   {
      destinationCommunicator.send(packet);
   }
   
   private T getPacketDestination(PacketCommunicator source, Packet<?> packet)
   {
      if(packet.getDestination() < 0 || packet.getDestination() >= destinationConstants.length)
      {
         System.err.println("Invalid destination: " + packet.getDestination() + " sending to Broadcast");
         return destinationConstants[0];
      }
      
      T destination = destinationConstants[packet.getDestination()]; 
      if (redirects.containsKey(destination))
      {
         destination = getRedirectDestination(destination);
         if(destination != communicatorDestinations.get(source))
         {
            packet.setDestination(destination.ordinal());
         }
      }
      return destinationConstants[packet.getDestination()];
   }
   
   /**
    * sends the packet to every communicator once, except the sender or any
    * communicators with redirects
   **/
   private void broadcastPacket(PacketCommunicator source, Packet<?> packet)
   {
      for(T destination : destinationConstants)
      {
         if(isBroadcast(destination) || !communicators.containsKey(destination))
         {
            continue;
         }
         
         PacketCommunicator destinationCommunicator = communicators.get(destination);
         if(source != destinationCommunicator && !redirects.containsKey(destination))
         {
            if (destinationCommunicator != null && destinationCommunicator.isConnected())
            {
               if(shouldPrintDebugStatement(source, destination.ordinal(), packet.getClass()))
               {
                  PrintTools.debug(this, "Sending " + packet.getClass().getSimpleName() + " from " + communicatorDestinations.get(source) + " to " + destination);
               }
               forwardPacket(packet, destinationCommunicator);
            }
         }
      }
   }
   
   /**
    * possible infinite recursion for funs.
    */
   private T getRedirectDestination(T destination)
   {
      if (redirects.containsKey(destination))
      {
         destination = getRedirectDestination(redirects.get(destination));
      }
      return destination;
   }

   public void detatchObjectCommunicator(T id)
   {
      PacketCommunicator communicator = communicators.remove(id);
      GlobalPacketConsumer consumer = consumers.remove(id);
      
      if(communicator != null)
      {
         communicator.detachGlobalListener(consumer);
      }
      
      redirects.remove(id);

      for(T destination : destinationConstants)
      {
         if(redirects.get(destination) == id)
         {
            redirects.remove(destination);
         }
      }
   }

   public void setPacketRedirects(T redirectFrom, T redirectTo)
   {
      if(redirects.containsValue(redirectFrom))
      {
         throw new IllegalArgumentException(redirectTo + " is currently redirecting and can't be chained");
      }
      redirects.put(redirectFrom, redirectTo);
   }

   public void removePacketRedirect(int redirectFrom)
   {
      redirects.remove(redirectFrom);
   }
   
   private boolean shouldPrintDebugStatement(PacketCommunicator source, int destinationCommunicatorId, Class<?> packetType)
   {
      if (!DEBUG)
         return false;
      
      if(sourceCommunicatorIdToDebug != Integer.MIN_VALUE && sourceCommunicatorIdToDebug != communicatorDestinations.get(source).ordinal())
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
            if(packetTypesToDebug[i].equals(packetType.getSimpleName())) {
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
   
   public void setPacketTypeToDebug(Class<?> packetTypeToDebug)
   {
      this.packetTypesToDebug = new String[]{ packetTypeToDebug.getSimpleName() };
   }
   
//   public void setPacketTypesToDebug(Class<?>[] packetTypesToDebug)
//   {
//      this.packetTypesToDebug = packetTypesToDebug;
//   }
   
   private class PacketRoutingAction implements GlobalPacketConsumer
   {
      private final PacketCommunicator communicator;
      private PacketRoutingAction(PacketCommunicator packetCommunicator)
      {
         this.communicator = packetCommunicator;
      }
      
      @Override
      public void receivedPacket(Packet<?> packet)
      {
         processPacketRouting(communicator, packet);
      }
   }
}
