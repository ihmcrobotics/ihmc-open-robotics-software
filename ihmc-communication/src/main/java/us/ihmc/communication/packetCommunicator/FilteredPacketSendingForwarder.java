package us.ihmc.communication.packetCommunicator;

import java.util.HashMap;
import java.util.HashSet;

import us.ihmc.communication.net.AtomicSettableTimestampProvider;
import us.ihmc.communication.packetCommunicator.interfaces.GlobalPacketConsumer;
import us.ihmc.communication.packets.Packet;

/**
 * SelectivePacketForwarder connects two packetCommunicators send functions with filtering
 * If you send from the source it will filter the packet then call send on the destination
 * communicator.
 */
public class FilteredPacketSendingForwarder implements GlobalPacketConsumer
{
   private static final boolean DEBUG = false;
   
   private final PacketCommunicator communicatorToForwardFrom;
   private final PacketCommunicator communicatorToForwardTo;
   private final HashMap<Class, TimedElapsedChecker> inclusiveClassesWithElapsedTimeTriggers = new HashMap<Class, FilteredPacketSendingForwarder.TimedElapsedChecker>();
   private final HashSet<Class> classesToInclude = new HashSet<Class>();
   private final HashSet<Class> classesToExclude = new HashSet<Class>();
   private enum ForwarderMode { INCLUSIVE, EXCLUSIVE, DISABLED };
   private final AtomicSettableTimestampProvider timestampProvider;
   private ForwarderMode mode;

   
   public FilteredPacketSendingForwarder(PacketCommunicator communicatorToForwardFrom, PacketCommunicator communicatorToForwardTo, AtomicSettableTimestampProvider timestampProvider)
   {
      this.communicatorToForwardFrom = communicatorToForwardFrom;
      this.communicatorToForwardTo = communicatorToForwardTo;
      communicatorToForwardFrom.attachGlobalListener(this);
      this.timestampProvider = timestampProvider;
   }

   /**
    * Will forward all the packets except those types defined in clazzesToExclude
    * @param clazzesToExclude
    */
   public void enableExclusiveForwarding(Class[] clazzesToExclude)
   {
      mode = ForwarderMode.EXCLUSIVE;
      for(int i = 0; i < clazzesToExclude.length; i++)
      {
         classesToExclude.add(clazzesToExclude[i]);
         if (DEBUG)
         {
            System.out.println(getClass().getSimpleName() + ": excluding " + clazzesToExclude[i].getSimpleName() + " forwarding from " + communicatorToForwardFrom + " to " + communicatorToForwardTo);
         }
      }
   }
   
   /**
    * Will forward the packets with types defined in clazzesToInclude
    * @param clazzesToInclude
    */
   public void enableInclusiveForwarding(Class[] clazzesToInclude)
   {
      mode = ForwarderMode.INCLUSIVE;
      for(int i = 0; i < clazzesToInclude.length; i++)
      {
         classesToInclude.add(clazzesToInclude[i]);
         if (DEBUG)
         {
            System.out.println(getClass().getSimpleName() + ": including " + clazzesToInclude[i].getSimpleName() + " forwarding from " + communicatorToForwardFrom + " to " + communicatorToForwardTo);
         }
      }
   }
   
   /**
    * Forwards packets of type clazz with minimum intervals between sends
    * This will forward a packet if minimumTimeBetweenForwardInNanos has elapsed
    * since the last forwarded packet of type clazz
    * @param clazz
    * @param minimumTimeBetweenForwardInNanos
    * @throws IllegalArgumentException If the class type was already added to the inclusive not intervaled list 
    */
   public void enableInclusiveForwardingWithMinimumRobotTimeIntervals(Class clazz, long minimumTimeBetweenForwardInNanos)
   {
      if(classesToInclude.contains(clazz))
      {
         throw new IllegalArgumentException(clazz.getSimpleName() + " was already added to the inclusive set. It should only be included in the inclusive or the timeElapsed inclusive. Not both hommie");
      }
      inclusiveClassesWithElapsedTimeTriggers.put(clazz, new TimedElapsedChecker(minimumTimeBetweenForwardInNanos));
      if (DEBUG)
      {
         System.out.println(getClass().getSimpleName() + ": including " + clazz.getSimpleName() + " " + minimumTimeBetweenForwardInNanos + " nanoseconds timelapsed forwarding from " + communicatorToForwardFrom + " to " + communicatorToForwardTo);
      }
   }
   
   public void disableForwarding()
   {
      mode = ForwarderMode.DISABLED;
      if (DEBUG)
      {
         System.out.println(getClass().getSimpleName() + ": disabled forwarder" + communicatorToForwardFrom);
      }
   }

   @Override
   public void receivedPacket(Packet packet)
   {
      switch(mode)
      {
         case EXCLUSIVE:
            exclusiveForward(packet);
            break;
         case INCLUSIVE:
            inclusiveForward(packet);
            break;
         case DISABLED:
            break;
      }
   }

   private void inclusiveForward(Packet packet)
   {
      if(classesToInclude.contains(packet.getClass()))
      {
         if (DEBUG)
         {
            System.out.println(getClass().getSimpleName() + ": forwarding " +packet.getClass() + " from " + communicatorToForwardFrom + " to " + communicatorToForwardTo);
         }
         communicatorToForwardTo.send(packet);
         return;
      }
      
      if(inclusiveClassesWithElapsedTimeTriggers.containsKey(packet.getClass()))
      {
         if(inclusiveClassesWithElapsedTimeTriggers.get(packet.getClass()).getAndResetTimeElapsedIfElapsed())
         {
            if (DEBUG)
            {
               System.out.println(getClass().getSimpleName() + ": time lapsed forwarding " + packet.getClass() + " from " + communicatorToForwardFrom + " to " + communicatorToForwardTo);
            }
            communicatorToForwardTo.send(packet);
         }
         return;
      }
      
      if (DEBUG)
      {
         System.out.println(getClass().getSimpleName() + ": excluded on inclusive send " + packet.getClass() + " from " + communicatorToForwardFrom + " to " + communicatorToForwardTo);
      }
   }

   private void exclusiveForward(Packet packet)
   {
      if(!classesToExclude.contains(packet.getClass()))
      {
         if (DEBUG)
         {
            System.out.println(getClass().getSimpleName() + ": forwarding " +packet.getClass() + " from " + communicatorToForwardFrom + " to " + communicatorToForwardTo);
         }
         communicatorToForwardTo.send(packet);
      }
   }
   
   //Could be a problem with System.nanoTime when switching between cores :(
   private class TimedElapsedChecker
   {
      private long startTime;
      private final long triggerTimeInNanoSeconds;
      
      public TimedElapsedChecker(long timeInMilliSeconds)
      {
         startTime = timestampProvider.getTimestamp();
         triggerTimeInNanoSeconds = timeInMilliSeconds * 1000000L;
      }
      
      public boolean getAndResetTimeElapsedIfElapsed()
      {
         long currentTime = timestampProvider.getTimestamp();
         if(currentTime - startTime >= triggerTimeInNanoSeconds)
         {
            startTime = currentTime;
            return true;
         }
         
         return false;
     }
   }

   public void enableUnfilteredForwarding()
   {
      mode = ForwarderMode.EXCLUSIVE;
   }

}
