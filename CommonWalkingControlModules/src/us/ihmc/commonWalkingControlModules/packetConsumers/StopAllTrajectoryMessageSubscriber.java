package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.atomic.AtomicBoolean;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.StopAllTrajectoryMessage;

public class StopAllTrajectoryMessageSubscriber implements PacketConsumer<StopAllTrajectoryMessage>
{
   private final List<Object> listeners = new ArrayList<>();
   private final Map<Object, AtomicBoolean> latestMessageReferences = new HashMap<>(10);

   public StopAllTrajectoryMessageSubscriber()
   {

   }

   public void registerNewListener(Object newListener)
   {
      if (latestMessageReferences.containsKey(newListener))
         throw new RuntimeException("Subscriber already resgistered: " + newListener.getClass().getSimpleName());
      listeners.add(newListener);
      latestMessageReferences.put(newListener, new AtomicBoolean(false));
   }

   public boolean pollMessage(Object listener)
   {
      return latestMessageReferences.get(listener).getAndSet(false);
   }

   public void clearMessagesInQueue()
   {
      for (int i = 0; i < listeners.size(); i++)
      {
         Object listener = listeners.get(i);
         latestMessageReferences.get(listener).set(false);
      }
   }

   @Override
   public void receivedPacket(StopAllTrajectoryMessage stopAllTrajectoryMessage)
   {
      if (stopAllTrajectoryMessage == null)
         return;

      for (int i = 0; i < listeners.size(); i++)
      {
         Object listener = listeners.get(i);
         latestMessageReferences.get(listener).set(true);
      }
   }
}
