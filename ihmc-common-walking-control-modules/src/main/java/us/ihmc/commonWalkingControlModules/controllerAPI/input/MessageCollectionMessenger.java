package us.ihmc.commonWalkingControlModules.controllerAPI.input;

import java.lang.reflect.InvocationTargetException;
import java.lang.reflect.Method;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import controller_msgs.msg.dds.MessageCollection;
import controller_msgs.msg.dds.MessageCollectionNotification;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.packets.Packet;

/**
 * MessageOfMessages provides a generic way to send a collection of messages to the controller.
 */
@SuppressWarnings("rawtypes")
public class MessageCollectionMessenger
{
   private long sequenceID;

   private PublishingTask publishingTask = null;

   private MessageCollection collection = new MessageCollection();
   private List<Packet<?>> packets = new ArrayList<>();
   private final Map<Class, Method> sequenceIDSetterMap = new HashMap<>();

   public MessageCollectionMessenger()
   {
      this(new Random(1651).nextLong());
   }

   public MessageCollectionMessenger(long startSequenceID)
   {
      sequenceID = startSequenceID;
      collection.setSequenceId(sequenceID);
      incrementSequenceID();
   }

   public boolean addMessage(Packet<?> packet)
   {
      Class<? extends Packet> packetType = packet.getClass();

      Method sequenceIDSetter = sequenceIDSetterMap.get(packetType);

      if (sequenceIDSetter == null)
      {
         try
         {
            sequenceIDSetter = packetType.getMethod("setSequenceId", long.class);
            sequenceIDSetterMap.put(packetType, sequenceIDSetter);
         }
         catch (NoSuchMethodException | SecurityException e)
         {
            PrintTools.error("The message type: " + packetType.getSimpleName()
                  + " cannot be sent with a MessageCollection, it needs to declare a sequenceID field.");
            return false;
         }
      }

      try
      {
         sequenceIDSetter.invoke(packet, sequenceID);
      }
      catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e)
      {
         throw new RuntimeException("Something went wrong when setting the sequenceID for the message " + packetType.getSimpleName(), e);
      }

      packets.add(packet);
      collection.getSequences().add(sequenceID);

      incrementSequenceID();

      return true;
   }

   private void incrementSequenceID()
   {
      sequenceID++;

      if (sequenceID == 0)
         sequenceID++;
   }

   public void addMessages(Iterable<Packet<?>> messages)
   {
      for (Packet<?> packet : messages)
      {
         addMessage(packet);
      }
   }

   public void addMessages(Packet<?>... messages)
   {
      for (Packet<?> packet : messages)
      {
         addMessage(packet);
      }
   }

   public void sendMessageCollectionSafe(PacketCommunicator packetCommunicator, boolean runOnThread)
   {
      PacketConsumer<MessageCollectionNotification> listener = this::receivedNotification;
      packetCommunicator.attachListener(MessageCollectionNotification.class, listener);

      sendMessageCollectionSafe(packetCommunicator::send, runOnThread, () -> packetCommunicator.detachListener(MessageCollectionNotification.class, listener));
   }

   public void sendMessageCollectionSafe(Messenger messenger, boolean runOnThread)
   {
      sendMessageCollectionSafe(messenger, runOnThread, null);
   }

   public void sendMessageCollectionSafe(Messenger messenger, boolean runOnThread, Runnable postProcess)
   {
      publishingTask = new PublishingTask(messenger, collection, packets, postProcess);

      if (runOnThread)
      {
         Thread thread = new Thread(publishingTask, getClass().getSimpleName());
         thread.setDaemon(true);
         thread.start();
      }
      else
      {
         publishingTask.run();
      }
   }

   public void sendMessageCollectionUnsafe(Messenger messenger)
   {
      messenger.sendMessage(collection);

      for (int i = 0; i < packets.size(); i++)
      {
         messenger.sendMessage(packets.get(i));
      }
      collection.getSequences().reset();
      packets.clear();
   }

   public void receivedNotification(MessageCollectionNotification notification)
   {
      if (collection == null)
         return;

      if (notification.getMessageCollectionSequenceId() == collection.getSequenceId() && publishingTask != null)
      {
         synchronized (publishingTask.notificationSync)
         {
            publishingTask.hasReceivedNotification = true;
            publishingTask.notificationSync.notify();
         }
      }
   }

   public static interface Messenger
   {
      void sendMessage(Packet<?> message);
   }

   private class PublishingTask implements Runnable
   {
      private volatile boolean hasReceivedNotification = false;
      private final Object notificationSync = new Object();
      private final Messenger messenger;
      private final MessageCollection collection;
      private final List<Packet<?>> packets;
      private final Runnable postProcess;

      public PublishingTask(Messenger messenger, MessageCollection collection, List<Packet<?>> packets, Runnable postProcess)
      {
         this.messenger = messenger;
         this.collection = collection;
         this.packets = packets;
         this.postProcess = postProcess;
      }

      @Override
      public void run()
      {
         synchronized (notificationSync)
         {
            messenger.sendMessage(collection);

            try
            {
               notificationSync.wait(5000);
            }
            catch (InterruptedException e)
            {
               e.printStackTrace();
               collection.getSequences().reset();
               packets.clear();
               return;
            }

            if (hasReceivedNotification)
            {
               for (int i = 0; i < packets.size(); i++)
               {
                  messenger.sendMessage(packets.get(i));
               }
            }
            else
            {
               PrintTools.error(getClass().getSimpleName() + " did not receive " + MessageCollectionNotification.class.getSimpleName()
                     + ". Clearing the internal messages.");
            }

            hasReceivedNotification = false;
            collection.getSequences().reset();
            packets.clear();
            if (postProcess != null)
               postProcess.run();
         }
      }
   }
}
