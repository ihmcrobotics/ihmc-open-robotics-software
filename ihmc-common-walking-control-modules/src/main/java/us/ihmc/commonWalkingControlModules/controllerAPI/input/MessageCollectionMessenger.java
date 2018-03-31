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
 * Tool for setting up and sending collections of messages that are to be processed by the
 * controller at the same time.
 */
@SuppressWarnings("rawtypes")
public class MessageCollectionMessenger
{
   /**
    * Maximum duration in milliseconds to wait for a {@link MessageCollectionNotification} once a
    * {@link MessageCollection} has been sent.
    */
   private static final int NOTIFICATION_TIMEOUT = 5000;

   private long sequenceID;

   private PublishingTask publishingTask = null;

   private MessageCollection collection = new MessageCollection();
   private List<Packet<?>> packets = new ArrayList<>();
   private final Map<Class, Method> sequenceIDSetterMap = new HashMap<>();

   /**
    * Creates a new messenger ready to register a new collection of messages to be sent to the
    * controller.
    */
   public MessageCollectionMessenger()
   {
      this(new Random(1651).nextLong());
   }

   /**
    * Creates a new messenger ready to register a new collection of messages to be sent to the
    * controller.
    * 
    * @param startSequenceID the ID of the collection message and start ID for all the messages of
    *           the collection.
    */
   public MessageCollectionMessenger(long startSequenceID)
   {
      sequenceID = startSequenceID;
      collection.setSequenceId(sequenceID);
      incrementSequenceID();
   }

   /**
    * Convenience method for adding several messages.
    * 
    * @param messages the messages to add.
    * @see #addMessage(Packet)
    */
   public void addMessages(Iterable<Packet<?>> messages)
   {
      messages.forEach(this::addMessage);
   }

   /**
    * Convenience method for adding several messages.
    * 
    * @param messages the messages to add.
    * @see #addMessage(Packet)
    */
   public void addMessages(Packet<?>... messages)
   {
      for (Packet<?> message : messages)
      {
         addMessage(message);
      }
   }

   /**
    * Add a message to the current collection.
    * <p>
    * Only messages with a field "{@code long sequence_id_}" and a setter
    * "{@code setUniqueId(long)}" can be added to a collection.
    * </p>
    * 
    * @param message the message to add.
    * @return {@code true} if the message was properly added to the collection, {@code false}
    *         otherwise.
    */
   public boolean addMessage(Packet<?> message)
   {
      Class<? extends Packet> messageType = message.getClass();

      Method sequenceIDSetter = sequenceIDSetterMap.get(messageType);

      if (sequenceIDSetter == null)
      {
         try
         {
            sequenceIDSetter = messageType.getMethod("setSequenceId", long.class);
            sequenceIDSetterMap.put(messageType, sequenceIDSetter);
         }
         catch (NoSuchMethodException | SecurityException e)
         {
            PrintTools.error("The message type: " + messageType.getSimpleName()
                  + " cannot be sent with a MessageCollection, it needs to declare a sequenceID field.");
            return false;
         }
      }

      try
      {
         sequenceIDSetter.invoke(message, sequenceID);
      }
      catch (IllegalAccessException | IllegalArgumentException | InvocationTargetException e)
      {
         throw new RuntimeException("Something went wrong when setting the sequenceID for the message " + messageType.getSimpleName(), e);
      }

      packets.add(message);
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

   /**
    * Sends, in a safe manner, first a {@code MessageCollection} to indicate that the following
    * messages should be gathered and processed at the same time, and then the collection of
    * messages that have been added via {@link #addMessage(Packet)}.
    * <p>
    * This method is safer than {@link #sendMessageCollectionUnsafe(Messenger)} as before actually
    * sending the messages, it will wait for first receiving a
    * {@link MessageCollectionNotification}. The notification indicates that the controller is ready
    * to collect the collection of messages.
    * </p>
    * <p>
    * If the controller does not send a notification within a given duration, the task is cancelled
    * and the internal memory of {@code this} is cleared.
    * </p>
    * <p>
    * If this method is executed on the local thread, i.e. {@code runOnThread = false}, the
    * execution will be temporarily paused to wait for a notification. If this is undesirable,
    * running the task on a different thread, i.e. {@code runOnThread = true} will prevent this
    * effect.
    * </p>
    * 
    * @param packetCommunicator used to communicate with the controller.
    * @param runOnThread whether this method should be executed on a separate thread.
    * @see #NOTIFICATION_TIMEOUT
    */
   public void sendMessageCollectionSafe(PacketCommunicator packetCommunicator, boolean runOnThread)
   {
      PacketConsumer<MessageCollectionNotification> listener = this::receivedNotification;
      packetCommunicator.attachListener(MessageCollectionNotification.class, listener);

      sendMessageCollectionSafe(packetCommunicator::send, runOnThread, () -> packetCommunicator.detachListener(MessageCollectionNotification.class, listener));
   }

   /**
    * Sends, in a safe manner, first a {@code MessageCollection} to indicate that the following
    * messages should be gathered and processed at the same time, and then the collection of
    * messages that have been added via {@link #addMessage(Packet)}.
    * <p>
    * When using this method, the user should register the method
    * {@link #receivedNotification(MessageCollectionNotification)} as a listener to the
    * communication, so {@code this} can receive the notification from the controller.
    * </p>
    * 
    * @param messenger the protocol for sending messages to the controller.
    * @param runOnThread whether this method should be executed on a separate thread.
    * @see #sendMessageCollectionSafe(PacketCommunicator, boolean)
    */
   public void sendMessageCollectionSafe(Messenger messenger, boolean runOnThread)
   {
      sendMessageCollectionSafe(messenger, runOnThread, null);
   }

   private void sendMessageCollectionSafe(Messenger messenger, boolean runOnThread, Runnable postProcess)
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

   /**
    * Sends immediately a {@code MessageCollection} directly followed by the collection of messages
    * to the controller.
    * <p>
    * This method strongly rely on the reliability of the communication and expect the
    * {@code MessageCollection} to arrive first. Prefer using
    * {@link #sendMessageCollectionSafe(PacketCommunicator, boolean)}.
    * </p>
    * 
    * @param messenger the protocol for sending messages to the controller.
    */
   public void sendMessageCollectionUnsafe(Messenger messenger)
   {
      messenger.sendMessage(collection);

      for (int i = 0; i < packets.size(); i++)
      {
         messenger.sendMessage(packets.get(i));
      }
      reset();
   }

   /**
    * Notifies {@code this} that the controller just received a {@link MessageCollection}.
    * <p>
    * This method triggers the publishing of the collection of messages when using
    * {@link #sendMessageCollectionSafe(Messenger, boolean)}.
    * </p>
    * 
    * @param notification the notification.
    */
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

   private void reset()
   {
      collection.getSequences().reset();
      incrementSequenceID();
      collection.setSequenceId(sequenceID);
      incrementSequenceID();
      packets.clear();
   }

   /**
    * Implements this interface to provide a protocol for sending messages.
    * 
    * @author Sylvain Bertrand
    */
   public static interface Messenger
   {
      /**
       * Request for sending the given message.
       * 
       * @param message the message to be sent.
       */
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
               notificationSync.wait(NOTIFICATION_TIMEOUT);
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
            reset();
            if (postProcess != null)
               postProcess.run();
         }
      }
   }
}
