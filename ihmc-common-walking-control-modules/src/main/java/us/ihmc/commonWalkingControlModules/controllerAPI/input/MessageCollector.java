package us.ihmc.commonWalkingControlModules.controllerAPI.input;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import controller_msgs.msg.dds.MessageCollection;
import controller_msgs.msg.dds.MessageCollectionNotification;
import gnu.trove.set.hash.TLongHashSet;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.idl.IDLSequence;

/**
 * A {@code MessageCollector} can be added to a {@link ControllerNetworkSubscriber}.
 * <p>
 * It works with {@link MessageCollection} and allows the user to ensure that messages sent in group
 * are also processed in group.
 * </p>
 * <p>
 * See {@link MessageCollectionMessenger} for setting up and sending a collection of messages that
 * are to be processed together.
 * </p>
 * 
 * @author Sylvain Bertrand
 */
public class MessageCollector
{
   /** Whether this collector is currently waiting for additional messages of a collection. */
   private boolean isCollecting = false;
   /**
    * The ID set of the messages that are part of a collection. This set is empty when this
    * collector is not active, when active the set contains the ID of the messages that are still to
    * be collected.
    */
   private final TLongHashSet expectedMessageIDs;
   /** The list of the collection messages that have already been collected. */
   private final List<Settable<?>> interceptedMessages;
   /** A view list of {@link #interceptedMessages}, it is for read-only purposes. */
   private final List<Settable<?>> interceptedMessagesView;
   /**
    * The internal buffer used to save a copy of the intercepted messages without holding onto the
    * reference of a message that was created outside this collector.
    */
   private final MessagePool messagePool;
   /** Extractor used to get the ID of the messages. */
   private final MessageIDExtractor messageIDExtractor;
   /**
    * Notification that sent to notify the user that this collector has just received a
    * {@code MessageCollection} and is now active and waiting to collect the messages.
    */
   private final MessageCollectionNotification notification;

   /**
    * Dummy collector that is never active, for convenience purposes.
    * 
    * @return the dummy collector.
    */
   public static MessageCollector createDummyCollector()
   {
      return new MessageCollector();
   }

   private MessageCollector()
   {
      messageIDExtractor = null;
      messagePool = null;
      expectedMessageIDs = null;
      interceptedMessages = null;
      interceptedMessagesView = Collections.emptyList();
      notification = null;
   }

   /**
    * Creates a message collector.
    * <p>
    * A message collector is necessary to support {@code MessageCollection}.
    * </p>
    * 
    * @param messageIDExtractor provides the method for getting the IDs from the messages.
    * @param supportedMessages the list of messages that this collector can handle.
    */
   public MessageCollector(MessageIDExtractor messageIDExtractor, List<Class<? extends Settable<?>>> supportedMessages)
   {
      this.messageIDExtractor = messageIDExtractor;
      messagePool = new MessagePool(supportedMessages);
      expectedMessageIDs = new TLongHashSet();
      interceptedMessages = new ArrayList<>();
      interceptedMessagesView = Collections.unmodifiableList(interceptedMessages);
      notification = new MessageCollectionNotification();
   }

   /**
    * Disables this collector and marks its internal memory as available.
    * <p>
    * Should be called only once the intercepted messages have been processed.
    * </p>
    */
   public void reset()
   {
      if (messagePool == null)
         return;

      isCollecting = false;
      expectedMessageIDs.clear();
      interceptedMessages.clear();
      messagePool.reset();
   }

   /**
    * Enables this collector.
    * <p>
    * This collector is now waiting for the messages referred by the IDs contained in the given
    * {@code collection}.
    * </p>
    * 
    * @param collection the message holding onto the IDs of the messages to collect.
    * @return a notification ready to be sent back to the user.
    */
   public MessageCollectionNotification startCollecting(MessageCollection collection)
   {
      if (messagePool == null)
      {
         PrintTools.error("This is a dummy collector, it cannot collect messages.");
         return null;
      }
      reset();
      isCollecting = true;
      notification.setMessageCollectionSequenceId(collection.getSequenceId());

      IDLSequence.Long sequences = collection.getSequences();
      for (int i = 0; i < sequences.size(); i++)
      {
         expectedMessageIDs.add(sequences.get(i));
      }
      return notification;
   }

   /**
    * Tests the given message and hold onto it if this collector is active and was waiting for the
    * given message.
    * <p>
    * If this collector is inactive or not waiting for the given message, the message is not saved
    * and this method returns {@code false}.
    * </p>
    * 
    * @param message the message to possibly intercept.
    * @return {@code true} if the message is part of a collection that this collector is expecting,
    *         {@code false} if this collector is inactive or if the message is not part of the
    *         current collection gathered by this collector.
    */
   @SuppressWarnings({"unchecked", "rawtypes"})
   public boolean interceptMessage(Settable<?> message)
   {
      if (messagePool == null)
         return false;

      long messageID = messageIDExtractor.getMessageID(message);

      if (messageID == MessageIDExtractor.NO_ID)
         return false;

      boolean intercept = expectedMessageIDs.remove(messageID);

      if (intercept)
      {
         Settable copy = messagePool.requestMessage(message.getClass());
         copy.set(message);
         interceptedMessages.add(copy);
      }

      if (expectedMessageIDs.isEmpty())
         isCollecting = false;

      return intercept;
   }

   /**
    * Whether this collector is currently gathering messages.
    * 
    * @return {@code true} if collecting, {@code false} otherwise.
    */
   public boolean isCollecting()
   {
      if (messagePool == null)
         return true;

      return isCollecting;
   }

   /**
    * Gets a view of the collected messages.
    * <p>
    * These messages should only be processed once this collector is done collecting, i.e.
    * {@link #isCollecting()} returns {@code false}.
    * </p>
    * 
    * @return the collected messages.
    */
   public List<Settable<?>> getCollectedMessages()
   {
      return interceptedMessagesView;
   }

   /**
    * Implement this interface to provide a method for getting the ID of messages.
    * 
    * @author Sylvain Bertrand
    */
   public static interface MessageIDExtractor
   {
      /**
       * Default value for marking a message as without an ID. A message without an ID can never be
       * part of a collection.
       */
      static final long NO_ID = 0;

      /**
       * Gets the ID of the given message.
       * 
       * @param message the message to get the ID of.
       * @return the ID or {@link #NO_ID} if the message should not be processed.
       */
      public long getMessageID(Object message);
   }
}
