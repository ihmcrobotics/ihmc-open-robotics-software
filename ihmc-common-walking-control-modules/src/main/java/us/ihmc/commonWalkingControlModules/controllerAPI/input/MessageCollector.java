package us.ihmc.commonWalkingControlModules.controllerAPI.input;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import controller_msgs.msg.dds.MessageCollection;
import gnu.trove.set.hash.TLongHashSet;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.packets.Packet;
import us.ihmc.idl.IDLSequence;

public class MessageCollector
{
   private boolean isCollecting = false;

   private final TLongHashSet expectedMessageIDs;
   private final List<Packet<?>> interceptedMessages;
   private final List<Packet<?>> interceptedMessagesView;
   private final MessagePool messagePool;
   private final MessageIDExtractor messageIDExtractor;

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
   }

   public MessageCollector(MessageIDExtractor messageIDExtractor, List<Class<? extends Packet<?>>> supportedMessages)
   {
      this.messageIDExtractor = messageIDExtractor;
      messagePool = new MessagePool(supportedMessages);
      expectedMessageIDs = new TLongHashSet();
      interceptedMessages = new ArrayList<>();
      interceptedMessagesView = Collections.unmodifiableList(interceptedMessages);
   }

   public void reset()
   {
      if (messagePool == null)
         return;

      isCollecting = false;
      expectedMessageIDs.clear();
      interceptedMessages.clear();
      messagePool.reset();
   }

   public void startCollecting(MessageCollection collection)
   {
      if (messagePool == null)
      {
         PrintTools.error("This is a dummy collector, it cannot collect messages.");
         return;
      }
      reset();
      isCollecting = true;

      IDLSequence.Long sequences = collection.getSequences();
      for (int i = 0; i < sequences.size(); i++)
      {
         expectedMessageIDs.add(sequences.get(i));
      }
   }

   @SuppressWarnings({"unchecked", "rawtypes"})
   public boolean interceptMessage(Packet<?> message)
   {
      if (messagePool == null)
         return false;

      long messageID = messageIDExtractor.getMessageID(message);

      if (messageID == MessageIDExtractor.NO_ID)
         return false;

      boolean intercept = expectedMessageIDs.remove(messageID);

      if (intercept)
      {
         Packet copy = messagePool.requestMessage(message.getClass());
         copy.set(message);
         interceptedMessages.add(copy);
      }

      if (expectedMessageIDs.isEmpty())
         isCollecting = false;

      return intercept;
   }

   public boolean isCollecting()
   {
      if (messagePool == null)
         return true;

      return isCollecting;
   }

   public boolean hasCollectedMessages()
   {
      return !interceptedMessagesView.isEmpty();
   }

   public List<Packet<?>> getCollectedMessages()
   {
      return interceptedMessagesView;
   }

   public static interface MessageIDExtractor
   {
      static final long NO_ID = 0;
      public long getMessageID(Packet<?> message);
   }
}
