package us.ihmc.communication.controllerAPI;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.concurrent.Builder;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.log.LogTools;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

/**
 * ConcurrentMessageInputBuffer is used to generate a thread-safe input API reading data from another thread. Messages
 * can be submitted through the methods {@link #submitMessage(Settable)}. Only registered inputs (Packet) will make it through
 * to controller side. Unregistered inputs are ignored and the user is averted by a message error
 * with the information on the input class. Registering inputs is done in the constructor
 * {@link ConcurrentMessageInputBuffer}, this is how one wants to define the API. The list of
 * supported inputs can be accessed using {@link #getListOfSupportedMessages()}. ConcurrentMessageInputBuffer assumes that the different methods for
 * submitting a inputs are called from another thread. ABSOLUTELY NO Packet should be
 * directly passed to controller, any Packet has to go through this API to ensure that
 * multi-threading is done properly.
 *
 * @author Robert
 */
public class ConcurrentMessageInputBuffer
{
   private final String printStatementPrefix;
   private final int buffersCapacity;

   /**
    * List of all the buffers that allows the user to easily flush all new messages using
    * {@link #clearAllMessages()}. These buffers CANNOT be visible or accessed from outside this class.
    */
   private final List<ConcurrentRingBuffer<?>> allBuffers = new ArrayList<>();
   /**
    * Map from the registered messages to their associated buffer. These buffers CANNOT be visible or
    * accessed from outside this class.
    */
   private final Map<Class<? extends Settable<?>>, ConcurrentRingBuffer<? extends Settable<?>>> messageClassToBufferMap = new HashMap<>();

   /**
    * Controller's copy of the new message to be processed.
    */
   private final Map<Class<? extends Settable<?>>, RecyclingArrayList<? extends Settable<?>>> messagesMap = new HashMap<>();

   /**
    * Exhaustive list of all the supported messages that this API can process.
    */
   private final List<Class<? extends Settable<?>>> listOfSupportedMessages = new ArrayList<>();

   /**
    * List of the listeners that should get notified when receiving a new valid message.
    */
   private final List<HasReceivedInputListener> hasReceivedInputListeners = new ArrayList<>();

   /**
    * Only constructor to build a new API. No new constructors will be tolerated.
    *
    * @param messagesToRegister list of the messages that this API should support.
    */
   public ConcurrentMessageInputBuffer(List<Class<? extends Settable<?>>> messagesToRegister)
   {
      this(null, messagesToRegister);
   }

   /**
    * Only constructor to build a new API. No new constructors will be tolerated.
    *
    * @param name               name used when printing statements. It should preferably be unique to
    *                           distinguish the different modules using this class.
    * @param messagesToRegister list of the messages that this API should support.
    */
   public ConcurrentMessageInputBuffer(String name, List<Class<? extends Settable<?>>> messagesToRegister)
   {
      this(name, messagesToRegister, 16);
   }

   /**
    * Only constructor to build a new API. No new constructors will be tolerated.
    *
    * @param name               name used when printing statements. It should preferably be unique to
    *                           distinguish the different modules using this class.
    * @param messagesToRegister list of the messages that this API should support.
    * @param buffersCapacity    the capacity of the internal buffers, should be a power of 2.
    */
   public ConcurrentMessageInputBuffer(String name, List<Class<? extends Settable<?>>> messagesToRegister, int buffersCapacity)
   {
      this.printStatementPrefix = name == null ? "" : name + ": ";
      this.buffersCapacity = buffersCapacity;
      registerNewMessages(messagesToRegister);
   }

   /**
    * This method has to remain private. It is used to register in the API a list of messages.
    *
    * @param messagesToRegister
    */
   @SuppressWarnings("unchecked")
   private <C extends Settable<C>, M extends Settable<M>> void registerNewMessages(List<Class<? extends Settable<?>>> messagesToRegister)
   {
      for (int i = 0; i < messagesToRegister.size(); i++)
         registerNewMessage((Class<C>) messagesToRegister.get(i));
   }

   /**
    * This method has to remain private. It is used to register in the API a command.
    *
    * @param messageClass
    */
   private <M extends Settable<M>> void registerNewMessage(Class<M> messageClass)
   {
      Builder<M> builder = CommandInputManager.createBuilderWithEmptyConstructor(messageClass);
      ConcurrentRingBuffer<M> newBuffer = new ConcurrentRingBuffer<>(builder, buffersCapacity);
      allBuffers.add(newBuffer);
      // This is silly, but I could not find another way that is more elegant.
      messageClassToBufferMap.put(messageClass, newBuffer);
      messagesMap.put(messageClass, new RecyclingArrayList<>(buffersCapacity, messageClass));

      listOfSupportedMessages.add(messageClass);
   }

   public void registerHasReceivedInputListener(HasReceivedInputListener hasReceivedInputListener)
   {
      hasReceivedInputListeners.add(hasReceivedInputListener);
   }

   /**
    * Submit a new {@link M} to be processed by the controller. This method can be called from any
    * thread. The message is first copied locally and only the copy will be visible for the controller.
    * No reference of this message will be held after calling this method. The user is free to modify
    * the message afterwards.
    *
    * @param message message to be submitted to the controller.
    */
   public <M extends Settable<M>> void submitMessage(M message)
   {
      submitMessageInternal(message);
   }

   @SuppressWarnings({"unchecked", "rawtypes"})
   private <M extends Settable<M>> void submitMessageInternal(M message)
   {
      if (message == null)
      {
         LogTools.warn("{}Received a null message, ignored.", printStatementPrefix);
         return;
      }

      ConcurrentRingBuffer buffer = messageClassToBufferMap.get(message.getClass());
      if (buffer == null)
      {
         LogTools.error("{}The message type {} is not supported.", printStatementPrefix, message.getClass().getSimpleName());
         return;
      }
      M nextMessage = (M) buffer.next();
      if (nextMessage == null)
      {
         LogTools.warn("{}The buffer for the message: {} is full. Message ignored.", printStatementPrefix, message.getClass().getSimpleName());
         return;
      }

      Class<M> messageClass = (Class<M>) nextMessage.getClass();
      nextMessage.set(message);

      buffer.commit();

      for (int i = 0; i < hasReceivedInputListeners.size(); i++)
         hasReceivedInputListeners.get(i).hasReceivedInput(messageClass);
   }

   /**
    * Submit a new list of {@link Settable} to be processed by the controller. This method can be called
    * from any thread. The message is first copied locally and only the copy will be visible for the
    * controller. No reference of this message will be held after calling this method. The user is free
    * to modify the message afterwards.
    *
    * @param messages list of messages to be submitted to the controller.
    */
   @SuppressWarnings("unchecked")
   public <M extends Settable<M>> void submitMessages(List<? extends Settable<?>> messages)
   {
      for (int i = 0; i < messages.size(); i++)
         submitMessage((M) messages.get(i));
   }

   public boolean isNewMessageAvailable()
   {
      for (int i = 0; i < allBuffers.size(); i++)
      {
         if (allBuffers.get(i).poll())
            return true;
      }
      return false;
   }

   /**
    * Check if a new message to be processed is available.
    *
    * @param messageClassToCheck class of the message to check availability.
    * @return true if at least one new message is available.
    */
   public boolean isNewMessageAvailable(Class<? extends Settable<?>> messageClassToCheck)
   {
      return messageClassToBufferMap.get(messageClassToCheck).poll();
   }

   /**
    * Throw away any new available messages.
    */
   public void clearAllMessages()
   {
      for (int i = 0; i < allBuffers.size(); i++)
         clearBuffer(allBuffers.get(i));
   }

   /**
    * Throw away any new available message of a certain type.
    *
    * @param messageClassToCLear Used to know what type of message is to be thrown away.
    */
   public <C extends Settable<C>> void clearMessages(Class<C> messageClassToCLear)
   {
      clearBuffer(messageClassToBufferMap.get(messageClassToCLear));
   }

   /**
    * Poll the most recent available message. After calling this method, no new message will be
    * available.
    *
    * @param messageClassToPoll Used to know what type of message is to be polled.
    * @return the new message to be processed, returns null if there is no new available message.
    */
   public <C extends Settable<C>> C pollNewestMessage(Class<C> messageClassToPoll)
   {
      return ((RecyclingArrayList<C>) pollNewMessages(messageClassToPoll)).getLast();
   }

   /**
    * Poll all the new available messages. After calling this method, no new message will be available.
    *
    * @param messageClassToPoll Used to know what type of message is to be polled.
    * @return the new messages to be processed stored in a list, returns an empty list if there is no
    *       new available message.
    */
   @SuppressWarnings("unchecked")
   public <C extends Settable<C>> List<C> pollNewMessages(Class<C> messageClassToPoll)
   {
      RecyclingArrayList<C> messages = (RecyclingArrayList<C>) messagesMap.get(messageClassToPoll);
      ConcurrentRingBuffer<C> buffer = (ConcurrentRingBuffer<C>) messageClassToBufferMap.get(messageClassToPoll);
      pollNewMessages(buffer, messages);
      return messages;
   }

   /**
    * Reads all the available elements from the buffer and then flushes it.
    *
    * @param bufferToClear the buffer to be cleared from the reader perspective. Modified.
    */
   private static void clearBuffer(ConcurrentRingBuffer<?> bufferToClear)
   {
      if (bufferToClear.poll())
      {
         for (int i = 0; i < bufferToClear.getCapacity(); i++)
         {
            if (bufferToClear.read() == null)
               break;
         }
         bufferToClear.flush();
      }
   }

   /**
    * This method has to remain private. Reads all the new available messages from a buffer and copy
    * them in a list.
    *
    * @param buffer         Buffer in which the new available messages are stored.
    * @param messagesToPack Used to copy and store all the new available messages. This list will be
    *                       empty is there is no new available messages.
    */
   private static <C extends Settable<C>> void pollNewMessages(ConcurrentRingBuffer<C> buffer, RecyclingArrayList<C> messagesToPack)
   {
      messagesToPack.clear();

      if (buffer.poll())
      {
         C message;
         while ((message = buffer.read()) != null)
         {
            messagesToPack.add().set(message);
         }
         buffer.flush();
      }
   }

   /**
    * @return The list of all the messages supported by this API.
    */
   public List<Class<? extends Settable<?>>> getListOfSupportedMessages()
   {
      return listOfSupportedMessages;
   }

   /**
    * Use this interface to get notified when this API has received a new valid message.
    */
   public static interface HasReceivedInputListener
   {
      public void hasReceivedInput(Class<? extends Settable<?>> messageClass);
   }
}
