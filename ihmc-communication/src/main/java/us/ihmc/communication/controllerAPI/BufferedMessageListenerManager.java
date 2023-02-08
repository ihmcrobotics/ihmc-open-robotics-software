package us.ihmc.communication.controllerAPI;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.concurrent.Builder;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.log.LogTools;
import us.ihmc.tools.lists.PairList;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.function.BiConsumer;

/**
 * BufferedMessageListenerManager is used to generate a thread-safe input API reading data from another thread. Messages
 * can be submitted through the methods {@link #submitMessage(Settable)}. Only registered inputs (Packet) will make it through
 * to controller side. Unregistered inputs are ignored and the user is averted by a message error
 * with the information on the input class. Registering inputs is done in the constructor
 * {@link BufferedMessageListenerManager}, this is how one wants to define the API. The list of
 * supported inputs can be accessed using {@link #getListOfSupportedMessages()}. BufferedMessageListenerManager assumes that the different methods for
 * submitting a inputs are called from another thread. ABSOLUTELY NO Packet should be
 * directly passed to controller, any Packet has to go through this API to ensure that
 * multi-threading is done properly.
 *
 * @author Robert
 */
public class BufferedMessageListenerManager
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
    * Protocols for unpacking certain types of messages.
    */
   private final Map<Class<? extends Settable<?>>, BiConsumer<? extends Settable<?>, ? extends Settable<?>>> messageUnpackers = new HashMap<>();

   /**
    * Only constructor to build a new API. No new constructors will be tolerated.
    *
    * @param messagesAndUnpackersToRegister list of the messages and set methods that this API should support.
    */
   public BufferedMessageListenerManager(PairList<Class<? extends Settable<?>>, BiConsumer<? extends Settable<?>, ? extends Settable<?>>> messagesAndUnpackersToRegister)
   {
      this(null, messagesAndUnpackersToRegister);
   }

   /**
    * Only constructor to build a new API. No new constructors will be tolerated.
    *
    * @param name                           name used when printing statements. It should preferably be unique to
    *                                       distinguish the different modules using this class.
    * @param messagesAndUnpackersToRegister list of the messages and set methods that this API should support.
    */
   public BufferedMessageListenerManager(String name,
                                         PairList<Class<? extends Settable<?>>, BiConsumer<? extends Settable<?>, ? extends Settable<?>>> messagesAndUnpackersToRegister)
   {
      this(name, messagesAndUnpackersToRegister, 16);
   }

   /**
    * Only constructor to build a new API. No new constructors will be tolerated.
    *
    * @param name                           name used when printing statements. It should preferably be unique to
    *                                       distinguish the different modules using this class.
    * @param messagesAndUnpackersToRegister list of the messages and set methods that this API should support.
    * @param buffersCapacity                the capacity of the internal buffers, should be a power of 2.
    */
   public BufferedMessageListenerManager(String name,
                                         PairList<Class<? extends Settable<?>>, BiConsumer<? extends Settable<?>, ? extends Settable<?>>> messagesAndUnpackersToRegister,
                                         int buffersCapacity)
   {
      this.printStatementPrefix = name == null ? "" : name + ": ";
      this.buffersCapacity = buffersCapacity;
      registerNewMessages(messagesAndUnpackersToRegister);
   }

   /**
    * This method has to remain private. It is used to register in the API a list of messages.
    *
    * @param messagesAndUnpackersToRegister
    */
   @SuppressWarnings("unchecked")
   private <C extends Settable<C>, M extends Settable<M>> void registerNewMessages(PairList<Class<? extends Settable<?>>, BiConsumer<? extends Settable<?>, ? extends Settable<?>>> messagesAndUnpackersToRegister)
   {
      for (int i = 0; i < messagesAndUnpackersToRegister.size(); i++)
         registerNewMessage((Class<C>) messagesAndUnpackersToRegister.get(i).getLeft(), messagesAndUnpackersToRegister.get(i).getRight());
   }

   /**
    * This method has to remain private. It is used to register in the API a command.
    *
    * @param messageClass
    */
   private <M extends Settable<M>> void registerNewMessage(Class<M> messageClass, BiConsumer<? extends Settable<?>, ? extends Settable<?>> messageUnpacker)
   {
      Builder<M> builer = createBuilderWithEmptyConstructor(messageClass);
      ConcurrentRingBuffer<M> newBuffer = new ConcurrentRingBuffer<>(builer, buffersCapacity);
      allBuffers.add(newBuffer);
      // This is silly, but I could not find another way that is more elegant.
      messageClassToBufferMap.put(messageClass, newBuffer);
      messagesMap.put(messageClass, new RecyclingArrayList<>(buffersCapacity, messageClass));
      messageUnpackers.put(messageClass, messageUnpacker);

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
      BiConsumer<M, M> messageUnpacker = (BiConsumer<M, M>) messageUnpackers.get(message.getClass());
      if (messageUnpacker == null)
      {
         LogTools.warn("{}The unpacker for the message type {} is not supported.", printStatementPrefix, message.getClass().getSimpleName());
         return;
      }

      Class<M> messageClass = (Class<M>) nextMessage.getClass();

      messageUnpacker.accept(message, nextMessage);

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
    * Method to help creating a {@link ConcurrentRingBuffer} for a given class. The class has to have
    * an empty constructor.
    *
    * @param clazz For which a new builder needs to be created.
    * @return The new builder.
    */
   public static <U> Builder<U> createBuilderWithEmptyConstructor(Class<U> clazz)
   {
      final Constructor<U> emptyConstructor;
      // Trying to get an empty constructor from clazz
      try
      {
         emptyConstructor = clazz.getConstructor();
      }
      catch (NoSuchMethodException | SecurityException e)
      {
         throw new RuntimeException("Could not find a visible empty constructor in the class: " + clazz.getSimpleName());
      }

      Builder<U> builder = new Builder<U>()
      {
         @Override
         public U newInstance()
         {
            U newInstance = null;

            try
            {
               newInstance = emptyConstructor.newInstance();
            }
            catch (InstantiationException | IllegalAccessException | IllegalArgumentException | InvocationTargetException e)
            {
               e.printStackTrace();
               throw new RuntimeException(
                     "Something went wrong the empty constructor implemented in the class: " + emptyConstructor.getDeclaringClass().getSimpleName());
            }

            return newInstance;
         }
      };
      return builder;
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
