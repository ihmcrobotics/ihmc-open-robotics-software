package us.ihmc.communication.controllerAPI;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.communication.controllerAPI.MessageUnpackingTools.MessageUnpacker;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.concurrent.Builder;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.log.LogTools;

/**
 * CommandInputManager is used to generate a thread-safe input API for a controller. Message
 * and {@link Command} can be submitted through the methods {@link #submitMessage(Object)} and
 * {@link #submitCommand(Command)}. Only registered inputs (Packet or Command) will make it through
 * to controller side. Unregistered inputs are ignored and the user is averted by a message error
 * with the information on the input class.
 * 
 * Registering inputs is done in the constructor {@link #CommandInputManager(List)}, this is how one
 * wants to define the API.
 * 
 * The list of supported inputs can be accessed using {@link #getListOfSupportedMessages()} and
 * {@link #getListOfSupportedCommands()}.
 * 
 * CommandInputManager assumes that the different methods for submitting a inputs are called from
 * another thread. ABSOLUTELY NO Packet/Command should be directly passed to controller, any
 * Packet/Command has to go through this API to ensure that multi-threading is done properly.
 * 
 * @author Sylvain
 *
 */
public class CommandInputManager
{
   private final String printStatementPrefix;
   private final int buffersCapacity;

   /**
    * List of all the buffers that allows the user to easily flush all new commands using
    * {@link #clearAllCommands()}. These buffers CANNOT be visible or accessed from outside this
    * class.
    */
   private final List<ConcurrentRingBuffer<?>> allBuffers = new ArrayList<>();
   /**
    * Map from the registered commands to their associated buffer. These buffers CANNOT be visible
    * or accessed from outside this class.
    */
   private final Map<Class<? extends Command<?, ?>>, ConcurrentRingBuffer<? extends Command<?, ?>>> commandClassToBufferMap = new HashMap<>();
   /**
    * Map from the registered messages to their associated buffer. These buffers CANNOT be visible
    * or accessed from outside this class.
    */
   private final Map<Class<? extends Settable<?>>, ConcurrentRingBuffer<? extends Command<?, ?>>> messageClassToBufferMap = new HashMap<>();

   /** Controller's copy of the new commands to be processed. */
   private final Map<Class<? extends Command<?, ?>>, RecyclingArrayList<? extends Command<?, ?>>> commandsMap = new HashMap<>();

   /** Exhaustive list of all the supported commands that this API can process. */
   private final List<Class<? extends Command<?, ?>>> listOfSupportedCommands = new ArrayList<>();
   /** Exhaustive list of all the supported messages that this API can process. */
   private final List<Class<? extends Settable<?>>> listOfSupportedMessages = new ArrayList<>();

   /** List of the listeners that should get notified when receiving a new valid command. */
   private final List<HasReceivedInputListener> hasReceivedInputListeners = new ArrayList<>();

   /** Converters used to perform custom conversion to commands for specific messages. **/
   private final List<CommandConversionInterface> commandConverters = new ArrayList<>();

   /**
    * Protocols for unpacking certain types of messages. These messages do not have to be registered
    * at construction time.
    */
   private final Map<Class<? extends Settable<?>>, MessageUnpacker<? extends Settable<?>>> messageUnpackers = new HashMap<>();
   /** Buffer used to unpack messages without making garbage. */
   private final List<Settable<?>> unpackedMessages = new ArrayList<>();

   /**
    * Only constructor to build a new API. No new constructors will be tolerated.
    * 
    * @param commandsToRegister list of the commands that this API should support.
    */
   public CommandInputManager(List<Class<? extends Command<?, ?>>> commandsToRegister)
   {
      this(null, commandsToRegister);
   }

   /**
    * Only constructor to build a new API. No new constructors will be tolerated.
    *
    * @param name name used when printing statements. It should preferably be unique to distinguish
    *           the different modules using this class.
    * @param commandsToRegister list of the commands that this API should support.
    */
   public CommandInputManager(String name, List<Class<? extends Command<?, ?>>> commandsToRegister)
   {
      this(name, commandsToRegister, 16);
   }

   /**
    * Only constructor to build a new API. No new constructors will be tolerated.
    * 
    * @param name name used when printing statements. It should preferably be unique to distinguish
    *           the different modules using this class.
    * @param commandsToRegister list of the commands that this API should support.
    * @param buffersCapacity the capacity of the internal buffers, should be a power of 2.
    */
   public CommandInputManager(String name, List<Class<? extends Command<?, ?>>> commandsToRegister, int buffersCapacity)
   {
      this.printStatementPrefix = name == null ? "" : name + ": ";
      this.buffersCapacity = buffersCapacity;
      registerNewCommands(commandsToRegister);
   }

   /**
    * Registers a converter that helps convert a packet to a command. Currently only supports one
    * converter
    * 
    * @param commandConversionHelper
    */
   public void registerConversionHelper(CommandConversionInterface commandConversionHelper)
   {
      this.commandConverters.add(commandConversionHelper);
   }

   /**
    * Registers a protocol for unpacking a certain of message.
    * 
    * @param messageClass the type of messages that holds onto multiple messages which should be
    *           unpacked at reception.
    * @param messageUnpacker the protocol for unpacking the message.
    */
   public <T extends Settable<T>> void registerMessageUnpacker(Class<T> messageClass, MessageUnpacker<T> messageUnpacker)
   {
      messageUnpackers.put(messageClass, messageUnpacker);
   }

   /**
    * This method has to remain private. It is used to register in the API a list of commands.
    * 
    * @param commandClasses
    */
   @SuppressWarnings("unchecked")
   private <C extends Command<C, M>, M extends Settable<M>> void registerNewCommands(List<Class<? extends Command<?, ?>>> commandClasses)
   {
      for (int i = 0; i < commandClasses.size(); i++)
         registerNewCommand((Class<C>) commandClasses.get(i));
   }

   /**
    * This method has to remain private. It is used to register in the API a command.
    * 
    * @param commandClass
    */
   private <C extends Command<C, M>, M extends Settable<M>> void registerNewCommand(Class<C> commandClass)
   {
      Builder<C> builer = createBuilderWithEmptyConstructor(commandClass);
      ConcurrentRingBuffer<C> newBuffer = new ConcurrentRingBuffer<>(builer, buffersCapacity);
      allBuffers.add(newBuffer);
      // This is retarded, but I could not find another way that is more elegant.
      Class<M> messageClass = builer.newInstance().getMessageClass();
      commandClassToBufferMap.put(commandClass, newBuffer);
      messageClassToBufferMap.put(messageClass, newBuffer);
      commandsMap.put(commandClass, new RecyclingArrayList<>(buffersCapacity, commandClass));

      listOfSupportedCommands.add(commandClass);
      listOfSupportedMessages.add(messageClass);
   }

   public void registerHasReceivedInputListener(HasReceivedInputListener hasReceivedInputListener)
   {
      hasReceivedInputListeners.add(hasReceivedInputListener);
   }

   /**
    * Submit a new {@link Packet} to be processed by the controller. This method can be called from
    * any thread. The message is first copied locally and only the copy will be visible for the
    * controller. No reference of this message will be held after calling this method. The user is
    * free to modify the message afterwards.
    * 
    * @param message message to be submitted to the controller.
    */
   public <M extends Settable<M>> void submitMessage(M message)
   {
      submitMessageInternal(message);
   }

   @SuppressWarnings({"unchecked", "rawtypes"})
   private void submitMessageInternal(Settable message)
   {
      if (message == null)
      {
         LogTools.warn("{}Received a null message, ignored.", printStatementPrefix);
         return;
      }

      MessageUnpacker messageUnpacker = messageUnpackers.get(message.getClass());

      if (messageUnpacker != null)
      {
         messageUnpacker.unpackMessage(message, unpackedMessages);

         while (!unpackedMessages.isEmpty())
         {
            submitMessageInternal(unpackedMessages.remove(unpackedMessages.size() - 1));
         }
         return;
      }

      ConcurrentRingBuffer buffer = messageClassToBufferMap.get(message.getClass());
      if (buffer == null)
      {
         LogTools.error("{}The message type {} is not supported.", printStatementPrefix, message.getClass().getSimpleName());
         return;
      }
      Command nextCommand = (Command) buffer.next();
      if (nextCommand == null)
      {
         LogTools.warn("{}The buffer for the message: {} is full. Message ignored.", printStatementPrefix, message.getClass().getSimpleName());
         return;
      }

      Class<?> commandClass = nextCommand.getClass();

      if (!performCustomConversion(message, nextCommand))
      {
         nextCommand.setFromMessage(message);
      }

      buffer.commit();

      for (int i = 0; i < hasReceivedInputListeners.size(); i++)
         hasReceivedInputListeners.get(i).hasReceivedInput((Class<? extends Command<?, ?>>) commandClass);
   }

   /**
    * Finds the first converter in {@link #commandConverters} that can convert {@code message} into
    * {@code commandToStoreMessage} and use it to convert the message.
    * <p>
    * If no command converter is found, this method does nothing and returns {@code false}.
    * </p>
    * 
    * @param message the message to be converted into a command. Not modified.
    * @param commandToStoreMessage the command in which the message is converted. Modified.
    * @return whether the message was converted or not.
    */
   private <M extends Settable<M>> boolean performCustomConversion(M message, Command<?, M> commandToStoreMessage)
   {
      for (int i = 0; i < commandConverters.size(); i++)
      {
         CommandConversionInterface commandConverter = commandConverters.get(i);

         if (commandConverter.isConvertible(commandToStoreMessage, message))
         {
            commandConverter.process(commandToStoreMessage, message);
            return true;
         }
      }
      return false;
   }

   /**
    * Submit a new list of {@link Packet} to be processed by the controller. This method can be
    * called from any thread. The message is first copied locally and only the copy will be visible
    * for the controller. No reference of this message will be held after calling this method. The
    * user is free to modify the message afterwards.
    * 
    * @param messages list of messages to be submitted to the controller.
    */
   @SuppressWarnings("unchecked")
   public <M extends Settable<M>> void submitMessages(List<Settable<?>> messages)
   {
      for (int i = 0; i < messages.size(); i++)
         submitMessage((M) messages.get(i));
   }

   /**
    * Submit a new {@link Command} to be processed by the controller. This method can be called from
    * any thread. The command is first copied locally and only the copy will be visible for the
    * controller. No reference of this command will be held after calling this method. The user is
    * free to modify the command afterwards.
    * 
    * @param command command to be submitted to the controller.
    */
   @SuppressWarnings("unchecked")
   public <C extends Command<C, ?>> void submitCommand(C command)
   {
      if (!command.isCommandValid())
         return;

      ConcurrentRingBuffer<? extends Command<?, ?>> buffer = commandClassToBufferMap.get(command.getClass());
      if (buffer == null)
      {
         LogTools.error("{}The command type {} is not supported.", printStatementPrefix, command.getClass().getSimpleName());
         return;
      }

      Command<C, ?> nextModifiableMessage = (Command<C, ?>) buffer.next();
      if (nextModifiableMessage == null)
      {
         LogTools.warn("{}The buffer for the command: {} is full. Command ignored.", printStatementPrefix, command.getClass().getSimpleName());
         return;
      }
      nextModifiableMessage.set(command);
      buffer.commit();

      for (int i = 0; i < hasReceivedInputListeners.size(); i++)
         hasReceivedInputListeners.get(i).hasReceivedInput((Class<? extends Command<?, ?>>) command.getClass());
   }

   /**
    * Submit a new list of {@link Command} to be processed by the controller. This method can be
    * called from any thread. The command is first copied locally and only the copy will be visible
    * for the controller. No reference of this command will be held after calling this method. The
    * user is free to modify the command afterwards.
    * 
    * @param commands list of commands to be submitted to the controller.
    */
   @SuppressWarnings("unchecked")
   public <C extends Command<C, ?>> void submitControllerCommands(List<Command<?, ?>> commands)
   {
      for (int i = 0; i < commands.size(); i++)
         submitCommand((C) commands.get(i));
   }

   public boolean isNewCommandAvailable()
   {
      for (int i = 0; i < allBuffers.size(); i++)
      {
         if (allBuffers.get(i).poll())
            return true;
      }
      return false;
   }

   /**
    * Check if a new command to be processed is available.
    * 
    * @param commandClassToCheck class of the command to check availability.
    * @return true if at least one new command is available.
    */
   public boolean isNewCommandAvailable(Class<? extends Command<?, ?>> commandClassToCheck)
   {
      return commandClassToBufferMap.get(commandClassToCheck).poll();
   }

   /**
    * Throw away any new available commands.
    * @deprecated Use {@link #clearAllCommands()} instead
    */
   @Deprecated
   public void flushAllCommands()
   {
      clearAllCommands();
   }

   /**
    * Throw away any new available commands.
    */
   public void clearAllCommands()
   {
      for (int i = 0; i < allBuffers.size(); i++)
         clearBuffer(allBuffers.get(i));
   }

   /**
    * Throw away any new available commands of a certain type.
    * 
    * @param commandClassToFlush Used to know what type of command is to be thrown away.
    * @deprecated Use {@link #clearCommands(Class<C>)} instead
    */
   @Deprecated
   public <C extends Command<C, ?>> void flushCommands(Class<C> commandClassToFlush)
   {
      clearCommands(commandClassToFlush);
   }

   /**
    * Throw away any new available commands of a certain type.
    * 
    * @param commandClassToClear Used to know what type of command is to be thrown away.
    */
   public <C extends Command<C, ?>> void clearCommands(Class<C> commandClassToClear)
   {
      clearBuffer(commandClassToBufferMap.get(commandClassToClear));
   }

   /**
    * Poll the most recent available command. After calling this method, no new command will be
    * available.
    * 
    * @param commandClassToPoll Used to know what type of command is to be polled.
    * @return the new command to be processed, returns null if there is no new available command.
    */
   public <C extends Command<C, ?>> C pollNewestCommand(Class<C> commandClassToPoll)
   {
      return ((RecyclingArrayList<C>) pollNewCommands(commandClassToPoll)).getLast();
   }

   /**
    * Poll all the new available commands. After calling this method, no new command will be
    * available.
    * 
    * @param commandClassToPoll Used to know what type of command is to be polled.
    * @return the new commands to be processed stored in a list, returns an empty list if there is
    *         no new available command.
    */
   @SuppressWarnings("unchecked")
   public <C extends Command<C, ?>> List<C> pollNewCommands(Class<C> commandClassToPoll)
   {
      RecyclingArrayList<C> commands = (RecyclingArrayList<C>) commandsMap.get(commandClassToPoll);
      ConcurrentRingBuffer<C> buffer = (ConcurrentRingBuffer<C>) commandClassToBufferMap.get(commandClassToPoll);
      pollNewCommands(buffer, commands);
      return commands;
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
    * This method has to remain private. Reads all the new available commands from a buffer and copy
    * them in a list.
    * 
    * @param buffer Buffer in which the new available commands are stored.
    * @param commandsToPack Used to copy and store all the new available commands. This list will be
    *           empty is there is no new available command.
    */
   private static <C extends Command<C, ?>> void pollNewCommands(ConcurrentRingBuffer<C> buffer, RecyclingArrayList<C> commandsToPack)
   {
      commandsToPack.clear();

      if (buffer.poll())
      {
         C command;
         while ((command = buffer.read()) != null)
         {
            commandsToPack.add().set(command);
            command.clear();
         }
         buffer.flush();
      }
   }

   /**
    * Method to help creating a {@link ConcurrentRingBuffer} for a given class. The class has to
    * have an empty constructor.
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
               throw new RuntimeException("Something went wrong the empty constructor implemented in the class: "
                     + emptyConstructor.getDeclaringClass().getSimpleName());
            }

            return newInstance;
         }
      };
      return builder;
   }

   /**
    * @return The list of all the commands supported by this API.
    */
   public List<Class<? extends Command<?, ?>>> getListOfSupportedCommands()
   {
      return listOfSupportedCommands;
   }

   /**
    * @return The list of all the messages supported by this API.
    */
   public List<Class<? extends Settable<?>>> getListOfSupportedMessages()
   {
      return listOfSupportedMessages;
   }

   /**
    * Use this interface to get notified when this API has received a new valid command.
    */
   public static interface HasReceivedInputListener
   {
      public void hasReceivedInput(Class<? extends Command<?, ?>> commandClass);
   }
}
