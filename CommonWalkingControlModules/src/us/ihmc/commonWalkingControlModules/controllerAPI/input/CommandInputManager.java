package us.ihmc.commonWalkingControlModules.controllerAPI.input;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.Command;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.CompilableCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.MultipleCommandHolder;
import us.ihmc.communication.packets.MultiplePacketHolder;
import us.ihmc.communication.packets.Packet;
import us.ihmc.concurrent.Builder;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.tools.io.printing.PrintTools;

/**
 * CommandInputManager is used to generate a thread-safe input API for a controller.
 * {@link Packet} and {@link Command} can be submitted through the methods {@link #submitMessage(Packet)} and {@link #submitCommand(Command)}.
 * Only registered inputs (Packet or Command) will make it through to controller side.
 * Unregistered inputs are ignored and the user is averted by a message error with the information on the input class.
 * 
 * Registering inputs is done in the constructor {@link #CommandInputManager(List)}, this is how one wants to define the API.
 * 
 * The list of supported inputs can be accessed using {@link #getListOfSupportedMessages()} and {@link #getListOfSupportedCommands()}.
 * 
 * CommandInputManager assumes that the different methods for submitting a inputs are called from another thread.
 * ABSOLUTELY NO Packet/Command should be directly passed to controller, any Packet/Command has to go through this API to ensure that multi-threading is done properly.
 * 
 * @author Sylvain
 *
 */
public class CommandInputManager
{
   private final int buffersCapacity = 8;

   /**
    * List of all the buffers that allows the user to easily flush all new commands using {@link #flushAllCommands()}.
    * These buffers CANNOT be visible or accessed from outside this class.
    */
   private final List<ConcurrentRingBuffer<?>> allBuffers = new ArrayList<>();
   /**
    * Map from the registered commands to their associated buffer.
    * These buffers CANNOT be visible or accessed from outside this class.
    */
   private final Map<Class<? extends Command<?, ?>>, ConcurrentRingBuffer<? extends Command<?, ?>>> commandClassToBufferMap = new HashMap<>();
   /**
    * Map from the registered messages to their associated buffer.
    * These buffers CANNOT be visible or accessed from outside this class.
    */
   private final Map<Class<? extends Packet<?>>, ConcurrentRingBuffer<? extends Command<?, ?>>> messageClassToBufferMap = new HashMap<>();

   /** Controller's copy of the new commands to be processed. */
   private final Map<Class<? extends Command<?, ?>>, RecyclingArrayList<? extends Command<?, ?>>> commandsMap = new HashMap<>();

   /** Exhaustive list of all the supported commands that this API can process. */
   private final List<Class<? extends Command<?, ?>>> listOfSupportedCommands = new ArrayList<>();
   /** Exhaustive list of all the supported messages that this API can process. */
   private final List<Class<? extends Packet<?>>> listOfSupportedMessages = new ArrayList<>();

   /**
    * Only constructor to build a new API. No new constructors will be tolerated.
    * 
    * @param commandsToRegister list of the commands that this API should support.
    */
   public CommandInputManager(List<Class<? extends Command<?, ?>>> commandsToRegister)
   {
      registerNewCommands(commandsToRegister);
   }

   /**
    * This method has to remain private.
    * It is used to register in the API a list of commands.
    * @param commandClazzes
    */
   @SuppressWarnings("unchecked")
   private <C extends Command<C, M>, M extends Packet<M>> void registerNewCommands(List<Class<? extends Command<?, ?>>> commandClazzes)
   {
      for (int i = 0; i < commandClazzes.size(); i++)
         registerNewCommand((Class<C>) commandClazzes.get(i));
   }

   /**
    * This method has to remain private.
    * It is used to register in the API a command.
    * @param commandClazzes
    */
   private <C extends Command<C, M>, M extends Packet<M>> void registerNewCommand(Class<C> commandClass)
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

   public <M extends Packet<M>> void submitMessage(M message)
   {
      if (message == null || message.getUniqueId() == Packet.INVALID_MESSAGE_ID)
         return;

      if (message instanceof MultiplePacketHolder)
      {
         submitMessages(((MultiplePacketHolder) message).getPackets());
         return;
      }

      ConcurrentRingBuffer<? extends Command<?, ?>> buffer = messageClassToBufferMap.get(message.getClass());
      if (buffer == null)
      {
         PrintTools.error(this, "The message type " + message.getClass().getSimpleName() + " is not supported.");
         return;
      }
      @SuppressWarnings("unchecked")
      Command<?, M> nextCommand = (Command<?, M>) buffer.next();
      if (nextCommand == null)
         return;
      nextCommand.set(message);
      buffer.commit();
   }

   @SuppressWarnings("unchecked")
   public <M extends Packet<M>> void submitMessages(List<Packet<?>> messages)
   {
      for (int i = 0; i < messages.size(); i++)
         submitMessage((M) messages.get(i));
   }

   public <C extends Command<C, ?>> void submitCommand(C command)
   {
      if (!command.isCommandValid())
         return;

      if (command instanceof MultipleCommandHolder)
         submitControllerCommands(((MultipleCommandHolder<?, ?>) command).getControllerCommands());

      ConcurrentRingBuffer<? extends Command<?, ?>> buffer = commandClassToBufferMap.get(command.getClass());
      if (buffer == null)
      {
         PrintTools.error(this, "The message type " + command.getClass().getSimpleName() + " is not supported.");
         return;
      }
      @SuppressWarnings("unchecked")
      Command<C, ?> nextModifiableMessage = (Command<C, ?>) buffer.next();
      if (nextModifiableMessage == null)
         return;
      nextModifiableMessage.set(command);
      buffer.commit();
   }

   @SuppressWarnings("unchecked")
   public <C extends Command<C, ?>> void submitControllerCommands(List<Command<?, ?>> controllerCommands)
   {
      for (int i = 0; i < controllerCommands.size(); i++)
         submitCommand((C) controllerCommands.get(i));
   }

   public boolean isNewCommandAvailable(Class<? extends Command<?, ?>> commandClassToCheck)
   {
      return commandClassToBufferMap.get(commandClassToCheck).poll();
   }

   public void flushAllCommands()
   {
      for (int i = 0; i < allBuffers.size(); i++)
         allBuffers.get(i).flush();
   }

   public <C extends Command<C, ?>> void flushCommands(Class<C> commandClassToFlush)
   {
      commandClassToBufferMap.get(commandClassToFlush).flush();
   }

   public <C extends CompilableCommand<C, ?>> C pollAndCompileCommands(Class<C> commandClassToPoll)
   {
      List<C> commands = pollNewCommands(commandClassToPoll);
      for (int i = 1; i < commands.size(); i++)
         commands.get(0).compile(commands.get(i));
      return commands.get(0);
   }

   public <C extends Command<C, ?>> C pollNewestCommand(Class<C> commandClassToFlush)
   {
      return ((RecyclingArrayList<C>) pollNewCommands(commandClassToFlush)).getLast();
   }

   @SuppressWarnings("unchecked")
   public <C extends Command<C, ?>> List<C> pollNewCommands(Class<C> commandClassToPoll)
   {
      RecyclingArrayList<C> commands = (RecyclingArrayList<C>) commandsMap.get(commandClassToPoll);
      commands.clear();
      ConcurrentRingBuffer<C> buffer = (ConcurrentRingBuffer<C>) commandClassToBufferMap.get(commandClassToPoll);
      pollNewCommands(buffer, commands);
      return commands;
   }

   private static <C extends Command<C, ?>> void pollNewCommands(ConcurrentRingBuffer<C> buffer, RecyclingArrayList<C> commandsToPack)
   {
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

   public List<Class<? extends Command<?, ?>>> getListOfSupportedCommands()
   {
      return listOfSupportedCommands;
   }

   public List<Class<? extends Packet<?>>> getListOfSupportedMessages()
   {
      return listOfSupportedMessages;
   }
}
