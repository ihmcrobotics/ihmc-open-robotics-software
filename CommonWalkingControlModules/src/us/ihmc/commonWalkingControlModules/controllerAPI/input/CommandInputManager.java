package us.ihmc.commonWalkingControlModules.controllerAPI.input;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.AbortWalkingCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ArmDesiredAccelerationsCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ArmTrajectoryCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.AutomaticManipulationAbortCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ChestTrajectoryCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.Command;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.CompilableCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.EndEffectorLoadBearingCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.FootTrajectoryCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.FootstepDataListCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.GoHomeCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.HandComplianceControlParametersCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.HandTrajectoryCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.HeadTrajectoryCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.HighLevelStateCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.MultipleCommandHolder;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.PauseWalkingCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.PelvisHeightTrajectoryCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.PelvisOrientationTrajectoryCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.PelvisTrajectoryCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.StopAllTrajectoryCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.WholeBodyTrajectoryCommand;
import us.ihmc.communication.packets.MultiplePacketHolder;
import us.ihmc.communication.packets.Packet;
import us.ihmc.concurrent.Builder;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.tools.io.printing.PrintTools;

public class CommandInputManager
{
   private final int buffersCapacity = 8;

   private final List<ConcurrentRingBuffer<?>> allBuffers = new ArrayList<>();
   private final Map<Class<? extends Command<?, ?>>, ConcurrentRingBuffer<? extends Command<?, ?>>> commandClassToBufferMap = new HashMap<>();
   private final Map<Class<? extends Packet<?>>, ConcurrentRingBuffer<? extends Command<?, ?>>> messageClassToBufferMap = new HashMap<>();
   private final Map<Class<? extends Command<?, ?>>, RecyclingArrayList<? extends Command<?, ?>>> commandsMap = new HashMap<>();

   private final List<Class<? extends Packet<?>>> listOfSupportedMessages;

   public CommandInputManager()
   {
      registerNewCommand(ArmTrajectoryCommand.class);
      registerNewCommand(HandTrajectoryCommand.class);
      registerNewCommand(FootTrajectoryCommand.class);
      registerNewCommand(HeadTrajectoryCommand.class);
      registerNewCommand(ChestTrajectoryCommand.class);
      registerNewCommand(PelvisTrajectoryCommand.class);
      registerNewCommand(PelvisOrientationTrajectoryCommand.class);
      registerNewCommand(PelvisHeightTrajectoryCommand.class);
      registerNewCommand(StopAllTrajectoryCommand.class);
      registerNewCommand(FootstepDataListCommand.class);
      registerNewCommand(GoHomeCommand.class);
      registerNewCommand(EndEffectorLoadBearingCommand.class);
      registerNewCommand(ArmDesiredAccelerationsCommand.class);
      registerNewCommand(AutomaticManipulationAbortCommand.class);
      registerNewCommand(HandComplianceControlParametersCommand.class);
      registerNewCommand(HighLevelStateCommand.class);
      registerNewCommand(AbortWalkingCommand.class);
      registerNewCommand(PauseWalkingCommand.class);
      registerNewCommand(WholeBodyTrajectoryCommand.class);

      listOfSupportedMessages = new ArrayList<>(messageClassToBufferMap.keySet());
   }

   private <C extends Command<C, M>, M extends Packet<M>> ConcurrentRingBuffer<C> registerNewCommand(Class<C> commandClazz)
   {
      Builder<C> builer = createBuilderWithEmptyConstructor(commandClazz);
      ConcurrentRingBuffer<C> newBuffer = new ConcurrentRingBuffer<>(builer, buffersCapacity);
      allBuffers.add(newBuffer);
      // This is retarded, but I could not find another way that is more elegant.
      Class<M> messageClass = builer.newInstance().getMessageClass();
      commandClassToBufferMap.put(commandClazz, newBuffer);
      messageClassToBufferMap.put(messageClass, newBuffer);
      commandsMap.put(commandClazz, new RecyclingArrayList<>(buffersCapacity, commandClazz));

      return newBuffer;
   }

   public <M extends Packet<M>> void submitMessage(M message)
   {
      if (message.getUniqueId() == Packet.INVALID_MESSAGE_ID)
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
         submitControllerCommands(((MultipleCommandHolder) command).getControllerCommands());

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

   public List<Class<? extends Packet<?>>> getListOfSupportedMessages()
   {
      return listOfSupportedMessages;
   }
}
