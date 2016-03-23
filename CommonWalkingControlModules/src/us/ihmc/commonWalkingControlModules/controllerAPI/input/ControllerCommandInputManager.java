package us.ihmc.commonWalkingControlModules.controllerAPI.input;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.AbortWalkingControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ArmDesiredAccelerationsControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ArmTrajectoryControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.AutomaticManipulationAbortControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ChestTrajectoryControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.EndEffectorLoadBearingControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.FootTrajectoryControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.FootstepDataListControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.GoHomeControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.HandComplianceControlParametersControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.HandTrajectoryControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.HeadTrajectoryControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.HighLevelStateControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.MultipleControllerCommandHolder;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.PauseWalkingControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.PelvisHeightTrajectoryControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.PelvisOrientationTrajectoryControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.PelvisTrajectoryControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.StopAllTrajectoryControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.WholeBodyTrajectoryControllerCommand;
import us.ihmc.communication.packets.MultiplePacketHolder;
import us.ihmc.communication.packets.Packet;
import us.ihmc.concurrent.Builder;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.tools.io.printing.PrintTools;

public class ControllerCommandInputManager
{
   private final int buffersCapacity = 8;

   private final List<ConcurrentRingBuffer<?>> allBuffers = new ArrayList<>();
   private final Map<Class<? extends ControllerCommand<?, ?>>, ConcurrentRingBuffer<? extends ControllerCommand<?, ?>>> commandClassToBufferMap = new HashMap<>();
   private final Map<Class<? extends Packet<?>>, ConcurrentRingBuffer<? extends ControllerCommand<?, ?>>> messageClassToBufferMap = new HashMap<>();
   private final Map<Class<? extends ControllerCommand<?, ?>>, RecyclingArrayList<? extends ControllerCommand<?, ?>>> controllerCommandsMap = new HashMap<>();

   private final List<Class<? extends Packet<?>>> listOfSupportedMessages;

   public ControllerCommandInputManager()
   {
      registerNewControllerCommand(ArmTrajectoryControllerCommand.class);
      registerNewControllerCommand(HandTrajectoryControllerCommand.class);
      registerNewControllerCommand(FootTrajectoryControllerCommand.class);
      registerNewControllerCommand(HeadTrajectoryControllerCommand.class);
      registerNewControllerCommand(ChestTrajectoryControllerCommand.class);
      registerNewControllerCommand(PelvisTrajectoryControllerCommand.class);
      registerNewControllerCommand(PelvisOrientationTrajectoryControllerCommand.class);
      registerNewControllerCommand(PelvisHeightTrajectoryControllerCommand.class);
      registerNewControllerCommand(StopAllTrajectoryControllerCommand.class);
      registerNewControllerCommand(FootstepDataListControllerCommand.class);
      registerNewControllerCommand(GoHomeControllerCommand.class);
      registerNewControllerCommand(EndEffectorLoadBearingControllerCommand.class);
      registerNewControllerCommand(ArmDesiredAccelerationsControllerCommand.class);
      registerNewControllerCommand(AutomaticManipulationAbortControllerCommand.class);
      registerNewControllerCommand(HandComplianceControlParametersControllerCommand.class);
      registerNewControllerCommand(HighLevelStateControllerCommand.class);
      registerNewControllerCommand(AbortWalkingControllerCommand.class);
      registerNewControllerCommand(PauseWalkingControllerCommand.class);
      registerNewControllerCommand(WholeBodyTrajectoryControllerCommand.class);

      listOfSupportedMessages = new ArrayList<>(messageClassToBufferMap.keySet());
   }

   private <T extends ControllerCommand<T, M>, M extends Packet<M>> ConcurrentRingBuffer<T> registerNewControllerCommand(Class<T> clazz)
   {
      Builder<T> builer = createBuilderWithEmptyConstructor(clazz);
      ConcurrentRingBuffer<T> newBuffer = new ConcurrentRingBuffer<>(builer, buffersCapacity);
      allBuffers.add(newBuffer);
      // This is retarded, but I could not find another way that is more elegant.
      Class<M> messageClass = builer.newInstance().getMessageClass();
      commandClassToBufferMap.put(clazz, newBuffer);
      messageClassToBufferMap.put(messageClass, newBuffer);
      controllerCommandsMap.put(clazz, new RecyclingArrayList<>(buffersCapacity, clazz));

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

      ConcurrentRingBuffer<? extends ControllerCommand<?, ?>> buffer = messageClassToBufferMap.get(message.getClass());
      if (buffer == null)
      {
         PrintTools.error(this, "The message type " + message.getClass().getSimpleName() + " is not supported.");
         return;
      }
      @SuppressWarnings("unchecked")
      ControllerCommand<?, M> nextControllerCommand = (ControllerCommand<?, M>) buffer.next();
      if (nextControllerCommand == null)
         return;
      nextControllerCommand.set(message);
      buffer.commit();
   }

   @SuppressWarnings("unchecked")
   public <M extends Packet<M>> void submitMessages(List<Packet<?>> messages)
   {
      for (int i = 0; i < messages.size(); i++)
         submitMessage((M) messages.get(i));
   }

   public <T extends ControllerCommand<T, ?>> void submitControllerCommand(T controllerCommand)
   {
      if (!controllerCommand.isCommandValid())
         return;

      if (controllerCommand instanceof MultipleControllerCommandHolder)
         submitControllerCommands(((MultipleControllerCommandHolder) controllerCommand).getControllerCommands());

      ConcurrentRingBuffer<? extends ControllerCommand<?, ?>> buffer = commandClassToBufferMap.get(controllerCommand.getClass());
      if (buffer == null)
      {
         PrintTools.error(this, "The message type " + controllerCommand.getClass().getSimpleName() + " is not supported.");
         return;
      }
      @SuppressWarnings("unchecked")
      ControllerCommand<T, ?> nextModifiableMessage = (ControllerCommand<T, ?>) buffer.next();
      if (nextModifiableMessage == null)
         return;
      nextModifiableMessage.set(controllerCommand);
      buffer.commit();
   }

   @SuppressWarnings("unchecked")
   public <T extends ControllerCommand<T, ?>> void submitControllerCommands(List<ControllerCommand<?, ?>> controllerCommands)
   {
      for (int i = 0; i < controllerCommands.size(); i++)
         submitControllerCommand((T) controllerCommands.get(i));
   }

   public boolean isNewMessageAvailable(Class<? extends ControllerCommand<?, ?>> commandClassToCheck)
   {
      return commandClassToBufferMap.get(commandClassToCheck).poll();
   }

   public EndEffectorLoadBearingControllerCommand pollAndCompileEndEffectorLoadBearingMessages()
   {
      List<EndEffectorLoadBearingControllerCommand> messages = pollNewMessages(EndEffectorLoadBearingControllerCommand.class);
      for (int i = 1; i < messages.size(); i++)
         messages.get(0).set(messages.get(i));
      return messages.get(0);
   }

   public GoHomeControllerCommand pollAndCompileGoHomeMessages()
   {
      List<GoHomeControllerCommand> messages = pollNewMessages(GoHomeControllerCommand.class);
      for (int i = 1; i < messages.size(); i++)
         messages.get(0).set(messages.get(i));
      return messages.get(0);
   }

   public void flushManipulationBuffers()
   {
      flushMessages(HandTrajectoryControllerCommand.class);
      flushMessages(ArmTrajectoryControllerCommand.class);
      flushMessages(ArmDesiredAccelerationsControllerCommand.class);
      flushMessages(HandComplianceControlParametersControllerCommand.class);
   }

   public void flushPelvisBuffers()
   {
      flushMessages(PelvisTrajectoryControllerCommand.class);
      flushMessages(PelvisOrientationTrajectoryControllerCommand.class);
      flushMessages(PelvisHeightTrajectoryControllerCommand.class);
   }

   public void flushFootstepBuffers()
   {
      flushMessages(FootstepDataListControllerCommand.class);
   }

   public void flushFlamingoBuffers()
   {
      flushMessages(FootTrajectoryControllerCommand.class);
   }

   public void flushBuffers()
   {
      for (int i = 0; i < allBuffers.size(); i++)
         allBuffers.get(i).flush();
   }

   public <T extends ControllerCommand<T, ?>> void flushMessages(Class<T> messageToFlushClass)
   {
      commandClassToBufferMap.get(messageToFlushClass).flush();
   }

   public <T extends ControllerCommand<T, ?>> T pollNewestMessage(Class<T> messageToPollClass)
   {
      return ((RecyclingArrayList<T>) pollNewMessages(messageToPollClass)).getLast();
   }

   @SuppressWarnings("unchecked")
   public <T extends ControllerCommand<T, ?>> List<T> pollNewMessages(Class<T> messageToPollClass)
   {
      RecyclingArrayList<T> messages = (RecyclingArrayList<T>) controllerCommandsMap.get(messageToPollClass);
      messages.clear();
      ConcurrentRingBuffer<T> buffer = (ConcurrentRingBuffer<T>) commandClassToBufferMap.get(messageToPollClass);
      pollNewMessages(buffer, messages);
      return messages;
   }

   private static <T extends ControllerCommand<T, ?>> void pollNewMessages(ConcurrentRingBuffer<T> buffer, RecyclingArrayList<T> messagesToPack)
   {
      if (buffer.poll())
      {
         T message;
         while ((message = buffer.read()) != null)
         {
            messagesToPack.add().set(message);
            message.clear();
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
