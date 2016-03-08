package us.ihmc.commonWalkingControlModules.controllerAPI.input;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.AbortWalkingControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ArmDesiredAccelerationsControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ArmTrajectoryControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.AutomaticManipulationAbortControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ChestTrajectoryControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.EndEffectorLoadBearingControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.FootTrajectoryControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.FootstepDataListControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.GoHomeControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.HandComplianceControlParametersControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.HandTrajectoryControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.HeadTrajectoryControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.HighLevelStateControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.PauseWalkingControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.PelvisHeightTrajectoryControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.PelvisOrientationTrajectoryControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.PelvisTrajectoryControllerCommand;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.StopAllTrajectoryControllerCommand;
import us.ihmc.communication.packets.Packet;
import us.ihmc.concurrent.Builder;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.printing.PrintTools;

public class ControllerCommandInputManager
{
   private final int buffersCapacity = 8;

   private final List<ConcurrentRingBuffer<?>> allBuffers = new ArrayList<>();
   private final Map<Class<? extends ControllerCommand<?, ?>>, ConcurrentRingBuffer<? extends ControllerCommand<?, ?>>> modifiableMessageClassToBufferMap = new HashMap<>();
   private final Map<Class<? extends Packet<?>>, ConcurrentRingBuffer<? extends ControllerCommand<?, ?>>> messageClassToBufferMap = new HashMap<>();
   private final Map<Class<? extends ControllerCommand<?, ?>>, RecyclingArrayList<? extends ControllerCommand<?, ?>>> controllerMessagesMap = new HashMap<>();

   private final List<Class<? extends Packet<?>>> listOfSupportedMessages;

   public ControllerCommandInputManager()
   {
      createBuffer(ArmTrajectoryControllerCommand.class);
      createBuffer(HandTrajectoryControllerCommand.class);
      createBuffer(FootTrajectoryControllerCommand.class);
      createBuffer(HeadTrajectoryControllerCommand.class);
      createBuffer(ChestTrajectoryControllerCommand.class);
      createBuffer(PelvisTrajectoryControllerCommand.class);
      createBuffer(PelvisOrientationTrajectoryControllerCommand.class);
      createBuffer(PelvisHeightTrajectoryControllerCommand.class);
      createBuffer(StopAllTrajectoryControllerCommand.class);
      createBuffer(FootstepDataListControllerCommand.class);
      createBuffer(GoHomeControllerCommand.class);
      createBuffer(EndEffectorLoadBearingControllerCommand.class);
      createBuffer(ArmDesiredAccelerationsControllerCommand.class);
      createBuffer(AutomaticManipulationAbortControllerCommand.class);
      createBuffer(HandComplianceControlParametersControllerCommand.class);
      createBuffer(HighLevelStateControllerCommand.class);
      createBuffer(AbortWalkingControllerCommand.class);
      createBuffer(PauseWalkingControllerCommand.class);

      listOfSupportedMessages = new ArrayList<>(messageClassToBufferMap.keySet());
      // This message has to be added manually as it is handled in a different way to the others.
      listOfSupportedMessages.add(WholeBodyTrajectoryMessage.class);
   }

   private <T extends ControllerCommand<T, M>, M extends Packet<M>> ConcurrentRingBuffer<T> createBuffer(Class<T> clazz)
   {
      Builder<T> builer = createBuilderWithEmptyConstructor(clazz);
      ConcurrentRingBuffer<T> newBuffer = new ConcurrentRingBuffer<>(builer, buffersCapacity);
      allBuffers.add(newBuffer);
      // This is retarded, but I could not find another way that is more elegant.
      Class<M> messageClass = builer.newInstance().getMessageClass();
      modifiableMessageClassToBufferMap.put(clazz, newBuffer);
      messageClassToBufferMap.put(messageClass, newBuffer);
      controllerMessagesMap.put(clazz, new RecyclingArrayList<>(buffersCapacity, clazz));

      return newBuffer;
   }

   public <M extends Packet<M>> void submitMessage(M message)
   {
      if (message instanceof WholeBodyTrajectoryMessage)
      {
         submitWholeBodyTrajectoryMessage((WholeBodyTrajectoryMessage) message);
         return;
      }

      ConcurrentRingBuffer<? extends ControllerCommand<?, ?>> buffer = messageClassToBufferMap.get(message.getClass());
      if (buffer == null)
      {
         PrintTools.error(this, "The message type " + message.getClass().getSimpleName() + " is not supported.");
         return;
      }
      @SuppressWarnings("unchecked")
      ControllerCommand<?, M> nextModifiableMessage = (ControllerCommand<?, M>) buffer.next();
      if (nextModifiableMessage == null)
         return;
      nextModifiableMessage.set(message);
      buffer.commit();
   }

   public <T extends ControllerCommand<T, ?>> void submitModifiableMessage(T modifiableMessage)
   {
      ConcurrentRingBuffer<? extends ControllerCommand<?, ?>> buffer = modifiableMessageClassToBufferMap.get(modifiableMessage.getClass());
      if (buffer == null)
      {
         PrintTools.error(this, "The message type " + modifiableMessage.getClass().getSimpleName() + " is not supported.");
         return;
      }
      @SuppressWarnings("unchecked")
      ControllerCommand<T, ?> nextModifiableMessage = (ControllerCommand<T, ?>) buffer.next();
      if (nextModifiableMessage == null)
         return;
      nextModifiableMessage.set(modifiableMessage);
      buffer.commit();
   }

   public void submitWholeBodyTrajectoryMessage(WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         ArmTrajectoryMessage armTrajectoryMessage = wholeBodyTrajectoryMessage.getArmTrajectoryMessage(robotSide);
         if (armTrajectoryMessage != null && armTrajectoryMessage.getUniqueId() != Packet.INVALID_MESSAGE_ID)
            submitMessage(armTrajectoryMessage);
         HandTrajectoryMessage handTrajectoryMessage = wholeBodyTrajectoryMessage.getHandTrajectoryMessage(robotSide);
         if (handTrajectoryMessage != null && handTrajectoryMessage.getUniqueId() != Packet.INVALID_MESSAGE_ID)
            submitMessage(handTrajectoryMessage);
         FootTrajectoryMessage footTrajectoryMessage = wholeBodyTrajectoryMessage.getFootTrajectoryMessage(robotSide);
         if (footTrajectoryMessage != null && footTrajectoryMessage.getUniqueId() != Packet.INVALID_MESSAGE_ID)
            submitMessage(footTrajectoryMessage);
      }

      PelvisTrajectoryMessage pelvisTrajectoryMessage = wholeBodyTrajectoryMessage.getPelvisTrajectoryMessage();
      if (pelvisTrajectoryMessage != null && pelvisTrajectoryMessage.getUniqueId() != Packet.INVALID_MESSAGE_ID)
         submitMessage(pelvisTrajectoryMessage);
      ChestTrajectoryMessage chestTrajectoryMessage = wholeBodyTrajectoryMessage.getChestTrajectoryMessage();
      if (chestTrajectoryMessage != null && chestTrajectoryMessage.getUniqueId() != Packet.INVALID_MESSAGE_ID)
         submitMessage(chestTrajectoryMessage);
   }

   public boolean isNewMessageAvailable(Class<? extends ControllerCommand<?, ?>> messageClassToCheck)
   {
      return modifiableMessageClassToBufferMap.get(messageClassToCheck).poll();
   }

   public EndEffectorLoadBearingControllerCommand pollAndCompileEndEffectorLoadBearingMessages()
   {
      RecyclingArrayList<EndEffectorLoadBearingControllerCommand> messages = pollNewMessages(EndEffectorLoadBearingControllerCommand.class);
      for (int i = 1; i < messages.size(); i++)
         messages.get(0).set(messages.get(i));
      return messages.get(0);
   }

   public GoHomeControllerCommand pollAndCompileGoHomeMessages()
   {
      RecyclingArrayList<GoHomeControllerCommand> messages = pollNewMessages(GoHomeControllerCommand.class);
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
      modifiableMessageClassToBufferMap.get(messageToFlushClass).flush();
   }

   public <T extends ControllerCommand<T, ?>> T pollNewestMessage(Class<T> messageToPollClass)
   {
      return pollNewMessages(messageToPollClass).getLast();
   }

   @SuppressWarnings("unchecked")
   public <T extends ControllerCommand<T, ?>> RecyclingArrayList<T> pollNewMessages(Class<T> messageToPollClass)
   {
      RecyclingArrayList<T> messages = (RecyclingArrayList<T>) controllerMessagesMap.get(messageToPollClass);
      messages.clear();
      ConcurrentRingBuffer<T> buffer = (ConcurrentRingBuffer<T>) modifiableMessageClassToBufferMap.get(messageToPollClass);
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
