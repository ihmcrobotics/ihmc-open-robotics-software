package us.ihmc.commonWalkingControlModules.controllerAPI.input;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ControllerMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableArmDesiredAccelerationsMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableArmTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableAutomaticManipulationAbortMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableChestTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableEndEffectorLoadBearingMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableFootTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableFootstepDataListMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableGoHomeMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableHandComplianceControlParametersMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableHandTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableHeadTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiablePelvisHeightTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiablePelvisOrientationTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiablePelvisTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableStopAllTrajectoryMessage;
import us.ihmc.communication.packets.Packet;
import us.ihmc.concurrent.Builder;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmDesiredAccelerationsMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandComplianceControlParametersMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.StopAllTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.AutomaticManipulationAbortMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.EndEffectorLoadBearingMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootstepDataListMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.HeadTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisHeightTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisOrientationTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.PelvisTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;
import us.ihmc.robotics.lists.RecyclingArrayList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.printing.PrintTools;

public class ControllerCommandInputManager
{
   private final int buffersCapacity = 8;

   private final List<ConcurrentRingBuffer<?>> allBuffers = new ArrayList<>();
   private final Map<Class<? extends ControllerMessage<?, ?>>, ConcurrentRingBuffer<? extends ControllerMessage<?, ?>>> modifiableMessageClassToBufferMap = new HashMap<>();
   private final Map<Class<? extends Packet<?>>, ConcurrentRingBuffer<? extends ControllerMessage<?, ?>>> messageClassToBufferMap = new HashMap<>();
   private final Map<Class<? extends ControllerMessage<?, ?>>, RecyclingArrayList<? extends ControllerMessage<?, ?>>> controllerMessagesMap = new HashMap<>();

   private final ConcurrentRingBuffer<ModifiableArmTrajectoryMessage> armTrajectoryMessageBuffer;
   private final ConcurrentRingBuffer<ModifiableHandTrajectoryMessage> handTrajectoryMessageBuffer;
   private final ConcurrentRingBuffer<ModifiableFootTrajectoryMessage> footTrajectoryMessageBuffer;
   private final ConcurrentRingBuffer<ModifiableHeadTrajectoryMessage> headTrajectoryMessageBuffer;
   private final ConcurrentRingBuffer<ModifiableChestTrajectoryMessage> chestTrajectoryMessageBuffer;
   private final ConcurrentRingBuffer<ModifiablePelvisTrajectoryMessage> pelvisTrajectoryMessageBuffer;
   private final ConcurrentRingBuffer<ModifiablePelvisOrientationTrajectoryMessage> pelvisOrientationTrajectoryMessageBuffer;
   private final ConcurrentRingBuffer<ModifiablePelvisHeightTrajectoryMessage> pelvisHeightTrajectoryMessageBuffer;
   private final ConcurrentRingBuffer<ModifiableGoHomeMessage> goHomeMessageBuffer;
   private final ConcurrentRingBuffer<ModifiableFootstepDataListMessage> footstepDataListMessageBuffer;
   private final ConcurrentRingBuffer<ModifiableEndEffectorLoadBearingMessage> endEffectorLoadBearingMessageBuffer;
   private final ConcurrentRingBuffer<ModifiableStopAllTrajectoryMessage> stopAllTrajectoryMessageBuffer;
   private final ConcurrentRingBuffer<ModifiableArmDesiredAccelerationsMessage> armDesiredAccelerationsMessageBuffer;
   private final ConcurrentRingBuffer<ModifiableAutomaticManipulationAbortMessage> automaticManipulationAbortMessageBuffer;
   private final ConcurrentRingBuffer<ModifiableHandComplianceControlParametersMessage> handComplianceControlParametersMessageBuffer;

   private final List<Class<? extends Packet<?>>> listOfSupportedMessages;

   public ControllerCommandInputManager()
   {
      armTrajectoryMessageBuffer = createBuffer(ModifiableArmTrajectoryMessage.class);
      handTrajectoryMessageBuffer = createBuffer(ModifiableHandTrajectoryMessage.class);
      footTrajectoryMessageBuffer = createBuffer(ModifiableFootTrajectoryMessage.class);
      headTrajectoryMessageBuffer = createBuffer(ModifiableHeadTrajectoryMessage.class);
      chestTrajectoryMessageBuffer = createBuffer(ModifiableChestTrajectoryMessage.class);
      pelvisTrajectoryMessageBuffer = createBuffer(ModifiablePelvisTrajectoryMessage.class);
      pelvisOrientationTrajectoryMessageBuffer = createBuffer(ModifiablePelvisOrientationTrajectoryMessage.class);
      pelvisHeightTrajectoryMessageBuffer = createBuffer(ModifiablePelvisHeightTrajectoryMessage.class);
      goHomeMessageBuffer = createBuffer(ModifiableGoHomeMessage.class);
      footstepDataListMessageBuffer = createBuffer(ModifiableFootstepDataListMessage.class);
      endEffectorLoadBearingMessageBuffer = createBuffer(ModifiableEndEffectorLoadBearingMessage.class);
      stopAllTrajectoryMessageBuffer = createBuffer(ModifiableStopAllTrajectoryMessage.class);
      armDesiredAccelerationsMessageBuffer = createBuffer(ModifiableArmDesiredAccelerationsMessage.class);
      automaticManipulationAbortMessageBuffer = createBuffer(ModifiableAutomaticManipulationAbortMessage.class);
      handComplianceControlParametersMessageBuffer = createBuffer(ModifiableHandComplianceControlParametersMessage.class);

      listOfSupportedMessages = new ArrayList<>(messageClassToBufferMap.keySet());
   }

   private <T extends ControllerMessage<T, M>, M extends Packet<M>> ConcurrentRingBuffer<T> createBuffer(Class<T> clazz)
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
         submitWholeBodyTrajectoryMessage((WholeBodyTrajectoryMessage) message);

      ConcurrentRingBuffer<? extends ControllerMessage<?, ?>> buffer = messageClassToBufferMap.get(message.getClass());
      if (buffer == null)
      {
         PrintTools.error(this, "The message type " + message.getClass().getSimpleName() + " is not supported.");
         return;
      }
      @SuppressWarnings("unchecked")
      ControllerMessage<?, M> nextModifiableMessage = (ControllerMessage<?, M>) buffer.next();
      if (nextModifiableMessage == null)
         return;
      nextModifiableMessage.set(message);
      buffer.commit();
   }

   public <T extends ControllerMessage<T, ?>> void submitModifiableMessage(T modifiableMessage)
   {
      if (modifiableMessage instanceof WholeBodyTrajectoryMessage)
         submitWholeBodyTrajectoryMessage((WholeBodyTrajectoryMessage) modifiableMessage);

      ConcurrentRingBuffer<? extends ControllerMessage<?, ?>> buffer = messageClassToBufferMap.get(modifiableMessage.getClass());
      if (buffer == null)
      {
         PrintTools.error(this, "The message type " + modifiableMessage.getClass().getSimpleName() + " is not supported.");
         return;
      }
      @SuppressWarnings("unchecked")
      ControllerMessage<T, ?> nextModifiableMessage = (ControllerMessage<T, ?>) buffer.next();
      if (nextModifiableMessage == null)
         return;
      nextModifiableMessage.set(modifiableMessage);
      buffer.commit();
   }

   public void submitArmTrajectoryMessage(ArmTrajectoryMessage message)
   {
      submitMessage(message);
   }

   public void submitHandTrajectoryMessage(HandTrajectoryMessage message)
   {
      submitMessage(message);
   }

   public void submitFootTrajectoryMessage(FootTrajectoryMessage message)
   {
      submitMessage(message);
   }

   public void submitHeadTrajectoryMessage(HeadTrajectoryMessage message)
   {
      submitMessage(message);
   }

   public void submitChestTrajectoryMessage(ChestTrajectoryMessage message)
   {
      submitMessage(message);
   }

   public void submitPelvisTrajectoryMessage(PelvisTrajectoryMessage message)
   {
      submitMessage(message);
   }

   public void submitPelvisOrientationTrajectoryMessage(PelvisOrientationTrajectoryMessage message)
   {
      submitMessage(message);
   }

   public void submitPelvisHeightTrajectoryMessage(PelvisHeightTrajectoryMessage message)
   {
      submitMessage(message);
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

   public void submitGoHomeMessage(GoHomeMessage message)
   {
      submitMessage(message);
   }

   public void submitFootstepDataListMessage(FootstepDataListMessage message)
   {
      submitMessage(message);
   }

   public void submitEndEffectorLoadBearingMessage(EndEffectorLoadBearingMessage message)
   {
      submitMessage(message);
   }

   public void submitStopAllTrajectoryMessage(StopAllTrajectoryMessage message)
   {
      submitMessage(message);
   }

   public void submitArmDesiredAccelerationsMessage(ArmDesiredAccelerationsMessage message)
   {
      submitMessage(message);
   }

   public void submitAutomaticManipulationAbortMessage(AutomaticManipulationAbortMessage message)
   {
      submitMessage(message);
   }

   public void submitHandComplianceControlParametersMessage(HandComplianceControlParametersMessage message)
   {
      submitMessage(message);
   }

   public void submitFootstepDataListMessage(ModifiableFootstepDataListMessage modifiableMessage)
   {
      submitModifiableMessage(modifiableMessage);
   }

   public boolean isNewMessageAvailable(Class<? extends ControllerMessage<?, ?>> messageClassToCheck)
   {
      return modifiableMessageClassToBufferMap.get(messageClassToCheck).poll();
   }

   public RecyclingArrayList<ModifiableHandTrajectoryMessage> pollHandTrajectoryMessages()
   {
      return pollNewMessages(ModifiableHandTrajectoryMessage.class);
   }

   public RecyclingArrayList<ModifiableArmTrajectoryMessage> pollArmTrajectoryMessages()
   {
      return pollNewMessages(ModifiableArmTrajectoryMessage.class);
   }

   public RecyclingArrayList<ModifiableArmDesiredAccelerationsMessage> pollArmDesiredAccelerationsMessages()
   {
      return pollNewMessages(ModifiableArmDesiredAccelerationsMessage.class);
   }

   public RecyclingArrayList<ModifiableHandComplianceControlParametersMessage> pollHandComplianceControlParametersMessages()
   {
      return pollNewMessages(ModifiableHandComplianceControlParametersMessage.class);
   }

   public RecyclingArrayList<ModifiableHeadTrajectoryMessage> pollHeadTrajectoryMessages()
   {
      return pollNewMessages(ModifiableHeadTrajectoryMessage.class);
   }

   public ModifiableHeadTrajectoryMessage pollNewestHeadTrajectoryMessage()
   {
      return pollNewestMessage(ModifiableHeadTrajectoryMessage.class);
   }

   public ModifiableChestTrajectoryMessage pollNewestChestTrajectoryMessage()
   {
      return pollNewestMessage(ModifiableChestTrajectoryMessage.class);
   }

   public ModifiablePelvisHeightTrajectoryMessage pollNewestPelvisHeightTrajectoryMessage()
   {
      return pollNewestMessage(ModifiablePelvisHeightTrajectoryMessage.class);
   }

   public void flushManipulationBuffers()
   {
      handTrajectoryMessageBuffer.flush();
      armTrajectoryMessageBuffer.flush();
      armDesiredAccelerationsMessageBuffer.flush();
      handComplianceControlParametersMessageBuffer.flush();
   }

   public void flushBuffers()
   {
      for (int i = 0; i < allBuffers.size(); i++)
         allBuffers.get(i).flush();
   }

   public <T extends ControllerMessage<T, ?>> T pollNewestMessage(Class<T> messageToPollClass)
   {
      return pollNewMessages(messageToPollClass).getLast();
   }

   @SuppressWarnings("unchecked")
   public <T extends ControllerMessage<T, ?>> RecyclingArrayList<T> pollNewMessages(Class<T> messageToPollClass)
   {
      RecyclingArrayList<T> messages = (RecyclingArrayList<T>) controllerMessagesMap.get(messageToPollClass);
      messages.clear();
      ConcurrentRingBuffer<T> buffer = (ConcurrentRingBuffer<T>) modifiableMessageClassToBufferMap.get(messageToPollClass);
      pollNewMessages(buffer, messages);
      return messages;
   }

   private static <T extends ControllerMessage<T, ?>> void pollNewMessages(ConcurrentRingBuffer<T> buffer, RecyclingArrayList<T> messagesToPack)
   {
      if (buffer.poll())
      {
         T message;
         while ((message = buffer.read()) != null)
         {
            messagesToPack.add().set(message);
         }
         buffer.flush();
      }
   }

   private static <U> Builder<U> createBuilderWithEmptyConstructor(Class<U> clazz)
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
