package us.ihmc.commonWalkingControlModules.controllerAPI.input;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.List;

import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableArmDesiredAccelerationsMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableArmTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableChestTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableEndEffectorLoadBearingMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableFootTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableFootstepDataListMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableGoHomeMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableHandTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableHeadTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiablePelvisHeightTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiablePelvisOrientationTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiablePelvisTrajectoryMessage;
import us.ihmc.communication.packets.Packet;
import us.ihmc.concurrent.Builder;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmDesiredAccelerationsMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
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
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class ControllerCommandInputManager
{
   private final int buffersCapacity = 8;

   private final SideDependentList<ConcurrentRingBuffer<ModifiableArmTrajectoryMessage>> armTrajectoryMessageBuffers = new SideDependentList<>();
   private final ModifiableArmTrajectoryMessage controllerArmTrajectoryMessage = new ModifiableArmTrajectoryMessage();
   private final SideDependentList<ConcurrentRingBuffer<ModifiableHandTrajectoryMessage>> handTrajectoryMessageBuffers = new SideDependentList<>();
   private final ModifiableHandTrajectoryMessage controllerHandTrajectoryMessage = new ModifiableHandTrajectoryMessage();
   private final SideDependentList<ConcurrentRingBuffer<ModifiableFootTrajectoryMessage>> footTrajectoryMessageBuffers = new SideDependentList<>();
   private final ModifiableFootTrajectoryMessage controllerFootTrajectoryMessage = new ModifiableFootTrajectoryMessage();
   private final ConcurrentRingBuffer<ModifiableHeadTrajectoryMessage> headTrajectoryMessageBuffer;
   private final ModifiableHeadTrajectoryMessage controllerHeadTrajectoryMessage = new ModifiableHeadTrajectoryMessage();
   private final ConcurrentRingBuffer<ModifiableChestTrajectoryMessage> chestTrajectoryMessageBuffer;
   private final ModifiableChestTrajectoryMessage controllerChestTrajectoryMessage = new ModifiableChestTrajectoryMessage();
   private final ConcurrentRingBuffer<ModifiablePelvisTrajectoryMessage> pelvisTrajectoryMessageBuffer;
   private final ModifiablePelvisTrajectoryMessage controllerPelvisTrajectoryMessage = new ModifiablePelvisTrajectoryMessage();
   private final ConcurrentRingBuffer<ModifiablePelvisOrientationTrajectoryMessage> pelvisOrientationTrajectoryMessageBuffer;
   private final ModifiablePelvisOrientationTrajectoryMessage controllerPelvisOrientationTrajectoryMessage = new ModifiablePelvisOrientationTrajectoryMessage();
   private final ConcurrentRingBuffer<ModifiablePelvisHeightTrajectoryMessage> pelvisHeightTrajectoryMessageBuffer;
   private final ModifiablePelvisHeightTrajectoryMessage controllerPelvisHeightTrajectoryMessage = new ModifiablePelvisHeightTrajectoryMessage();

   private final ConcurrentRingBuffer<ModifiableGoHomeMessage> goHomeMessageBuffer;
   private final ModifiableGoHomeMessage controllerGoHomeMessage = new ModifiableGoHomeMessage();

   private final ConcurrentRingBuffer<ModifiableFootstepDataListMessage> footstepDataListMessageBuffer;
   private final ModifiableFootstepDataListMessage controllerFootstepDataListMessage = new ModifiableFootstepDataListMessage();

   private final ConcurrentRingBuffer<ModifiableEndEffectorLoadBearingMessage> endEffectorLoadBearingMessageBuffer;
   private final ModifiableEndEffectorLoadBearingMessage controllerEndEffectorLoadBearingMessage = new ModifiableEndEffectorLoadBearingMessage();

   private final ConcurrentRingBuffer<StopAllTrajectoryMessage> stopAllTrajectoryMessageBuffer;

   private final SideDependentList<ConcurrentRingBuffer<ModifiableArmDesiredAccelerationsMessage>> armDesiredAccelerationsMessageBuffers = new SideDependentList<>();
   private final ModifiableArmDesiredAccelerationsMessage controllerArmDesiredAccelerationsMessage = new ModifiableArmDesiredAccelerationsMessage();

   private final ConcurrentRingBuffer<AutomaticManipulationAbortMessage> automaticManipulationAbortMessageBuffer;
   private final AutomaticManipulationAbortMessage controllerAutomaticManipulationAbortMessage = new AutomaticManipulationAbortMessage();

   private final List<ConcurrentRingBuffer<?>> allBuffers = new ArrayList<>();

   public ControllerCommandInputManager()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         armTrajectoryMessageBuffers.put(robotSide, createBuffer(ModifiableArmTrajectoryMessage.class));
         handTrajectoryMessageBuffers.put(robotSide, createBuffer(ModifiableHandTrajectoryMessage.class));
         footTrajectoryMessageBuffers.put(robotSide, createBuffer(ModifiableFootTrajectoryMessage.class));
         armDesiredAccelerationsMessageBuffers.put(robotSide, createBuffer(ModifiableArmDesiredAccelerationsMessage.class)); 
      }
      headTrajectoryMessageBuffer = createBuffer(ModifiableHeadTrajectoryMessage.class);
      chestTrajectoryMessageBuffer = createBuffer(ModifiableChestTrajectoryMessage.class);
      pelvisTrajectoryMessageBuffer = createBuffer(ModifiablePelvisTrajectoryMessage.class);
      pelvisOrientationTrajectoryMessageBuffer = createBuffer(ModifiablePelvisOrientationTrajectoryMessage.class);
      pelvisHeightTrajectoryMessageBuffer = createBuffer(ModifiablePelvisHeightTrajectoryMessage.class);

      goHomeMessageBuffer = createBuffer(ModifiableGoHomeMessage.class);

      footstepDataListMessageBuffer = createBuffer(ModifiableFootstepDataListMessage.class);

      endEffectorLoadBearingMessageBuffer = createBuffer(ModifiableEndEffectorLoadBearingMessage.class);
      stopAllTrajectoryMessageBuffer = createBuffer(StopAllTrajectoryMessage.class);

      automaticManipulationAbortMessageBuffer = createBuffer(AutomaticManipulationAbortMessage.class);
   }

   private <T> ConcurrentRingBuffer<T> createBuffer(Class<T> clazz)
   {
      ConcurrentRingBuffer<T> newBuffer = new ConcurrentRingBuffer<>(createBuilderWithEmptyConstructor(clazz), buffersCapacity);
      allBuffers.add(newBuffer);
      return newBuffer;
   }

   public void submitArmTrajectoryMessage(ArmTrajectoryMessage armTrajectoryMessage)
   {
      ConcurrentRingBuffer<ModifiableArmTrajectoryMessage> buffer = armTrajectoryMessageBuffers.get(armTrajectoryMessage.getRobotSide());

      ModifiableArmTrajectoryMessage nextModifiableMessage = buffer.next();
      if (nextModifiableMessage == null)
         return;
      nextModifiableMessage.set(armTrajectoryMessage);
      buffer.commit();
   }

   public void submitHandTrajectoryMessage(HandTrajectoryMessage handTrajectoryMessage)
   {
      ConcurrentRingBuffer<ModifiableHandTrajectoryMessage> buffer = handTrajectoryMessageBuffers.get(handTrajectoryMessage.getRobotSide());

      ModifiableHandTrajectoryMessage nextModifiableMessage = buffer.next();
      if (nextModifiableMessage == null)
         return;
      nextModifiableMessage.set(handTrajectoryMessage);
      buffer.commit();
   }

   public void submitFootTrajectoryMessage(FootTrajectoryMessage footTrajectoryMessage)
   {
      ConcurrentRingBuffer<ModifiableFootTrajectoryMessage> buffer = footTrajectoryMessageBuffers.get(footTrajectoryMessage.getRobotSide());

      ModifiableFootTrajectoryMessage nextModifiableMessage = buffer.next();
      if (nextModifiableMessage == null)
         return;
      nextModifiableMessage.set(footTrajectoryMessage);
      buffer.commit();
   }

   public void submitHeadTrajectoryMessage(HeadTrajectoryMessage headTrajectoryMessage)
   {
      ModifiableHeadTrajectoryMessage nextModifiableMessage = headTrajectoryMessageBuffer.next();
      if (nextModifiableMessage == null)
         return;
      nextModifiableMessage.set(headTrajectoryMessage);
      headTrajectoryMessageBuffer.commit();
   }

   public void submitChestTrajectoryMessage(ChestTrajectoryMessage chestTrajectoryMessage)
   {
      ModifiableChestTrajectoryMessage nextModifiableMessage = chestTrajectoryMessageBuffer.next();
      if (nextModifiableMessage == null)
         return;
      nextModifiableMessage.set(chestTrajectoryMessage);
      chestTrajectoryMessageBuffer.commit();
   }

   public void submitPelvisTrajectoryMessage(PelvisTrajectoryMessage pelvisTrajectoryMessage)
   {
      ModifiablePelvisTrajectoryMessage nextModifiableMessage = pelvisTrajectoryMessageBuffer.next();
      if (nextModifiableMessage == null)
         return;
      nextModifiableMessage.set(pelvisTrajectoryMessage);
      pelvisTrajectoryMessageBuffer.commit();
   }

   public void submitPelvisOrientationTrajectoryMessage(PelvisOrientationTrajectoryMessage pelvisOrientationTrajectoryMessage)
   {
      ModifiablePelvisOrientationTrajectoryMessage nextModifiableMessage = pelvisOrientationTrajectoryMessageBuffer.next();
      if (nextModifiableMessage == null)
         return;
      nextModifiableMessage.set(pelvisOrientationTrajectoryMessage);
      pelvisOrientationTrajectoryMessageBuffer.commit();
   }

   public void submitPelvisHeightTrajectoryMessage(PelvisHeightTrajectoryMessage pelvisHeightTrajectoryMessage)
   {
      ModifiablePelvisHeightTrajectoryMessage nextModifiableMessage = pelvisHeightTrajectoryMessageBuffer.next();
      if (nextModifiableMessage == null)
         return;
      nextModifiableMessage.set(pelvisHeightTrajectoryMessage);
      pelvisHeightTrajectoryMessageBuffer.commit();
   }

   public void submitWholeBodyTrajectoryMessage(WholeBodyTrajectoryMessage wholeBodyTrajectoryMessage)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         ArmTrajectoryMessage armTrajectoryMessage = wholeBodyTrajectoryMessage.getArmTrajectoryMessage(robotSide);
         if (armTrajectoryMessage != null && armTrajectoryMessage.getUniqueId() != Packet.INVALID_MESSAGE_ID)
            submitArmTrajectoryMessage(armTrajectoryMessage);
         HandTrajectoryMessage handTrajectoryMessage = wholeBodyTrajectoryMessage.getHandTrajectoryMessage(robotSide);
         if (handTrajectoryMessage != null && handTrajectoryMessage.getUniqueId() != Packet.INVALID_MESSAGE_ID)
            submitHandTrajectoryMessage(handTrajectoryMessage);
         FootTrajectoryMessage footTrajectoryMessage = wholeBodyTrajectoryMessage.getFootTrajectoryMessage(robotSide);
         if (footTrajectoryMessage != null && footTrajectoryMessage.getUniqueId() != Packet.INVALID_MESSAGE_ID)
            submitFootTrajectoryMessage(footTrajectoryMessage);
      }

      PelvisTrajectoryMessage pelvisTrajectoryMessage = wholeBodyTrajectoryMessage.getPelvisTrajectoryMessage();
      if (pelvisTrajectoryMessage != null && pelvisTrajectoryMessage.getUniqueId() != Packet.INVALID_MESSAGE_ID)
         submitPelvisTrajectoryMessage(pelvisTrajectoryMessage);
      ChestTrajectoryMessage chestTrajectoryMessage = wholeBodyTrajectoryMessage.getChestTrajectoryMessage();
      if (chestTrajectoryMessage != null && chestTrajectoryMessage.getUniqueId() != Packet.INVALID_MESSAGE_ID)
         submitChestTrajectoryMessage(chestTrajectoryMessage);
   }

   public void submitGoHomeMessage(GoHomeMessage goHomeMessage)
   {
      ModifiableGoHomeMessage nextModifiableMessage = goHomeMessageBuffer.next();
      if (nextModifiableMessage == null)
         return;
      goHomeMessageBuffer.commit();
   }

   public void submitFootstepDataListMessage(FootstepDataListMessage footstepDataListMessage)
   {
      ModifiableFootstepDataListMessage nextModifiableMessage = footstepDataListMessageBuffer.next();
      if (nextModifiableMessage == null)
         return;
      nextModifiableMessage.set(footstepDataListMessage);
      footstepDataListMessageBuffer.commit();
   }

   public void submitFootstepDataListMessage(ModifiableFootstepDataListMessage footstepDataListMessage)
   {
      ModifiableFootstepDataListMessage nextModifiableMessage = footstepDataListMessageBuffer.next();
      if (nextModifiableMessage == null)
         return;
      nextModifiableMessage.set(footstepDataListMessage);
      footstepDataListMessageBuffer.commit();
   }

   public void submitEndEffectorLoadBearingMessage(EndEffectorLoadBearingMessage endEffectorLoadBearingMessage)
   {
      ModifiableEndEffectorLoadBearingMessage nextModifiableMessage = endEffectorLoadBearingMessageBuffer.next();
      if (nextModifiableMessage == null)
         return;
      nextModifiableMessage.set(endEffectorLoadBearingMessage);
      endEffectorLoadBearingMessageBuffer.commit();
   }

   public void submitStopAllTrajectoryMessage(StopAllTrajectoryMessage stopAllTrajectoryMessage)
   {
      StopAllTrajectoryMessage nextModifiableMessage = stopAllTrajectoryMessageBuffer.next();
      if (nextModifiableMessage == null)
         return;
      stopAllTrajectoryMessageBuffer.commit();
   }

   public void submitArmDesiredAccelerationsMessage(ArmDesiredAccelerationsMessage armDesiredAccelerationsMessage)
   {
      RobotSide robotSide = armDesiredAccelerationsMessage.getRobotSide();
      ConcurrentRingBuffer<ModifiableArmDesiredAccelerationsMessage> buffer = armDesiredAccelerationsMessageBuffers.get(robotSide);
      ModifiableArmDesiredAccelerationsMessage nextModifiableMessage = buffer.next();
      if (nextModifiableMessage == null)
         return;
      nextModifiableMessage.set(armDesiredAccelerationsMessage);
      buffer.commit();
   }

   public void submitAutomaticManipulationAbortMessage(AutomaticManipulationAbortMessage automaticManipulationAbortMessage)
   {
      AutomaticManipulationAbortMessage nextModifiableMessage = automaticManipulationAbortMessageBuffer.next();
      if (nextModifiableMessage == null)
         return;
      automaticManipulationAbortMessageBuffer.commit();
   }

   public boolean isNewHandTrajectoryMessageAvailable(RobotSide robotSide)
   {
      return handTrajectoryMessageBuffers.get(robotSide).poll();
   }

   public boolean isNewArmTrajectoryMessageAvailable(RobotSide robotSide)
   {
      return armTrajectoryMessageBuffers.get(robotSide).poll();
   }

   public boolean isNewFootTrajectoryMessageAvailable(RobotSide robotSide)
   {
      return footTrajectoryMessageBuffers.get(robotSide).poll();
   }

   public boolean isNewHeadTrajectoryMessageAvailable()
   {
      return headTrajectoryMessageBuffer.poll();
   }

   public boolean isNewChestTrajectoryMessageAvailable()
   {
      return chestTrajectoryMessageBuffer.poll();
   }

   public boolean isNewPelvisTrajectoryMessageAvailable()
   {
      return pelvisTrajectoryMessageBuffer.poll();
   }

   public boolean isNewPelvisOrientationTrajectoryMessageAvailable()
   {
      return pelvisOrientationTrajectoryMessageBuffer.poll();
   }

   public boolean isNewPelvisHeightTrajectoryMessageAvailable()
   {
      return pelvisHeightTrajectoryMessageBuffer.poll();
   }

   public boolean isNewGoHomeMessageAvailable()
   {
      return goHomeMessageBuffer.poll();
   }

   public boolean isNewFootstepDataListMessageAvailable()
   {
      return footstepDataListMessageBuffer.poll();
   }

   public boolean isNewEndEffectorLoadBearingMessageAvailable()
   {
      return endEffectorLoadBearingMessageBuffer.poll();
   }

   public boolean isNewStopAllTrajectoryMessageAvailable()
   {
      return stopAllTrajectoryMessageBuffer.poll();
   }

   public boolean isNewArmDesiredAccelerationsMessageAvailable(RobotSide robotSide)
   {
      return armDesiredAccelerationsMessageBuffers.get(robotSide).poll();
   }

   public boolean isNewAutomaticManipulationAbortMessageAvailable()
   {
      return automaticManipulationAbortMessageBuffer.poll();
   }

   public ModifiableHandTrajectoryMessage pollHandTrajectoryMessage(RobotSide robotSide)
   {
      controllerHandTrajectoryMessage.set(pollNewestMessage(handTrajectoryMessageBuffers.get(robotSide)));
      return controllerHandTrajectoryMessage;
   }

   public ModifiableArmTrajectoryMessage pollArmTrajectoryMessage(RobotSide robotSide)
   {
      controllerArmTrajectoryMessage.set(pollNewestMessage(armTrajectoryMessageBuffers.get(robotSide)));
      return controllerArmTrajectoryMessage;
   }

   public ModifiableFootTrajectoryMessage pollFootTrajectoryMessage(RobotSide robotSide)
   {
      controllerFootTrajectoryMessage.set(pollNewestMessage(footTrajectoryMessageBuffers.get(robotSide)));
      return controllerFootTrajectoryMessage;
   }

   public ModifiableHeadTrajectoryMessage pollHeadTrajectoryMessage()
   {
      controllerHeadTrajectoryMessage.set(pollNewestMessage(headTrajectoryMessageBuffer));
      return controllerHeadTrajectoryMessage;
   }

   public ModifiableChestTrajectoryMessage pollChestTrajectoryMessage()
   {
      controllerChestTrajectoryMessage.set(pollNewestMessage(chestTrajectoryMessageBuffer));
      return controllerChestTrajectoryMessage;
   }

   public ModifiablePelvisTrajectoryMessage pollPelvisTrajectoryMessage()
   {
      controllerPelvisTrajectoryMessage.set(pollNewestMessage(pelvisTrajectoryMessageBuffer));
      return controllerPelvisTrajectoryMessage;
   }

   public ModifiablePelvisOrientationTrajectoryMessage pollPelvisOrientationTrajectoryMessage()
   {
      controllerPelvisOrientationTrajectoryMessage.set(pollNewestMessage(pelvisOrientationTrajectoryMessageBuffer));
      return controllerPelvisOrientationTrajectoryMessage;
   }

   public ModifiablePelvisHeightTrajectoryMessage pollPelvisHeightTrajectoryMessage()
   {
      controllerPelvisHeightTrajectoryMessage.set(pollNewestMessage(pelvisHeightTrajectoryMessageBuffer));
      return controllerPelvisHeightTrajectoryMessage;
   }

   public ModifiableGoHomeMessage pollGoHomeMessage()
   {
      controllerGoHomeMessage.set(pollNewestMessage(goHomeMessageBuffer));
      return controllerGoHomeMessage;
   }

   public ModifiableFootstepDataListMessage pollFootstepDataListMessage()
   {
      controllerFootstepDataListMessage.set(pollNewestMessage(footstepDataListMessageBuffer));
      return controllerFootstepDataListMessage;
   }

   public ModifiableEndEffectorLoadBearingMessage pollEndEffectorLoadBearingMessage()
   {
      controllerEndEffectorLoadBearingMessage.set(pollNewestMessage(endEffectorLoadBearingMessageBuffer));
      return controllerEndEffectorLoadBearingMessage;
   }

   public StopAllTrajectoryMessage pollStopAllTrajectoryMessage()
   {
      return pollNewestMessage(stopAllTrajectoryMessageBuffer);
   }

   public ModifiableArmDesiredAccelerationsMessage pollArmDesiredAccelerationsMessage(RobotSide robotSide)
   {
      controllerArmDesiredAccelerationsMessage.set(pollNewestMessage(armDesiredAccelerationsMessageBuffers.get(robotSide)));
      return controllerArmDesiredAccelerationsMessage;
   }

   public AutomaticManipulationAbortMessage pollAutomaticManipulationAbortMessage()
   {
      controllerAutomaticManipulationAbortMessage.set(pollNewestMessage(automaticManipulationAbortMessageBuffer));
      return controllerAutomaticManipulationAbortMessage;
   }

   public void clearManipulationMessagesInQueue()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         handTrajectoryMessageBuffers.get(robotSide).flush();
         armTrajectoryMessageBuffers.get(robotSide).flush();
         armDesiredAccelerationsMessageBuffers.get(robotSide).flush();
      }
   }

   public void clearMessagesInQueue()
   {
      for (int i = 0; i < allBuffers.size(); i++)
         allBuffers.get(i).flush();
   }

   private static <T> T pollNewestMessage(ConcurrentRingBuffer<T> buffer)
   {
      if (buffer.poll())
      {
         T newestMessage = null;
         T message;
         while ((message = buffer.read()) != null)
         {
            newestMessage = message;
         }
         buffer.flush();
         return newestMessage;
      }
      return null;
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
}
