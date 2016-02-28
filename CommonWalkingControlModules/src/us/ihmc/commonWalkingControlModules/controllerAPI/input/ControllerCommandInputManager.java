package us.ihmc.commonWalkingControlModules.controllerAPI.input;

import java.lang.reflect.Constructor;
import java.lang.reflect.InvocationTargetException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableArmTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableChestTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableEndEffectorLoadBearingMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableFootTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableHandTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableHeadTrajectoryMessage;
import us.ihmc.commonWalkingControlModules.controllerAPI.input.command.ModifiableStopAllTrajectoryMessage;
import us.ihmc.concurrent.Builder;
import us.ihmc.concurrent.ConcurrentRingBuffer;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.ArmTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.HandTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.StopAllTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.ChestTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.EndEffectorLoadBearingMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.FootTrajectoryMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.HeadTrajectoryMessage;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class ControllerCommandInputManager
{
   private final SideDependentList<ConcurrentRingBuffer<ModifiableArmTrajectoryMessage>> armTrajectoryMessageBuffers = new SideDependentList<>();
   private final SideDependentList<ConcurrentRingBuffer<ModifiableHandTrajectoryMessage>> handTrajectoryMessageBuffers = new SideDependentList<>();
   private final SideDependentList<ConcurrentRingBuffer<ModifiableFootTrajectoryMessage>> footTrajectoryMessageBuffers = new SideDependentList<>();
   private final ConcurrentRingBuffer<ModifiableHeadTrajectoryMessage> headTrajectoryMessageBuffer;
   private final ConcurrentRingBuffer<ModifiableChestTrajectoryMessage> chestTrajectoryMessageBuffer;

   private final ConcurrentRingBuffer<ModifiableEndEffectorLoadBearingMessage> endEffectorLoadBearingMessageBuffer;

   private final List<Object> stopAllTrajectoryMessageListeners = new ArrayList<>();
   private final Map<Object, ConcurrentRingBuffer<ModifiableStopAllTrajectoryMessage>> stopAllTrajectoryMessageBuffers = new HashMap<>();

   public ControllerCommandInputManager()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         armTrajectoryMessageBuffers.put(robotSide, new ConcurrentRingBuffer<>(createBuilderWithEmptyConstructor(ModifiableArmTrajectoryMessage.class), 8));
         handTrajectoryMessageBuffers.put(robotSide, new ConcurrentRingBuffer<>(createBuilderWithEmptyConstructor(ModifiableHandTrajectoryMessage.class), 8));
         footTrajectoryMessageBuffers.put(robotSide, new ConcurrentRingBuffer<>(createBuilderWithEmptyConstructor(ModifiableFootTrajectoryMessage.class), 8));
      }
      headTrajectoryMessageBuffer = new ConcurrentRingBuffer<>(createBuilderWithEmptyConstructor(ModifiableHeadTrajectoryMessage.class), 8);
      chestTrajectoryMessageBuffer = new ConcurrentRingBuffer<>(createBuilderWithEmptyConstructor(ModifiableChestTrajectoryMessage.class), 8);
      endEffectorLoadBearingMessageBuffer = new ConcurrentRingBuffer<>(createBuilderWithEmptyConstructor(ModifiableEndEffectorLoadBearingMessage.class), 8);
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
      for (int i = 0; i < stopAllTrajectoryMessageListeners.size(); i++)
      {
         ConcurrentRingBuffer<ModifiableStopAllTrajectoryMessage> buffer = stopAllTrajectoryMessageBuffers.get(stopAllTrajectoryMessageListeners.get(i));
         if (buffer == null)
         {
            buffer = new ConcurrentRingBuffer<>(createBuilderWithEmptyConstructor(ModifiableStopAllTrajectoryMessage.class), 10);
            stopAllTrajectoryMessageBuffers.put(stopAllTrajectoryMessageListeners.get(i), buffer);
         }

         ModifiableStopAllTrajectoryMessage nextModifiableMessage = buffer.next();
         if (nextModifiableMessage == null)
            continue;
         nextModifiableMessage.set(stopAllTrajectoryMessage);
         buffer.commit();
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
               throw new RuntimeException("Something went wrong the empty constructor implemented in the class: " + emptyConstructor.getDeclaringClass().getSimpleName());
            }

            return newInstance;
         }
      };
      return builder;
   }
}
