package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.LinkedHashMap;
import java.util.concurrent.atomic.AtomicInteger;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.walking.EndEffectorLoadBearingMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.EndEffectorLoadBearingMessage.EndEffector;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.robotics.robotSide.RobotSide;

public class EndEffectorLoadBearingMessageSubscriber implements PacketConsumer<EndEffectorLoadBearingMessage>
{
   private static final int NO_LOAD_BEARING_REQUEST = -1;
   private static final int LOAD_BEARING_REQUEST_FOR_BOTH_SIDES = 2;
   private static final int LOAD_BEARING_REQUEST_FOR_SINGLE_SIDED_END_EFFECTOR = 1;

   private final HumanoidGlobalDataProducer globalDataProducer;

   private final LinkedHashMap<EndEffector, AtomicInteger> latestMessageReferencePerEndEffector = new LinkedHashMap<>();
   
   public EndEffectorLoadBearingMessageSubscriber(HumanoidGlobalDataProducer globalDataProducer)
   {
      this.globalDataProducer = globalDataProducer;

      for (EndEffector endEffector : EndEffector.values)
         latestMessageReferencePerEndEffector.put(endEffector, new AtomicInteger(-1));

      globalDataProducer.attachListener(EndEffectorLoadBearingMessage.class, this);
   }

   public boolean pollMessage(EndEffector endEffector)
   {
      if (endEffector.isRobotSideNeeded())
         throw new RuntimeException("Need to provide robotSide for the endEffector: " + endEffector);
      boolean isNewMessageAvailable = latestMessageReferencePerEndEffector.get(endEffector).get() == LOAD_BEARING_REQUEST_FOR_SINGLE_SIDED_END_EFFECTOR;
      clearMessageInQueue(endEffector);
      return isNewMessageAvailable;
   }

   public boolean pollMessage(EndEffector endEffector, RobotSide robotSide)
   {
      if (endEffector.isRobotSideNeeded())
      {
         boolean isNewMessageAvailable = latestMessageReferencePerEndEffector.get(endEffector).get() == robotSide.ordinal();
         clearMessageInQueue(endEffector, robotSide);
         return isNewMessageAvailable;
      }
      else
         return pollMessage(endEffector);
   }

   public void clearMessageInQueue(EndEffector endEffector)
   {
      if (endEffector.isRobotSideNeeded())
         throw new RuntimeException("Need to provide robotSide for the endEffector: " + endEffector);
      latestMessageReferencePerEndEffector.get(endEffector).set(NO_LOAD_BEARING_REQUEST);
   }

   public void clearMessageInQueue(EndEffector endEffector, RobotSide robotSide)
   {
      if (endEffector.isRobotSideNeeded())
      {
         AtomicInteger latestMessage = latestMessageReferencePerEndEffector.get(endEffector);
         if (latestMessage.get() == robotSide.ordinal())
         {
            latestMessage.set(NO_LOAD_BEARING_REQUEST);
         }
         else if (latestMessage.get() == LOAD_BEARING_REQUEST_FOR_BOTH_SIDES)
         {
            latestMessage.set(robotSide.getOppositeSide().ordinal());
         }
      }
      else
         clearMessageInQueue(endEffector);
   }

   public void clearMessagesInQueue()
   {
      for (EndEffector endEffector : EndEffector.values)
         latestMessageReferencePerEndEffector.get(endEffector).set(NO_LOAD_BEARING_REQUEST);
   }

   @Override
   public void receivedPacket(EndEffectorLoadBearingMessage endEffectorLoadBearingMessage)
   {
      if (!PacketValidityChecker.validateEndEffectorLoadBearingMessage(endEffectorLoadBearingMessage, globalDataProducer))
         return;

      EndEffector endEffector = endEffectorLoadBearingMessage.getEndEffector();
      RobotSide robotSide = endEffectorLoadBearingMessage.getRobotSide();
      
      AtomicInteger latestMessage = latestMessageReferencePerEndEffector.get(endEffector);

      if (endEffector.isRobotSideNeeded())
      {
         if (latestMessage.get() == NO_LOAD_BEARING_REQUEST) // non of the sides were requested to switch to loadbearing
         {
            latestMessage.set(robotSide.ordinal()); // 0 for left side, 1 for right side.
         }
         else
         {
            latestMessage.set(LOAD_BEARING_REQUEST_FOR_BOTH_SIDES); // Both sides are requested to switching to loadbearing.
         }
      }
      else
      {
         latestMessage.set(LOAD_BEARING_REQUEST_FOR_SINGLE_SIDED_END_EFFECTOR); // No robotSide smartness needed here, just need to set to something else than -1.
      }
   }
}
