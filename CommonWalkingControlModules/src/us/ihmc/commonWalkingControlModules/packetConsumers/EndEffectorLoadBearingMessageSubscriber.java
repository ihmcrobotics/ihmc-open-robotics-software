package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.EnumMap;
import java.util.HashMap;
import java.util.Map;
import java.util.concurrent.atomic.AtomicReference;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.walking.EndEffectorLoadBearingMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.EndEffectorLoadBearingMessage.EndEffector;
import us.ihmc.humanoidRobotics.communication.packets.walking.EndEffectorLoadBearingMessage.LoadBearingRequest;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class EndEffectorLoadBearingMessageSubscriber implements PacketConsumer<EndEffectorLoadBearingMessage>
{
   private final HumanoidGlobalDataProducer globalDataProducer;

   private final SideDependentList<EnumMap<EndEffector, AtomicReference<LoadBearingRequest>>> latestMessageReferenceForSideDependentEndEffectors = SideDependentList.createListOfEnumMaps(EndEffector.class);
   private final Map<EndEffector, AtomicReference<LoadBearingRequest>> latestMessageReferenceForOtherEndEffectors = new HashMap<>();

   public EndEffectorLoadBearingMessageSubscriber(HumanoidGlobalDataProducer globalDataProducer)
   {
      this.globalDataProducer = globalDataProducer;

      for (EndEffector endEffector : EndEffector.values)
      {
         if (endEffector.isRobotSideNeeded())
         {
            for (RobotSide robotSide : RobotSide.values)
            {
               latestMessageReferenceForSideDependentEndEffectors.get(robotSide).put(endEffector, new AtomicReference<LoadBearingRequest>(null));
            }
         }
         else
         {
            latestMessageReferenceForOtherEndEffectors.put(endEffector, new AtomicReference<LoadBearingRequest>(null));
         }
      }

      globalDataProducer.attachListener(EndEffectorLoadBearingMessage.class, this);
   }

   public LoadBearingRequest pollMessage(EndEffector endEffector)
   {
      if (endEffector.isRobotSideNeeded())
         throw new RuntimeException("Need to provide robotSide for the endEffector: " + endEffector);

      return latestMessageReferenceForOtherEndEffectors.get(endEffector).getAndSet(null);
   }

   public LoadBearingRequest pollMessage(EndEffector endEffector, RobotSide robotSide)
   {
      if (endEffector.isRobotSideNeeded())
      {
         return latestMessageReferenceForSideDependentEndEffectors.get(robotSide).get(endEffector).getAndSet(null);
      }
      else
         return pollMessage(endEffector);
   }

   public void clearMessageInQueue(EndEffector endEffector)
   {
      if (endEffector.isRobotSideNeeded())
         throw new RuntimeException("Need to provide robotSide for the endEffector: " + endEffector);
      latestMessageReferenceForOtherEndEffectors.get(endEffector).set(null);
   }

   public void clearMessageInQueue(EndEffector endEffector, RobotSide robotSide)
   {
      if (endEffector.isRobotSideNeeded())
      {
         latestMessageReferenceForSideDependentEndEffectors.get(robotSide).get(endEffector).set(null);
      }
      else
         clearMessageInQueue(endEffector);
   }

   public void clearMessagesInQueue()
   {
      for (EndEffector endEffector : EndEffector.values)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            clearMessageInQueue(endEffector, robotSide);
         }
      }
   }

   @Override
   public void receivedPacket(EndEffectorLoadBearingMessage endEffectorLoadBearingMessage)
   {
      String errorMessage = PacketValidityChecker.validateEndEffectorLoadBearingMessage(endEffectorLoadBearingMessage);
      if (errorMessage != null)
      {
         if (globalDataProducer != null)
            globalDataProducer.notifyInvalidPacketReceived(endEffectorLoadBearingMessage.getClass(), errorMessage);
         return;
      }

      EndEffector endEffector = endEffectorLoadBearingMessage.getEndEffector();
      LoadBearingRequest request = endEffectorLoadBearingMessage.getRequest();

      if (endEffector.isRobotSideNeeded())
      {
         RobotSide robotSide = endEffectorLoadBearingMessage.getRobotSide();
         latestMessageReferenceForSideDependentEndEffectors.get(robotSide).get(endEffector).set(request);
      }
      else
      {
         latestMessageReferenceForOtherEndEffectors.get(endEffector).set(request);
      }
   }
}
