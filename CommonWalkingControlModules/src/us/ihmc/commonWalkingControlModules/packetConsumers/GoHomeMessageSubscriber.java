package us.ihmc.commonWalkingControlModules.packetConsumers;

import java.util.EnumMap;
import java.util.HashMap;
import java.util.Map;

import com.google.common.util.concurrent.AtomicDouble;

import us.ihmc.communication.net.PacketConsumer;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage;
import us.ihmc.humanoidRobotics.communication.packets.walking.GoHomeMessage.BodyPart;
import us.ihmc.humanoidRobotics.communication.streamingData.HumanoidGlobalDataProducer;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class GoHomeMessageSubscriber implements PacketConsumer<GoHomeMessage>
{
   private final HumanoidGlobalDataProducer globalDataProducer;

   private final SideDependentList<EnumMap<BodyPart, AtomicDouble>> latestMessageReferenceForSideDependentBodyParts = SideDependentList.createListOfEnumMaps(BodyPart.class);
   private final Map<BodyPart, AtomicDouble> latestMessageReferenceForOtherBodyParts = new HashMap<>();

   public GoHomeMessageSubscriber(HumanoidGlobalDataProducer globalDataProducer)
   {
      this.globalDataProducer = globalDataProducer;

      for (BodyPart bodyPart : BodyPart.values)
      {
         if (bodyPart.isRobotSideNeeded())
         {
            for (RobotSide robotSide : RobotSide.values)
            {
               latestMessageReferenceForSideDependentBodyParts.get(robotSide).put(bodyPart, new AtomicDouble(Double.NaN));
            }
         }
         else
         {
            latestMessageReferenceForOtherBodyParts.put(bodyPart, new AtomicDouble(Double.NaN));
         }
      }

      globalDataProducer.attachListener(GoHomeMessage.class, this);
   }

   public boolean isNewMessageAvailable(BodyPart bodyPart)
   {
      if (bodyPart.isRobotSideNeeded())
         throw new RuntimeException("Need to provide robotSide for the bodyPart: " + bodyPart);

      return !Double.isNaN(latestMessageReferenceForOtherBodyParts.get(bodyPart).get());
   }

   public boolean isNewMessageAvailable(BodyPart bodyPart, RobotSide robotSide)
   {
      if (bodyPart.isRobotSideNeeded())
      {
         return !Double.isNaN(latestMessageReferenceForSideDependentBodyParts.get(robotSide).get(bodyPart).get());
      }
      else
         return isNewMessageAvailable(bodyPart);
   }

   public double pollMessage(BodyPart bodyPart)
   {
      if (bodyPart.isRobotSideNeeded())
         throw new RuntimeException("Need to provide robotSide for the bodyPart: " + bodyPart);

      return latestMessageReferenceForOtherBodyParts.get(bodyPart).getAndSet(Double.NaN);
   }

   public double pollMessage(BodyPart bodyPart, RobotSide robotSide)
   {
      if (bodyPart.isRobotSideNeeded())
      {
         return latestMessageReferenceForSideDependentBodyParts.get(robotSide).get(bodyPart).getAndSet(Double.NaN);
      }
      else
         return pollMessage(bodyPart);
   }

   public void clearMessageInQueue(BodyPart bodyPart)
   {
      if (bodyPart.isRobotSideNeeded())
         throw new RuntimeException("Need to provide robotSide for the bodyPart: " + bodyPart);
      latestMessageReferenceForOtherBodyParts.get(bodyPart).set(Double.NaN);
   }

   public void clearMessageInQueue(BodyPart bodyPart, RobotSide robotSide)
   {
      if (bodyPart.isRobotSideNeeded())
      {
         latestMessageReferenceForSideDependentBodyParts.get(robotSide).get(bodyPart).set(Double.NaN);
      }
      else
         clearMessageInQueue(bodyPart);
   }

   public void clearMessagesInQueue()
   {
      for (BodyPart bodyPart : BodyPart.values)
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            clearMessageInQueue(bodyPart, robotSide);
         }
      }
   }

   @Override
   public void receivedPacket(GoHomeMessage goHomeMessage)
   {
      String errorMessage = PacketValidityChecker.validateGoHomeMessage(goHomeMessage);
      if (errorMessage != null)
      {
         if (globalDataProducer != null)
            globalDataProducer.notifyInvalidPacketReceived(goHomeMessage.getClass(), errorMessage);
         return;
      }

      BodyPart bodyPart = goHomeMessage.getBodyPart();
      double trajectoryTime = goHomeMessage.trajectoryTime;

      if (bodyPart.isRobotSideNeeded())
      {
         RobotSide robotSide = goHomeMessage.getRobotSide();
         latestMessageReferenceForSideDependentBodyParts.get(robotSide).get(bodyPart).set(trajectoryTime);
      }
      else
      {
         latestMessageReferenceForOtherBodyParts.get(bodyPart).set(trajectoryTime);
      }
   }
}
