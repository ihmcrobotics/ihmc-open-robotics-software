package us.ihmc.humanoidRobotics.communication.util;

import java.util.List;

import us.ihmc.communication.packets.Packet;
import us.ihmc.humanoidRobotics.communication.packets.manipulation.wholeBodyTrajectory.WholeBodyTrajectoryToolboxMessage;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.MessageOfMessages;
import us.ihmc.humanoidRobotics.communication.packets.wholebody.WholeBodyTrajectoryMessage;

public final class MessageUnpackingTools
{
   private MessageUnpackingTools()
   {
   }

   public static MessageUnpacker<WholeBodyTrajectoryMessage> createWholeBodyTrajectoryMessageUnpacker()
   {
      return new MessageUnpacker<WholeBodyTrajectoryMessage>()
      {
         @Override
         public void unpackMessage(WholeBodyTrajectoryMessage multipleMessageHolder, List<Packet<?>> messagesToPack)
         {
            if (multipleMessageHolder.leftHandTrajectoryMessage != null && !multipleMessageHolder.leftHandTrajectoryMessage.se3Trajectory.taskspaceTrajectoryPoints.isEmpty())
               messagesToPack.add(multipleMessageHolder.leftHandTrajectoryMessage);
            if (multipleMessageHolder.rightHandTrajectoryMessage != null && !multipleMessageHolder.leftHandTrajectoryMessage.se3Trajectory.taskspaceTrajectoryPoints.isEmpty())
               messagesToPack.add(multipleMessageHolder.rightHandTrajectoryMessage);
            if (multipleMessageHolder.leftArmTrajectoryMessage != null && !multipleMessageHolder.leftArmTrajectoryMessage.jointspaceTrajectory.jointTrajectoryMessages.isEmpty())
               messagesToPack.add(multipleMessageHolder.leftArmTrajectoryMessage);
            if (multipleMessageHolder.rightArmTrajectoryMessage != null && !multipleMessageHolder.rightArmTrajectoryMessage.jointspaceTrajectory.jointTrajectoryMessages.isEmpty())
               messagesToPack.add(multipleMessageHolder.rightArmTrajectoryMessage);
            if (multipleMessageHolder.chestTrajectoryMessage != null && !multipleMessageHolder.chestTrajectoryMessage.so3Trajectory.taskspaceTrajectoryPoints.isEmpty())
               messagesToPack.add(multipleMessageHolder.chestTrajectoryMessage);
            if (multipleMessageHolder.pelvisTrajectoryMessage != null && !multipleMessageHolder.pelvisTrajectoryMessage.se3Trajectory.taskspaceTrajectoryPoints.isEmpty())
               messagesToPack.add(multipleMessageHolder.pelvisTrajectoryMessage);
            if (multipleMessageHolder.headTrajectoryMessage != null && !multipleMessageHolder.headTrajectoryMessage.so3Trajectory.taskspaceTrajectoryPoints.isEmpty())
               messagesToPack.add(multipleMessageHolder.headTrajectoryMessage);
            if (multipleMessageHolder.leftFootTrajectoryMessage != null && !multipleMessageHolder.leftFootTrajectoryMessage.se3Trajectory.taskspaceTrajectoryPoints.isEmpty())
               messagesToPack.add(multipleMessageHolder.leftFootTrajectoryMessage);
            if (multipleMessageHolder.rightFootTrajectoryMessage != null && !multipleMessageHolder.rightFootTrajectoryMessage.se3Trajectory.taskspaceTrajectoryPoints.isEmpty())
               messagesToPack.add(multipleMessageHolder.rightFootTrajectoryMessage);
         }
      };
   }

   public static MessageUnpacker<MessageOfMessages> createMessageOfMessagesUnpacker()
   {
      return new MessageUnpacker<MessageOfMessages>()
      {
         @Override
         public void unpackMessage(MessageOfMessages multipleMessageHolder, List<Packet<?>> messagesToPack)
         {
            List<Packet<?>> packets = multipleMessageHolder.getPackets();
            for (int i = 0; i < packets.size(); i++)
            {
               Packet<?> unpackedMessage = packets.get(i);
               messagesToPack.add(unpackedMessage);
            }
         }
      };
   }

   public static MessageUnpacker<WholeBodyTrajectoryToolboxMessage> createWholeBodyTrajectoryToolboxMessageUnpacker()
   {
      return new MessageUnpacker<WholeBodyTrajectoryToolboxMessage>()
      {
         @Override
         public void unpackMessage(WholeBodyTrajectoryToolboxMessage multipleMessageHolder, List<Packet<?>> messagesToPack)
         {
            if (multipleMessageHolder.configuration != null)
               messagesToPack.add(multipleMessageHolder.configuration);
            if (multipleMessageHolder.endEffectorTrajectories != null)
            {
               for (int i = 0; i < multipleMessageHolder.endEffectorTrajectories.size(); i++)
                  messagesToPack.add(multipleMessageHolder.endEffectorTrajectories.get(i));
            }
            if (multipleMessageHolder.explorationConfigurations != null)
            {
               for (int i = 0; i < multipleMessageHolder.explorationConfigurations.size(); i++)
                  messagesToPack.add(multipleMessageHolder.explorationConfigurations.get(i));
            }
            if (multipleMessageHolder.reachingManifolds != null)
            {
               for (int i = 0; i < multipleMessageHolder.reachingManifolds.size(); i++)
                  messagesToPack.add(multipleMessageHolder.reachingManifolds.get(i));
            }
         }
      };
   }

   public static interface MessageUnpacker<T extends Packet<T>>
   {
      /**
       * Unpack the messages in the given list.
       * 
       * @param messagesToPack the list the messages should be stored once unpacked.
       */
      public void unpackMessage(T multipleMessageHolder, List<Packet<?>> messagesToPack);
   }
}
