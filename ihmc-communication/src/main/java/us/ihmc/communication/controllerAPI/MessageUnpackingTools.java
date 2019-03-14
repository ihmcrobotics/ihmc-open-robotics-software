package us.ihmc.communication.controllerAPI;

import java.util.List;

import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryToolboxMessage;
import us.ihmc.euclid.interfaces.Settable;

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
         public void unpackMessage(WholeBodyTrajectoryMessage multipleMessageHolder, List<Settable<?>> messagesToPack)
         {
            if (multipleMessageHolder.getLeftHandTrajectoryMessage() != null && !multipleMessageHolder.getLeftHandTrajectoryMessage().getSe3Trajectory().getTaskspaceTrajectoryPoints().isEmpty())
               messagesToPack.add(multipleMessageHolder.getLeftHandTrajectoryMessage());
            if (multipleMessageHolder.getRightHandTrajectoryMessage() != null && !multipleMessageHolder.getLeftHandTrajectoryMessage().getSe3Trajectory().getTaskspaceTrajectoryPoints().isEmpty())
               messagesToPack.add(multipleMessageHolder.getRightHandTrajectoryMessage());
            if (multipleMessageHolder.getLeftArmTrajectoryMessage() != null && !multipleMessageHolder.getLeftArmTrajectoryMessage().getJointspaceTrajectory().getJointTrajectoryMessages().isEmpty())
               messagesToPack.add(multipleMessageHolder.getLeftArmTrajectoryMessage());
            if (multipleMessageHolder.getRightArmTrajectoryMessage() != null && !multipleMessageHolder.getRightArmTrajectoryMessage().getJointspaceTrajectory().getJointTrajectoryMessages().isEmpty())
               messagesToPack.add(multipleMessageHolder.getRightArmTrajectoryMessage());
            if (multipleMessageHolder.getChestTrajectoryMessage() != null && !multipleMessageHolder.getChestTrajectoryMessage().getSo3Trajectory().getTaskspaceTrajectoryPoints().isEmpty())
               messagesToPack.add(multipleMessageHolder.getChestTrajectoryMessage());
            if (multipleMessageHolder.getSpineTrajectoryMessage() != null && !multipleMessageHolder.getSpineTrajectoryMessage().getJointspaceTrajectory().getJointTrajectoryMessages().isEmpty())
               messagesToPack.add(multipleMessageHolder.getSpineTrajectoryMessage());
            if (multipleMessageHolder.getPelvisTrajectoryMessage() != null && !multipleMessageHolder.getPelvisTrajectoryMessage().getSe3Trajectory().getTaskspaceTrajectoryPoints().isEmpty())
               messagesToPack.add(multipleMessageHolder.getPelvisTrajectoryMessage());
            if (multipleMessageHolder.getHeadTrajectoryMessage() != null && !multipleMessageHolder.getHeadTrajectoryMessage().getSo3Trajectory().getTaskspaceTrajectoryPoints().isEmpty())
               messagesToPack.add(multipleMessageHolder.getHeadTrajectoryMessage());
            if (multipleMessageHolder.getLeftFootTrajectoryMessage() != null && !multipleMessageHolder.getLeftFootTrajectoryMessage().getSe3Trajectory().getTaskspaceTrajectoryPoints().isEmpty())
               messagesToPack.add(multipleMessageHolder.getLeftFootTrajectoryMessage());
            if (multipleMessageHolder.getRightFootTrajectoryMessage() != null && !multipleMessageHolder.getRightFootTrajectoryMessage().getSe3Trajectory().getTaskspaceTrajectoryPoints().isEmpty())
               messagesToPack.add(multipleMessageHolder.getRightFootTrajectoryMessage());
         }
      };
   }

   public static MessageUnpacker<WholeBodyTrajectoryToolboxMessage> createWholeBodyTrajectoryToolboxMessageUnpacker()
   {
      return new MessageUnpacker<WholeBodyTrajectoryToolboxMessage>()
      {
         @Override
         public void unpackMessage(WholeBodyTrajectoryToolboxMessage multipleMessageHolder, List<Settable<?>> messagesToPack)
         {
            if (multipleMessageHolder.getConfiguration() != null)
               messagesToPack.add(multipleMessageHolder.getConfiguration());
            if (multipleMessageHolder.getEndEffectorTrajectories() != null)
            {
               for (int i = 0; i < multipleMessageHolder.getEndEffectorTrajectories().size(); i++)
                  messagesToPack.add(multipleMessageHolder.getEndEffectorTrajectories().get(i));
            }
            if (multipleMessageHolder.getExplorationConfigurations() != null)
            {
               for (int i = 0; i < multipleMessageHolder.getExplorationConfigurations().size(); i++)
                  messagesToPack.add(multipleMessageHolder.getExplorationConfigurations().get(i));
            }
            if (multipleMessageHolder.getReachingManifolds() != null)
            {
               for (int i = 0; i < multipleMessageHolder.getReachingManifolds().size(); i++)
                  messagesToPack.add(multipleMessageHolder.getReachingManifolds().get(i));
            }
         }
      };
   }

   public static interface MessageUnpacker<T>
   {
      /**
       * Unpack the messages in the given list.
       * 
       * @param messagesToPack the list the messages should be stored once unpacked.
       */
      public void unpackMessage(T multipleMessageHolder, List<Settable<?>> messagesToPack);
   }
}
