package us.ihmc.communication.controllerAPI;

import java.util.List;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.ChestTrajectoryMessage;
import controller_msgs.msg.dds.FootTrajectoryMessage;
import controller_msgs.msg.dds.HandHybridJointspaceTaskspaceTrajectoryMessage;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import controller_msgs.msg.dds.HeadTrajectoryMessage;
import controller_msgs.msg.dds.PelvisTrajectoryMessage;
import controller_msgs.msg.dds.SpineTrajectoryMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryToolboxMessage;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.robotics.robotSide.RobotSide;

public final class MessageUnpackingTools
{
   private MessageUnpackingTools()
   {
   }

   public static MessageUnpacker<WholeBodyTrajectoryMessage> createWholeBodyTrajectoryMessageUnpacker()
   {
      return new MessageUnpacker<WholeBodyTrajectoryMessage>()
      {
         private final HandHybridJointspaceTaskspaceTrajectoryMessage leftHandHybridJointspaceTaskspaceTrajectoryMessage = new HandHybridJointspaceTaskspaceTrajectoryMessage();
         private final HandHybridJointspaceTaskspaceTrajectoryMessage rightHandHybridJointspaceTaskspaceTrajectoryMessage = new HandHybridJointspaceTaskspaceTrajectoryMessage();

         @Override
         public void unpackMessage(WholeBodyTrajectoryMessage multipleMessageHolder, List<Settable<?>> messagesToPack)
         {
            HandTrajectoryMessage leftHandTrajectoryMessage = multipleMessageHolder.getLeftHandTrajectoryMessage();
            HandTrajectoryMessage rightHandTrajectoryMessage = multipleMessageHolder.getRightHandTrajectoryMessage();
            ArmTrajectoryMessage leftArmTrajectoryMessage = multipleMessageHolder.getLeftArmTrajectoryMessage();
            ArmTrajectoryMessage rightArmTrajectoryMessage = multipleMessageHolder.getRightArmTrajectoryMessage();
            ChestTrajectoryMessage chestTrajectoryMessage = multipleMessageHolder.getChestTrajectoryMessage();
            SpineTrajectoryMessage spineTrajectoryMessage = multipleMessageHolder.getSpineTrajectoryMessage();
            PelvisTrajectoryMessage pelvisTrajectoryMessage = multipleMessageHolder.getPelvisTrajectoryMessage();
            HeadTrajectoryMessage headTrajectoryMessage = multipleMessageHolder.getHeadTrajectoryMessage();
            FootTrajectoryMessage leftFootTrajectoryMessage = multipleMessageHolder.getLeftFootTrajectoryMessage();
            FootTrajectoryMessage rightFootTrajectoryMessage = multipleMessageHolder.getRightFootTrajectoryMessage();

            if (!leftHandTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().isEmpty())
            {
               if (!leftArmTrajectoryMessage.getJointspaceTrajectory().getJointTrajectoryMessages().isEmpty())
               {
                  leftHandHybridJointspaceTaskspaceTrajectoryMessage.setRobotSide(RobotSide.LEFT.toByte());
                  leftHandHybridJointspaceTaskspaceTrajectoryMessage.setSequenceId(leftHandTrajectoryMessage.getSequenceId());
                  leftHandHybridJointspaceTaskspaceTrajectoryMessage.getTaskspaceTrajectoryMessage().set(leftHandTrajectoryMessage.getSe3Trajectory());
                  leftHandHybridJointspaceTaskspaceTrajectoryMessage.getJointspaceTrajectoryMessage().set(leftArmTrajectoryMessage.getJointspaceTrajectory());
                  messagesToPack.add(leftHandHybridJointspaceTaskspaceTrajectoryMessage);
               }
               else
               {
                  messagesToPack.add(leftHandTrajectoryMessage);
               }
            }
            else if (!leftArmTrajectoryMessage.getJointspaceTrajectory().getJointTrajectoryMessages().isEmpty())
            {
               messagesToPack.add(leftArmTrajectoryMessage);
            }

            if (!rightHandTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().isEmpty())
            {
               if (!rightArmTrajectoryMessage.getJointspaceTrajectory().getJointTrajectoryMessages().isEmpty())
               {
                  rightHandHybridJointspaceTaskspaceTrajectoryMessage.setRobotSide(RobotSide.RIGHT.toByte());
                  rightHandHybridJointspaceTaskspaceTrajectoryMessage.setSequenceId(rightHandTrajectoryMessage.getSequenceId());
                  rightHandHybridJointspaceTaskspaceTrajectoryMessage.getTaskspaceTrajectoryMessage().set(rightHandTrajectoryMessage.getSe3Trajectory());
                  rightHandHybridJointspaceTaskspaceTrajectoryMessage.getJointspaceTrajectoryMessage().set(rightArmTrajectoryMessage.getJointspaceTrajectory());
                  messagesToPack.add(rightHandHybridJointspaceTaskspaceTrajectoryMessage);
               }
               else
               {
                  messagesToPack.add(rightHandTrajectoryMessage);
               }
            }
            else if (!rightArmTrajectoryMessage.getJointspaceTrajectory().getJointTrajectoryMessages().isEmpty())
            {
               messagesToPack.add(rightArmTrajectoryMessage);
            }

            if (!chestTrajectoryMessage.getSo3Trajectory().getTaskspaceTrajectoryPoints().isEmpty())
               messagesToPack.add(chestTrajectoryMessage);
            if (!spineTrajectoryMessage.getJointspaceTrajectory().getJointTrajectoryMessages().isEmpty())
               messagesToPack.add(spineTrajectoryMessage);
            if (!pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().isEmpty())
               messagesToPack.add(pelvisTrajectoryMessage);
            if (!headTrajectoryMessage.getSo3Trajectory().getTaskspaceTrajectoryPoints().isEmpty())
               messagesToPack.add(headTrajectoryMessage);
            if (!leftFootTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().isEmpty())
               messagesToPack.add(leftFootTrajectoryMessage);
            if (!rightFootTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().isEmpty())
               messagesToPack.add(rightFootTrajectoryMessage);
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
