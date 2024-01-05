package us.ihmc.communication.controllerAPI;

import java.util.List;

import controller_msgs.msg.dds.ArmTrajectoryMessage;
import controller_msgs.msg.dds.ChestTrajectoryMessage;
import controller_msgs.msg.dds.FootTrajectoryMessage;
import controller_msgs.msg.dds.HandHybridJointspaceTaskspaceTrajectoryMessage;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import controller_msgs.msg.dds.HeadTrajectoryMessage;
import controller_msgs.msg.dds.JointspaceStreamingMessage;
import controller_msgs.msg.dds.JointspaceTrajectoryMessage;
import controller_msgs.msg.dds.LegTrajectoryMessage;
import controller_msgs.msg.dds.NeckTrajectoryMessage;
import controller_msgs.msg.dds.OneDoFJointTrajectoryMessage;
import controller_msgs.msg.dds.PelvisTrajectoryMessage;
import ihmc_common_msgs.msg.dds.QueueableMessage;
import ihmc_common_msgs.msg.dds.SE3StreamingMessage;
import ihmc_common_msgs.msg.dds.SE3TrajectoryMessage;
import ihmc_common_msgs.msg.dds.SE3TrajectoryPointMessage;
import ihmc_common_msgs.msg.dds.SO3StreamingMessage;
import ihmc_common_msgs.msg.dds.SO3TrajectoryMessage;
import ihmc_common_msgs.msg.dds.SO3TrajectoryPointMessage;
import controller_msgs.msg.dds.SpineTrajectoryMessage;
import ihmc_common_msgs.msg.dds.TrajectoryPoint1DMessage;
import controller_msgs.msg.dds.WholeBodyStreamingMessage;
import controller_msgs.msg.dds.WholeBodyTrajectoryMessage;
import toolbox_msgs.msg.dds.WholeBodyTrajectoryToolboxMessage;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.idl.IDLSequence;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

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
            NeckTrajectoryMessage neckTrajectoryMessage = multipleMessageHolder.getNeckTrajectoryMessage();
            FootTrajectoryMessage leftFootTrajectoryMessage = multipleMessageHolder.getLeftFootTrajectoryMessage();
            FootTrajectoryMessage rightFootTrajectoryMessage = multipleMessageHolder.getRightFootTrajectoryMessage();
            LegTrajectoryMessage leftLegTrajectoryMessage = multipleMessageHolder.getLeftLegTrajectoryMessage();
            LegTrajectoryMessage rightLegTrajectoryMessage = multipleMessageHolder.getRightLegTrajectoryMessage();

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
            
            if (!rightLegTrajectoryMessage.getJointspaceTrajectory().getJointTrajectoryMessages().isEmpty())
               messagesToPack.add(rightLegTrajectoryMessage);
            if (!leftLegTrajectoryMessage.getJointspaceTrajectory().getJointTrajectoryMessages().isEmpty())
               messagesToPack.add(leftLegTrajectoryMessage);

            if (!chestTrajectoryMessage.getSo3Trajectory().getTaskspaceTrajectoryPoints().isEmpty())
               messagesToPack.add(chestTrajectoryMessage);
            if (!spineTrajectoryMessage.getJointspaceTrajectory().getJointTrajectoryMessages().isEmpty())
               messagesToPack.add(spineTrajectoryMessage);
            if (!pelvisTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().isEmpty())
               messagesToPack.add(pelvisTrajectoryMessage);
            if (!neckTrajectoryMessage.getJointspaceTrajectory().getJointTrajectoryMessages().isEmpty())
               messagesToPack.add(neckTrajectoryMessage);
            if (!headTrajectoryMessage.getSo3Trajectory().getTaskspaceTrajectoryPoints().isEmpty())
               messagesToPack.add(headTrajectoryMessage);
            if (!leftFootTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().isEmpty())
               messagesToPack.add(leftFootTrajectoryMessage);
            if (!rightFootTrajectoryMessage.getSe3Trajectory().getTaskspaceTrajectoryPoints().isEmpty())
               messagesToPack.add(rightFootTrajectoryMessage);
         }
      };
   }

   public static MessageUnpacker<WholeBodyStreamingMessage> createWholeBodyStreamingMessageUnpacker()
   {
      return new MessageUnpacker<WholeBodyStreamingMessage>()
      {
         private final SideDependentList<HandTrajectoryMessage> handTrajectoryMessages = new SideDependentList<>(new HandTrajectoryMessage(),
                                                                                                                 new HandTrajectoryMessage());
         private final SideDependentList<ArmTrajectoryMessage> armTrajectoryMessages = new SideDependentList<>(new ArmTrajectoryMessage(),
                                                                                                               new ArmTrajectoryMessage());
         private final SideDependentList<HandHybridJointspaceTaskspaceTrajectoryMessage> handHybridJointspaceTaskspaceTrajectoryMessages = new SideDependentList<>(new HandHybridJointspaceTaskspaceTrajectoryMessage(),
                                                                                                                                                                   new HandHybridJointspaceTaskspaceTrajectoryMessage());
         private final ChestTrajectoryMessage chestTrajectoryMessage = new ChestTrajectoryMessage();
         private final PelvisTrajectoryMessage pelvisTrajectoryMessage = new PelvisTrajectoryMessage();
         private final NeckTrajectoryMessage neckTrajectoryMessage = new NeckTrajectoryMessage();

         @Override
         public void unpackMessage(WholeBodyStreamingMessage message, List<Settable<?>> messagesToPack)
         {
            long sequenceId = message.getSequenceId();
            long uniqueId = message.getUniqueId();
            float streamIntegrationDuration = message.getStreamIntegrationDuration();
            long sourceTimestamp = message.getTimestamp();

            if (message.getHasChestStreamingMessage())
            {
               chestTrajectoryMessage.setSequenceId(sequenceId);
               chestTrajectoryMessage.setUniqueId(uniqueId);
               toSO3TrajectoryMessage(message.getChestStreamingMessage(),
                                      chestTrajectoryMessage.getSo3Trajectory(),
                                      sequenceId,
                                      uniqueId,
                                      streamIntegrationDuration,
                                      sourceTimestamp);
               messagesToPack.add(chestTrajectoryMessage);
            }

            if (message.getHasPelvisStreamingMessage())
            {
               pelvisTrajectoryMessage.setSequenceId(sequenceId);
               pelvisTrajectoryMessage.setUniqueId(uniqueId);
               pelvisTrajectoryMessage.setEnableUserPelvisControl(message.getEnableUserPelvisControl());
               toSE3TrajectoryMessage(message.getPelvisStreamingMessage(),
                                      pelvisTrajectoryMessage.getSe3Trajectory(),
                                      sequenceId,
                                      uniqueId,
                                      streamIntegrationDuration,
                                      sourceTimestamp);
               messagesToPack.add(pelvisTrajectoryMessage);
            }

            if (message.getHasNeckStreamingMessage())
            {
               neckTrajectoryMessage.setSequenceId(sequenceId);
               neckTrajectoryMessage.setUniqueId(uniqueId);
               toJointspaceTrajectoryMessage(message.getNeckStreamingMessage(),
                                             neckTrajectoryMessage.getJointspaceTrajectory(),
                                             sequenceId,
                                             uniqueId,
                                             streamIntegrationDuration,
                                             sourceTimestamp);
               messagesToPack.add(neckTrajectoryMessage);
            }

            for (RobotSide robotSide : RobotSide.values)
            {
               boolean hasArmStreamingMessage = select(robotSide, message.getHasLeftArmStreamingMessage(), message.getHasRightArmStreamingMessage());
               boolean hasHandStreamingMessage = select(robotSide, message.getHasLeftHandStreamingMessage(), message.getHasRightHandStreamingMessage());
               SE3StreamingMessage handStreamingMessage = select(robotSide, message.getLeftHandStreamingMessage(), message.getRightHandStreamingMessage());

               if (hasArmStreamingMessage)
               {
                  JointspaceStreamingMessage armStreamingMessage = select(robotSide,
                                                                          message.getLeftArmStreamingMessage(),
                                                                          message.getRightArmStreamingMessage());

                  if (hasHandStreamingMessage)
                  {
                     HandHybridJointspaceTaskspaceTrajectoryMessage hybridTrajectoryMessage = handHybridJointspaceTaskspaceTrajectoryMessages.get(robotSide);
                     hybridTrajectoryMessage.setSequenceId(sequenceId);
                     hybridTrajectoryMessage.setUniqueId(uniqueId);
                     hybridTrajectoryMessage.setRobotSide(robotSide.toByte());
                     toSE3TrajectoryMessage(handStreamingMessage,
                                            hybridTrajectoryMessage.getTaskspaceTrajectoryMessage(),
                                            sequenceId,
                                            uniqueId,
                                            streamIntegrationDuration,
                                            sourceTimestamp);
                     toJointspaceTrajectoryMessage(armStreamingMessage,
                                                   hybridTrajectoryMessage.getJointspaceTrajectoryMessage(),
                                                   sequenceId,
                                                   uniqueId,
                                                   streamIntegrationDuration,
                                                   sourceTimestamp);
                     messagesToPack.add(hybridTrajectoryMessage);
                  }
                  else
                  {
                     ArmTrajectoryMessage armTrajectoryMessage = armTrajectoryMessages.get(robotSide);
                     armTrajectoryMessage.setSequenceId(sequenceId);
                     armTrajectoryMessage.setUniqueId(uniqueId);
                     armTrajectoryMessage.setRobotSide(robotSide.toByte());
                     toJointspaceTrajectoryMessage(armStreamingMessage,
                                                   armTrajectoryMessage.getJointspaceTrajectory(),
                                                   sequenceId,
                                                   uniqueId,
                                                   streamIntegrationDuration,
                                                   sourceTimestamp);
                     messagesToPack.add(armTrajectoryMessage);
                  }
               }
               else if (hasHandStreamingMessage)
               {
                  HandTrajectoryMessage handTrajectoryMessage = handTrajectoryMessages.get(robotSide);
                  handTrajectoryMessage.setSequenceId(sequenceId);
                  handStreamingMessage.setUniqueId(uniqueId);
                  handTrajectoryMessage.setRobotSide(robotSide.toByte());
                  toSE3TrajectoryMessage(handStreamingMessage,
                                         handTrajectoryMessage.getSe3Trajectory(),
                                         sequenceId,
                                         uniqueId,
                                         streamIntegrationDuration,
                                         sourceTimestamp);
                  messagesToPack.add(handTrajectoryMessage);
               }
            }
         }

         private <T> T select(RobotSide robotSide, T left, T right)
         {
            return robotSide == RobotSide.LEFT ? left : right;
         }

         private boolean select(RobotSide robotSide, boolean left, boolean right)
         {
            return robotSide == RobotSide.LEFT ? left : right;
         }

         private void toSO3TrajectoryMessage(SO3StreamingMessage source, SO3TrajectoryMessage destination, long sequenceId, long uniqueId,
                                             double streamIntegrationDuration, long sourceTimestamp)
         {
            destination.setSequenceId(sequenceId);
            destination.setUniqueId(uniqueId);
            destination.getTaskspaceTrajectoryPoints().clear();
            SO3TrajectoryPointMessage trajectoryPoint = destination.getTaskspaceTrajectoryPoints().add();
            trajectoryPoint.setSequenceId(sequenceId);
            trajectoryPoint.setUniqueId(uniqueId);
            trajectoryPoint.setTime(0.0);
            trajectoryPoint.getOrientation().set(source.getOrientation());
            trajectoryPoint.getAngularVelocity().set(source.getAngularVelocity());
            MessageTools.packSelectionMatrix3DMessage(true, destination.getSelectionMatrix());
            destination.getFrameInformation().set(source.getFrameInformation());
            MessageTools.packWeightMatrix3DMessage(-1.0, destination.getWeightMatrix());
            destination.setUseCustomControlFrame(source.getUseCustomControlFrame());
            destination.getControlFramePose().set(source.getControlFramePose());
            configureQueueableMessage(destination.getQueueingProperties(), sequenceId, uniqueId, streamIntegrationDuration, sourceTimestamp);
         }

         private void toSE3TrajectoryMessage(SE3StreamingMessage source, SE3TrajectoryMessage destination, long sequenceId, long uniqueId,
                                             double streamIntegrationDuration, long sourceTimestamp)
         {
            destination.setSequenceId(sequenceId);
            destination.setUniqueId(uniqueId);
            destination.getTaskspaceTrajectoryPoints().clear();
            SE3TrajectoryPointMessage trajectoryPoint = destination.getTaskspaceTrajectoryPoints().add();
            trajectoryPoint.setSequenceId(sequenceId);
            trajectoryPoint.setUniqueId(uniqueId);
            trajectoryPoint.setTime(0.0);
            trajectoryPoint.getPosition().set(source.getPosition());
            trajectoryPoint.getOrientation().set(source.getOrientation());
            trajectoryPoint.getLinearVelocity().set(source.getLinearVelocity());
            trajectoryPoint.getAngularVelocity().set(source.getAngularVelocity());
            MessageTools.packSelectionMatrix3DMessage(true, destination.getAngularSelectionMatrix());
            MessageTools.packSelectionMatrix3DMessage(true, destination.getLinearSelectionMatrix());
            destination.getFrameInformation().set(source.getFrameInformation());
            MessageTools.packWeightMatrix3DMessage(-1.0, destination.getAngularWeightMatrix());
            MessageTools.packWeightMatrix3DMessage(-1.0, destination.getLinearWeightMatrix());
            destination.setUseCustomControlFrame(source.getUseCustomControlFrame());
            destination.getControlFramePose().set(source.getControlFramePose());
            configureQueueableMessage(destination.getQueueingProperties(), sequenceId, uniqueId, streamIntegrationDuration, sourceTimestamp);
         }

         private void toJointspaceTrajectoryMessage(JointspaceStreamingMessage source, JointspaceTrajectoryMessage destination, long sequenceId, long uniqueId,
                                                    double streamIntegrationDuration, long sourceTimestamp)
         {
            destination.setSequenceId(sequenceId);
            destination.setUniqueId(uniqueId);
            destination.getJointTrajectoryMessages().clear();

            IDLSequence.Float positions = source.getPositions();
            IDLSequence.Float velocities = source.getVelocities();
            IDLSequence.Float accelerations = source.getAccelerations();

            for (int i = 0; i < Math.min(positions.size(), velocities.size()); i++)
            {
               OneDoFJointTrajectoryMessage jointTrajectoryMessage = destination.getJointTrajectoryMessages().add();
               jointTrajectoryMessage.setSequenceId(sequenceId);
               jointTrajectoryMessage.setUniqueId(uniqueId);
               jointTrajectoryMessage.getTrajectoryPoints().clear();
               TrajectoryPoint1DMessage trajectoryPoint = jointTrajectoryMessage.getTrajectoryPoints().add();
               trajectoryPoint.setSequenceId(sequenceId);
               trajectoryPoint.setUniqueId(uniqueId);
               trajectoryPoint.setTime(0.0);
               trajectoryPoint.setPosition(positions.get(i));
               trajectoryPoint.setVelocity(velocities.get(i));
               trajectoryPoint.setAcceleration(accelerations.get(i));
               jointTrajectoryMessage.setWeight(-1.0);
            }

            configureQueueableMessage(destination.getQueueingProperties(), sequenceId, uniqueId, streamIntegrationDuration, sourceTimestamp);
         }

         private void configureQueueableMessage(QueueableMessage messageToModify, long sequenceId, long uniqueId, double streamIntegrationDuration,
                                                long sourceTimestamp)
         {
            messageToModify.setExecutionMode(ExecutionMode.STREAM.toByte());
            messageToModify.setSequenceId(sequenceId);
            messageToModify.setUniqueId(uniqueId);
            messageToModify.setStreamIntegrationDuration(streamIntegrationDuration);
            messageToModify.setTimestamp(sourceTimestamp);
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
