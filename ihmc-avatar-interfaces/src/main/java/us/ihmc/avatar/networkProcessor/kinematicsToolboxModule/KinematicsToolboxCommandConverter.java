package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import controller_msgs.msg.dds.KinematicsToolboxSupportRegionMessage;
import controller_msgs.msg.dds.KinematicsToolboxInputCollectionMessage;
import controller_msgs.msg.dds.KinematicsToolboxOneDoFJointMessage;
import controller_msgs.msg.dds.KinematicsToolboxPrivilegedConfigurationMessage;
import controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import us.ihmc.communication.controllerAPI.CommandConversionInterface;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxSupportRegionCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxInputCollectionCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxOneDoFJointCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxPrivilegedConfigurationCommand;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.JointHashCodeResolver;
import us.ihmc.robotModels.RigidBodyHashCodeResolver;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;
import us.ihmc.sensorProcessing.frames.ReferenceFrames;

/**
 * This class allows the retrieve the rigid-body from its hash code when converting a
 * {@link KinematicsToolboxRigidBodyMessage} into a {@link KinematicsToolboxRigidBodyCommand}.
 * 
 * @author Sylvain Bertrand
 */
public class KinematicsToolboxCommandConverter implements CommandConversionInterface
{
   private final RigidBodyHashCodeResolver rigidBodyHashCodeResolver;
   private final JointHashCodeResolver jointHashCodeResolver;
   private final ReferenceFrameHashCodeResolver referenceFrameHashCodeResolver;

   public KinematicsToolboxCommandConverter(FullHumanoidRobotModel fullRobotModel, ReferenceFrames referenceFrames)
   {
      rigidBodyHashCodeResolver = new RigidBodyHashCodeResolver(fullRobotModel);
      referenceFrameHashCodeResolver = new ReferenceFrameHashCodeResolver(fullRobotModel, referenceFrames);
      jointHashCodeResolver = new JointHashCodeResolver(fullRobotModel);
   }

   public KinematicsToolboxCommandConverter(RigidBodyBasics rootBody)
   {
      rigidBodyHashCodeResolver = new RigidBodyHashCodeResolver();
      rigidBodyHashCodeResolver.putAllMultiBodySystemRigidBodies(rootBody);
      referenceFrameHashCodeResolver = new ReferenceFrameHashCodeResolver();
      referenceFrameHashCodeResolver.putAllMultiBodySystemReferenceFrames(rootBody);
      jointHashCodeResolver = new JointHashCodeResolver();
      jointHashCodeResolver.putAllMultiBodySystemJoints(rootBody);
   }

   /**
    * Only converting {@link KinematicsToolboxRigidBodyMessage}.
    */
   @Override
   public <C extends Command<?, M>, M extends Settable<M>> boolean isConvertible(C command, M message)
   {
      if (message instanceof KinematicsToolboxRigidBodyMessage)
         return true;
      if (message instanceof KinematicsToolboxOneDoFJointMessage)
         return true;
      if (message instanceof KinematicsToolboxPrivilegedConfigurationMessage)
         return true;
      if (message instanceof KinematicsToolboxSupportRegionMessage)
         return true;
      if (message instanceof KinematicsToolboxInputCollectionMessage)
         return true;
      return false;
   }

   /**
    * Retrieves the end-effector and convert the message into its command counterpart.
    */
   @Override
   public <C extends Command<?, M>, M extends Settable<M>> void process(C command, M message)
   {
      if (message instanceof KinematicsToolboxRigidBodyMessage)
      {
         KinematicsToolboxRigidBodyMessage rigidBodyMessage = (KinematicsToolboxRigidBodyMessage) message;
         KinematicsToolboxRigidBodyCommand rigidBodyCommand = (KinematicsToolboxRigidBodyCommand) command;
         rigidBodyCommand.set(rigidBodyMessage, rigidBodyHashCodeResolver, referenceFrameHashCodeResolver);
      }
      else if (message instanceof KinematicsToolboxOneDoFJointMessage)
      {
         KinematicsToolboxOneDoFJointMessage jointMessage = (KinematicsToolboxOneDoFJointMessage) message;
         KinematicsToolboxOneDoFJointCommand jointCommand = (KinematicsToolboxOneDoFJointCommand) command;
         jointCommand.set(jointMessage, jointHashCodeResolver);
      }
      else if (message instanceof KinematicsToolboxSupportRegionMessage)
      {
         KinematicsToolboxSupportRegionMessage contactStateMessage = (KinematicsToolboxSupportRegionMessage) message;
         KinematicsToolboxSupportRegionCommand contactStateCommand = (KinematicsToolboxSupportRegionCommand) command;
         contactStateCommand.set(contactStateMessage, referenceFrameHashCodeResolver);
      }
      else if (message instanceof KinematicsToolboxPrivilegedConfigurationMessage)
      {
         KinematicsToolboxPrivilegedConfigurationMessage privConfMessage = (KinematicsToolboxPrivilegedConfigurationMessage) message;
         KinematicsToolboxPrivilegedConfigurationCommand privConfCommand = (KinematicsToolboxPrivilegedConfigurationCommand) command;
         privConfCommand.set(privConfMessage, jointHashCodeResolver);
      }
      else if (message instanceof KinematicsToolboxInputCollectionMessage)
      {
         KinematicsToolboxInputCollectionMessage collectionMessage = (KinematicsToolboxInputCollectionMessage) message;
         KinematicsToolboxInputCollectionCommand collectionCommand = (KinematicsToolboxInputCollectionCommand) command;
         collectionCommand.set(collectionMessage, rigidBodyHashCodeResolver, referenceFrameHashCodeResolver, jointHashCodeResolver);
      }
   }
}
