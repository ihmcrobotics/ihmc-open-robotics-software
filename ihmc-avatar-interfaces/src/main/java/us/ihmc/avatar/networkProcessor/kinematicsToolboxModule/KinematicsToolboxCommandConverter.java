package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import us.ihmc.communication.controllerAPI.CommandConversionInterface;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.RigidBodyHashCodeResolver;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

/**
 * This class allows the retrieve the rigid-body from its hash code when converting a
 * {@link KinematicsToolboxRigidBodyMessage} into a {@link KinematicsToolboxRigidBodyCommand}.
 * 
 * @author Sylvain Bertrand
 *
 */
public class KinematicsToolboxCommandConverter implements CommandConversionInterface
{
   private final RigidBodyHashCodeResolver rigidBodyHashCodeResolver;
   private final ReferenceFrameHashCodeResolver referenceFrameHashCodeResolver;

   public KinematicsToolboxCommandConverter(FullHumanoidRobotModel fullRobotModel)
   {
      rigidBodyHashCodeResolver = new RigidBodyHashCodeResolver(fullRobotModel);
      referenceFrameHashCodeResolver = new ReferenceFrameHashCodeResolver(fullRobotModel, new HumanoidReferenceFrames(fullRobotModel));
   }

   public KinematicsToolboxCommandConverter(RigidBodyBasics rootBody)
   {
      rigidBodyHashCodeResolver = new RigidBodyHashCodeResolver();
      rigidBodyHashCodeResolver.putAllMultiBodySystemRigidBodies(rootBody);
      referenceFrameHashCodeResolver = new ReferenceFrameHashCodeResolver();
      referenceFrameHashCodeResolver.putAllMultiBodySystemReferenceFrames(rootBody);
   }

   /**
    * Only converting {@link KinematicsToolboxRigidBodyMessage}.
    */
   @Override
   public <C extends Command<?, M>, M extends Settable<M>> boolean isConvertible(C command, M message)
   {
      return message instanceof KinematicsToolboxRigidBodyMessage;
   }

   /**
    * Retrieves the end-effector and convert the message into its command counterpart.
    */
   @Override
   public <C extends Command<?, M>, M extends Settable<M>> void process(C command, M message)
   {
      KinematicsToolboxRigidBodyMessage rigiBodyMessage = (KinematicsToolboxRigidBodyMessage) message;
      KinematicsToolboxRigidBodyCommand rigiBodyCommand = (KinematicsToolboxRigidBodyCommand) command;
      rigiBodyCommand.set(rigiBodyMessage, rigidBodyHashCodeResolver, referenceFrameHashCodeResolver);
   }
}
