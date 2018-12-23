package us.ihmc.avatar.networkProcessor.kinematicsToolboxModule;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import us.ihmc.communication.controllerAPI.CommandConversionInterface;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
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
   private final Map<Integer, RigidBodyBasics> rigidBodyHashMap = new HashMap<>();
   private final ReferenceFrameHashCodeResolver referenceFrameHashCodeResolver;

   public KinematicsToolboxCommandConverter(FullHumanoidRobotModel fullRobotModel)
   {
      referenceFrameHashCodeResolver = new ReferenceFrameHashCodeResolver(fullRobotModel, new HumanoidReferenceFrames(fullRobotModel));

      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(fullRobotModel.getElevator());
      for (RigidBodyBasics rigidBody : rootBody.subtreeIterable())
         rigidBodyHashMap.put(rigidBody.hashCode(), rigidBody);
   }

   public KinematicsToolboxCommandConverter(RigidBodyBasics rootBody)
   {
      List<ReferenceFrame> referenceFrames = new ArrayList<>();
      for (JointBasics joint : rootBody.childrenSubtreeIterable())
      {
         referenceFrames.add(joint.getFrameAfterJoint());
         referenceFrames.add(joint.getFrameBeforeJoint());
      }

      for (RigidBodyBasics rigidBody : rootBody.subtreeIterable())
      {
         referenceFrames.add(rigidBody.getBodyFixedFrame());
      }

      referenceFrameHashCodeResolver = new ReferenceFrameHashCodeResolver(referenceFrames);

      for (RigidBodyBasics rigidBody : rootBody.subtreeIterable())
         rigidBodyHashMap.put(rigidBody.hashCode(), rigidBody);
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
      rigiBodyCommand.set(rigiBodyMessage, rigidBodyHashMap, referenceFrameHashCodeResolver);
   }
}
