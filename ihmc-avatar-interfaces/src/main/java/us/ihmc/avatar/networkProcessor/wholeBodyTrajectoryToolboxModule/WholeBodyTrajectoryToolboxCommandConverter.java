package us.ihmc.avatar.networkProcessor.wholeBodyTrajectoryToolboxModule;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import toolbox_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import us.ihmc.communication.controllerAPI.CommandConversionInterface;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.kinematicsToolboxAPI.KinematicsToolboxRigidBodyCommand;
import us.ihmc.humanoidRobotics.communication.wholeBodyTrajectoryToolboxAPI.WholeBodyTrajectoryToolboxAPI;
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
public class WholeBodyTrajectoryToolboxCommandConverter implements CommandConversionInterface
{
   private final Map<Integer, RigidBodyBasics> rigidBodyHashMap = new HashMap<>();
   private final ReferenceFrameHashCodeResolver referenceFrameHashCodeResolver;

   public WholeBodyTrajectoryToolboxCommandConverter(FullHumanoidRobotModel fullRobotModel)
   {
      referenceFrameHashCodeResolver = new ReferenceFrameHashCodeResolver(fullRobotModel, new HumanoidReferenceFrames(fullRobotModel));

      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(fullRobotModel.getElevator());
      for (RigidBodyBasics rigidBody : rootBody.subtreeIterable())
         rigidBodyHashMap.put(rigidBody.hashCode(), rigidBody);
   }

   public WholeBodyTrajectoryToolboxCommandConverter(RigidBodyBasics rootBody)
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

   public RigidBodyBasics getRigidBody(int hashCode)
   {
      return rigidBodyHashMap.get(hashCode);
   }

   /**
    * Only converting {@link KinematicsToolboxRigidBodyMessage}.
    */
   @Override
   public <C extends Command<?, M>, M extends Settable<M>> boolean isConvertible(C command, M message)
   {
      if (command instanceof WholeBodyTrajectoryToolboxAPI<?>)
         return true;
      return false;
   }

   /**
    * Retrieves the end-effector and convert the message into its command counterpart.
    */
   //@SuppressWarnings("unchecked")
   @Override
   public <C extends Command<?, M>, M extends Settable<M>> void process(C command, M message)
   {
      WholeBodyTrajectoryToolboxAPI<M> wholeBodyTrajectoryCommand = (WholeBodyTrajectoryToolboxAPI<M>) command;
      wholeBodyTrajectoryCommand.set(message, rigidBodyHashMap, referenceFrameHashCodeResolver);
   }
}
