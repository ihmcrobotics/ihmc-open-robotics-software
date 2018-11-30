package us.ihmc.avatar.networkProcessor.kinematicsPlanningToolboxModule;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import controller_msgs.msg.dds.KinematicsPlanningToolboxRigidBodyMessage;
import controller_msgs.msg.dds.KinematicsToolboxRigidBodyMessage;
import us.ihmc.communication.controllerAPI.CommandConversionInterface;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.kinematicsPlanningToolboxAPI.KinematicsPlanningToolboxRigidBodyCommand;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class KinematicsPlanningToolboxCommandConverter implements CommandConversionInterface
{
   private final Map<Integer, RigidBodyBasics> rigidBodyHashMap = new HashMap<>();
   private final ReferenceFrameHashCodeResolver referenceFrameHashCodeResolver;

   public KinematicsPlanningToolboxCommandConverter(FullHumanoidRobotModel fullRobotModel)
   {
      referenceFrameHashCodeResolver = new ReferenceFrameHashCodeResolver(fullRobotModel, new HumanoidReferenceFrames(fullRobotModel));

      RigidBodyBasics rootBody = MultiBodySystemTools.getRootBody(fullRobotModel.getElevator());
      for (RigidBodyBasics rigidBody : rootBody.subtreeIterable())
         rigidBodyHashMap.put(rigidBody.hashCode(), rigidBody);
   }

   public KinematicsPlanningToolboxCommandConverter(RigidBodyBasics rootBody)
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

   @Override
   public <C extends Command<?, M>, M extends Settable<M>> boolean isConvertible(C command, M message)
   {
      return message instanceof KinematicsToolboxRigidBodyMessage;
   }

   @Override
   public <C extends Command<?, M>, M extends Settable<M>> void process(C command, M message)
   {
      KinematicsPlanningToolboxRigidBodyMessage rigiBodyMessage = (KinematicsPlanningToolboxRigidBodyMessage) message;
      KinematicsPlanningToolboxRigidBodyCommand rigiBodyCommand = (KinematicsPlanningToolboxRigidBodyCommand) command;
      rigiBodyCommand.set(rigiBodyMessage, rigidBodyHashMap, referenceFrameHashCodeResolver);
   }

}
