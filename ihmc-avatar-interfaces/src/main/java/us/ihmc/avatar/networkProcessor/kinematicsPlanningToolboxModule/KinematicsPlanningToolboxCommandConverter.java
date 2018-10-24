package us.ihmc.avatar.networkProcessor.kinematicsPlanningToolboxModule;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import controller_msgs.msg.dds.KinematicsPlanningToolboxRigidBodyMessage;
import us.ihmc.communication.controllerAPI.CommandConversionInterface;
import us.ihmc.communication.controllerAPI.command.Command;
import us.ihmc.euclid.interfaces.Settable;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.kinematicsPlanningToolboxAPI.KinematicsPlanningToolboxRigidBodyCommand;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.sensorProcessing.frames.ReferenceFrameHashCodeResolver;

public class KinematicsPlanningToolboxCommandConverter implements CommandConversionInterface
{
   private final Map<Long, RigidBody> rigidBodyNamedBasedHashMap = new HashMap<>();
   private final ReferenceFrameHashCodeResolver referenceFrameHashCodeResolver;

   public KinematicsPlanningToolboxCommandConverter(FullHumanoidRobotModel fullRobotModel)
   {
      referenceFrameHashCodeResolver = new ReferenceFrameHashCodeResolver(fullRobotModel, new HumanoidReferenceFrames(fullRobotModel));

      RigidBody rootBody = ScrewTools.getRootBody(fullRobotModel.getElevator());
      RigidBody[] allRigidBodies = ScrewTools.computeSupportAndSubtreeSuccessors(rootBody);
      for (RigidBody rigidBody : allRigidBodies)
         rigidBodyNamedBasedHashMap.put(rigidBody.getNameBasedHashCode(), rigidBody);
   }

   public KinematicsPlanningToolboxCommandConverter(RigidBody rootBody)
   {
      List<ReferenceFrame> referenceFrames = new ArrayList<>();
      for (InverseDynamicsJoint joint : ScrewTools.computeSubtreeJoints(rootBody))
      {
         referenceFrames.add(joint.getFrameAfterJoint());
         referenceFrames.add(joint.getFrameBeforeJoint());
      }

      for (RigidBody rigidBody : ScrewTools.computeSupportAndSubtreeSuccessors(rootBody))
      {
         referenceFrames.add(rigidBody.getBodyFixedFrame());
      }

      referenceFrameHashCodeResolver = new ReferenceFrameHashCodeResolver(referenceFrames);

      RigidBody[] allRigidBodies = ScrewTools.computeSupportAndSubtreeSuccessors(rootBody);
      for (RigidBody rigidBody : allRigidBodies)
         rigidBodyNamedBasedHashMap.put(rigidBody.getNameBasedHashCode(), rigidBody);
   }

   @Override
   public <C extends Command<?, M>, M extends Settable<M>> boolean isConvertible(C command, M message)
   {
      return message instanceof KinematicsPlanningToolboxRigidBodyMessage;
   }

   @Override
   public <C extends Command<?, M>, M extends Settable<M>> void process(C command, M message)
   {
      KinematicsPlanningToolboxRigidBodyMessage rigiBodyMessage = (KinematicsPlanningToolboxRigidBodyMessage) message;
      KinematicsPlanningToolboxRigidBodyCommand rigiBodyCommand = (KinematicsPlanningToolboxRigidBodyCommand) command;
      rigiBodyCommand.set(rigiBodyMessage, rigidBodyNamedBasedHashMap, referenceFrameHashCodeResolver);
   }
}
