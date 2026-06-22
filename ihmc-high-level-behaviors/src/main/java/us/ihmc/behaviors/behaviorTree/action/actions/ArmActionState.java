package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.ArmActionStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeState;
import us.ihmc.communication.crdt.*;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameMissingTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import static us.ihmc.behaviors.behaviorTree.action.actions.ArmActionDefinition.MAX_NUMBER_OF_JOINTS;

public class ArmActionState extends ActionNodeState<ArmActionDefinition>
{
   private final CRDTDetachableReferenceFrame palmFrame;
   /**
    * This is the estimated goal chest frame as the robot executes a potential whole body action.
    * This is used to compute joint angles that achieve the desired and previewed end pose
    * even when the pelvis and/or chest might also move.
    */
   private final CRDTStatusRigidBodyTransform goalChestToWorldTransform;
   private final ReferenceFrame goalChestFrame;
   private final CRDTStatusDoubleArray previewJointAngles;
   private final CRDTStatusDouble solutionQuality;
   private final ScrewPrimitiveState screwPrimitive;
   private final CRDTStatusVector3D force;
   private final CRDTStatusVector3D torque;
   private final SideDependentList<Integer> numberOfJoints = new SideDependentList<>();

   public ArmActionState(long id, BehaviorTreeRootNodeState rootNode)
   {
      super(id, new ArmActionDefinition(rootNode.getDefinition()), rootNode);

      palmFrame = new CRDTDetachableReferenceFrame(scene::findFrameByName,
                                                   definition.getCRDTPalmParentFrameName(),
                                                   definition.getPalmTransformToParent());
      goalChestToWorldTransform = new CRDTStatusRigidBodyTransform(ROS2ActorDesignation.ROBOT, crdtInfo);
      goalChestFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(),
                                                                                              goalChestToWorldTransform.getValueReadOnly());
      previewJointAngles = new CRDTStatusDoubleArray(ROS2ActorDesignation.ROBOT, crdtInfo, MAX_NUMBER_OF_JOINTS);
      solutionQuality = new CRDTStatusDouble(ROS2ActorDesignation.ROBOT, crdtInfo, Double.NaN);
      screwPrimitive = new ScrewPrimitiveState(scene, crdtInfo, definition.getScrewPrimitive());
      force = new CRDTStatusVector3D(ROS2ActorDesignation.ROBOT, crdtInfo);
      torque = new CRDTStatusVector3D(ROS2ActorDesignation.ROBOT, crdtInfo);

      for (RobotSide side : RobotSide.values)
         numberOfJoints.put(side, robotModel.getJointMap().getArmJointNamesAsStrings(side).size());
   }

   @Override
   public void update()
   {
      palmFrame.update();
      screwPrimitive.update(definition.getPalmParentFrameName());
   }

   @Override
   public boolean hasStatus()
   {
      boolean hasStatus = super.hasStatus();
      hasStatus |= goalChestToWorldTransform.pollHasStatus();
      hasStatus |= previewJointAngles.pollHasStatus();
      hasStatus |= solutionQuality.pollHasStatus();
      hasStatus |= screwPrimitive.pollHasStatus();
      hasStatus |= force.pollHasStatus();
      hasStatus |= torque.pollHasStatus();
      return hasStatus;
   }

   public void toMessage(ArmActionStateMessage message)
   {
      definition.toMessage(message.getDefinition());

      super.toMessage(message.getState());

      goalChestToWorldTransform.toMessage(message.getGoalChestTransformToWorld());
      previewJointAngles.toMessage(message.getJointAngles());
      message.setSolutionQuality(solutionQuality.toMessage());
      screwPrimitive.toMessage(message);
      message.getForce().set(force.getValueReadOnly());
      message.getTorque().set(torque.getValueReadOnly());
   }

   public void fromMessage(ArmActionStateMessage message)
   {
      definition.fromMessage(message.getDefinition());

      super.fromMessage(message.getState());

      goalChestToWorldTransform.fromMessage(message.getGoalChestTransformToWorld());
      goalChestFrame.update();
      previewJointAngles.fromMessage(message.getJointAngles());
      solutionQuality.fromMessage(message.getSolutionQuality());
      screwPrimitive.fromMessage(message);
      force.fromMessage(message.getForce().getVector());
      torque.fromMessage(message.getTorque().getVector());
   }

   public CRDTDetachableReferenceFrame getPalmFrame()
   {
      return palmFrame;
   }

   public ScrewPrimitiveState getScrewPrimitive()
   {
      return screwPrimitive;
   }

   public DetachableReferenceFrame getScrewFrame()
   {
      return screwPrimitive.getScrewFrame();
   }

   public CRDTStatusRigidBodyTransform getGoalChestToWorldTransform()
   {
      return goalChestToWorldTransform;
   }

   public ReferenceFrame getGoalChestFrame()
   {
      return goalChestFrame;
   }

   public CRDTStatusVector3D getForce()
   {
      return force;
   }

   public CRDTStatusVector3D getTorque()
   {
      return torque;
   }

   public CRDTStatusDoubleArray getPreviewJointAngles()
   {
      return previewJointAngles;
   }

   public CRDTStatusPoseList getPreviewTrajectory()
   {
      return screwPrimitive.getPreviewTrajectory();
   }

   public CRDTStatusDouble getPreviewTrajectoryDuration()
   {
      return screwPrimitive.getPreviewTrajectoryDuration();
   }

   public CRDTStatusDouble getPreviewTrajectoryLinearVelocity()
   {
      return screwPrimitive.getPreviewTrajectoryLinearVelocity();
   }

   public CRDTStatusDouble getPreviewTrajectoryAngularVelocity()
   {
      return screwPrimitive.getPreviewTrajectoryAngularVelocity();
   }

   public CRDTStatusDouble getPreviewRequestedTime()
   {
      return screwPrimitive.getPreviewRequestedTime();
   }

   public CRDTStatusDoubleArray getScrewPreviewJointAngles()
   {
      return screwPrimitive.getScrewPreviewJointAngles();
   }

   public CRDTStatusDouble getPreviewSolutionQuality()
   {
      return screwPrimitive.getPreviewSolutionQuality();
   }

   public int getNumberOfJoints()
   {
      return numberOfJoints.get(definition.getSide());
   }

   public double getSolutionQuality()
   {
      return solutionQuality.getValue();
   }

   public void setSolutionQuality(double solutionQuality)
   {
      this.solutionQuality.setValue(solutionQuality);
   }
}
