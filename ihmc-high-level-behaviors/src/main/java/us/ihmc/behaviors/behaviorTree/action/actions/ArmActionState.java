package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.msg.dds.ArmActionStateMessage;
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
   /** This limit is defined in the .msg file and limited to the size in the SE3TrajectoryMessage. */
   public static final int TRAJECTORY_SIZE_LIMIT = new ArmActionStateMessage().getPreviewTrajectory().getCurrentCapacity();

   private final CRDTDetachableReferenceFrame palmFrame;
   private final DetachableReferenceFrame screwFrame;
   /**
    * This is the estimated goal chest frame as the robot executes a potential whole body action.
    * This is used to compute joint angles that achieve the desired and previewed end pose
    * even when the pelvis and/or chest might also move.
    */
   private final CRDTStatusRigidBodyTransform goalChestToWorldTransform;
   private final ReferenceFrame goalChestFrame;
   private final CRDTStatusVector3D force;
   private final CRDTStatusVector3D torque;
   private final CRDTStatusDoubleArray previewJointAngles;
   private final CRDTStatusDouble solutionQuality;
   private final CRDTStatusPoseList previewTrajectory;
   private final CRDTStatusDouble previewTrajectoryDuration;
   private final CRDTStatusDouble previewTrajectoryLinearVelocity;
   private final CRDTStatusDouble previewTrajectoryAngularVelocity;
   private final CRDTStatusDouble previewRequestedTime;
   private final CRDTStatusDoubleArray screwPreviewJointAngles;
   private final CRDTStatusDouble previewSolutionQuality;
   private final SideDependentList<Integer> numberOfJoints = new SideDependentList<>();

   public ArmActionState(long id, BehaviorTreeRootNodeState rootNode)
   {
      super(id, new ArmActionDefinition(rootNode.getDefinition()), rootNode);

      palmFrame = new CRDTDetachableReferenceFrame(scene::findFrameByName,
                                                   definition.getCRDTPalmParentFrameName(),
                                                   definition.getPalmTransformToParent());
      screwFrame = new DetachableReferenceFrame(scene::findFrameByName, definition.getScrewAxisPoseInObjectFrame().getValueReadOnly());
      goalChestToWorldTransform = new CRDTStatusRigidBodyTransform(ROS2ActorDesignation.ROBOT, crdtInfo);
      goalChestFrame = ReferenceFrameMissingTools.constructFrameWithChangingTransformToParent(ReferenceFrame.getWorldFrame(),
                                                                                              goalChestToWorldTransform.getValueReadOnly());
      force = new CRDTStatusVector3D(ROS2ActorDesignation.ROBOT, crdtInfo);
      torque = new CRDTStatusVector3D(ROS2ActorDesignation.ROBOT, crdtInfo);
      previewJointAngles = new CRDTStatusDoubleArray(ROS2ActorDesignation.ROBOT, crdtInfo, MAX_NUMBER_OF_JOINTS);
      solutionQuality = new CRDTStatusDouble(ROS2ActorDesignation.ROBOT, crdtInfo, Double.NaN);
      previewTrajectory = new CRDTStatusPoseList(ROS2ActorDesignation.ROBOT, crdtInfo);
      previewTrajectoryDuration = new CRDTStatusDouble(ROS2ActorDesignation.ROBOT, crdtInfo, -1.0);
      previewTrajectoryLinearVelocity = new CRDTStatusDouble(ROS2ActorDesignation.ROBOT, crdtInfo, -1.0);
      previewTrajectoryAngularVelocity = new CRDTStatusDouble(ROS2ActorDesignation.ROBOT, crdtInfo, -1.0);
      previewRequestedTime = new CRDTStatusDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, 1.0);
      screwPreviewJointAngles = new CRDTStatusDoubleArray(ROS2ActorDesignation.ROBOT, crdtInfo, MAX_NUMBER_OF_JOINTS);
      previewSolutionQuality = new CRDTStatusDouble(ROS2ActorDesignation.ROBOT, crdtInfo, 0.0);

      for (RobotSide side : RobotSide.values)
         numberOfJoints.put(side, robotModel.getJointMap().getArmJointNamesAsStrings(side).size());
   }

   @Override
   public void update()
   {
      palmFrame.update();
      screwFrame.update(definition.getPalmParentFrameName());
   }

   @Override
   public boolean hasStatus()
   {
      boolean hasStatus = super.hasStatus();
      hasStatus |= goalChestToWorldTransform.pollHasStatus();
      hasStatus |= force.pollHasStatus();
      hasStatus |= torque.pollHasStatus();
      hasStatus |= previewJointAngles.pollHasStatus();
      hasStatus |= solutionQuality.pollHasStatus();
      hasStatus |= previewTrajectory.pollHasStatus();
      hasStatus |= previewTrajectoryDuration.pollHasStatus();
      hasStatus |= previewTrajectoryLinearVelocity.pollHasStatus();
      hasStatus |= previewTrajectoryAngularVelocity.pollHasStatus();
      hasStatus |= previewRequestedTime.pollHasStatus();
      hasStatus |= screwPreviewJointAngles.pollHasStatus();
      hasStatus |= previewSolutionQuality.pollHasStatus();
      return hasStatus;
   }

   public void toMessage(ArmActionStateMessage message)
   {
      definition.toMessage(message.getDefinition());

      super.toMessage(message.getState());

      goalChestToWorldTransform.toMessage(message.getGoalChestTransformToWorld());
      force.toMessage(message.getForce());
      torque.toMessage(message.getTorque());
      previewJointAngles.toMessage(message.getJointAngles());
      message.setSolutionQuality(solutionQuality.toMessage());
      previewTrajectory.toMessage(message.getPreviewTrajectory());
      message.setPreviewTrajectoryDuration(previewTrajectoryDuration.toMessage());
      message.setPreviewTrajectoryLinearVelocity(previewTrajectoryLinearVelocity.toMessage());
      message.setPreviewTrajectoryAngularVelocity(previewTrajectoryAngularVelocity.toMessage());
      message.setPreviewRequestedTime(previewRequestedTime.toMessage());
      screwPreviewJointAngles.toMessage(message.getPreviewJointAngles());
      message.setPreviewSolutionQuality(previewSolutionQuality.toMessage());
   }

   public void fromMessage(ArmActionStateMessage message)
   {
      definition.fromMessage(message.getDefinition());

      super.fromMessage(message.getState());

      force.fromMessage(message.getForce());
      torque.fromMessage(message.getTorque());
      previewJointAngles.fromMessage(message.getJointAngles());
      solutionQuality.fromMessage(message.getSolutionQuality());
      goalChestToWorldTransform.fromMessage(message.getGoalChestTransformToWorld());
      goalChestFrame.update();
      previewTrajectory.fromMessage(message.getPreviewTrajectory());
      previewTrajectoryDuration.fromMessage(message.getPreviewTrajectoryDuration());
      previewTrajectoryLinearVelocity.fromMessage(message.getPreviewTrajectoryLinearVelocity());
      previewTrajectoryAngularVelocity.fromMessage(message.getPreviewTrajectoryAngularVelocity());
      previewRequestedTime.fromMessage(message.getPreviewRequestedTime());
      screwPreviewJointAngles.fromMessage(message.getPreviewJointAngles());
      previewSolutionQuality.fromMessage(message.getPreviewSolutionQuality());
   }

   public CRDTDetachableReferenceFrame getPalmFrame()
   {
      return palmFrame;
   }

   public DetachableReferenceFrame getScrewFrame()
   {
      return screwFrame;
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
      return previewTrajectory;
   }

   public CRDTStatusDouble getPreviewTrajectoryDuration()
   {
      return previewTrajectoryDuration;
   }

   public CRDTStatusDouble getPreviewTrajectoryLinearVelocity()
   {
      return previewTrajectoryLinearVelocity;
   }

   public CRDTStatusDouble getPreviewTrajectoryAngularVelocity()
   {
      return previewTrajectoryAngularVelocity;
   }

   public CRDTStatusDouble getPreviewRequestedTime()
   {
      return previewRequestedTime;
   }

   public CRDTStatusDoubleArray getScrewPreviewJointAngles()
   {
      return screwPreviewJointAngles;
   }

   public CRDTStatusDouble getPreviewSolutionQuality()
   {
      return previewSolutionQuality;
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
