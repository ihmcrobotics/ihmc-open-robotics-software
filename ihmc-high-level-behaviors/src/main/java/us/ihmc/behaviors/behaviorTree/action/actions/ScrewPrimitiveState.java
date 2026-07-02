package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.msg.dds.ArmActionStateMessage;
import us.ihmc.behaviors.behaviorTree.scene.BehaviorTreeSceneState;
import us.ihmc.communication.crdt.*;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;

import static us.ihmc.behaviors.behaviorTree.action.actions.ArmActionDefinition.MAX_NUMBER_OF_JOINTS;

public class ScrewPrimitiveState
{
   /** This limit is defined in the .msg file and limited to the size in the SE3TrajectoryMessage. */
   public static final int TRAJECTORY_SIZE_LIMIT = new ArmActionStateMessage().getPreviewTrajectory().getCurrentCapacity();

   private final DetachableReferenceFrame screwFrame;
   private final CRDTStatusPoseList previewTrajectory;
   private final CRDTStatusDouble previewTrajectoryDuration;
   private final CRDTStatusDouble previewTrajectoryLinearVelocity;
   private final CRDTStatusDouble previewTrajectoryAngularVelocity;
   private final CRDTStatusDouble previewRequestedTime;
   private final CRDTStatusDoubleArray screwPreviewJointAngles;
   private final CRDTStatusDouble previewSolutionQuality;

   public ScrewPrimitiveState(BehaviorTreeSceneState scene, CRDTInfo crdtInfo, ScrewPrimitiveDefinition definition)
   {
      screwFrame = new DetachableReferenceFrame(scene::findFrameByName, definition.getScrewAxisPoseInObjectFrame().getValueReadOnly());
      previewTrajectory = new CRDTStatusPoseList(ROS2ActorDesignation.ROBOT, crdtInfo);
      previewTrajectoryDuration = new CRDTStatusDouble(ROS2ActorDesignation.ROBOT, crdtInfo, -1.0);
      previewTrajectoryLinearVelocity = new CRDTStatusDouble(ROS2ActorDesignation.ROBOT, crdtInfo, -1.0);
      previewTrajectoryAngularVelocity = new CRDTStatusDouble(ROS2ActorDesignation.ROBOT, crdtInfo, -1.0);
      previewRequestedTime = new CRDTStatusDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, 1.0);
      screwPreviewJointAngles = new CRDTStatusDoubleArray(ROS2ActorDesignation.ROBOT, crdtInfo, MAX_NUMBER_OF_JOINTS);
      previewSolutionQuality = new CRDTStatusDouble(ROS2ActorDesignation.ROBOT, crdtInfo, 0.0);
   }

   public void update(String parentFrameName)
   {
      screwFrame.update(parentFrameName);
   }

   public boolean pollHasStatus()
   {
      boolean hasStatus = false;
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
      previewTrajectory.fromMessage(message.getPreviewTrajectory());
      previewTrajectoryDuration.fromMessage(message.getPreviewTrajectoryDuration());
      previewTrajectoryLinearVelocity.fromMessage(message.getPreviewTrajectoryLinearVelocity());
      previewTrajectoryAngularVelocity.fromMessage(message.getPreviewTrajectoryAngularVelocity());
      previewRequestedTime.fromMessage(message.getPreviewRequestedTime());
      screwPreviewJointAngles.fromMessage(message.getPreviewJointAngles());
      previewSolutionQuality.fromMessage(message.getPreviewSolutionQuality());
   }

   public DetachableReferenceFrame getScrewFrame()
   {
      return screwFrame;
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
}
