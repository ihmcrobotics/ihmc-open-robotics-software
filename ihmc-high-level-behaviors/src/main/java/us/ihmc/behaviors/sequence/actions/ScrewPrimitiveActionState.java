package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTStatusDouble;
import us.ihmc.communication.crdt.CRDTStatusDoubleArray;
import us.ihmc.communication.crdt.CRDTStatusPoseList;
import us.ihmc.communication.crdt.CRDTStatusVector3D;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class ScrewPrimitiveActionState extends ActionNodeState<ScrewPrimitiveActionDefinition>
{
   /** This limit is defined in the .msg file and limited to the size in the SE3TrajectoryMessage. */
   public static final int TRAJECTORY_SIZE_LIMIT = new ScrewPrimitiveActionStateMessage().getPreviewTrajectory().getCurrentCapacity();

   private final ScrewPrimitiveActionDefinition definition;
   private final DetachableReferenceFrame screwFrame;
   private final CRDTStatusPoseList previewTrajectory;
   private final CRDTStatusVector3D force;
   private final CRDTStatusVector3D torque;
   private final CRDTStatusDouble previewTrajectoryDuration;
   private final CRDTStatusDouble previewTrajectoryLinearVelocity;
   private final CRDTStatusDouble previewTrajectoryAngularVelocity;
   private final CRDTStatusDouble previewRequestedTime;
   private final CRDTStatusDoubleArray previewJointAngles;
   private final CRDTStatusDouble previewSolutionQuality;

   public ScrewPrimitiveActionState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory, ReferenceFrameLibrary referenceFrameLibrary)
   {
      super(id, new ScrewPrimitiveActionDefinition(crdtInfo, saveFileDirectory), crdtInfo);

      definition = getDefinition();

      screwFrame = new DetachableReferenceFrame(referenceFrameLibrary, getDefinition().getScrewAxisPoseInObjectFrame().getValueReadOnly());
      previewTrajectory = new CRDTStatusPoseList(ROS2ActorDesignation.ROBOT, crdtInfo);
      force = new CRDTStatusVector3D(ROS2ActorDesignation.ROBOT, crdtInfo);
      torque = new CRDTStatusVector3D(ROS2ActorDesignation.ROBOT, crdtInfo);
      previewTrajectoryDuration = new CRDTStatusDouble(ROS2ActorDesignation.ROBOT, crdtInfo, -1.0);
      previewTrajectoryLinearVelocity = new CRDTStatusDouble(ROS2ActorDesignation.ROBOT, crdtInfo, -1.0);
      previewTrajectoryAngularVelocity = new CRDTStatusDouble(ROS2ActorDesignation.ROBOT, crdtInfo, -1.0);
      previewRequestedTime = new CRDTStatusDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, 1.0);
      previewJointAngles = new CRDTStatusDoubleArray(ROS2ActorDesignation.ROBOT, crdtInfo, HandPoseActionDefinition.MAX_NUMBER_OF_JOINTS);
      previewSolutionQuality = new CRDTStatusDouble(ROS2ActorDesignation.ROBOT, crdtInfo, 0.0);
   }

   @Override
   public void update()
   {
      screwFrame.update(getDefinition().getObjectFrameName());
   }

   @Override
   public boolean hasStatus()
   {
      boolean hasStatus = false;
      hasStatus |= previewTrajectory.pollHasStatus();
      hasStatus |= force.pollHasStatus();
      hasStatus |= torque.pollHasStatus();
      hasStatus |= previewTrajectoryDuration.pollHasStatus();
      hasStatus |= previewTrajectoryLinearVelocity.pollHasStatus();
      hasStatus |= previewTrajectoryAngularVelocity.pollHasStatus();
      hasStatus |= previewRequestedTime.pollHasStatus();
      hasStatus |= previewJointAngles.pollHasStatus();
      hasStatus |= previewSolutionQuality.pollHasStatus();
      return hasStatus;
   }

   public void toMessage(ScrewPrimitiveActionStateMessage message)
   {
      getDefinition().toMessage(message.getDefinition());

      super.toMessage(message.getState());

      previewTrajectory.toMessage(message.getPreviewTrajectory());
      force.toMessage(message.getForce());
      torque.toMessage(message.getTorque());
      message.setPreviewTrajectoryDuration(previewTrajectoryDuration.toMessage());
      message.setPreviewTrajectoryLinearVelocity(previewTrajectoryLinearVelocity.toMessage());
      message.setPreviewTrajectoryAngularVelocity(previewTrajectoryAngularVelocity.toMessage());
      message.setPreviewRequestedTime(previewRequestedTime.toMessage());
      for (int i = 0; i < HandPoseActionDefinition.MAX_NUMBER_OF_JOINTS; i++)
      {
         previewJointAngles.toMessage(message.getPreviewJointAngles());
      }
      message.setPreviewSolutionQuality(previewSolutionQuality.toMessage());
   }

   public void fromMessage(ScrewPrimitiveActionStateMessage message)
   {
      super.fromMessage(message.getState());

      getDefinition().fromMessage(message.getDefinition());

      previewTrajectory.fromMessage(message.getPreviewTrajectory());
      force.fromMessage(message.getForce());
      torque.fromMessage(message.getTorque());
      previewTrajectoryDuration.fromMessage(message.getPreviewTrajectoryDuration());
      previewTrajectoryLinearVelocity.fromMessage(message.getPreviewTrajectoryLinearVelocity());
      previewTrajectoryAngularVelocity.fromMessage(message.getPreviewTrajectoryAngularVelocity());
      previewRequestedTime.fromMessage(message.getPreviewRequestedTime());
      previewJointAngles.fromMessage(message.getPreviewJointAngles());
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

   public CRDTStatusVector3D getForce()
   {
      return force;
   }

   public CRDTStatusVector3D getTorque()
   {
      return torque;
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

   public CRDTStatusDoubleArray getPreviewJointAngles()
   {
      return previewJointAngles;
   }

   public CRDTStatusDouble getPreviewSolutionQuality()
   {
      return previewSolutionQuality;
   }
}
