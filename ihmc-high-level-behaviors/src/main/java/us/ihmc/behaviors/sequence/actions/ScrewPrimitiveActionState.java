package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTUnidirectionalDouble;
import us.ihmc.communication.crdt.CRDTUnidirectionalDoubleArray;
import us.ihmc.communication.crdt.CRDTUnidirectionalPoseList;
import us.ihmc.communication.crdt.CRDTUnidirectionalVector3D;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class ScrewPrimitiveActionState extends ActionNodeState<ScrewPrimitiveActionDefinition>
{
   /** This limit is defined in the .msg file and limited to the size in the SE3TrajectoryMessage. */
   public static final int TRAJECTORY_SIZE_LIMIT = new ScrewPrimitiveActionStateMessage().getPreviewTrajectory().getCurrentCapacity();

   private final ScrewPrimitiveActionDefinition definition;
   private final DetachableReferenceFrame screwFrame;
   private final CRDTUnidirectionalPoseList previewTrajectory;
   private final CRDTUnidirectionalVector3D force;
   private final CRDTUnidirectionalVector3D torque;
   private final CRDTUnidirectionalDouble previewTrajectoryDuration;
   private final CRDTUnidirectionalDouble previewTrajectoryLinearVelocity;
   private final CRDTUnidirectionalDouble previewTrajectoryAngularVelocity;
   private final CRDTUnidirectionalDouble previewRequestedTime;
   private final CRDTUnidirectionalDoubleArray previewJointAngles;
   private final CRDTUnidirectionalDouble previewSolutionQuality;

   public ScrewPrimitiveActionState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory, ReferenceFrameLibrary referenceFrameLibrary)
   {
      super(id, new ScrewPrimitiveActionDefinition(crdtInfo, saveFileDirectory), crdtInfo);

      definition = getDefinition();

      screwFrame = new DetachableReferenceFrame(referenceFrameLibrary, getDefinition().getScrewAxisPoseInObjectFrame().getValueReadOnly());
      previewTrajectory = new CRDTUnidirectionalPoseList(ROS2ActorDesignation.ROBOT, definition);
      force = new CRDTUnidirectionalVector3D(ROS2ActorDesignation.ROBOT, definition);
      torque = new CRDTUnidirectionalVector3D(ROS2ActorDesignation.ROBOT, definition);
      previewTrajectoryDuration = new CRDTUnidirectionalDouble(ROS2ActorDesignation.ROBOT, definition, -1.0);
      previewTrajectoryLinearVelocity = new CRDTUnidirectionalDouble(ROS2ActorDesignation.ROBOT, definition, -1.0);
      previewTrajectoryAngularVelocity = new CRDTUnidirectionalDouble(ROS2ActorDesignation.ROBOT, definition, -1.0);
      previewRequestedTime = new CRDTUnidirectionalDouble(ROS2ActorDesignation.OPERATOR, definition, 1.0);
      previewJointAngles = new CRDTUnidirectionalDoubleArray(ROS2ActorDesignation.ROBOT, definition, HandPoseActionDefinition.MAX_NUMBER_OF_JOINTS);
      previewSolutionQuality = new CRDTUnidirectionalDouble(ROS2ActorDesignation.ROBOT, definition, 0.0);
   }

   @Override
   public void update()
   {
      screwFrame.update(getDefinition().getObjectFrameName());
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

   public CRDTUnidirectionalPoseList getPreviewTrajectory()
   {
      return previewTrajectory;
   }

   public CRDTUnidirectionalVector3D getForce()
   {
      return force;
   }

   public CRDTUnidirectionalVector3D getTorque()
   {
      return torque;
   }

   public CRDTUnidirectionalDouble getPreviewTrajectoryDuration()
   {
      return previewTrajectoryDuration;
   }

   public CRDTUnidirectionalDouble getPreviewTrajectoryLinearVelocity()
   {
      return previewTrajectoryLinearVelocity;
   }

   public CRDTUnidirectionalDouble getPreviewTrajectoryAngularVelocity()
   {
      return previewTrajectoryAngularVelocity;
   }

   public CRDTUnidirectionalDouble getPreviewRequestedTime()
   {
      return previewRequestedTime;
   }

   public CRDTUnidirectionalDoubleArray getPreviewJointAngles()
   {
      return previewJointAngles;
   }

   public CRDTUnidirectionalDouble getPreviewSolutionQuality()
   {
      return previewSolutionQuality;
   }
}
