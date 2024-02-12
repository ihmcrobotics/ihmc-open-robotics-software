package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTUnidirectionalDouble;
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

   private final DetachableReferenceFrame screwFrame;
   private final CRDTUnidirectionalPoseList previewTrajectory;
   private final CRDTUnidirectionalVector3D force;
   private final CRDTUnidirectionalVector3D torque;
   private final CRDTUnidirectionalDouble previewTrajectoryDuration;
   private final CRDTUnidirectionalDouble previewTrajectoryLinearVelocity;
   private final CRDTUnidirectionalDouble previewTrajectoryAngularVelocity;

   public ScrewPrimitiveActionState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory, ReferenceFrameLibrary referenceFrameLibrary)
   {
      super(id, new ScrewPrimitiveActionDefinition(crdtInfo, saveFileDirectory), crdtInfo);

      screwFrame = new DetachableReferenceFrame(referenceFrameLibrary, getDefinition().getScrewAxisPoseInObjectFrame().getValueReadOnly());
      previewTrajectory = new CRDTUnidirectionalPoseList(ROS2ActorDesignation.ROBOT, crdtInfo);
      force = new CRDTUnidirectionalVector3D(ROS2ActorDesignation.ROBOT, crdtInfo);
      torque = new CRDTUnidirectionalVector3D(ROS2ActorDesignation.ROBOT, crdtInfo);
      previewTrajectoryDuration = new CRDTUnidirectionalDouble(ROS2ActorDesignation.ROBOT, crdtInfo, -1.0);
      previewTrajectoryLinearVelocity = new CRDTUnidirectionalDouble(ROS2ActorDesignation.ROBOT, crdtInfo, -1.0);
      previewTrajectoryAngularVelocity = new CRDTUnidirectionalDouble(ROS2ActorDesignation.ROBOT, crdtInfo, -1.0);
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
}
