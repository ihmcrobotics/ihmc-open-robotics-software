package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.communication.crdt.CRDTInfo;
import us.ihmc.communication.crdt.CRDTUnidirectionalPoseList;
import us.ihmc.communication.crdt.CRDTUnidirectionalVector3D;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class ScrewPrimitiveActionState extends ActionNodeState<ScrewPrimitiveActionDefinition>
{
   /** This limit is defined in the .msg file and limited to the size in the SE3TrajectoryMessage. */
   public static final int TRAJECTORY_SIZE_LIMIT = new ScrewPrimitiveActionStateMessage().getTrajectory().getCurrentCapacity();

   private final DetachableReferenceFrame screwFrame;
   private final CRDTUnidirectionalPoseList trajectory;
   private final CRDTUnidirectionalVector3D force;
   private final CRDTUnidirectionalVector3D torque;

   public ScrewPrimitiveActionState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory, ReferenceFrameLibrary referenceFrameLibrary)
   {
      super(id, new ScrewPrimitiveActionDefinition(crdtInfo, saveFileDirectory), crdtInfo);

      screwFrame = new DetachableReferenceFrame(referenceFrameLibrary, getDefinition().getScrewAxisPoseInObjectFrame().getValueReadOnly());
      trajectory = new CRDTUnidirectionalPoseList(ROS2ActorDesignation.ROBOT, crdtInfo);
      force = new CRDTUnidirectionalVector3D(ROS2ActorDesignation.ROBOT, crdtInfo);
      torque = new CRDTUnidirectionalVector3D(ROS2ActorDesignation.ROBOT, crdtInfo);
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

      trajectory.toMessage(message.getTrajectory());
      force.toMessage(message.getForce());
      torque.toMessage(message.getTorque());
   }

   public void fromMessage(ScrewPrimitiveActionStateMessage message)
   {
      super.fromMessage(message.getState());

      getDefinition().fromMessage(message.getDefinition());

      trajectory.fromMessage(message.getTrajectory());
      force.fromMessage(message.getForce());
      torque.fromMessage(message.getTorque());
   }

   public DetachableReferenceFrame getScrewFrame()
   {
      return screwFrame;
   }

   public CRDTUnidirectionalPoseList getTrajectory()
   {
      return trajectory;
   }

   public CRDTUnidirectionalVector3D getForce()
   {
      return force;
   }

   public CRDTUnidirectionalVector3D getTorque()
   {
      return torque;
   }
}
