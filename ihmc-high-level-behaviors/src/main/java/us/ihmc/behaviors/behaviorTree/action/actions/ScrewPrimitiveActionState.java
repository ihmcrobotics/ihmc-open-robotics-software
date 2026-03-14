package us.ihmc.behaviors.behaviorTree.action.actions;

import behavior_msgs.msg.dds.ScrewPrimitiveActionStateMessage;
import us.ihmc.behaviors.behaviorTree.BehaviorTreeRootNodeState;
import us.ihmc.behaviors.behaviorTree.action.ActionNodeState;
import us.ihmc.communication.crdt.CRDTStatusDouble;
import us.ihmc.communication.crdt.CRDTStatusDoubleArray;
import us.ihmc.communication.crdt.CRDTStatusPoseList;
import us.ihmc.communication.crdt.CRDTStatusVector3D;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.robotics.referenceFrames.DetachableReferenceFrame;

public class ScrewPrimitiveActionState extends ActionNodeState<ScrewPrimitiveActionDefinition>
{
   /** This limit is defined in the .msg file and limited to the size in the SE3TrajectoryMessage. */
   public static final int TRAJECTORY_SIZE_LIMIT = new ScrewPrimitiveActionStateMessage().getPreviewTrajectory().getCurrentCapacity();

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

   public ScrewPrimitiveActionState(long id, BehaviorTreeRootNodeState rootNode)
   {
      super(id, new ScrewPrimitiveActionDefinition(rootNode.getDefinition()), rootNode);

      screwFrame = new DetachableReferenceFrame(scene::findFrameByName, definition.getScrewAxisPoseInObjectFrame().getValueReadOnly());
      previewTrajectory = new CRDTStatusPoseList(ROS2ActorDesignation.ROBOT, crdtInfo);
      force = new CRDTStatusVector3D(ROS2ActorDesignation.ROBOT, crdtInfo);
      torque = new CRDTStatusVector3D(ROS2ActorDesignation.ROBOT, crdtInfo);
      previewTrajectoryDuration = new CRDTStatusDouble(ROS2ActorDesignation.ROBOT, crdtInfo, -1.0);
      previewTrajectoryLinearVelocity = new CRDTStatusDouble(ROS2ActorDesignation.ROBOT, crdtInfo, -1.0);
      previewTrajectoryAngularVelocity = new CRDTStatusDouble(ROS2ActorDesignation.ROBOT, crdtInfo, -1.0);
      previewRequestedTime = new CRDTStatusDouble(ROS2ActorDesignation.OPERATOR, crdtInfo, 1.0);
      previewJointAngles = new CRDTStatusDoubleArray(ROS2ActorDesignation.ROBOT, crdtInfo, ArmActionDefinition.MAX_NUMBER_OF_JOINTS);
      previewSolutionQuality = new CRDTStatusDouble(ROS2ActorDesignation.ROBOT, crdtInfo, 0.0);
   }

   @Override
   public void update()
   {
      screwFrame.update(definition.getObjectFrameName());
   }

   @Override
   public boolean hasStatus()
   {
      boolean hasStatus = super.hasStatus();
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
      definition.toMessage(message.getDefinition());

      super.toMessage(message.getState());

      previewTrajectory.toMessage(message.getPreviewTrajectory());
      force.toMessage(message.getForce());
      torque.toMessage(message.getTorque());
      message.setPreviewTrajectoryDuration(previewTrajectoryDuration.toMessage());
      message.setPreviewTrajectoryLinearVelocity(previewTrajectoryLinearVelocity.toMessage());
      message.setPreviewTrajectoryAngularVelocity(previewTrajectoryAngularVelocity.toMessage());
      message.setPreviewRequestedTime(previewRequestedTime.toMessage());
      for (int i = 0; i < ArmActionDefinition.MAX_NUMBER_OF_JOINTS; i++)
      {
         previewJointAngles.toMessage(message.getPreviewJointAngles());
      }
      message.setPreviewSolutionQuality(previewSolutionQuality.toMessage());
   }

   public void fromMessage(ScrewPrimitiveActionStateMessage message)
   {
      definition.fromMessage(message.getDefinition());

      super.fromMessage(message.getState());

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
