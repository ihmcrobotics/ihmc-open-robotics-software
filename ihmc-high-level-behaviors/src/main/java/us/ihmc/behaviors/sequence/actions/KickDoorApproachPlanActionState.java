package us.ihmc.behaviors.sequence.actions;

import behavior_msgs.msg.dds.KickDoorApproachPlanStateMessage;
import us.ihmc.behaviors.sequence.ActionNodeState;
import us.ihmc.communication.crdt.*;
import us.ihmc.communication.ros2.ROS2ActorDesignation;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.io.WorkspaceResourceDirectory;

public class KickDoorApproachPlanActionState extends ActionNodeState<KickDoorApproachPlanActionDefinition>
{
   private final KickDoorApproachPlanActionDefinition definition;
   private final ReferenceFrameLibrary referenceFrameLibrary;
   private final CRDTUnidirectionalPose3D leftFootGoalPose;
   private final CRDTUnidirectionalPose3D rightFootGoalPose;
   private final ReferenceFrame parentFrame;
   private final CRDTUnidirectionalInteger totalNumberOfFootsteps;
   private final CRDTUnidirectionalInteger numberOfIncompleteFootsteps;
   private final SideDependentList<CRDTUnidirectionalSE3Trajectory> desiredFootPoses = new SideDependentList<>();
   private final SideDependentList<CRDTUnidirectionalPose3D> currentFootPoses = new SideDependentList<>();
   private final CRDTUnidirectionalEnumField<FootstepPlanActionExecutionState> executionState;

   public KickDoorApproachPlanActionState(long id, CRDTInfo crdtInfo, WorkspaceResourceDirectory saveFileDirectory, ReferenceFrameLibrary referenceFrameLibrary)
   {
      super(id, new KickDoorApproachPlanActionDefinition(crdtInfo, saveFileDirectory), crdtInfo);

      definition = getDefinition();

      this.referenceFrameLibrary = referenceFrameLibrary;

      parentFrame = referenceFrameLibrary.findFrameByName(definition.getParentFrameName());

      leftFootGoalPose = new CRDTUnidirectionalPose3D(ROS2ActorDesignation.ROBOT, crdtInfo);
      rightFootGoalPose = new CRDTUnidirectionalPose3D(ROS2ActorDesignation.ROBOT, crdtInfo);
      totalNumberOfFootsteps = new CRDTUnidirectionalInteger(ROS2ActorDesignation.ROBOT, crdtInfo, 0);
      numberOfIncompleteFootsteps = new CRDTUnidirectionalInteger(ROS2ActorDesignation.ROBOT, crdtInfo, 0);
      for (RobotSide side : RobotSide.values)
      {
         desiredFootPoses.set(side, new CRDTUnidirectionalSE3Trajectory(ROS2ActorDesignation.ROBOT, crdtInfo));
         currentFootPoses.set(side, new CRDTUnidirectionalPose3D(ROS2ActorDesignation.ROBOT, crdtInfo));
      }
      executionState = new CRDTUnidirectionalEnumField<>(ROS2ActorDesignation.ROBOT, crdtInfo, FootstepPlanActionExecutionState.PLANNING_SUCCEEDED);
   }

   @Override
   public void update()
   {
   }

   public void toMessage(KickDoorApproachPlanStateMessage message)
   {
      definition.toMessage(message.getDefinition());

      super.toMessage(message.getState());

      leftFootGoalPose.toMessage(message.getLeftFootGoalPose());
      rightFootGoalPose.toMessage(message.getRightFootGoalPose());
      message.setTotalNumberOfFootsteps(totalNumberOfFootsteps.toMessage());
      message.setNumberOfIncompleteFootsteps(numberOfIncompleteFootsteps.toMessage());
      desiredFootPoses.get(RobotSide.LEFT).toMessage(message.getDesiredLeftFootsteps());
      desiredFootPoses.get(RobotSide.RIGHT).toMessage(message.getDesiredRightFootsteps());
      currentFootPoses.get(RobotSide.LEFT).toMessage(message.getCurrentLeftFootPose());
      currentFootPoses.get(RobotSide.RIGHT).toMessage(message.getCurrentRightFootPose());


      message.setExecutionState(executionState.toMessage().toByte());
   }

   public void fromMessage(KickDoorApproachPlanStateMessage message)
   {
      super.fromMessage(message.getState());

      definition.fromMessage(message.getDefinition());

      leftFootGoalPose.fromMessage(message.getLeftFootGoalPose());
      rightFootGoalPose.fromMessage(message.getRightFootGoalPose());
      totalNumberOfFootsteps.fromMessage(message.getTotalNumberOfFootsteps());
      numberOfIncompleteFootsteps.fromMessage(message.getNumberOfIncompleteFootsteps());
      desiredFootPoses.get(RobotSide.LEFT).fromMessage(message.getDesiredLeftFootsteps());
      desiredFootPoses.get(RobotSide.RIGHT).fromMessage(message.getDesiredRightFootsteps());
      currentFootPoses.get(RobotSide.LEFT).fromMessage(message.getCurrentLeftFootPose());
      currentFootPoses.get(RobotSide.RIGHT).fromMessage(message.getCurrentRightFootPose());

      executionState.fromMessage(FootstepPlanActionExecutionState.fromByte(message.getExecutionState()));
   }

   public boolean areFramesInWorld()
   {
      return referenceFrameLibrary.containsFrame(definition.getParentFrameName()) && parentFrame.getRootFrame() == ReferenceFrame.getWorldFrame();
   }

   public ReferenceFrame getParentFrame()
   {
      return parentFrame;
   }

   public int getTotalNumberOfFootsteps()
   {
      return totalNumberOfFootsteps.getValue();
   }

   public void setTotalNumberOfFootsteps(int totalNumberOfFootsteps)
   {
      this.totalNumberOfFootsteps.setValue(totalNumberOfFootsteps);
   }

   public int getNumberOfIncompleteFootsteps()
   {
      return numberOfIncompleteFootsteps.getValue();
   }

   public void setNumberOfIncompleteFootsteps(int numberOfIncompleteFootsteps)
   {
      this.numberOfIncompleteFootsteps.setValue(numberOfIncompleteFootsteps);
   }

   public CRDTUnidirectionalPose3D getLeftFootGoalPose()
   {
      return leftFootGoalPose;
   }

   public CRDTUnidirectionalPose3D getRightFootGoalPose()
   {
      return rightFootGoalPose;
   }

   public SideDependentList<CRDTUnidirectionalSE3Trajectory> getDesiredFootPoses()
   {
      return desiredFootPoses;
   }

   public SideDependentList<CRDTUnidirectionalPose3D> getCurrentFootPoses()
   {
      return currentFootPoses;
   }

   public CRDTUnidirectionalEnumField<FootstepPlanActionExecutionState> getExecutionState()
   {
      return executionState;
   }
}
