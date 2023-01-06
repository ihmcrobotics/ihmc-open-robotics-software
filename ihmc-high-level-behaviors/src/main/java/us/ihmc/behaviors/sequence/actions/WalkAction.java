package us.ihmc.behaviors.sequence.actions;

import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.BehaviorAction;
import us.ihmc.behaviors.sequence.ReferenceFrameLibrary;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.footstepPlanning.tools.FootstepPlannerRejectionReasonReport;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.referenceFrames.ModifiableReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

import java.util.UUID;

public class WalkAction extends WalkActionData implements BehaviorAction
{
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final ReferenceFrameLibrary referenceFrameLibrary;
   private ReferenceFrame parentReferenceFrame;
   private final ModifiableReferenceFrame gizmoFrame = new ModifiableReferenceFrame(ReferenceFrame.getWorldFrame());
   private final SideDependentList<FramePose3D> goalFeetPoses = new SideDependentList<>(() -> new FramePose3D());
   private final FootstepPlanningModule footstepPlanner;
   private final FootstepPlannerParametersBasics footstepPlannerParameters;
   private FootstepDataListMessage footstepDataListMessage;

   public WalkAction(ROS2ControllerHelper ros2ControllerHelper,
                     ROS2SyncedRobotModel syncedRobot,
                     FootstepPlanningModule footstepPlanner,
                     FootstepPlannerParametersBasics footstepPlannerParameters,
                     ReferenceFrameLibrary referenceFrameLibrary)
   {
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.syncedRobot = syncedRobot;
      this.footstepPlanner = footstepPlanner;
      this.footstepPlannerParameters = footstepPlannerParameters;
      this.referenceFrameLibrary = referenceFrameLibrary;
   }

   @Override
   public void update()
   {
      ReferenceFrame previousParentReferenceFrame = parentReferenceFrame;
      parentReferenceFrame = referenceFrameLibrary.findFrameByName(getParentFrameName());
      if (parentReferenceFrame != previousParentReferenceFrame)
      {
         gizmoFrame.changeParentFrame(parentReferenceFrame);
      }

      gizmoFrame.getTransformToParent().set(getTransformToParent());
      gizmoFrame.getReferenceFrame().update();

      for (RobotSide side : RobotSide.values)
      {
         goalFeetPoses.get(side).setIncludingFrame(gizmoFrame.getReferenceFrame(), getGoalFootstepToGizmos().get(side));
         goalFeetPoses.get(side).changeFrame(ReferenceFrame.getWorldFrame());
      }
   }

   public void plan()
   {
      FramePose3D leftFootPose = new FramePose3D(syncedRobot.getReferenceFrames().getSoleFrame(RobotSide.LEFT));
      FramePose3D rightFootPose = new FramePose3D(syncedRobot.getReferenceFrames().getSoleFrame(RobotSide.RIGHT));
      leftFootPose.changeFrame(ReferenceFrame.getWorldFrame());
      rightFootPose.changeFrame(ReferenceFrame.getWorldFrame());

      footstepPlannerParameters.setFinalTurnProximity(1.0);

      FootstepPlannerRequest footstepPlannerRequest = new FootstepPlannerRequest();
      footstepPlannerRequest.setPlanBodyPath(false);
      footstepPlannerRequest.setStartFootPoses(leftFootPose, rightFootPose);
      // TODO: Set start footholds!!
      for (RobotSide side : RobotSide.values)
      {
         footstepPlannerRequest.setGoalFootPose(side, goalFeetPoses.get(side));
      }

      //      footstepPlannerRequest.setPlanarRegionsList(...);
      footstepPlannerRequest.setAssumeFlatGround(true); // FIXME Assuming flat ground

      footstepPlanner.getFootstepPlannerParameters().set(footstepPlannerParameters);
      double idealFootstepLength = 0.5;
      footstepPlanner.getFootstepPlannerParameters().setIdealFootstepLength(idealFootstepLength);
      footstepPlanner.getFootstepPlannerParameters().setMaximumStepReach(idealFootstepLength);
      LogTools.info("Planning footsteps...");
      FootstepPlannerOutput footstepPlannerOutput = footstepPlanner.handleRequest(footstepPlannerRequest);
      LogTools.info("Footstep planner completed with {}, {} step(s)",
                    footstepPlannerOutput.getFootstepPlanningResult(),
                    footstepPlannerOutput.getFootstepPlan().getNumberOfSteps());

      FootstepPlannerLogger footstepPlannerLogger = new FootstepPlannerLogger(footstepPlanner);
      footstepPlannerLogger.logSession();
      ThreadTools.startAThread(FootstepPlannerLogger::deleteOldLogs, "FootstepPlanLogDeletion");

      if (footstepPlannerOutput.getFootstepPlan().getNumberOfSteps() < 1) // failed
      {
         FootstepPlannerRejectionReasonReport rejectionReasonReport = new FootstepPlannerRejectionReasonReport(footstepPlanner);
         rejectionReasonReport.update();
         for (BipedalFootstepPlannerNodeRejectionReason reason : rejectionReasonReport.getSortedReasons())
         {
            double rejectionPercentage = rejectionReasonReport.getRejectionReasonPercentage(reason);
            LogTools.info("Rejection {}%: {}", FormattingTools.getFormattedToSignificantFigures(rejectionPercentage, 3), reason);
         }
         LogTools.info("Footstep planning failure...");
         footstepDataListMessage = null;
      }
      else
      {
         footstepDataListMessage = FootstepDataMessageConverter.createFootstepDataListFromPlan(footstepPlannerOutput.getFootstepPlan(),
                                                                                               getSwingDuration(),
                                                                                               getTransferDuration());
         footstepDataListMessage.getQueueingProperties().setExecutionMode(ExecutionMode.OVERRIDE.toByte());
         footstepDataListMessage.getQueueingProperties().setMessageId(UUID.randomUUID().getLeastSignificantBits());
      }
   }

   @Override
   public void executeAction()
   {
      plan();
      if (footstepDataListMessage != null)
      {
         ros2ControllerHelper.publishToController(footstepDataListMessage);
      }
   }
}
