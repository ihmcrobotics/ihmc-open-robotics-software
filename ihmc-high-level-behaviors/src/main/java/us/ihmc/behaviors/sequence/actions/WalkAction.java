package us.ihmc.behaviors.sequence.actions;

import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.ros2.ROS2ControllerHelper;
import us.ihmc.behaviors.sequence.BehaviorAction;
import us.ihmc.behaviors.sequence.BehaviorActionSequenceTools;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commons.FormattingTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.packets.ExecutionMode;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.footstepPlanning.*;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParametersBasics;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogger;
import us.ihmc.footstepPlanning.tools.FootstepPlannerRejectionReasonReport;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.log.LogTools;
import us.ihmc.robotics.referenceFrames.ReferenceFrameLibrary;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.tools.Timer;
import us.ihmc.tools.thread.Throttler;

import java.util.UUID;

public class WalkAction extends WalkActionData implements BehaviorAction
{
   private final ROS2ControllerHelper ros2ControllerHelper;
   private final ROS2SyncedRobotModel syncedRobot;
   private final SideDependentList<FramePose3D> goalFeetPoses = new SideDependentList<>(() -> new FramePose3D());
   private final SideDependentList<FramePose3D> syncedFeetPoses = new SideDependentList<>(() -> new FramePose3D());
   private final SideDependentList<FramePose3D> commandedGoalFeetTransformToWorld = new SideDependentList<>(() -> new FramePose3D());
   private final FootstepPlanningModule footstepPlanner;
   private final FootstepPlannerParametersBasics footstepPlannerParameters;
   private final WalkingControllerParameters walkingControllerParameters;
   private FootstepDataListMessage footstepDataListMessage;
   private final Timer executionTimer = new Timer();
   private final Throttler warningThrottler = new Throttler().setFrequency(2.0);

   public WalkAction(ROS2ControllerHelper ros2ControllerHelper,
                     ROS2SyncedRobotModel syncedRobot,
                     FootstepPlanningModule footstepPlanner,
                     FootstepPlannerParametersBasics footstepPlannerParameters,
                     WalkingControllerParameters walkingControllerParameters,
                     ReferenceFrameLibrary referenceFrameLibrary)
   {
      this.ros2ControllerHelper = ros2ControllerHelper;
      this.syncedRobot = syncedRobot;
      this.footstepPlanner = footstepPlanner;
      this.footstepPlannerParameters = footstepPlannerParameters;
      this.walkingControllerParameters = walkingControllerParameters;
      setReferenceFrameLibrary(referenceFrameLibrary);
   }

   @Override
   public void update(int actionIndex, int nextExecutionIndex)
   {
      for (RobotSide side : RobotSide.values)
      {
         goalFeetPoses.get(side).setIncludingFrame(getReferenceFrame(), getGoalFootstepToParentTransforms().get(side));
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
         goalFeetPoses.get(side).get(commandedGoalFeetTransformToWorld.get(side));
      }

      //      footstepPlannerRequest.setPlanarRegionsList(...);
      footstepPlannerRequest.setAssumeFlatGround(true); // FIXME Assuming flat ground

      footstepPlanner.getFootstepPlannerParameters().set(footstepPlannerParameters);
      double idealFootstepLength = 0.5;
      footstepPlanner.getFootstepPlannerParameters().setIdealFootstepLength(idealFootstepLength);
      footstepPlanner.getFootstepPlannerParameters().setMaximumStepReach(idealFootstepLength);
      LogTools.info("Planning footsteps...");
      FootstepPlannerOutput footstepPlannerOutput = footstepPlanner.handleRequest(footstepPlannerRequest);
      FootstepPlan footstepPlan = footstepPlannerOutput.getFootstepPlan();
      LogTools.info("Footstep planner completed with {}, {} step(s)",
                    footstepPlannerOutput.getFootstepPlanningResult(),
                    footstepPlan.getNumberOfSteps());

      FootstepPlannerLogger footstepPlannerLogger = new FootstepPlannerLogger(footstepPlanner);
      footstepPlannerLogger.logSession();
      ThreadTools.startAThread(FootstepPlannerLogger::deleteOldLogs, "FootstepPlanLogDeletion");

      if (footstepPlan.getNumberOfSteps() < 1) // failed
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
         footstepPlan.getFootstep(0).setTransferDuration(getTransferDuration() / 2.0);

         if (footstepPlan.getNumberOfSteps() > 1)
            footstepPlan.getFootstep(footstepPlan.getNumberOfSteps() - 1).setTransferDuration(getTransferDuration() / 2.0);

         footstepDataListMessage = FootstepDataMessageConverter.createFootstepDataListFromPlan(footstepPlan,
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
         executionTimer.reset();
      }
   }

   @Override
   public boolean isExecuting()
   {
      double nominalExecutionDuration = PlannerTools.calculateNominalTotalPlanExecutionDuration(footstepPlanner.getOutput().getFootstepPlan(),
                                                                                                getSwingDuration(),
                                                                                                walkingControllerParameters.getDefaultInitialTransferTime(),
                                                                                                getTransferDuration(),
                                                                                                walkingControllerParameters.getDefaultFinalTransferTime());
      boolean isExecuting = false;
      for (RobotSide side : RobotSide.values)
      {
         syncedFeetPoses.get(side).setFromReferenceFrame(syncedRobot.getReferenceFrames().getSoleFrame(side));

         double translationTolerance = 0.15;
         double rotationTolerance = Math.toRadians(10.0);
         isExecuting |= BehaviorActionSequenceTools.isExecuting(commandedGoalFeetTransformToWorld.get(side),
                                                                syncedFeetPoses.get(side),
                                                                translationTolerance,
                                                                rotationTolerance,
                                                                nominalExecutionDuration,
                                                                executionTimer,
                                                                warningThrottler);
      }
      return isExecuting;
   }
}
