package us.ihmc.behaviors.activeMapping.ContinuousHikingStatesAndTransitions;

import controller_msgs.msg.dds.FootstepDataListMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.networkProcessor.footstepPlanningModule.FootstepPlanningModuleLauncher;
import us.ihmc.behaviors.activeMapping.ContinuousPlanner;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.footstepPlanning.graphSearch.FootstepPlannerEnvironmentHandler;
import us.ihmc.footstepPlanning.graphSearch.footstepSnapping.FootstepSnapAndWiggler;
import us.ihmc.footstepPlanning.graphSearch.graph.visualization.BipedalFootstepPlannerNodeRejectionReason;
import us.ihmc.footstepPlanning.graphSearch.stepChecking.FootstepPoseHeuristicChecker;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.yoVariables.registry.YoRegistry;

public class StepValidityChecker
{
   private boolean isNextStepValid = false;

   private final ContinuousPlanner continuousPlanner;
   private final FootstepPoseHeuristicChecker stepChecker;
   private final HumanoidReferenceFrames referenceFrames;

   public StepValidityChecker(ContinuousPlanner continuousPlanner, DRCRobotModel robotModel, HumanoidReferenceFrames referenceFrames, YoRegistry parentRegistry)
   {
      this.continuousPlanner = continuousPlanner;
      this.referenceFrames = referenceFrames;

      FootstepPlannerEnvironmentHandler environmentHandler = new FootstepPlannerEnvironmentHandler();
      FootstepSnapAndWiggler snapper = new FootstepSnapAndWiggler(FootstepPlanningModuleLauncher.createFootPolygons(robotModel),
                                                                  continuousPlanner.getFootstepPlannerParameters(),
                                                                  environmentHandler);
      stepChecker = new FootstepPoseHeuristicChecker(continuousPlanner.getFootstepPlannerParameters(), snapper, parentRegistry);

   }

   public boolean checkNextStepIsValid(FootstepDataListMessage footstepDataList)
   {
      FramePose3DReadOnly stanceFootPose = new FramePose3D(ReferenceFrame.getWorldFrame(), continuousPlanner.getImminentFootstepPose());

      FramePose3DReadOnly candidateStepPose = new FramePose3D(ReferenceFrame.getWorldFrame(),
                                                              footstepDataList.getFootstepDataList().get(0).getLocation(),
                                                              footstepDataList.getFootstepDataList().get(0).getOrientation());

      BipedalFootstepPlannerNodeRejectionReason reason;
      FramePose3D startOfSwingPose = new FramePose3D();

      RobotSide stepSide = continuousPlanner.getImminentFootstepSide();
      assert stepSide != null;
      startOfSwingPose.setFromReferenceFrame(referenceFrames.getSoleFrame(stepSide.getOppositeSide()));
      reason = stepChecker.checkValidity(stepSide.getOppositeSide(), candidateStepPose, stanceFootPose, startOfSwingPose);

      isNextStepValid = reason == null;

      return isNextStepValid;
   }

   public boolean isNextStepValid()
   {
      return isNextStepValid;
   }

}
