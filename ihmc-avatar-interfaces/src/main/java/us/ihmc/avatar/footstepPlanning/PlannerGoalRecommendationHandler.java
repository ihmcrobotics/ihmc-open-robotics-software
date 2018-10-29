package us.ihmc.avatar.footstepPlanning;

import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlannerGoalType;
import us.ihmc.footstepPlanning.FootstepPlannerObjective;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;

public class PlannerGoalRecommendationHandler
{
   private final List<FootstepPlanningStage> allPlanningStages;
   private final HashMap<FootstepPlanningStage, FootstepPlannerObjective> stepPlanningStagesInProgress;

   private final List<FootstepPlannerObjective> footstepPlannerObjectives = new ArrayList<>();

   public PlannerGoalRecommendationHandler(List<FootstepPlanningStage> allPlanningStages,
                                           HashMap<FootstepPlanningStage, FootstepPlannerObjective> stepPlanningStagesInProgress,
                                           FootstepPlannerParameters parameters)
   {
      this.allPlanningStages = allPlanningStages;
      this.stepPlanningStagesInProgress = stepPlanningStagesInProgress;
   }

   public void notifyWithPlannerGoalRecommendation(FootstepPlannerGoal newIntermediateGoal, int stageId)
   {
      FootstepPlanningStage currentPlanningStage = allPlanningStages.get(stageId);

      FootstepPlannerObjective currentPlanningObjective = stepPlanningStagesInProgress.get(currentPlanningStage);
      FootstepPlannerObjective newFinalPlanningObjective = new FootstepPlannerObjective();

      FootstepPlannerGoal newFinalPlanningGoal = new FootstepPlannerGoal();
      newFinalPlanningGoal.set(currentPlanningObjective.getGoal());


      FramePose3D newLeftFootPoseGoal = new FramePose3D();
      FramePose3D newRightFootPoseGoal = new FramePose3D();
      FramePose3D newMidstancePoseGoal = new FramePose3D();

      newIntermediateGoal.getDoubleFootstepGoal().get(RobotSide.LEFT).getSoleFramePose(newLeftFootPoseGoal);
      newIntermediateGoal.getDoubleFootstepGoal().get(RobotSide.RIGHT).getSoleFramePose(newRightFootPoseGoal);
      newMidstancePoseGoal.interpolate(newLeftFootPoseGoal, newRightFootPoseGoal, 0.5);

      double currentPlanningDistance = currentPlanningObjective.getInitialStanceFootPose().getPositionDistance(newMidstancePoseGoal);
      double remainingHorizonLength = Math.max(0.0, currentPlanningObjective.getHorizonLength() - currentPlanningDistance);

      RobotSide newInitialSide = RobotSide.LEFT;

      newFinalPlanningObjective.setTimeout(currentPlanningObjective.getTimeout());
      newFinalPlanningObjective.setHorizonLength(remainingHorizonLength);
      newFinalPlanningObjective.setGoal(newFinalPlanningGoal);
      newFinalPlanningObjective.setInitialStanceFootPose(newLeftFootPoseGoal);
      newFinalPlanningObjective.setInitialStanceFootSide(newInitialSide);

      footstepPlannerObjectives.add(newFinalPlanningObjective);

      currentPlanningObjective.setGoal(newIntermediateGoal);
      currentPlanningStage.setGoal(newIntermediateGoal);
   }

   public boolean hasNewFootstepPlannerObjectives()
   {
      return !footstepPlannerObjectives.isEmpty();
   }

   public FootstepPlannerObjective pollNextFootstepPlannerObjective()
   {
      return footstepPlannerObjectives.remove(0);
   }
}
