package us.ihmc.avatar.footstepPlanning;

import us.ihmc.avatar.footstepPlanning.MultiStageFootstepPlanningManager.ConcurrentList;
import us.ihmc.avatar.footstepPlanning.MultiStageFootstepPlanningManager.ConcurrentMap;
import us.ihmc.commons.PrintTools;
import us.ihmc.concurrent.ConcurrentCopier;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.footstepPlanning.FootstepPlannerGoal;
import us.ihmc.footstepPlanning.FootstepPlannerObjective;
import us.ihmc.robotics.robotSide.RobotSide;

import java.util.ArrayList;
import java.util.List;
import java.util.Map;

public class PlannerGoalRecommendationHandler
{
   private static final boolean debug = false;
   private static final boolean reinitializeOnGoalChange = true;

   private final ConcurrentList<FootstepPlanningStage> allPlanningStages;
   private final ConcurrentMap<FootstepPlanningStage, FootstepPlannerObjective> stepPlanningStagesInProgress;

   private final ConcurrentCopier<List<FootstepPlannerObjective>> footstepPlannerObjectives = new ConcurrentCopier<>(ArrayList::new);

   public PlannerGoalRecommendationHandler(ConcurrentList<FootstepPlanningStage> allPlanningStages,
                                           ConcurrentMap<FootstepPlanningStage, FootstepPlannerObjective> stepPlanningStagesInProgress)
   {
      this.allPlanningStages = allPlanningStages;
      this.stepPlanningStagesInProgress = stepPlanningStagesInProgress;
   }

   public void notifyWithPlannerGoalRecommendation(FootstepPlannerGoal newIntermediateGoal, RobotSide lastStepSide, int stageId)
   {
      FootstepPlanningStage currentPlanningStage = allPlanningStages.getCopyForReading().get(stageId);

      Map<FootstepPlanningStage, FootstepPlannerObjective> currentPlanningObjectives = stepPlanningStagesInProgress.getCopyForReading();
      Map<FootstepPlanningStage, FootstepPlannerObjective> updatedPlanningObjectives = stepPlanningStagesInProgress.getCopyForWriting();
      updatedPlanningObjectives.putAll(currentPlanningObjectives);

      FootstepPlannerObjective currentPlanningObjective = updatedPlanningObjectives.get(currentPlanningStage);
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

      RobotSide newInitialSide = lastStepSide.getOppositeSide();
      FramePose3D newInitialPose = newInitialSide.equals(RobotSide.LEFT) ? newLeftFootPoseGoal : newRightFootPoseGoal;

      newFinalPlanningObjective.setTimeout(currentPlanningObjective.getTimeout());
      newFinalPlanningObjective.setHorizonLength(remainingHorizonLength);
      newFinalPlanningObjective.setGoal(newFinalPlanningGoal);
      newFinalPlanningObjective.setInitialStanceFootPose(newInitialPose);
      newFinalPlanningObjective.setInitialStanceFootSide(newInitialSide);

      List<FootstepPlannerObjective> currentObjectives = footstepPlannerObjectives.getCopyForReading();
      List<FootstepPlannerObjective> newSetOfObjectives = footstepPlannerObjectives.getCopyForWriting();
      newSetOfObjectives.clear();
      if (currentObjectives != null)
         newSetOfObjectives.addAll(currentObjectives);
      newSetOfObjectives.add(newFinalPlanningObjective);
      footstepPlannerObjectives.commit();

      currentPlanningObjective.setGoal(newIntermediateGoal);

      if (reinitializeOnGoalChange)
      {
         currentPlanningStage.requestInitialize();
      }
      currentPlanningStage.setGoalUnsafe(newIntermediateGoal);

      if (debug)
         PrintTools.info("Adding a new objective.");

      stepPlanningStagesInProgress.commit();
   }

   public boolean hasNewFootstepPlannerObjectives()
   {
      List<FootstepPlannerObjective> objectives = footstepPlannerObjectives.getCopyForReading();
      if (objectives != null)
         return !objectives.isEmpty();
      else
         return false;
   }

   public FootstepPlannerObjective pollNextFootstepPlannerObjective()
   {
      List<FootstepPlannerObjective> currentObjectives = footstepPlannerObjectives.getCopyForReading();
      List<FootstepPlannerObjective> shortenedObjectives = footstepPlannerObjectives.getCopyForWriting();

      if (currentObjectives == null)
         return null;

      FootstepPlannerObjective objective = currentObjectives.remove(0);

      shortenedObjectives.clear();
      shortenedObjectives.addAll(currentObjectives);
      footstepPlannerObjectives.commit();

      return objective;
   }
}
