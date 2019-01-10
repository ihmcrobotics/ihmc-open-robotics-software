package us.ihmc.quadrupedPlanning.footstepPlanning;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedOrientedStep;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.robotics.robotSide.EndDependentList;
import us.ihmc.robotics.robotSide.RobotEnd;

public class QuadrupedFootstepPlan
{
   private final EndDependentList<QuadrupedTimedOrientedStep> currentSteps = new EndDependentList<>(new QuadrupedTimedOrientedStep(), new QuadrupedTimedOrientedStep());
   private final RecyclingArrayList<QuadrupedTimedOrientedStep> plannedSteps = new RecyclingArrayList<>(QuadrupedTimedOrientedStep::new);

   private final RecyclingArrayList<QuadrupedTimedOrientedStep> completeStepSequence = new RecyclingArrayList<>(QuadrupedTimedOrientedStep::new);

   public void initializeCurrentStepsFromPlannedSteps()
   {
      for (int i = 0; i < 2; i++)
      {
         RobotEnd robotEnd = plannedSteps.get(i).getRobotQuadrant().getEnd();
         currentSteps.get(robotEnd).set(plannedSteps.get(i));
      }
   }

   public void updateCurrentSteps(double currentTime)
   {
      for (int i = 0; i < plannedSteps.size(); i++)
      {
         QuadrupedTimedStep xGaitPreviewStep = plannedSteps.get(i);
         if (xGaitPreviewStep.getTimeInterval().getStartTime() <= currentTime)
         {
            currentSteps.get(xGaitPreviewStep.getRobotQuadrant().getEnd()).set(xGaitPreviewStep);
         }
      }
   }

   public RecyclingArrayList<QuadrupedTimedOrientedStep> getStepPlan()
   {
      return plannedSteps;
   }

   public EndDependentList<QuadrupedTimedOrientedStep> getCurrentSteps()
   {
      return currentSteps;
   }

   public RecyclingArrayList<QuadrupedTimedOrientedStep> getCompleteStepSequence(double currentTime)
   {
      completeStepSequence.clear();
      for (RobotEnd robotEnd : RobotEnd.values)
      {
         if (currentSteps.get(robotEnd).getTimeInterval().getEndTime() >= currentTime)
         {
            completeStepSequence.add();
            completeStepSequence.get(completeStepSequence.size() - 1).set(currentSteps.get(robotEnd));
         }
      }
      for (int i = 0; i < plannedSteps.size(); i++)
      {
         if (plannedSteps.get(i).getTimeInterval().getEndTime() >= currentTime)
         {
            completeStepSequence.add();
            completeStepSequence.get(completeStepSequence.size() - 1).set(plannedSteps.get(i));
         }
      }

      return completeStepSequence;
   }
}
