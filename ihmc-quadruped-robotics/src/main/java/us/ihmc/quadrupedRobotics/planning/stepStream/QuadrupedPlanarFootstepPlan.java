package us.ihmc.quadrupedRobotics.planning.stepStream;

import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedOrientedStep;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.util.PreallocatedList;
import us.ihmc.robotics.robotSide.EndDependentList;
import us.ihmc.robotics.robotSide.RobotEnd;

public class QuadrupedPlanarFootstepPlan
{
   private final EndDependentList<QuadrupedTimedOrientedStep> currentSteps;
   private final PreallocatedList<QuadrupedTimedOrientedStep> plannedSteps;

   private final PreallocatedList<QuadrupedTimedOrientedStep> completeStepSequence;

   public QuadrupedPlanarFootstepPlan(int planCapacity)
   {
      currentSteps = new EndDependentList<>(new QuadrupedTimedOrientedStep(), new QuadrupedTimedOrientedStep());
      plannedSteps = new PreallocatedList<>(planCapacity, QuadrupedTimedOrientedStep::new);
      completeStepSequence = new PreallocatedList<>(planCapacity + 2, QuadrupedTimedOrientedStep::new);
   }

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

   public PreallocatedList<QuadrupedTimedOrientedStep> getPlannedSteps()
   {
      return plannedSteps;
   }

   public EndDependentList<QuadrupedTimedOrientedStep> getCurrentSteps()
   {
      return currentSteps;
   }

   public PreallocatedList<QuadrupedTimedOrientedStep> getCompleteStepSequence(double currentTime)
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
