package us.ihmc.quadrupedRobotics.planning.icp;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedBasics.utils.TimeIntervalTools;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.tools.lists.ListSorter;

import java.util.Comparator;
import java.util.List;

public class QuadrupedContactSequenceTools
{
   public static void computeStepTransitionsFromStepSequence(RecyclingArrayList<QuadrupedStepTransition> stepTransitionsToPack, double currentTime,
                                                             List<? extends QuadrupedTimedStep> stepSequence)
   {
      stepTransitionsToPack.clear();
      for (int i = 0; i < stepSequence.size(); i++)
      {
         QuadrupedTimedStep step = stepSequence.get(i);

         if (step.getTimeInterval().getStartTime() >= currentTime)
         {
            QuadrupedStepTransition stepTransition = stepTransitionsToPack.add();

            stepTransition.setTransitionTime(step.getTimeInterval().getStartTime());
            stepTransition.addTransition(QuadrupedStepTransitionType.LIFT_OFF, step.getRobotQuadrant(), step.getGoalPosition());
         }

         if (step.getTimeInterval().getEndTime() >= currentTime)
         {
            QuadrupedStepTransition stepTransition = stepTransitionsToPack.add();

            stepTransition.setTransitionTime(step.getTimeInterval().getEndTime());
            stepTransition.addTransition(QuadrupedStepTransitionType.TOUCH_DOWN, step.getRobotQuadrant(), step.getGoalPosition());
         }
      }

      // sort step transitions in ascending order as a function of time
      stepTransitionsToPack.sort(Comparator.comparingDouble(QuadrupedStepTransition::getTransitionTime));

      // collapse the transitions that occur at the same time
      collapseTransitionEvents(stepTransitionsToPack);
   }

   public static void collapseTransitionEvents(List<QuadrupedStepTransition> stepTransitionsToPack)
   {
      // collapse the transitions that occur at the same time
      int transitionNumber = 0;
      while (transitionNumber < stepTransitionsToPack.size() - 1)
      {
         QuadrupedStepTransition currentTransition = stepTransitionsToPack.get(transitionNumber);
         QuadrupedStepTransition nextTransition = stepTransitionsToPack.get(transitionNumber + 1);
         if (MathTools.epsilonEquals(currentTransition.getTransitionTime(), nextTransition.getTransitionTime(), QuadrupedStepTransition.sameTimeEpsilon))
         {
            currentTransition.addTransition(nextTransition);
            stepTransitionsToPack.remove(transitionNumber + 1);
         }
         else
         {
            transitionNumber++;
         }
      }
   }


   public static void trimPastContactSequences(RecyclingArrayList<QuadrupedContactPhase> contactSequenceToPack, double currentTime,
                                               List<RobotQuadrant> currentFeetInContact,
                                               QuadrantDependentList<? extends FramePoint3DReadOnly> currentSolePositions, int contactPhasesToRetain)
   {
      // retain desired number of past contact phases
      TimeIntervalTools.removeStartTimesGreaterThanOrEqualTo(currentTime, contactSequenceToPack);
      while (contactSequenceToPack.size() > contactPhasesToRetain + 1)
      {
         contactSequenceToPack.remove(0);
      }

      QuadrupedContactPhase contactPhase;
      if (contactSequenceToPack.isEmpty())
      {
         contactPhase = contactSequenceToPack.add();
         contactPhase.getTimeInterval().setStartTime(currentTime);
         contactPhase.setFeetInContact(currentFeetInContact);
         contactPhase.setSolePositions(currentSolePositions);

         contactPhase.update();
      }
      else
      {
         QuadrupedContactPhase lastContactPhase = contactSequenceToPack.getLast();
         if (isEqualContactState(lastContactPhase.getFeetInContact(), currentFeetInContact))
         {
            // extend current contact phase
            contactPhase = lastContactPhase;
            contactPhase.setSolePositions(currentSolePositions);
         }
         else
         {
            // end previous contact phase
            lastContactPhase.getTimeInterval().setEndTime(currentTime);
            contactSequenceToPack.remove(0);

            contactPhase = contactSequenceToPack.add();
            contactPhase.getTimeInterval().setStartTime(currentTime);
            contactPhase.setFeetInContact(currentFeetInContact);
            contactPhase.setSolePositions(currentSolePositions);

            contactPhase.update();
         }
      }
   }

   public static boolean isEqualContactState(List<RobotQuadrant> contactStateA, List<RobotQuadrant> contactStateB)
   {
      if (contactStateA.size() != contactStateB.size())
         return false;

      for (int i = 0; i < contactStateA.size(); i++)
      {
         if (!contactStateB.contains(contactStateA.get(i)))
            return false;
      }
      return true;
   }
}
