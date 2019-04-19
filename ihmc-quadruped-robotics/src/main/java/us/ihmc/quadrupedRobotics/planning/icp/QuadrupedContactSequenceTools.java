package us.ihmc.quadrupedRobotics.planning.icp;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.time.TimeIntervalTools;

import java.util.Comparator;
import java.util.List;

/**
 * This is a tools class for computing a list of {@link ContactStateProvider} from a list of {@link QuadrupedTimedStep}. It is used by
 * the {@link QuadrupedContactSequenceUpdater} class.
 */
public class QuadrupedContactSequenceTools
{
   private static boolean debug = false;

   public static void setDebug(boolean debug)
   {
      QuadrupedContactSequenceTools.debug = debug;
   }

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
            stepTransition.reset();

            stepTransition.setTransitionTime(step.getTimeInterval().getStartTime());
            stepTransition.addTransition(QuadrupedStepTransitionType.LIFT_OFF, step.getRobotQuadrant(), step.getGoalPosition());
         }

         if (step.getTimeInterval().getEndTime() >= currentTime)
         {
            QuadrupedStepTransition stepTransition = stepTransitionsToPack.add();
            stepTransition.reset();

            stepTransition.setTransitionTime(step.getTimeInterval().getEndTime());
            stepTransition.addTransition(QuadrupedStepTransitionType.TOUCH_DOWN, step.getRobotQuadrant(), step.getGoalPosition());
         }
      }

      // sort step transitions in ascending order as a function of time
      stepTransitionsToPack.sort(Comparator.comparingDouble(QuadrupedStepTransition::getTransitionTime));

      // collapse the transitions that occur at the same time
      collapseTransitionEvents(stepTransitionsToPack);

      int currentNumberOfTransitions = stepTransitionsToPack.size();
      // remove any transitions that already happened
      stepTransitionsToPack.removeIf(transition -> transition.getTransitionTime() < currentTime);

      if (debug && stepTransitionsToPack.size() < currentNumberOfTransitions)
         throw new RuntimeException("Somehow a past transition was generated from what is supposed to be a pruned step sequence.");
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
                                               QuadrantDependentList<? extends FramePoint3DReadOnly> currentSolePositions)
   {
      // remove all future steps, as they don't affect the past sequence and could have been changed.
      TimeIntervalTools.removeStartTimesGreaterThanOrEqualTo(currentTime, contactSequenceToPack);

      // remove all steps that are already complete, as they don't matter any more.
      TimeIntervalTools.removeEndTimesLessThan(currentTime, contactSequenceToPack);

      if (contactSequenceToPack.isEmpty())
      { // if there aren't any past sequences, add the current one in.
         addCurrentStateAsAContactPhase(contactSequenceToPack, currentFeetInContact, currentSolePositions, currentTime, currentTime);
      }
      else
      { // there are some contact phases that are currently in progress
         if (debug && contactSequenceToPack.size() > 1)
            throw new RuntimeException("You shouldn't have more than one sequence remaining in the plan after trimming.");

         for (int i = 0; i < contactSequenceToPack.size(); i++)
         {
            QuadrupedContactPhase contactPhase = contactSequenceToPack.get(i);

            if (isEqualContactState(contactPhase.getFeetInContact(), currentFeetInContact))
            { // the last phase hasn't been completed, so update the sole positions
               contactPhase.setSolePositions(currentSolePositions);
               contactPhase.update();
            }
            else
            { // end the previous contact phase and add a new one for the current state
               contactSequenceToPack.remove(i);
               addCurrentStateAsAContactPhase(contactSequenceToPack, currentFeetInContact, currentSolePositions, currentTime, currentTime);
            }
         }
      }
   }

   public static void addCurrentStateAsAContactPhase(RecyclingArrayList<QuadrupedContactPhase> contactSequenceToPack, List<RobotQuadrant> currentFeetInContact,
                                                     QuadrantDependentList<? extends FramePoint3DReadOnly> solePositions, double currentTime, double endTime)
   {
      QuadrupedContactPhase contactPhase = contactSequenceToPack.add();
      contactPhase.reset();
      contactPhase.setFeetInContact(currentFeetInContact);
      contactPhase.setSolePositions(solePositions);
      contactPhase.getTimeInterval().setInterval(currentTime, endTime);
      contactPhase.update();
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

   public static void shiftContactSequencesToRelativeTime(List<QuadrupedContactPhase> contactSequenceToPack, double currentAbsoluteTime)
   {
      double shiftTime = -currentAbsoluteTime;
      for (int sequence = 0; sequence < contactSequenceToPack.size(); sequence++)
         contactSequenceToPack.get(sequence).getTimeInterval().shiftInterval(shiftTime);
   }
}
