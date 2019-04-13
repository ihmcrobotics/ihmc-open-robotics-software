package us.ihmc.commonWalkingControlModules.capturePoint.comBasedPlanner;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.time.TimeIntervalTools;

import java.util.Comparator;
import java.util.List;

/**
 * This is a tools class for computing a list of {@link ContactStateProvider} from a list of {@link BipedTimedStep}. It is used by
 * the {@link BipedContactSequenceUpdater} class.
 */
public class BipedContactSequenceTools
{
   public static void collapseTransitionEvents(List<BipedStepTransition> stepTransitionsToPack)
   {
      // collapse the transitions that occur at the same time
      int transitionNumber = 0;
      while (transitionNumber < stepTransitionsToPack.size() - 1)
      {
         BipedStepTransition currentTransition = stepTransitionsToPack.get(transitionNumber);
         BipedStepTransition nextTransition = stepTransitionsToPack.get(transitionNumber + 1);
         if (MathTools.epsilonEquals(currentTransition.getTransitionTime(), nextTransition.getTransitionTime(), BipedStepTransition.sameTimeEpsilon))
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

   /**
    * <p>
    * WARNING: This method generates garbage.
    * </p>
    */
   public static void computeStepTransitionsFromStepSequence(RecyclingArrayList<BipedStepTransition> stepTransitionsToPack, double currentTime,
                                                             List<? extends BipedTimedStep> stepSequence)
   {
      stepTransitionsToPack.clear();
      for (int i = 0; i < stepSequence.size(); i++)
      {
         BipedTimedStep step = stepSequence.get(i);

         if (step.getTimeInterval().getStartTime() >= currentTime)
         {
            BipedStepTransition stepTransition = stepTransitionsToPack.add();
            stepTransition.reset();

            stepTransition.setTransitionTime(step.getTimeInterval().getStartTime());

            stepTransition.addTransition(BipedStepTransitionType.LIFT_OFF, step.getRobotSide(), step.getGoalPose());
         }

         if (step.getTimeInterval().getEndTime() >= currentTime)
         {
            BipedStepTransition stepTransition = stepTransitionsToPack.add();
            stepTransition.reset();

            stepTransition.setTransitionTime(step.getTimeInterval().getEndTime());
            stepTransition.addTransition(BipedStepTransitionType.TOUCH_DOWN, step.getRobotSide(), step.getGoalPose());
         }
      }

      // sort step transitions in ascending order as a function of time
      stepTransitionsToPack.sort(Comparator.comparingDouble(BipedStepTransition::getTransitionTime));

      // collapse the transitions that occur at the same time
      BipedContactSequenceTools.collapseTransitionEvents(stepTransitionsToPack);

      // remove any transitions that already happened
      stepTransitionsToPack.removeIf(transition -> transition.getTransitionTime() < currentTime);
   }

   public static void trimPastContactSequences(RecyclingArrayList<SimpleBipedContactPhase> contactSequenceToPack, double currentTime,
                                               List<RobotSide> currentFeetInContact, SideDependentList<? extends FramePose3DReadOnly> currentSolePoses)
   {
      // remove all future steps, as they don't affect the past sequence and could have been changed.
      TimeIntervalTools.removeStartTimesGreaterThanOrEqualTo(currentTime, contactSequenceToPack);

      // remove all steps that are already complete, as they don't matter any more.
      TimeIntervalTools.removeEndTimesLessThan(currentTime, contactSequenceToPack);

      if (contactSequenceToPack.isEmpty())
      { // if there aren't any past sequences, add the current one in.
         addCurrentStateAsAContactPhase(contactSequenceToPack, currentFeetInContact, currentSolePoses, currentTime);
      }
      else
      { // there are some contact phases that are currently in progress
         for (int i = 0; i < contactSequenceToPack.size(); i++)
         {
            SimpleBipedContactPhase contactPhase = contactSequenceToPack.get(i);

            if (isEqualContactState(contactPhase.getFeetInContact(), currentFeetInContact))
            { // the last phase hasn't been completed, so update the sole positions
               contactPhase.resetEnd();
               contactPhase.setStartFootPoses(currentSolePoses);
            }
            else
            { // end the previous contact phase and add a new one for the current state
               contactSequenceToPack.remove(i);
               addCurrentStateAsAContactPhase(contactSequenceToPack, currentFeetInContact, currentSolePoses, currentTime);
            }
         }
      }
   }

   public static void addCurrentStateAsAContactPhase(RecyclingArrayList<SimpleBipedContactPhase> contactSequenceToPack, List<RobotSide> currentFeetInContact,
                                                     SideDependentList<? extends FramePose3DReadOnly> solePoses, double currentTime)
   {
      SimpleBipedContactPhase contactPhase = contactSequenceToPack.add();
      contactPhase.reset();
      contactPhase.setFeetInContact(currentFeetInContact);
      for (int i = 0; i < currentFeetInContact.size(); i++)
         contactPhase.addStartFoot(currentFeetInContact.get(i), solePoses.get(currentFeetInContact.get(i)));
      contactPhase.getTimeInterval().setStartTime(currentTime);
      contactPhase.update();
   }

   public static boolean isEqualContactState(List<RobotSide> contactStateA, List<RobotSide> contactStateB)
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

   public static void shiftContactSequencesToRelativeTime(List<SimpleBipedContactPhase> contactSequenceToPack, double currentAbsoluteTime)
   {
      double shiftTime = -currentAbsoluteTime;
      for (int sequence = 0; sequence < contactSequenceToPack.size(); sequence++)
         contactSequenceToPack.get(sequence).getTimeInterval().shiftInterval(shiftTime);
   }
}
