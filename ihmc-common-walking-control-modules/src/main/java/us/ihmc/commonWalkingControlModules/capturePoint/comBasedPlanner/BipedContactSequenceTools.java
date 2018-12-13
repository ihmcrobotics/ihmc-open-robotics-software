package us.ihmc.commonWalkingControlModules.capturePoint.comBasedPlanner;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.time.TimeIntervalTools;

import java.util.Comparator;
import java.util.List;

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

   // FIXME
   public static void trimPastContactSequences(RecyclingArrayList<SimpleBipedContactPhase> contactSequenceToPack, double currentTime,
                                               List<RobotSide> currentFeetInContact,
                                               SideDependentList<? extends FramePose3DReadOnly> currentSolePoses)
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
               for (int j = 0; j < currentFeetInContact.size(); j++)
                  contactPhase.addStartFoot(currentFeetInContact.get(j), currentSolePoses.get(currentFeetInContact.get(j)));
               contactPhase.update();
            }
            else
            { // end the previous contact phase and add a new one for the current state
               contactSequenceToPack.remove(i);
               addCurrentStateAsAContactPhase(contactSequenceToPack, currentFeetInContact, currentSolePoses, currentTime);
            }
         }
      }
   }

   // FIXME
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
}
