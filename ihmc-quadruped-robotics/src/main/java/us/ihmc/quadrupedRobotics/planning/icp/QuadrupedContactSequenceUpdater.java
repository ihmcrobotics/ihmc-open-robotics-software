package us.ihmc.quadrupedRobotics.planning.icp;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedBasics.utils.TimeIntervalTools;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.tools.lists.ListSorter;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class QuadrupedContactSequenceUpdater
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public static final double finalTransferDuration = 1.0;

   private Comparator<NewQuadrupedStepTransition> compareByTime = Comparator.comparingDouble(NewQuadrupedStepTransition::getTransitionTime);

   // internal states
   private final int pastContactPhaseCapacity;
   private final RecyclingArrayList<NewQuadrupedStepTransition> stepTransitions = new RecyclingArrayList<>(NewQuadrupedStepTransition::new);
   private final List<RobotQuadrant> feetInContact = new ArrayList<>();
   private final QuadrantDependentList<FramePoint3D> solePositions = new QuadrantDependentList<>();
   private final QuadrantDependentList<ReferenceFrame> soleFrames;

   private final int maxCapacity;

   private final RecyclingArrayList<NewQuadrupedContactPhase> contactSequence;

   public QuadrupedContactSequenceUpdater(QuadrantDependentList<ReferenceFrame> soleFrames, int pastContactPhaseCapacity, int futureContactPhaseCapacity)
   {
      contactSequence = new RecyclingArrayList<>(pastContactPhaseCapacity + futureContactPhaseCapacity + 1, NewQuadrupedContactPhase::new);
      contactSequence.clear();

      this.soleFrames = soleFrames;

      maxCapacity = pastContactPhaseCapacity + futureContactPhaseCapacity + 1;
      this.pastContactPhaseCapacity = pastContactPhaseCapacity;

      if (maxCapacity < 1)
      {
         throw new RuntimeException("Sequence capacity must be greater than zero.");
      }

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         feetInContact.add(robotQuadrant);
         solePositions.set(robotQuadrant, new FramePoint3D());
      }
   }

   public void initialize()
   {
      contactSequence.clear();
   }

   public RecyclingArrayList<NewQuadrupedContactPhase> getContactSequence()
   {
      return contactSequence;
   }

   public void update(List<? extends QuadrupedTimedStep> stepSequence, List<RobotQuadrant> currentFeetInContact,
                                                              double currentTime)
   {
      initializeCalculationConditions(currentFeetInContact);
      computeStepTransitionsFromStepSequence(currentTime, stepSequence);

      trimPastContactSequences(currentTime, currentFeetInContact);

      computeContactPhasesFromStepTransitions();
   }

   private void initializeCalculationConditions(List<RobotQuadrant> currentFeetInContact)
   {
      // initialize contact state and sole positions
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePositions.get(robotQuadrant).setToZero(soleFrames.get(robotQuadrant));
         solePositions.get(robotQuadrant).changeFrame(worldFrame);
      }
      feetInContact.clear();
      for (int footIndex = 0; footIndex < currentFeetInContact.size(); footIndex++)
         feetInContact.add(currentFeetInContact.get(footIndex));
   }

   private void computeStepTransitionsFromStepSequence(double currentTime, List<? extends QuadrupedTimedStep> stepSequence)
   {
      stepTransitions.clear();
      for (int i = 0; i < stepSequence.size(); i++)
      {
         QuadrupedTimedStep step = stepSequence.get(i);

         if (step.getTimeInterval().getStartTime() >= currentTime)
         {
            NewQuadrupedStepTransition stepTransition = stepTransitions.add();

            stepTransition.setTransitionTime(step.getTimeInterval().getStartTime());
            stepTransition.addTransition(QuadrupedStepTransitionType.LIFT_OFF, step.getRobotQuadrant(), step.getGoalPosition());
         }

         if (step.getTimeInterval().getEndTime() >= currentTime)
         {
            NewQuadrupedStepTransition stepTransition = stepTransitions.add();

            stepTransition.setTransitionTime(step.getTimeInterval().getEndTime());
            stepTransition.addTransition(QuadrupedStepTransitionType.TOUCH_DOWN, step.getRobotQuadrant(), step.getGoalPosition());
         }
      }

      // sort step transitions in ascending order as a function of time
      ListSorter.sort(stepTransitions, compareByTime);

      // collapse the transitions that occur at the same time
      int transitionNumber = 0;
      while (transitionNumber < stepTransitions.size() - 1)
      {
         NewQuadrupedStepTransition currentTransition = stepTransitions.get(transitionNumber);
         NewQuadrupedStepTransition nextTransition = stepTransitions.get(transitionNumber + 1);
         if (MathTools.epsilonEquals(currentTransition.getTransitionTime(), nextTransition.getTransitionTime(), NewQuadrupedStepTransition.sameTimeEpsilon))
         {
            currentTransition.addTransition(nextTransition);
            stepTransitions.remove(transitionNumber + 1);
         }
         else
         {
            transitionNumber++;
         }
      }
   }

   private void trimPastContactSequences(double currentTime, List<RobotQuadrant> currentFeetInContact)
   {
      // retain desired number of past contact phases
      TimeIntervalTools.removeStartTimesGreaterThanOrEqualTo(currentTime, contactSequence);
      while (contactSequence.size() > pastContactPhaseCapacity + 1)
      {
         contactSequence.remove(0);
      }

      NewQuadrupedContactPhase contactPhase;
      if (contactSequence.isEmpty())
      {
         contactPhase = contactSequence.add();
         contactPhase.getTimeInterval().setStartTime(currentTime);
         contactPhase.setFeetInContact(feetInContact);
         contactPhase.setSolePosition(solePositions);

         contactPhase.update();
      }
      else
      {
         NewQuadrupedContactPhase lastContactPhase = contactSequence.getLast();
         if (isEqualContactState(lastContactPhase.getFeetInContact(), currentFeetInContact))
         {
            // extend current contact phase
            contactPhase = lastContactPhase;
            contactPhase.setSolePosition(solePositions);
         }
         else
         {
            // end previous contact phase
            lastContactPhase.getTimeInterval().setEndTime(currentTime);
            contactSequence.remove(0);

            contactPhase = contactSequence.add();
            contactPhase.getTimeInterval().setStartTime(currentTime);
            contactPhase.setFeetInContact(feetInContact);
            contactPhase.setSolePosition(solePositions);

            contactPhase.update();
         }
      }
   }

   private void computeContactPhasesFromStepTransitions()
   {
      int numberOfTransitions = stepTransitions.size();
      // compute transition time and center of pressure for each time interval
      for (int transitionNumber = 0; transitionNumber < numberOfTransitions; transitionNumber++)
      {
         NewQuadrupedStepTransition stepTransition = stepTransitions.get(transitionNumber);

         for (int transitioningFootNumber = 0; transitioningFootNumber < stepTransition.getNumberOfFeetInTransition(); transitioningFootNumber++)
         {
            RobotQuadrant transitionQuadrant = stepTransition.getTransitionQuadrant(transitioningFootNumber);
            switch (stepTransition.getTransitionType(transitioningFootNumber))
            {
            case LIFT_OFF:
               feetInContact.remove(transitionQuadrant);
               break;
            case TOUCH_DOWN:
               feetInContact.add(transitionQuadrant);
               solePositions.get(transitionQuadrant).setIncludingFrame(worldFrame, stepTransition.getTransitionPosition(transitionQuadrant));
               break;
            }
         }

         NewQuadrupedContactPhase contactPhase = contactSequence.add();

         boolean isLastContact = (transitionNumber == numberOfTransitions - 1) || (contactSequence.size() == maxCapacity);

         contactPhase.setFeetInContact(feetInContact);
         contactPhase.setSolePosition(solePositions);
         contactPhase.update();

         if (isLastContact)
         {
            contactPhase.getTimeInterval().setInterval(stepTransition.getTransitionTime(), stepTransition.getTransitionTime() + finalTransferDuration);
            return;
         }
         else
         {
            contactPhase.getTimeInterval().setInterval(stepTransition.getTransitionTime(), stepTransitions.get(transitionNumber + 1).getTransitionTime());
         }
      }

      return;
   }


   private boolean isEqualContactState(List<RobotQuadrant> contactStateA, List<RobotQuadrant> contactStateB)
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
