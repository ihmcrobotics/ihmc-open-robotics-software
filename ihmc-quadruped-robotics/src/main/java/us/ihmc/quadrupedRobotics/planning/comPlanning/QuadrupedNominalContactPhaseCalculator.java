package us.ihmc.quadrupedRobotics.planning.comPlanning;

import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.Comparator;
import java.util.List;

import static us.ihmc.quadrupedRobotics.planning.comPlanning.QuadrupedStepTransitionType.LIFT_OFF;
import static us.ihmc.quadrupedRobotics.planning.comPlanning.QuadrupedStepTransitionType.TOUCH_DOWN;

public class QuadrupedNominalContactPhaseCalculator
{
   private static final double minimumTimeInFutureForTouchdown = 1.0e-3;

   private boolean resetStartingFootPositions = true;

   private final RecyclingArrayList<QuadrupedStepTransition> stepTransitions = new RecyclingArrayList<>(QuadrupedStepTransition::new);
   private final RecyclingArrayList<QuadrupedTimedContactInterval> contactPhases = new RecyclingArrayList<>(QuadrupedTimedContactInterval::new);

   private final QuadrantDependentList<ContactState> contactState;
   private final QuadrantDependentList<FramePoint3D> solePosition;
   private final QuadrantDependentList<FramePoint3D> startingSolePosition;

   private final int maxCapacity;

   public QuadrupedNominalContactPhaseCalculator(int maxCapacity)
   {
      this.maxCapacity = maxCapacity;

      contactState = new QuadrantDependentList<>();
      solePosition = new QuadrantDependentList<>();
      startingSolePosition = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         contactState.set(robotQuadrant, ContactState.IN_CONTACT);
         solePosition.set(robotQuadrant, new FramePoint3D());
         startingSolePosition.set(robotQuadrant, new FramePoint3D());
      }
   }

   public void initialize()
   {
      contactPhases.clear();
      resetStartingFootPositions = true;
   }

   public List<QuadrupedTimedContactInterval> getContactPhases()
   {
      return contactPhases;
   }

   public List<QuadrupedTimedContactInterval> computeFromSteps(List<? extends QuadrupedTimedStep> stepSequence,
                                                               QuadrantDependentList<? extends ReferenceFrame> soleFrames,
                                                               List<RobotQuadrant> currentFeetInContact, double currentTime, double timeAtStartOfState)
   {
      stepTransitions.clear();
      contactPhases.clear();

      // initialize contact state and sole positions
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         boolean inContact = currentFeetInContact.contains(robotQuadrant);
         if (resetStartingFootPositions || inContact)
         {
            startingSolePosition.get(robotQuadrant).setToZero(soleFrames.get(robotQuadrant));
            startingSolePosition.get(robotQuadrant).changeFrame(ReferenceFrame.getWorldFrame());
         }
         contactState.set(robotQuadrant, inContact ? ContactState.IN_CONTACT : ContactState.NO_CONTACT);
      }

      resetStartingFootPositions = false;

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         solePosition.get(robotQuadrant).set(startingSolePosition.get(robotQuadrant));

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (!currentFeetInContact.contains(robotQuadrant))
         {
            QuadrupedTimedStep firstStepForQuadrant = getFirstStepForQuadrant(robotQuadrant, stepSequence);

            // end the current step if there are no active steps for it.
            if (firstStepForQuadrant == null || firstStepForQuadrant.getTimeInterval().getStartTime() > timeAtStartOfState)
            {
               double timeOfTransition = Math.max(currentTime, timeAtStartOfState + minimumTimeInFutureForTouchdown);
               QuadrupedStepTransition stepTransition = stepTransitions.add();
               stepTransition.reset();
               stepTransition.setTransitionTime(timeOfTransition);
               stepTransition.addTransition(TOUCH_DOWN, robotQuadrant, solePosition.get(robotQuadrant));
            }
         }
      }

      for (int i = 0; i < stepSequence.size(); i++)
      {
         QuadrupedTimedStep step = stepSequence.get(i);

         if (step.getTimeInterval().getStartTime() >= timeAtStartOfState)
         {
            QuadrupedStepTransition stepTransition = stepTransitions.add();
            stepTransition.reset();
            stepTransition.setTransitionTime(step.getTimeInterval().getStartTime());
            stepTransition.addTransition(LIFT_OFF, step.getRobotQuadrant(), step.getGoalPosition());
         }

         if (step.getTimeInterval().getEndTime() >= timeAtStartOfState)
         {
            QuadrupedStepTransition stepTransition = stepTransitions.add();
            stepTransition.reset();
            stepTransition.setTransitionTime(step.getTimeInterval().getEndTime());
            stepTransition.addTransition(TOUCH_DOWN, step.getRobotQuadrant(), step.getGoalPosition());
         }
      }

      // sort step transitions in ascending order as a function of time
      stepTransitions.sort(Comparator.comparingDouble(QuadrupedStepTransition::getTransitionTime));

      // collapse the transitions that occur at the same time
      QuadrupedContactSequenceTools.collapseTransitionEvents(stepTransitions);

      // remove any transitions that already happened
      stepTransitions.removeIf(transition -> transition.getTransitionTime() < timeAtStartOfState);

      QuadrupedTimedContactInterval contactPhase = createNewContactPhase(timeAtStartOfState, contactState, solePosition);

      // compute transition time and center of pressure for each time interval
      for (int i = 0; i < stepTransitions.size(); i++)
      {
         QuadrupedStepTransition stepTransition = stepTransitions.get(i);

         for (int transitionNumber = 0; transitionNumber < stepTransition.getNumberOfFeetInTransition(); transitionNumber++)
         {
            RobotQuadrant transitionQuadrant = stepTransition.getTransitionQuadrant(transitionNumber);

            switch (stepTransition.getTransitionType(transitionNumber))
            {
            case LIFT_OFF:
               contactState.set(transitionQuadrant, ContactState.NO_CONTACT);
               break;
            case TOUCH_DOWN:
               contactState.set(transitionQuadrant, ContactState.IN_CONTACT);
               solePosition.get(transitionQuadrant).changeFrame(ReferenceFrame.getWorldFrame());
               solePosition.get(transitionQuadrant).set(stepTransition.getTransitionPosition(transitionQuadrant));
               break;
            }
         }

         if ((i + 1 == stepTransitions.size()) || (!MathTools
               .epsilonEquals(stepTransition.getTransitionTime(), stepTransitions.get(i + 1).getTransitionTime(), 1e-2)))
         {
            contactPhase.getTimeInterval().setEndTime(stepTransition.getTransitionTime());
            contactPhase.getTimeInterval().checkInterval();
            contactPhase = createNewContactPhase(stepTransition.getTransitionTime(), contactState, solePosition);
            if (contactPhase == null)
            {
               return contactPhases;
            }
         }
      }
//      contactPhase.getTimeInterval().setEndTime(contactPhase.getTimeInterval().getStartTime() + 1.0);
      contactPhase.getTimeInterval().setEndTime(Double.POSITIVE_INFINITY);
      contactPhase.getTimeInterval().checkInterval();

      return contactPhases;
   }

   private final ConvexPolygon2D tempPolygon = new ConvexPolygon2D();

   private QuadrupedTimedContactInterval createNewContactPhase(double startTime, QuadrantDependentList<ContactState> contactState,
                                                               QuadrantDependentList<FramePoint3D> solePositions)
   {
      if (contactPhases.size() < maxCapacity)
      {
         QuadrupedTimedContactInterval contactPhase = contactPhases.add();
         contactPhase.getTimeInterval().setStartTime(startTime);
         contactPhase.setContactState(contactState);
         contactPhase.setSolePosition(solePositions);
         tempPolygon.clear();
         for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         {
            if (contactState.get(robotQuadrant).isLoadBearing())
               tempPolygon.addVertex(solePositions.get(robotQuadrant));
         }
         tempPolygon.update();
         contactPhase.setSupportPolygon(tempPolygon);
         return contactPhase;
      }
      else
      {
         return null;
      }
   }

   private QuadrupedTimedStep getFirstStepForQuadrant(RobotQuadrant robotQuadrant, List<? extends QuadrupedTimedStep> stepSequence)
   {
      for (int i = 0; i < stepSequence.size(); i++)
      {
         if (stepSequence.get(i).getRobotQuadrant() == robotQuadrant)
            return stepSequence.get(i);
      }

      return null;
   }
}
