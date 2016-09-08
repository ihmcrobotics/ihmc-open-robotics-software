package us.ihmc.quadrupedRobotics.planning;

import us.ihmc.quadrupedRobotics.util.ArraySorter;
import us.ihmc.quadrupedRobotics.util.PreallocatedList;
import us.ihmc.quadrupedRobotics.util.TimeIntervalTools;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.simulationconstructionset.Robot;

import javax.vecmath.Point3d;
import java.util.Comparator;
import java.util.List;

public class QuadrupedTimedContactSequence extends PreallocatedList<QuadrupedTimedContactPhase>
{
   private enum QuadrupedStepTransitionType
   {
      LIFT_OFF, TOUCH_DOWN
   }

   private class QuadrupedStepTransition
   {
      QuadrupedStepTransitionType type;
      RobotQuadrant robotQuadrant;
      Point3d solePosition;
      double time;

      public QuadrupedStepTransition()
      {
         time = 0.0;
         type = QuadrupedStepTransitionType.LIFT_OFF;
         solePosition = new Point3d();
         robotQuadrant = RobotQuadrant.FRONT_LEFT;
      }
   }

   private Comparator<QuadrupedStepTransition> compareByTime = new Comparator<QuadrupedStepTransition>()
   {
      @Override public int compare(QuadrupedStepTransition a, QuadrupedStepTransition b)
      {
         return Double.compare(a.time, b.time);
      }
   };

   // internal states
   private final int pastContactPhaseCapacity;
   private final QuadrupedStepTransition[] stepTransition;
   private final QuadrupedTimedStep[] transitionStep;
   private final QuadrantDependentList<ContactState> contactState;
   private final QuadrantDependentList<FramePoint> solePosition;

   public QuadrupedTimedContactSequence(int pastContactPhaseCapacity, int futureContactPhaseCapacity)
   {
      super(QuadrupedTimedContactPhase.class, pastContactPhaseCapacity + futureContactPhaseCapacity);
      this.pastContactPhaseCapacity = pastContactPhaseCapacity;

      stepTransition = new QuadrupedStepTransition[capacity()];
      transitionStep = new QuadrupedTimedStep[capacity()];
      for (int i = 0; i < capacity(); i++)
      {
         stepTransition[i] = new QuadrupedStepTransition();
      }
      contactState = new QuadrantDependentList<>();
      solePosition = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         contactState.set(robotQuadrant, ContactState.IN_CONTACT);
         solePosition.set(robotQuadrant, new FramePoint());
      }
   }

   public void initialize()
   {
      super.clear();
   }

   /**
    * compute piecewise center of pressure plan given an array of upcoming steps
    * @param stepSequence list of upcoming steps (input)
    * @param currentSolePosition current sole positions (input)
    * @param currentContactState current sole contact state (input)
    * @param currentTime current time (input)
    */
   public void update(List<QuadrupedTimedStep> stepSequence, QuadrantDependentList<FramePoint> currentSolePosition,
         QuadrantDependentList<ContactState> currentContactState, double currentTime)
   {
      // initialize contact state and sole positions
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePosition.get(robotQuadrant).setIncludingFrame(currentSolePosition.get(robotQuadrant));
         solePosition.get(robotQuadrant).changeFrame(ReferenceFrame.getWorldFrame());
         contactState.set(robotQuadrant, currentContactState.get(robotQuadrant));
      }

      // initialize step transitions
      for (int i = 0; i < stepTransition.length; i++)
      {
         stepTransition[i].time = Double.MAX_VALUE;
      }

      int numberOfStepTransitions = 0;
      for (int i = 0; i < stepSequence.size(); i++)
      {
         QuadrupedTimedStep step = stepSequence.get(i);

         if (step.getTimeInterval().getStartTime() >= currentTime)
         {
            stepTransition[numberOfStepTransitions].time = step.getTimeInterval().getStartTime();
            stepTransition[numberOfStepTransitions].type = QuadrupedStepTransitionType.LIFT_OFF;
            stepTransition[numberOfStepTransitions].robotQuadrant = step.getRobotQuadrant();
            stepTransition[numberOfStepTransitions].solePosition = step.getGoalPosition();
            transitionStep[numberOfStepTransitions] = step;
            numberOfStepTransitions++;
         }

         if (step.getTimeInterval().getEndTime() >= currentTime)
         {
            stepTransition[numberOfStepTransitions].time = step.getTimeInterval().getEndTime();
            stepTransition[numberOfStepTransitions].type = QuadrupedStepTransitionType.TOUCH_DOWN;
            stepTransition[numberOfStepTransitions].robotQuadrant = step.getRobotQuadrant();
            stepTransition[numberOfStepTransitions].solePosition = step.getGoalPosition();
            transitionStep[numberOfStepTransitions] = step;
            numberOfStepTransitions++;
         }
      }

      // sort step transitions in ascending order as a function of time
      ArraySorter.sort(stepTransition, compareByTime);

      // retain desired number of past contact phases
      TimeIntervalTools.removeStartTimesGreaterThanOrEqualTo(currentTime, this);
      while (super.size() > pastContactPhaseCapacity)
      {
         super.remove(0);
      }

      // initialize current contact phase
      QuadrupedTimedContactPhase contactPhase;
      if (super.size() > 0)
      {
         contactPhase = super.get(super.size() - 1);
         if (isEqualContactState(contactPhase.getContactState(), currentContactState))
         {
            contactPhase.setSolePosition(solePosition);
         }
         else
         {
            contactPhase.getTimeInterval().setEndTime(currentTime);
            super.add();
            contactPhase = super.get(super.size() - 1);
            contactPhase.getTimeInterval().setStartTime(currentTime);
            contactPhase.setContactState(contactState);
            contactPhase.setSolePosition(solePosition);
         }
      }
      else
      {
         super.add();
         contactPhase = super.get(super.size() - 1);
         contactPhase.getTimeInterval().setStartTime(currentTime);
         contactPhase.setContactState(contactState);
         contactPhase.setSolePosition(solePosition);
      }

      // compute transition time and center of pressure for each time interval
      for (int i = 0; i < numberOfStepTransitions; i++)
      {
         switch (stepTransition[i].type)
         {
         case LIFT_OFF:
            contactState.set(stepTransition[i].robotQuadrant, ContactState.NO_CONTACT);
            break;
         case TOUCH_DOWN:
            contactState.set(stepTransition[i].robotQuadrant, ContactState.IN_CONTACT);
            solePosition.get(stepTransition[i].robotQuadrant).changeFrame(ReferenceFrame.getWorldFrame());
            solePosition.get(stepTransition[i].robotQuadrant).setPoint(stepTransition[i].solePosition);
            break;
         }
         if ((i + 1 == numberOfStepTransitions) || (stepTransition[i].time != stepTransition[i + 1].time))
         {
            contactPhase.getTimeInterval().setEndTime(stepTransition[i].time);
            if (super.add())
            {
               contactPhase = super.get(super.size() - 1);
               contactPhase.setSolePosition(solePosition);
               contactPhase.setContactState(contactState);
               contactPhase.getTimeInterval().setStartTime(stepTransition[i].time);
            }
            else
            {
               return;
            }
         }
      }
      contactPhase.getTimeInterval().setEndTime(contactPhase.getTimeInterval().getStartTime() + 1.0);
   }

   private boolean isEqualContactState(QuadrantDependentList<ContactState> contactStateA, QuadrantDependentList<ContactState> contactStateB)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (contactStateA.get(robotQuadrant) != contactStateB.get(robotQuadrant))
            return false;
      }
      return true;
   }
}

