package us.ihmc.quadrupedRobotics.planning;

import us.ihmc.quadrupedRobotics.util.ArraySorter;
import us.ihmc.quadrupedRobotics.util.PreallocatedList;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

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
   private final QuadrupedStepTransition[] stepTransition;
   private final QuadrupedTimedStep[] transitionStep;
   private final QuadrantDependentList<ContactState> contactState;
   private final QuadrantDependentList<FramePoint> solePosition;

   public QuadrupedTimedContactSequence(int maximumNumberOfSteps)
   {
      super(QuadrupedTimedContactPhase.class, 2 * maximumNumberOfSteps + 2);

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
   }

   /**
    * compute piecewise center of pressure plan given an array of upcoming steps
    * @param stepSequence list of upcoming steps (input)
    * @param initialSolePosition initial sole positions (input)
    * @param initialContactState initial sole contact state (input)
    * @param initialTime initial time (input)
    */
   public void compute(List<QuadrupedTimedStep> stepSequence, QuadrantDependentList<FramePoint> initialSolePosition,
         QuadrantDependentList<ContactState> initialContactState, double initialTime)
   {
      QuadrupedTimedContactPhase contactPhase;
      super.clear();

      // initialize contact state and sole positions
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePosition.get(robotQuadrant).setIncludingFrame(initialSolePosition.get(robotQuadrant));
         solePosition.get(robotQuadrant).changeFrame(ReferenceFrame.getWorldFrame());
         contactState.set(robotQuadrant, initialContactState.get(robotQuadrant));
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

         if (step.getTimeInterval().getStartTime() >= initialTime)
         {
            stepTransition[numberOfStepTransitions].time = step.getTimeInterval().getStartTime();
            stepTransition[numberOfStepTransitions].type = QuadrupedStepTransitionType.LIFT_OFF;
            stepTransition[numberOfStepTransitions].robotQuadrant = step.getRobotQuadrant();
            stepTransition[numberOfStepTransitions].solePosition = step.getGoalPosition();
            transitionStep[numberOfStepTransitions] = step;
            numberOfStepTransitions++;
         }

         if (step.getTimeInterval().getEndTime() >= initialTime)
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

      // compute transition time and center of pressure for each time interval
      super.add();
      contactPhase = super.get(super.size() - 1);
      contactPhase.setContactState(contactState);
      contactPhase.setSolePosition(solePosition);
      contactPhase.getTimeInterval().setStartTime(initialTime);
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
            super.add();
            contactPhase = super.get(super.size() - 1);
            contactPhase.setSolePosition(solePosition);
            contactPhase.setContactState(contactState);
            contactPhase.getTimeInterval().setStartTime(stepTransition[i].time);
         }
      }
      contactPhase.getTimeInterval().setEndTime(contactPhase.getTimeInterval().getStartTime() + 1.0);
   }
}