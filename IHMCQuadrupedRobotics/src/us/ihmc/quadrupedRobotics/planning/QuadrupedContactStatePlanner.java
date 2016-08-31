package us.ihmc.quadrupedRobotics.planning;

import org.apache.commons.lang3.mutable.MutableDouble;
import us.ihmc.quadrupedRobotics.util.ArraySorter;
import us.ihmc.quadrupedRobotics.util.PreallocatedQueue;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import javax.vecmath.Point3d;
import java.util.Comparator;
import java.util.List;

public class QuadrupedContactStatePlanner
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

   // step plan
   private final QuadrupedTimedStep[] stepArray;
   private final QuadrupedStepTransition[] stepTransition;
   private final QuadrupedTimedStep[] transitionStep;

   // internal states
   private final QuadrantDependentList<FramePoint> solePosition;
   private final QuadrantDependentList<ContactState> contactState;
   private final QuadrantDependentList<MutableDouble> contactPressure;

   public QuadrupedContactStatePlanner(int maximumNumberOfSteps)
   {
      // step plan
      int maximumNumberOfIntervals = 2 * maximumNumberOfSteps + 2;
      stepArray = new QuadrupedTimedStep[maximumNumberOfIntervals];
      stepTransition = new QuadrupedStepTransition[maximumNumberOfIntervals];
      transitionStep = new QuadrupedTimedStep[maximumNumberOfIntervals];
      for (int i = 0; i < maximumNumberOfIntervals; i++)
      {
         stepTransition[i] = new QuadrupedStepTransition();
      }

      // internal states
      solePosition = new QuadrantDependentList<>();
      contactState = new QuadrantDependentList<>();
      contactPressure = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePosition.set(robotQuadrant, new FramePoint(ReferenceFrame.getWorldFrame()));
         contactState.set(robotQuadrant, ContactState.IN_CONTACT);
         contactPressure.set(robotQuadrant, new MutableDouble(0.0));
      }
   }

   /**
    * compute piecewise center of pressure plan given a preallocated queue of upcoming steps
    * @param contactStatePlan contact state plan (output)
    * @param numberOfSteps number of upcoming steps (input)
    * @param stepQueue queue of preview steps (input)
    * @param initialSolePosition initial sole positions (input)
    * @param initialContactState initial sole contact state (input)
    * @param initialTime initial time (input)
    */
   public void compute(QuadrupedContactStatePlan contactStatePlan, int numberOfSteps, PreallocatedQueue<QuadrupedTimedStep> stepQueue, QuadrantDependentList<FramePoint> initialSolePosition,
         QuadrantDependentList<ContactState> initialContactState, double initialTime)
   {
      for (int i = 0; i < numberOfSteps; i++)
      {
         stepArray[i] = stepQueue.get(i);
      }
      compute(contactStatePlan, numberOfSteps, stepArray, initialSolePosition, initialContactState, initialTime);
   }

   /**
    * compute piecewise center of pressure plan given a list of upcoming steps
    * @param contactStatePlan contact state plan (output)
    * @param numberOfSteps number of upcoming steps (input)
    * @param stepList list of upcoming steps (input)
    * @param initialSolePosition initial sole positions (input)
    * @param initialContactState initial sole contact state (input)
    * @param initialTime initial time (input)
    */
   public void compute(QuadrupedContactStatePlan contactStatePlan, int numberOfSteps, List<QuadrupedTimedStep> stepList, QuadrantDependentList<FramePoint> initialSolePosition,
         QuadrantDependentList<ContactState> initialContactState, double initialTime)
   {
      for (int i = 0; i < numberOfSteps; i++)
      {
         stepArray[i] = stepList.get(i);
      }
      compute(contactStatePlan, numberOfSteps, stepArray, initialSolePosition, initialContactState, initialTime);
   }

   /**
    * compute piecewise center of pressure plan given an array of upcoming steps
    * @param contactStatePlan contact state plan (output)
    * @param numberOfSteps number of upcoming steps (input)
    * @param stepArray array of upcoming steps (input)
    * @param initialSolePosition initial sole positions (input)
    * @param initialContactState initial sole contact state (input)
    * @param initialTime initial time (input)
    */
   public void compute(QuadrupedContactStatePlan contactStatePlan, int numberOfSteps, QuadrupedTimedStep[] stepArray, QuadrantDependentList<FramePoint> initialSolePosition,
         QuadrantDependentList<ContactState> initialContactState, double initialTime)
   {
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
      for (int i = 0; i < numberOfSteps; i++)
      {
         QuadrupedTimedStep step = stepArray[i];

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
      int numberOfIntervals = 1;
      updatePiecewisePressurePlan(contactStatePlan, 0, initialTime);
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
            updatePiecewisePressurePlan(contactStatePlan, numberOfIntervals, stepTransition[i].time);
            numberOfIntervals++;
         }
      }
      contactStatePlan.setNumberOfIntervals(numberOfIntervals);
   }

   private void updatePiecewisePressurePlan(QuadrupedContactStatePlan contactStatePlan, int interval, double intervalStartTime)
   {
      contactStatePlan.getTimeAtStartOfInterval().get(interval).setValue(intervalStartTime);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         contactStatePlan.getContactStateAtStartOfInterval().get(interval).set(robotQuadrant, contactState.get(robotQuadrant));
         contactStatePlan.getSolePositionAtStartOfInterval().get(interval).get(robotQuadrant).setIncludingFrame(solePosition.get(robotQuadrant));
      }
   }
}