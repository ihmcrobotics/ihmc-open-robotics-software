package us.ihmc.quadrupedRobotics.planning;

import org.apache.commons.lang3.mutable.MutableDouble;
import us.ihmc.quadrupedRobotics.util.ArraySorter;
import us.ihmc.quadrupedRobotics.util.PreallocatedDeque;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import javax.vecmath.Point3d;
import java.util.ArrayList;
import java.util.Comparator;
import java.util.List;

public class QuadrupedContactStateSequence
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
   private final QuadrupedTimedStep[] stepArray;
   private final QuadrupedStepTransition[] stepTransition;
   private final QuadrupedTimedStep[] transitionStep;
   private final QuadrantDependentList<FramePoint> solePosition;
   private final QuadrantDependentList<ContactState> contactState;
   private final QuadrantDependentList<MutableDouble> contactPressure;

   // contact state plan
   private int numberOfIntervals;
   private final ArrayList<MutableDouble> timeAtStartOfInterval;
   private final ArrayList<QuadrantDependentList<ContactState>> contactStateAtStartOfInterval;
   private final ArrayList<QuadrantDependentList<FramePoint>> solePositionAtStartOfInterval;

   public QuadrupedContactStateSequence(int maximumNumberOfSteps)
   {
      int maximumNumberOfIntervals = 2 * maximumNumberOfSteps + 2;

      // internal
      stepArray = new QuadrupedTimedStep[maximumNumberOfIntervals];
      stepTransition = new QuadrupedStepTransition[maximumNumberOfIntervals];
      transitionStep = new QuadrupedTimedStep[maximumNumberOfIntervals];
      for (int i = 0; i < maximumNumberOfIntervals; i++)
      {
         stepTransition[i] = new QuadrupedStepTransition();
      }
      solePosition = new QuadrantDependentList<>();
      contactState = new QuadrantDependentList<>();
      contactPressure = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePosition.set(robotQuadrant, new FramePoint(ReferenceFrame.getWorldFrame()));
         contactState.set(robotQuadrant, ContactState.IN_CONTACT);
         contactPressure.set(robotQuadrant, new MutableDouble(0.0));
      }

      // external
      numberOfIntervals = 0;
      timeAtStartOfInterval = new ArrayList<>(maximumNumberOfIntervals);
      solePositionAtStartOfInterval = new ArrayList<>(maximumNumberOfIntervals);
      contactStateAtStartOfInterval = new ArrayList<>(maximumNumberOfIntervals);
      for (int i = 0; i < maximumNumberOfIntervals; i++)
      {
         timeAtStartOfInterval.add(i, new MutableDouble(0.0));
         contactStateAtStartOfInterval.add(i, new QuadrantDependentList<ContactState>());
         solePositionAtStartOfInterval.add(i, new QuadrantDependentList<>(new FramePoint(), new FramePoint(), new FramePoint(), new FramePoint()));
      }
   }

   public int getNumberOfIntervals()
   {
      return numberOfIntervals;
   }

   public double getTimeAtStartOfInterval(int interval)
   {
      return timeAtStartOfInterval.get(interval).getValue();
   }

   public QuadrantDependentList<ContactState> getContactStateAtStartOfInterval(int interval)
   {
      return contactStateAtStartOfInterval.get(interval);
   }

   public QuadrantDependentList<FramePoint> getSolePositionAtStartOfInterval(int interval)
   {
      return solePositionAtStartOfInterval.get(interval);
   }

   public ArrayList<MutableDouble> getTimeAtStartOfInterval()
   {
      return timeAtStartOfInterval;
   }

   public ArrayList<QuadrantDependentList<ContactState>> getContactStateAtStartOfInterval()
   {
      return contactStateAtStartOfInterval;
   }

   public ArrayList<QuadrantDependentList<FramePoint>> getSolePositionAtStartOfInterval()
   {
      return solePositionAtStartOfInterval;
   }

   /**
    * compute piecewise center of pressure plan given an array of upcoming steps
    * @param steps list of upcoming steps (input)
    * @param initialSolePosition initial sole positions (input)
    * @param initialContactState initial sole contact state (input)
    * @param initialTime initial time (input)
    */
   public void compute(List<QuadrupedTimedStep> steps, QuadrantDependentList<FramePoint> initialSolePosition,
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
      for (int i = 0; i < steps.size(); i++)
      {
         QuadrupedTimedStep step = steps.get(i);

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
      numberOfIntervals = 1;
      timeAtStartOfInterval.get(0).setValue(initialTime);
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         contactStateAtStartOfInterval.get(0).set(robotQuadrant, contactState.get(robotQuadrant));
         solePositionAtStartOfInterval.get(0).get(robotQuadrant).setIncludingFrame(solePosition.get(robotQuadrant));
      }
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
            timeAtStartOfInterval.get(numberOfIntervals).setValue(stepTransition[i].time);
            for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
            {
               contactStateAtStartOfInterval.get(numberOfIntervals).set(robotQuadrant, contactState.get(robotQuadrant));
               solePositionAtStartOfInterval.get(numberOfIntervals).get(robotQuadrant).setIncludingFrame(solePosition.get(robotQuadrant));
            }
            numberOfIntervals++;
         }
      }
   }
}