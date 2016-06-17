package us.ihmc.quadrupedRobotics.planning;

import org.apache.commons.lang3.mutable.MutableBoolean;
import org.apache.commons.lang3.mutable.MutableDouble;
import us.ihmc.quadrupedRobotics.util.ArraySorter;
import us.ihmc.quadrupedRobotics.util.PreallocatedQueue;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.Comparator;
import java.util.List;

import javax.vecmath.Point3d;

public class QuadrupedTimedStepPressurePlanner
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
   private final QuadrantDependentList<MutableBoolean> isInitialContactState;

   // pressure plan
   private final double[] timeAtStartOfInterval;
   private final FramePoint[] centerOfPressureAtStartOfInterval;
   private final double[] normalizedPressureContributedByInitialContacts;
   private final double[] normalizedPressureContributedByQueuedSteps;
   private final QuadrantDependentList<double[]> normalizedPressureContributedByQuadrant;

   public QuadrupedTimedStepPressurePlanner(int maxSteps)
   {
      // step plan
      int maxIntervals = 2 * maxSteps + 2;
      stepArray = new QuadrupedTimedStep[maxIntervals];
      stepTransition = new QuadrupedStepTransition[maxIntervals];
      transitionStep = new QuadrupedTimedStep[maxIntervals];
      for (int i = 0; i < maxIntervals; i++)
      {
         stepTransition[i] = new QuadrupedStepTransition();
      }

      // internal states
      solePosition = new QuadrantDependentList<>();
      contactState = new QuadrantDependentList<>();
      contactPressure = new QuadrantDependentList<>();
      isInitialContactState = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePosition.set(robotQuadrant, new FramePoint(ReferenceFrame.getWorldFrame()));
         contactState.set(robotQuadrant, ContactState.IN_CONTACT);
         contactPressure.set(robotQuadrant, new MutableDouble(0.0));
         isInitialContactState.set(robotQuadrant, new MutableBoolean(true));
      }

      // pressure plan
      timeAtStartOfInterval = new double[maxIntervals];
      centerOfPressureAtStartOfInterval = new FramePoint[maxIntervals];
      for (int i = 0; i < maxIntervals; i++)
      {
         centerOfPressureAtStartOfInterval[i] = new FramePoint(ReferenceFrame.getWorldFrame());
         timeAtStartOfInterval[i] = 0.0;
      }
      normalizedPressureContributedByInitialContacts = new double[maxIntervals];
      normalizedPressureContributedByQueuedSteps = new double[maxIntervals];
      normalizedPressureContributedByQuadrant = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         normalizedPressureContributedByQuadrant.set(robotQuadrant, new double[maxIntervals]);
      }
   }

   public double getTimeAtStartOfInterval(int interval)
   {
      return timeAtStartOfInterval[interval];
   }

   public double[] getTimeAtStartOfInterval()
   {
      return timeAtStartOfInterval;
   }

   public FramePoint getCenterOfPressureAtStartOfInterval(int interval)
   {
      return centerOfPressureAtStartOfInterval[interval];
   }

   public FramePoint[] getCenterOfPressureAtStartOfInterval()
   {
      return centerOfPressureAtStartOfInterval;
   }

   public double getNormalizedPressureContributedByQuadrant(RobotQuadrant robotQuadrant, int interval)
   {
      return normalizedPressureContributedByQuadrant.get(robotQuadrant)[interval];
   }

   public double[] getNormalizedPressureContributedByQuadrant(RobotQuadrant robotQuadrant)
   {
      return normalizedPressureContributedByQuadrant.get(robotQuadrant);
   }

   public double getNormalizedPressureContributedByInitialContacts(int interval)
   {
      return normalizedPressureContributedByInitialContacts[interval];
   }

   public double[] getNormalizedPressureContributedByInitialContacts()
   {
      return normalizedPressureContributedByInitialContacts;
   }

   public double getNormalizedPressureContributedByQueuedSteps(int interval)
   {
      return normalizedPressureContributedByQueuedSteps[interval];
   }

   public double[] getNormalizedPressureContributedByQueuedSteps()
   {
      return normalizedPressureContributedByQueuedSteps;
   }

   /**
    * compute piecewise center of pressure plan given a preallocated queue of upcoming steps
    * @param numberOfSteps number of upcoming steps
    * @param stepQueue queue of preview steps
    * @param initialSolePosition initial sole positions
    * @param initialContactState initial sole contact state
    * @param initialTime initial time
    * @return numberOfTransitions
    */
   public int compute(int numberOfSteps, PreallocatedQueue<QuadrupedTimedStep> stepQueue, QuadrantDependentList<FramePoint> initialSolePosition,
         QuadrantDependentList<ContactState> initialContactState, double initialTime)
   {
      for (int i = 0; i < numberOfSteps; i++)
      {
         stepArray[i] = stepQueue.get(i);
      }
      return compute(numberOfSteps, stepArray, initialSolePosition, initialContactState, initialTime);
   }

   /**
    * compute piecewise center of pressure plan given a list of upcoming steps
    * @param numberOfSteps number of upcoming steps
    * @param stepList list of upcoming steps
    * @param initialSolePosition initial sole positions
    * @param initialContactState initial sole contact state
    * @param initialTime initial time
    * @return numberOfTransitions
    */
   public int compute(int numberOfSteps, List<QuadrupedTimedStep> stepList, QuadrantDependentList<FramePoint> initialSolePosition,
         QuadrantDependentList<ContactState> initialContactState, double initialTime)
   {
      for (int i = 0; i < numberOfSteps; i++)
      {
         stepArray[i] = stepList.get(i);
      }
      return compute(numberOfSteps, stepArray, initialSolePosition, initialContactState, initialTime);
   }

   /**
    * compute piecewise center of pressure plan given an array of upcoming steps
    * @param numberOfSteps number of upcoming steps
    * @param stepArray array of upcoming steps
    * @param initialSolePosition initial sole positions
    * @param initialContactState initial sole contact state
    * @param initialTime initial time
    * @return numberOfIntervals
    */
   public int compute(int numberOfSteps, QuadrupedTimedStep[] stepArray, QuadrantDependentList<FramePoint> initialSolePosition,
         QuadrantDependentList<ContactState> initialContactState, double initialTime)
   {
      // initialize contact state and sole positions
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePosition.get(robotQuadrant).setIncludingFrame(initialSolePosition.get(robotQuadrant));
         solePosition.get(robotQuadrant).changeFrame(ReferenceFrame.getWorldFrame());
         contactState.set(robotQuadrant, initialContactState.get(robotQuadrant));
         isInitialContactState.get(robotQuadrant).setTrue();
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
      updatePiecewisePressurePlan(0, initialTime);
      for (int i = 0; i < numberOfStepTransitions; i++)
      {
         switch (stepTransition[i].type)
         {
         case LIFT_OFF:
            contactState.set(stepTransition[i].robotQuadrant, ContactState.NO_CONTACT);
            isInitialContactState.get(stepTransition[i].robotQuadrant).setFalse();
            break;
         case TOUCH_DOWN:
            contactState.set(stepTransition[i].robotQuadrant, ContactState.IN_CONTACT);
            solePosition.get(stepTransition[i].robotQuadrant).changeFrame(ReferenceFrame.getWorldFrame());
            solePosition.get(stepTransition[i].robotQuadrant).setPoint(stepTransition[i].solePosition);
            isInitialContactState.get(stepTransition[i].robotQuadrant).setFalse();
            break;
         }
         if ((i + 1 == numberOfStepTransitions) || (stepTransition[i].time != stepTransition[i + 1].time))
         {
            updatePiecewisePressurePlan(numberOfIntervals, stepTransition[i].time);
            numberOfIntervals++;
         }
      }

      return numberOfIntervals;
   }

   private void updatePiecewisePressurePlan(int interval, double intervalStartTime)
   {
      timeAtStartOfInterval[interval] = intervalStartTime;
      computeNormalizedContactPressure(contactState, contactPressure);
      computeCenterOfPressure(solePosition, contactPressure, centerOfPressureAtStartOfInterval[interval]);
      normalizedPressureContributedByInitialContacts[interval] = 0.0;
      normalizedPressureContributedByQueuedSteps[interval] = 0.0;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         normalizedPressureContributedByQuadrant.get(robotQuadrant)[interval] = contactPressure.get(robotQuadrant).getValue();
         if (isInitialContactState.get(robotQuadrant).booleanValue())
         {
            normalizedPressureContributedByInitialContacts[interval] += contactPressure.get(robotQuadrant).getValue();
         }
         else
         {
            normalizedPressureContributedByQueuedSteps[interval] += contactPressure.get(robotQuadrant).getValue();
         }
      }
   }

   private void computeNormalizedContactPressure(QuadrantDependentList<ContactState> contactState, QuadrantDependentList<MutableDouble> contactPressure)
   {
      // Compute vertical force distribution assuming equal loading of hind and front ends.
      int numberOfHindFeetInContact = 0;
      int numberOfFrontFeetInContact = 0;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (contactState.get(robotQuadrant) == ContactState.IN_CONTACT)
         {
            if (robotQuadrant.isQuadrantInFront())
            {
               numberOfFrontFeetInContact++;
            }
            else
            {
               numberOfHindFeetInContact++;
            }
            contactPressure.get(robotQuadrant).setValue(1.0);
         }
         else
         {
            contactPressure.get(robotQuadrant).setValue(0.0);
         }
      }

      double numberOfEndsInContact = 0.0;
      if ((numberOfHindFeetInContact > 0) ^ (numberOfFrontFeetInContact > 0))
      {
         numberOfEndsInContact = 1.0;
      }
      if ((numberOfHindFeetInContact > 0) && (numberOfFrontFeetInContact > 0))
      {
         numberOfEndsInContact = 2.0;
      }

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         double pressure = contactPressure.get(robotQuadrant).doubleValue();
         pressure /= Math.max(numberOfEndsInContact, 1.0);
         pressure /= Math.max((robotQuadrant.isQuadrantInFront() ? numberOfFrontFeetInContact : numberOfHindFeetInContact), 1.0);
         contactPressure.get(robotQuadrant).setValue(pressure);
      }
   }

   private void computeCenterOfPressure(QuadrantDependentList<FramePoint> solePosition, QuadrantDependentList<MutableDouble> contactPressure,
         FramePoint copPosition)
   {
      // Compute center of pressure given the vertical force at each contact.
      double pressure = 1e-6;
      copPosition.setToZero(ReferenceFrame.getWorldFrame());
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         pressure += contactPressure.get(robotQuadrant).doubleValue();
         solePosition.get(robotQuadrant).changeFrame(ReferenceFrame.getWorldFrame());
         addPointWithScaleFactor(copPosition, solePosition.get(robotQuadrant), contactPressure.get(robotQuadrant).doubleValue());
      }
      copPosition.scale(1.0 / pressure);
   }

   private void addPointWithScaleFactor(FramePoint point, FramePoint pointToAdd, double scaleFactor)
   {
      point.checkReferenceFrameMatch(pointToAdd);
      point.add(scaleFactor * pointToAdd.getX(), scaleFactor * pointToAdd.getY(), scaleFactor * pointToAdd.getZ());
   }
}
