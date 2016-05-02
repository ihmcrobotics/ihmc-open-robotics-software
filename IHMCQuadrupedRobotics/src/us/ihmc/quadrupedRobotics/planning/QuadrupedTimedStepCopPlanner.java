package us.ihmc.quadrupedRobotics.planning;

import us.ihmc.quadrupedRobotics.util.ArraySorter;
import us.ihmc.quadrupedRobotics.util.PreallocatedQueue;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.Comparator;
import java.util.List;

public class QuadrupedTimedStepCopPlanner
{
   private enum QuadrupedStepTransitionType
   {
      LIFT_OFF, TOUCH_DOWN
   }

   private class QuadrupedStepTransition
   {
      QuadrupedStepTransitionType type;
      RobotQuadrant robotQuadrant;
      FramePoint solePosition;
      double time;

      public QuadrupedStepTransition()
      {
         time = 0.0;
         type = QuadrupedStepTransitionType.LIFT_OFF;
         solePosition = new FramePoint(ReferenceFrame.getWorldFrame());
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

   private final double[] timeAtTransition;
   private final FramePoint[] copAtTransition;
   private final QuadrupedStepTransition[] stepTransition;
   private final QuadrupedTimedStep[] stepArray;
   private final QuadrantDependentList<FramePoint> solePosition;
   private final QuadrantDependentList<ContactState> contactState;

   public QuadrupedTimedStepCopPlanner(int maxIntervals)
   {
      timeAtTransition = new double[maxIntervals];
      copAtTransition = new FramePoint[maxIntervals];
      stepTransition = new QuadrupedStepTransition[maxIntervals];
      stepArray = new QuadrupedTimedStep[maxIntervals];
      for (int i = 0; i < maxIntervals; i++)
      {
         timeAtTransition[i] = 0.0;
         copAtTransition[i] = new FramePoint(ReferenceFrame.getWorldFrame());
         stepTransition[i] = new QuadrupedStepTransition();
         stepArray[i] = new QuadrupedTimedStep();
      }
      solePosition = new QuadrantDependentList<>();
      contactState = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePosition.set(robotQuadrant, new FramePoint(ReferenceFrame.getWorldFrame()));
         contactState.set(robotQuadrant, ContactState.IN_CONTACT);
      }
   }

   public double getTimeAtTransition(int transition)
   {
      return timeAtTransition[transition];
   }

   public double[] getTimeAtTransitions()
   {
      return timeAtTransition;
   }

   public FramePoint getCopAtTransition(int transition)
   {
      return copAtTransition[transition];
   }

   public FramePoint[] getCopAtTransitions()
   {
      return copAtTransition;
   }

   /**
    * compute piecewise center of pressure plan given a preallocated queue of upcoming steps
    * @param stepQueue queue of upcoming steps
    * @param initialSolePosition initial sole positions
    * @param initialContactState initial sole contact state
    * @return numberOfTransitions
    */
   public int compute(PreallocatedQueue<QuadrupedTimedStep> stepQueue, QuadrantDependentList<FramePoint> initialSolePosition, QuadrantDependentList<ContactState> initialContactState)
   {
      int numberOfSteps = stepQueue.size();
      for (int i = 0; i < numberOfSteps; i++)
      {
         stepArray[i].set(stepQueue.get(i));
      }
      return compute(numberOfSteps, stepArray, initialSolePosition, initialContactState);
   }

   /**
    * compute piecewise center of pressure plan given a list of upcoming steps
    * @param stepList list of upcoming steps
    * @param initialSolePosition initial sole positions
    * @param initialContactState initial sole contact state
    * @return numberOfTransitions
    */
   public int compute(List<QuadrupedTimedStep> stepList, QuadrantDependentList<FramePoint> initialSolePosition, QuadrantDependentList<ContactState> initialContactState)
   {
      int numberOfSteps = stepList.size();
      for (int i = 0; i < numberOfSteps; i++)
      {
         stepArray[i].set(stepList.get(i));
      }
      return compute(numberOfSteps, stepArray, initialSolePosition, initialContactState);
   }

   /**
    * compute piecewise center of pressure plan given an array of upcoming steps
    * @param numberOfSteps number of upcoming steps
    * @param stepArray array of upcoming steps
    * @param initialSolePosition initial sole positions
    * @param initialContactState initial sole contact state
    * @return numberOfTransitions
    */
   public int compute(int numberOfSteps, QuadrupedTimedStep[] stepArray, QuadrantDependentList<FramePoint> initialSolePosition, QuadrantDependentList<ContactState> initialContactState)
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

      for (int i = 0; i < numberOfSteps; i++)
      {
         QuadrupedTimedStep step = stepArray[i];

         stepTransition[2 * i].time = step.getTimeInterval().getStartTime();
         stepTransition[2 * i].type = QuadrupedStepTransitionType.LIFT_OFF;
         stepTransition[2 * i].robotQuadrant = step.getRobotQuadrant();
         stepTransition[2 * i].solePosition = step.getGoalPosition();

         stepTransition[2 * i + 1].time = step.getTimeInterval().getEndTime();
         stepTransition[2 * i + 1].type = QuadrupedStepTransitionType.TOUCH_DOWN;
         stepTransition[2 * i + 1].robotQuadrant = step.getRobotQuadrant();
         stepTransition[2 * i + 1].solePosition = step.getGoalPosition();
      }

      // sort step transitions in ascending order as a function of time
      ArraySorter.sort(stepTransition, compareByTime);

      // compute transition time and center of pressure for each time interval
      int numberOfTransitions = 0;
      for (int i = 0; i < 2 * numberOfSteps; i++)
      {
         switch (stepTransition[i].type)
         {
         case LIFT_OFF:
            contactState.set(stepTransition[i].robotQuadrant, ContactState.NO_CONTACT);
            break;
         case TOUCH_DOWN:
            contactState.set(stepTransition[i].robotQuadrant, ContactState.IN_CONTACT);
            solePosition.get(stepTransition[i].robotQuadrant).setIncludingFrame(stepTransition[i].solePosition);
            solePosition.get(stepTransition[i].robotQuadrant).changeFrame(ReferenceFrame.getWorldFrame());
            break;
         }
         if ((i + 1 == 2 * numberOfSteps) || (stepTransition[i].time != stepTransition[i + 1].time))
         {
            timeAtTransition[numberOfTransitions] = stepTransition[i].time;
            computeNominalCopPosition(solePosition, contactState, copAtTransition[numberOfTransitions]);
            numberOfTransitions++;
         }
      }

      // pad transition time and center of pressure arrays
      if (numberOfTransitions > 0)
      {
         for (int i = numberOfTransitions; i < timeAtTransition.length; i++)
         {
            timeAtTransition[i] = timeAtTransition[numberOfTransitions - 1];
            copAtTransition[i].setIncludingFrame(copAtTransition[numberOfTransitions - 1]);
         }
      }

      return numberOfTransitions;
   }

   private void computeNominalCopPosition(QuadrantDependentList<FramePoint> solePosition, QuadrantDependentList<ContactState> contactState, FramePoint copPosition)
   {
      int hindContacts = 0;
      if (contactState.get(RobotQuadrant.HIND_LEFT) == ContactState.IN_CONTACT)
      {
         hindContacts++;
      }
      if (contactState.get(RobotQuadrant.HIND_RIGHT) == ContactState.IN_CONTACT)
      {
         hindContacts++;
      }

      int frontContacts = 0;
      if (contactState.get(RobotQuadrant.FRONT_LEFT) == ContactState.IN_CONTACT)
      {
         frontContacts++;
      }
      if (contactState.get(RobotQuadrant.FRONT_RIGHT) == ContactState.IN_CONTACT)
      {
         frontContacts++;
      }

      double endContactWeight = 1.0;
      if (hindContacts > 0 && frontContacts > 0)
      {
         endContactWeight = 0.5;
      }

      // compute total center of pressure assuming equal force distribution for hind and front ends
      copPosition.setToZero(ReferenceFrame.getWorldFrame());
      if (contactState.get(RobotQuadrant.HIND_LEFT) == ContactState.IN_CONTACT)
      {
         addPointWithScaleFactor(copPosition, solePosition.get(RobotQuadrant.HIND_LEFT), endContactWeight / hindContacts);
      }
      if (contactState.get(RobotQuadrant.HIND_RIGHT) == ContactState.IN_CONTACT)
      {
         addPointWithScaleFactor(copPosition, solePosition.get(RobotQuadrant.HIND_RIGHT), endContactWeight / hindContacts);
      }
      if (contactState.get(RobotQuadrant.FRONT_LEFT) == ContactState.IN_CONTACT)
      {
         addPointWithScaleFactor(copPosition, solePosition.get(RobotQuadrant.FRONT_LEFT), endContactWeight / frontContacts);
      }
      if (contactState.get(RobotQuadrant.FRONT_RIGHT) == ContactState.IN_CONTACT)
      {
         addPointWithScaleFactor(copPosition, solePosition.get(RobotQuadrant.FRONT_RIGHT), endContactWeight / frontContacts);
      }
   }

   private void addPointWithScaleFactor(FramePoint point, FramePoint pointToAdd, double scaleFactor)
   {
      point.checkReferenceFrameMatch(pointToAdd);
      point.add(scaleFactor * pointToAdd.getX(), scaleFactor * pointToAdd.getY(), scaleFactor * pointToAdd.getZ());
   }
}
