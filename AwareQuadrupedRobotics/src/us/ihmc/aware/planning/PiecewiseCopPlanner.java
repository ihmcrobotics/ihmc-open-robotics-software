package us.ihmc.aware.planning;

import us.ihmc.aware.util.*;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import java.util.Comparator;

public class PiecewiseCopPlanner
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
   private final QuadrantDependentList<FramePoint> solePosition;
   private final QuadrantDependentList<ContactState> contactState;

   public PiecewiseCopPlanner(int maxIntervals)
   {
      timeAtTransition = new double[maxIntervals];
      copAtTransition = new FramePoint[maxIntervals];
      stepTransition = new QuadrupedStepTransition[maxIntervals];
      for (int i = 0; i < maxIntervals; i++)
      {
         timeAtTransition[i] = 0.0;
         copAtTransition[i] = new FramePoint(ReferenceFrame.getWorldFrame());
         stepTransition[i] = new QuadrupedStepTransition();
      }
      solePosition = new QuadrantDependentList<>();
      contactState = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePosition.set(robotQuadrant, new FramePoint(ReferenceFrame.getWorldFrame()));
         contactState.set(robotQuadrant, ContactState.IN_CONTACT);
      }
   }

   public int compute(QuadrantDependentList<FramePoint> initialSolePosition, QuadrantDependentList<ContactState> initialContactState, PreallocatedQueue<QuadrupedTimedStep> stepQueue)
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

      for (int i = 0; i < stepQueue.size(); i++)
      {
         QuadrupedTimedStep step = stepQueue.get(i);

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
      for (int i = 0; i < 2 * stepQueue.size(); i++)
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
         if ((i + 1 == 2 * stepQueue.size()) || (stepTransition[i].time != stepTransition[i + 1].time))
         {
            timeAtTransition[numberOfTransitions] = stepTransition[i].time;
            computeNominalCop(solePosition, contactState, copAtTransition[numberOfTransitions]);
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

   private void computeNominalCop(QuadrantDependentList<FramePoint> solePosition, QuadrantDependentList<ContactState> contactState, FramePoint copPosition)
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
