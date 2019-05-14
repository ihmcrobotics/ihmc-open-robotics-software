package us.ihmc.quadrupedRobotics.planning;

import java.awt.*;
import java.util.Comparator;
import java.util.List;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.commons.lists.PreallocatedList;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.robotics.time.TimeIntervalTools;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.tools.lists.ArraySorter;
import us.ihmc.yoVariables.variable.YoEnum;

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
      Point3D solePosition;
      double time;

      public QuadrupedStepTransition()
      {
         time = 0.0;
         type = QuadrupedStepTransitionType.LIFT_OFF;
         solePosition = new Point3D();
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
   private final QuadrantDependentList<ContactState> contactState;
   private final QuadrantDependentList<FramePoint3D> solePosition;

   public QuadrupedTimedContactSequence(int futureContactPhaseCapacity)
   {
      super(QuadrupedTimedContactPhase.class, QuadrupedTimedContactPhase::new, futureContactPhaseCapacity + 1);

      if (capacity() < 1)
      {
         throw new RuntimeException("Sequence capacity must be greater than zero.");
      }

      stepTransition = new QuadrupedStepTransition[capacity()];
      for (int i = 0; i < capacity(); i++)
      {
         stepTransition[i] = new QuadrupedStepTransition();
      }
      contactState = new QuadrantDependentList<>();
      solePosition = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         contactState.set(robotQuadrant, ContactState.IN_CONTACT);
         solePosition.set(robotQuadrant, new FramePoint3D());
      }
   }

   public void initialize()
   {
      super.clear();
   }

   /**
    * compute piecewise center of pressure plan given an array of upcoming steps
    * @param stepSequence list of upcoming steps (input)
    * @param soleFrames current sole frames (input)
    * @param currentContactState current sole contact state (input)
    * @param currentTime current time (input)
    */
   public void update(List<? extends QuadrupedTimedStep> stepSequence, QuadrantDependentList<? extends ReferenceFrame> soleFrames,
                      QuadrantDependentList<YoEnum<ContactState>> currentContactState, double currentTime)
   {
      super.clear();

      // initialize contact state and sole positions
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePosition.get(robotQuadrant).setToZero(soleFrames.get(robotQuadrant));
         solePosition.get(robotQuadrant).changeFrame(ReferenceFrame.getWorldFrame());
         contactState.set(robotQuadrant, currentContactState.get(robotQuadrant).getEnumValue());
      }

      // initialize step transitions
      for (int i = 0; i < stepTransition.length; i++)
      {
         stepTransition[i].time = Double.MAX_VALUE;
      }

      int numberOfStepTransitions = 0;
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (!currentContactState.get(robotQuadrant).getEnumValue().isLoadBearing())
         {
            QuadrupedTimedStep firstStepForQuadrant = getFirstStepForQuadrant(robotQuadrant, stepSequence);

            // end the current step if there are no active steps for it.
            if (firstStepForQuadrant == null || firstStepForQuadrant.getTimeInterval().getStartTime() > currentTime)
            {
               stepTransition[numberOfStepTransitions].time = currentTime;
               stepTransition[numberOfStepTransitions].type = QuadrupedStepTransitionType.TOUCH_DOWN;
               stepTransition[numberOfStepTransitions].robotQuadrant = robotQuadrant;
               stepTransition[numberOfStepTransitions].solePosition.set(solePosition.get(robotQuadrant));
               contactState.set(robotQuadrant, ContactState.IN_CONTACT);
               numberOfStepTransitions++;
            }
         }
      }

      for (int i = 0; i < stepSequence.size(); i++)
      {
         QuadrupedTimedStep step = stepSequence.get(i);

         if (step.getTimeInterval().getStartTime() >= currentTime)
         {
            stepTransition[numberOfStepTransitions].time = step.getTimeInterval().getStartTime();
            stepTransition[numberOfStepTransitions].type = QuadrupedStepTransitionType.LIFT_OFF;
            stepTransition[numberOfStepTransitions].robotQuadrant = step.getRobotQuadrant();
            stepTransition[numberOfStepTransitions].solePosition.set(step.getGoalPosition());
            numberOfStepTransitions++;
         }

         if (step.getTimeInterval().getEndTime() >= currentTime)
         {
            stepTransition[numberOfStepTransitions].time = step.getTimeInterval().getEndTime();
            stepTransition[numberOfStepTransitions].type = QuadrupedStepTransitionType.TOUCH_DOWN;
            stepTransition[numberOfStepTransitions].robotQuadrant = step.getRobotQuadrant();
            stepTransition[numberOfStepTransitions].solePosition.set(step.getGoalPosition());
            numberOfStepTransitions++;
         }
      }

      // sort step transitions in ascending order as a function of time
      ArraySorter.sort(stepTransition, compareByTime);

      QuadrupedTimedContactPhase  contactPhase = createNewContactPhase(currentTime, contactState, solePosition);

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
            solePosition.get(stepTransition[i].robotQuadrant).set(stepTransition[i].solePosition);
            break;
         }
         if ((i + 1 == numberOfStepTransitions) || (stepTransition[i].time != stepTransition[i + 1].time))
         {
            contactPhase.getTimeInterval().setEndTime(stepTransition[i].time);
            contactPhase = createNewContactPhase(stepTransition[i].time, contactState, solePosition);
            if (contactPhase == null)
            {
               return;
            }
         }
      }
      contactPhase.getTimeInterval().setEndTime(contactPhase.getTimeInterval().getStartTime() + 1.0);
   }

   private boolean isEqualContactState(QuadrantDependentList<ContactState> contactStateA, QuadrantDependentList<YoEnum<ContactState>> contactStateB)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         if (contactStateA.get(robotQuadrant) != contactStateB.get(robotQuadrant).getEnumValue())
            return false;
      }
      return true;
   }

   private QuadrupedTimedContactPhase createNewContactPhase(double startTime, QuadrantDependentList<ContactState> contactState, QuadrantDependentList<FramePoint3D> solePosition)
   {
      if (remaining() > 0)
      {
         QuadrupedTimedContactPhase contactPhase = add();
         contactPhase.getTimeInterval().setStartTime(startTime);
         contactPhase.setContactState(contactState);
         contactPhase.setSolePosition(solePosition);
         return contactPhase;
      }
      else
      {
         return null;
      }
   }

   private QuadrupedTimedStep getFirstStepForQuadrant(RobotQuadrant robotQuadrant, List<? extends QuadrupedTimedStep> stepSequence)
   {
      for (int i = 0 ; i < stepSequence.size(); i++)
      {
         if (stepSequence.get(i).getRobotQuadrant() == robotQuadrant)
            return stepSequence.get(i);
      }

      return null;
   }
}

