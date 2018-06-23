package us.ihmc.quadrupedRobotics.planning.icp;

import org.apache.commons.lang3.mutable.MutableDouble;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.quadrupedRobotics.planning.QuadrupedCenterOfPressureTools;
import us.ihmc.quadrupedRobotics.planning.QuadrupedTimedStep;
import us.ihmc.quadrupedRobotics.util.TimeIntervalTools;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.tools.lists.ArraySorter;

import java.util.Comparator;
import java.util.List;

public class QuadrupedContactSequence extends RecyclingArrayList<QuadrupedContactPhase>
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

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
   private final int pastContactPhaseCapacity;
   private final QuadrupedStepTransition[] stepTransition;
   private final QuadrantDependentList<ContactState> contactStates = new QuadrantDependentList<>();
   private final QuadrantDependentList<FramePoint3D> solePositions = new QuadrantDependentList<>();
   private final QuadrantDependentList<MutableDouble> contactPressures = new QuadrantDependentList<>();

   private final FramePoint3D copPosition = new FramePoint3D();

   private final int maxCapacity;

   private final QuadrantDependentList<ReferenceFrame> soleFrames;

   public QuadrupedContactSequence(QuadrantDependentList<ReferenceFrame> soleFrames, int pastContactPhaseCapacity, int futureContactPhaseCapacity)
   {
      super(pastContactPhaseCapacity + futureContactPhaseCapacity + 1, QuadrupedContactPhase.class);
      clear();

      this.soleFrames = soleFrames;

      maxCapacity = pastContactPhaseCapacity + futureContactPhaseCapacity + 1;
      this.pastContactPhaseCapacity = pastContactPhaseCapacity;

      if (maxCapacity < 1)
      {
         throw new RuntimeException("Sequence capacity must be greater than zero.");
      }

      stepTransition = new QuadrupedStepTransition[maxCapacity];
      for (int i = 0; i < maxCapacity; i++)
      {
         stepTransition[i] = new QuadrupedStepTransition();
      }

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         contactStates.set(robotQuadrant, ContactState.IN_CONTACT);
         solePositions.set(robotQuadrant, new FramePoint3D());
         contactPressures.set(robotQuadrant, new MutableDouble());
      }
   }

   public void initialize()
   {
      clear();
   }

   /**
    * compute piecewise center of pressure plan given an array of upcoming steps
    * @param stepSequence list of upcoming steps (input)
    * @param currentContactState current sole contact state (input)
    * @param currentTime current time (input)
    */
   public void update(List<? extends QuadrupedTimedStep> stepSequence,
                      QuadrantDependentList<ContactState> currentContactState, double currentTime)
   {
      // initialize contact state and sole positions
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePositions.get(robotQuadrant).setToZero(soleFrames.get(robotQuadrant));
         solePositions.get(robotQuadrant).changeFrame(worldFrame);
         contactStates.set(robotQuadrant, currentContactState.get(robotQuadrant));
      }

      // initialize step transitions
      for (int i = 0; i < stepTransition.length; i++)
      {
         stepTransition[i].time = Double.MAX_VALUE;
      }

      // FIXME we might have some bugs in here
      int numberOfStepTransitions = 0;
      for (int i = 0; i < stepSequence.size(); i++)
      {
         QuadrupedTimedStep step = stepSequence.get(i);

         if (step.getTimeInterval().getStartTime() >= currentTime)
         {
            stepTransition[numberOfStepTransitions].time = step.getTimeInterval().getStartTime();
            stepTransition[numberOfStepTransitions].type = QuadrupedStepTransitionType.LIFT_OFF;
            stepTransition[numberOfStepTransitions].robotQuadrant = step.getRobotQuadrant();
            step.getGoalPosition(stepTransition[numberOfStepTransitions].solePosition);

            numberOfStepTransitions++;
         }

         if (step.getTimeInterval().getEndTime() >= currentTime)
         {
            stepTransition[numberOfStepTransitions].time = step.getTimeInterval().getEndTime();
            stepTransition[numberOfStepTransitions].type = QuadrupedStepTransitionType.TOUCH_DOWN;
            stepTransition[numberOfStepTransitions].robotQuadrant = step.getRobotQuadrant();
            step.getGoalPosition(stepTransition[numberOfStepTransitions].solePosition);

            numberOfStepTransitions++;
         }
      }

      // sort step transitions in ascending order as a function of time
      ArraySorter.sort(stepTransition, compareByTime);

      // retain desired number of past contact phases
      TimeIntervalTools.removeStartTimesGreaterThanOrEqualTo(currentTime, this);
      while (size() > pastContactPhaseCapacity + 1)
      {
         remove(0);
      }

      QuadrupedContactPhase contactPhase;
      if (isEmpty())
      {
         contactPhase = createNewContactPhase(currentTime, contactStates, solePositions);
      }
      else
      {
         QuadrupedContactPhase lastContactPhase = get(size() - 1);
         if (isEqualContactState(lastContactPhase.getContactStates(), currentContactState))
         {
            // extend current contact phase
            contactPhase = lastContactPhase;
            contactPhase.setSolePosition(solePositions);
         }
         else
         {
            // end previous contact phase
            lastContactPhase.getTimeInterval().setEndTime(currentTime);
            remove(0);

            contactPhase = createNewContactPhase(currentTime, contactStates, solePositions);
         }
      }

      // compute transition time and center of pressure for each time interval
      for (int i = 0; i < numberOfStepTransitions; i++)
      {
         switch (stepTransition[i].type)
         {
         case LIFT_OFF:
            contactStates.set(stepTransition[i].robotQuadrant, ContactState.NO_CONTACT);
            break;
         case TOUCH_DOWN:
            contactStates.set(stepTransition[i].robotQuadrant, ContactState.IN_CONTACT);
            solePositions.get(stepTransition[i].robotQuadrant).changeFrame(worldFrame);
            solePositions.get(stepTransition[i].robotQuadrant).set(stepTransition[i].solePosition);
            break;
         }
         if ((i + 1 == numberOfStepTransitions) || (stepTransition[i].time != stepTransition[i + 1].time))
         {
            contactPhase.getTimeInterval().setEndTime(stepTransition[i].time);

            contactPhase = createNewContactPhase(stepTransition[i].time, contactStates, solePositions);
            if (contactPhase == null) // made the full sequence
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

   private QuadrupedContactPhase createNewContactPhase(double startTime, QuadrantDependentList<ContactState> contactState, QuadrantDependentList<FramePoint3D> solePosition)
   {
      if (maxCapacity - size() > 0)
      {
         QuadrupedCenterOfPressureTools.computeNominalNormalizedContactPressure(contactPressures, contactState);
         QuadrupedCenterOfPressureTools.computeCenterOfPressure(copPosition, solePosition, contactPressures);

         QuadrupedContactPhase contactPhase = add();
         contactPhase.getTimeInterval().setStartTime(startTime);
         contactPhase.setContactStates(contactState);
         contactPhase.setSolePosition(solePosition);
         contactPhase.setContactPressures(contactPressures);
         contactPhase.setCopPosition(copPosition);

         if (copPosition.containsNaN())
            contactPhase.setContactState(ContactState.NO_CONTACT);
         else
            contactPhase.setContactState(ContactState.IN_CONTACT);

         return contactPhase;
      }
      else
      {
         return null;
      }
   }

}
