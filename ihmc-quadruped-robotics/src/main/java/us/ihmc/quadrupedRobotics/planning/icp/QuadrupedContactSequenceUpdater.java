package us.ihmc.quadrupedRobotics.planning.icp;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedBasics.utils.TimeIntervalTools;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.ArrayList;
import java.util.List;

public class QuadrupedContactSequenceUpdater
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   private final int pastContactPhaseCapacity;
   private final RecyclingArrayList<QuadrupedStepTransition> stepTransitionsInAbsoluteTime = new RecyclingArrayList<>(QuadrupedStepTransition::new);
   private final List<RobotQuadrant> feetInContact = new ArrayList<>();
   private final QuadrantDependentList<FramePoint3D> solePositions = new QuadrantDependentList<>();
   private final QuadrantDependentList<MovingReferenceFrame> soleFrames;

   private final int maxCapacity;

   private final RecyclingArrayList<QuadrupedContactPhase> contactSequenceInAbsoluteTime;
   private final RecyclingArrayList<QuadrupedContactPhase> contactSequenceInRelativeTime;

   public QuadrupedContactSequenceUpdater(QuadrantDependentList<MovingReferenceFrame> soleFrames, int pastContactPhaseCapacity, int futureContactPhaseCapacity)
   {
      contactSequenceInAbsoluteTime = new RecyclingArrayList<>(pastContactPhaseCapacity + futureContactPhaseCapacity + 1, QuadrupedContactPhase::new);
      contactSequenceInRelativeTime = new RecyclingArrayList<>(pastContactPhaseCapacity + futureContactPhaseCapacity + 1, QuadrupedContactPhase::new);
      contactSequenceInAbsoluteTime.clear();
      contactSequenceInRelativeTime.clear();

      this.soleFrames = soleFrames;

      maxCapacity = pastContactPhaseCapacity + futureContactPhaseCapacity + 1;
      this.pastContactPhaseCapacity = pastContactPhaseCapacity;

      if (maxCapacity < 1)
      {
         throw new RuntimeException("Sequence capacity must be greater than zero.");
      }

      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         feetInContact.add(robotQuadrant);
         solePositions.set(robotQuadrant, new FramePoint3D());
      }
   }

   public void initialize()
   {
      contactSequenceInAbsoluteTime.clear();
   }

   public List<QuadrupedContactPhase> getContactSequenceInAbsoluteTime()
   {
      return contactSequenceInRelativeTime;
   }

   public void update(List<? extends QuadrupedTimedStep> stepSequence, List<RobotQuadrant> currentFeetInContact, double currentTime)
   {
      // initialize contact state and sole positions
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         solePositions.get(robotQuadrant).setToZero(soleFrames.get(robotQuadrant));
         solePositions.get(robotQuadrant).changeFrame(worldFrame);
      }
      feetInContact.clear();
      for (int footIndex = 0; footIndex < currentFeetInContact.size(); footIndex++)
         feetInContact.add(currentFeetInContact.get(footIndex));

      QuadrupedContactSequenceTools.computeStepTransitionsFromStepSequence(stepTransitionsInAbsoluteTime, currentTime, stepSequence);
      QuadrupedContactSequenceTools
            .trimPastContactSequences(contactSequenceInAbsoluteTime, currentTime, currentFeetInContact, solePositions, pastContactPhaseCapacity);

      computeContactPhasesFromStepTransitions();

      contactSequenceInRelativeTime.clear();
      for (int i = 0; i < contactSequenceInAbsoluteTime.size(); i++)
         contactSequenceInRelativeTime.add().set(contactSequenceInAbsoluteTime.get(i));

      TimeIntervalTools.removeEndTimesLessThan(currentTime, contactSequenceInRelativeTime);
      QuadrupedContactSequenceTools.shiftContactSequencesToRelativeTime(contactSequenceInRelativeTime, currentTime);
   }

   private void computeContactPhasesFromStepTransitions()
   {
      int numberOfTransitions = stepTransitionsInAbsoluteTime.size();
      QuadrupedContactPhase contactPhase = contactSequenceInAbsoluteTime.getLast();

      // compute transition time and center of pressure for each time interval
      for (int transitionNumber = 0; transitionNumber < numberOfTransitions; transitionNumber++)
      {
         QuadrupedStepTransition stepTransition = stepTransitionsInAbsoluteTime.get(transitionNumber);

         for (int transitioningFootNumber = 0; transitioningFootNumber < stepTransition.getNumberOfFeetInTransition(); transitioningFootNumber++)
         {
            RobotQuadrant transitionQuadrant = stepTransition.getTransitionQuadrant(transitioningFootNumber);
            switch (stepTransition.getTransitionType(transitioningFootNumber))
            {
            case LIFT_OFF:
               feetInContact.remove(transitionQuadrant);
               break;
            case TOUCH_DOWN:
               feetInContact.add(transitionQuadrant);
               solePositions.get(transitionQuadrant).setIncludingFrame(worldFrame, stepTransition.getTransitionPosition(transitionQuadrant));
               break;
            }
         }

         contactPhase.getTimeInterval().setEndTime(stepTransition.getTransitionTime());
         contactPhase = contactSequenceInAbsoluteTime.add();

         contactPhase.setFeetInContact(feetInContact);
         contactPhase.setSolePositions(solePositions);
         contactPhase.update();

         contactPhase.getTimeInterval().setStartTime(stepTransition.getTransitionTime());

         boolean isLastContact = (transitionNumber == numberOfTransitions - 1) || (contactSequenceInAbsoluteTime.size() == maxCapacity);
         if (isLastContact)
            break;
      }

      contactPhase.getTimeInterval().setEndTime(Double.POSITIVE_INFINITY);

   }

}
