package us.ihmc.quadrupedRobotics.planning.icp;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.quadrupedBasics.gait.QuadrupedTimedStep;
import us.ihmc.quadrupedBasics.utils.TimeIntervalTools;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.ArrayList;
import java.util.List;

public class QuadrupedContactSequenceUpdater
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   static final double finalTransferDuration = 1.0;

   private final int pastContactPhaseCapacity;
   private final RecyclingArrayList<QuadrupedStepTransition> stepTransitions = new RecyclingArrayList<>(QuadrupedStepTransition::new);
   private final List<RobotQuadrant> feetInContact = new ArrayList<>();
   private final QuadrantDependentList<FramePoint3D> solePositions = new QuadrantDependentList<>();
   private final QuadrantDependentList<ReferenceFrame> soleFrames;

   private final int maxCapacity;

   private final RecyclingArrayList<QuadrupedContactPhase> contactSequence;

   public QuadrupedContactSequenceUpdater(QuadrantDependentList<ReferenceFrame> soleFrames, int pastContactPhaseCapacity, int futureContactPhaseCapacity)
   {
      contactSequence = new RecyclingArrayList<>(pastContactPhaseCapacity + futureContactPhaseCapacity + 1, QuadrupedContactPhase::new);
      contactSequence.clear();

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
      contactSequence.clear();
   }

   public List<QuadrupedContactPhase> getContactSequence()
   {
      return contactSequence;
   }

   public void update(List<? extends QuadrupedTimedStep> stepSequence, List<RobotQuadrant> currentFeetInContact, double currentTime)
   {
      initializeCalculationConditions(currentFeetInContact);

      QuadrupedContactSequenceTools.computeStepTransitionsFromStepSequence(stepTransitions, currentTime, stepSequence);
      QuadrupedContactSequenceTools.trimPastContactSequences(contactSequence, currentTime, currentFeetInContact, solePositions, pastContactPhaseCapacity);

      computeContactPhasesFromStepTransitions();
   }

   private void initializeCalculationConditions(List<RobotQuadrant> currentFeetInContact)
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
   }

   private void computeContactPhasesFromStepTransitions()
   {
      int numberOfTransitions = stepTransitions.size();
      // compute transition time and center of pressure for each time interval
      for (int transitionNumber = 0; transitionNumber < numberOfTransitions; transitionNumber++)
      {
         QuadrupedStepTransition stepTransition = stepTransitions.get(transitionNumber);

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

         QuadrupedContactPhase contactPhase = contactSequence.add();

         boolean isLastContact = (transitionNumber == numberOfTransitions - 1) || (contactSequence.size() == maxCapacity);

         contactPhase.setFeetInContact(feetInContact);
         contactPhase.setSolePositions(solePositions);
         contactPhase.update();

         if (isLastContact)
         {
            contactPhase.getTimeInterval().setInterval(stepTransition.getTransitionTime(), stepTransition.getTransitionTime() + finalTransferDuration);
            return;
         }
         else
         {
            contactPhase.getTimeInterval().setInterval(stepTransition.getTransitionTime(), stepTransitions.get(transitionNumber + 1).getTransitionTime());
         }
      }
   }

}
