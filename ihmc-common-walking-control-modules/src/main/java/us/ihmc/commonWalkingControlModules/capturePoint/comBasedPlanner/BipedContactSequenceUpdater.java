package us.ihmc.commonWalkingControlModules.capturePoint.comBasedPlanner;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.TimeInterval;
import us.ihmc.robotics.time.TimeIntervalTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint2D;

import java.util.ArrayList;
import java.util.List;

public class BipedContactSequenceUpdater
{
   private static final int maxCapacity = 7;
   private final RecyclingArrayList<BipedStepTransition> stepTransitionsInAbsoluteTime = new RecyclingArrayList<>(BipedStepTransition::new);

   private final RecyclingArrayList<SimpleBipedContactPhase> contactSequenceInRelativeTime = new RecyclingArrayList<>(maxCapacity,
                                                                                                                      SimpleBipedContactPhase::new);
   private final RecyclingArrayList<SimpleBipedContactPhase> contactSequenceInAbsoluteTime = new RecyclingArrayList<>(maxCapacity,
                                                                                                                      SimpleBipedContactPhase::new);

   private final List<YoFramePoint2D> startCoPs = new ArrayList<>();
   private final List<YoFramePoint2D> endCoPs = new ArrayList<>();

   private final List<RobotSide> feetInContact = new ArrayList<>();
   private final SideDependentList<FramePose3D> solePoses = new SideDependentList<>();
   private final SideDependentList<MovingReferenceFrame> soleFrames;

   public BipedContactSequenceUpdater(SideDependentList<MovingReferenceFrame> soleFrames, YoVariableRegistry registry, YoGraphicsListRegistry graphicsListRegistry)
   {
      this.soleFrames = soleFrames;
      contactSequenceInAbsoluteTime.clear();
      contactSequenceInRelativeTime.clear();

      for (RobotSide robotSide : RobotSide.values)
      {
         feetInContact.add(robotSide);
         solePoses.set(robotSide, new FramePose3D());
      }

      for (int i = 0; i < maxCapacity + 1; i++)
      {
         startCoPs.add(new YoFramePoint2D("startCoP" + i, ReferenceFrame.getWorldFrame(), registry));
         endCoPs.add(new YoFramePoint2D("endCoP" + i, ReferenceFrame.getWorldFrame(), registry));

         YoGraphicPosition startCoP = new YoGraphicPosition("start cop " + i, startCoPs.get(i), 0.01, YoAppearance.Green(), YoGraphicPosition.GraphicType.SOLID_BALL);
         YoGraphicPosition endCoP = new YoGraphicPosition("end cop " + i, startCoPs.get(i), 0.01, YoAppearance.Green(), YoGraphicPosition.GraphicType.SOLID_BALL);

         graphicsListRegistry.registerArtifact("Contact Sequence", startCoP.createArtifact());
         graphicsListRegistry.registerArtifact("Contact Sequence", endCoP.createArtifact());
      }
   }

   public void initialize()
   {
      for (int i = 0; i < contactSequenceInAbsoluteTime.size(); i++)
         contactSequenceInAbsoluteTime.get(i).reset();
      contactSequenceInAbsoluteTime.clear();
   }

   public List<SimpleBipedContactPhase> getContactSequence()
   {
      return contactSequenceInRelativeTime;
   }

   public List<SimpleBipedContactPhase> getAbsoluteContactSequence()
   {
      return contactSequenceInAbsoluteTime;
   }

   public void update(List<? extends BipedTimedStep> stepSequence, List<RobotSide> currentFeetInContact, double currentTime)
   {
      // initialize contact state and sole positions
      for (RobotSide robotSide : RobotSide.values)
      {
         solePoses.get(robotSide).setToZero(soleFrames.get(robotSide));
      }
      feetInContact.clear();
      for (int footIndex = 0; footIndex < currentFeetInContact.size(); footIndex++)
      {
         feetInContact.add(currentFeetInContact.get(footIndex));
      }

      BipedContactSequenceTools.computeStepTransitionsFromStepSequence(stepTransitionsInAbsoluteTime, currentTime, stepSequence);
      BipedContactSequenceTools.trimPastContactSequences(contactSequenceInAbsoluteTime, currentTime, currentFeetInContact, solePoses);

      computeContactPhasesFromStepTransitions();

      contactSequenceInRelativeTime.clear();
      for (int i = 0; i < contactSequenceInAbsoluteTime.size(); i++)
      {
         SimpleBipedContactPhase contactPhase = contactSequenceInRelativeTime.add();
         contactPhase.reset();
         contactPhase.set(contactSequenceInAbsoluteTime.get(i));
      }

      BipedContactSequenceTools.shiftContactSequencesToRelativeTime(contactSequenceInRelativeTime, currentTime);

      TimeIntervalTools.removeEndTimesLessThan(0.0, contactSequenceInRelativeTime);

      int i = 0;
      for (; i < contactSequenceInRelativeTime.size(); i++)
      {
         startCoPs.get(i).set(contactSequenceInRelativeTime.get(i).getCopStartPosition());
         endCoPs.get(i).set(contactSequenceInRelativeTime.get(i).getCopEndPosition());
      }
      for (; i < maxCapacity; i++)
      {
         startCoPs.get(i).setToNaN();
         endCoPs.get(i).setToNaN();
      }
   }


   private void computeContactPhasesFromStepTransitions()
   {
      int numberOfTransitions = stepTransitionsInAbsoluteTime.size();
      SimpleBipedContactPhase previousContactPhase = contactSequenceInAbsoluteTime.getLast();

      // compute transition time and center of pressure for each time interval
      for (int transitionNumber = 0; transitionNumber < numberOfTransitions; transitionNumber++)
      {

         BipedStepTransition stepTransition = stepTransitionsInAbsoluteTime.get(transitionNumber);

         // end the previous phase and add a new one
         previousContactPhase.getTimeInterval().setEndTime(stepTransition.getTransitionTime());

         SimpleBipedContactPhase contactPhase = contactSequenceInAbsoluteTime.add();
         contactPhase.reset();

         for (int transitioningFootNumber = 0; transitioningFootNumber < stepTransition.getNumberOfFeetInTransition(); transitioningFootNumber++)
         {
            RobotSide transitionSide = stepTransition.getTransitionSide(transitioningFootNumber);
            switch (stepTransition.getTransitionType(transitioningFootNumber))
            {
            case LIFT_OFF:
               feetInContact.remove(transitionSide);
               break;
            case TOUCH_DOWN:
               feetInContact.add(transitionSide);
               solePoses.get(transitionSide).setMatchingFrame(stepTransition.transitionPose(transitionSide));
               break;
            }
         }

         if (stepTransition.getNumberOfFeetInTransition() > 1 && stepTransition.getTransitionType(0) == stepTransition.getTransitionType(1))
         { // just started or landed from a jump
            throw new RuntimeException("Not handled.");
         }
         else
         {
            RobotSide transitionSide = stepTransition.getTransitionSide(0);
            BipedStepTransitionType transitionType = stepTransition.getTransitionType(0);

            RobotSide startSide, previousEndSide;

            if (transitionType == BipedStepTransitionType.LIFT_OFF)
            { // this is the end of transfer, with a foot lifting off the ground, meaning this phase is the start of swing
               startSide = transitionSide.getOppositeSide();
               previousEndSide = transitionSide.getOppositeSide();
            }
            else
            {
               startSide = transitionSide.getOppositeSide();
               previousEndSide = transitionSide.getOppositeSide();
            }

            if (feetInContact.size() > 0)
            {
               contactPhase.addStartFoot(startSide, solePoses.get(startSide));
            }

            previousContactPhase.addEndFoot(previousEndSide, solePoses.get(previousEndSide));
            previousContactPhase.update();
         }

         contactPhase.setFeetInContact(feetInContact);

         contactPhase.getTimeInterval().setStartTime(stepTransition.getTransitionTime());
//         contactPhase.update();

         previousContactPhase = contactPhase;
         boolean isLastContact = (transitionNumber == numberOfTransitions - 1) || (contactSequenceInAbsoluteTime.size() >= maxCapacity && feetInContact.size() > 0);
         if (isLastContact)
            break;
      }

      previousContactPhase.getTimeInterval().setEndTime(Double.POSITIVE_INFINITY);
      for (int i = 0; i < feetInContact.size(); i++)
         previousContactPhase.addEndFoot(feetInContact.get(i), solePoses.get(feetInContact.get(i)));
      previousContactPhase.update();
   }

}
