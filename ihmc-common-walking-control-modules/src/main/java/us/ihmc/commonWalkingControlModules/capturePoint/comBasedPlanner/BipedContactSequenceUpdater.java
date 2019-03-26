package us.ihmc.commonWalkingControlModules.capturePoint.comBasedPlanner;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicPosition;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.TimeIntervalTools;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoFramePoint2D;

import java.util.ArrayList;
import java.util.List;

/**
 * This class computes a list of {@link ContactStateProvider} from a list of {@link BipedTimedStep}. This uses transition trajectories from the
 * start CoP to the end CoP. This class should be made smarter for computing better VRP waypoints for things like heel-toe walking.
 */
public class BipedContactSequenceUpdater
{
   private static final boolean debug = true;
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

   public BipedContactSequenceUpdater(SideDependentList<MovingReferenceFrame> soleFrames, YoVariableRegistry registry,
                                      YoGraphicsListRegistry graphicsListRegistry)
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

         YoGraphicPosition startCoP = new YoGraphicPosition("start cop " + i, startCoPs.get(i), 0.01, YoAppearance.Green(),
                                                            YoGraphicPosition.GraphicType.SOLID_BALL);
         YoGraphicPosition endCoP = new YoGraphicPosition("end cop " + i, startCoPs.get(i), 0.01, YoAppearance.Green(),
                                                          YoGraphicPosition.GraphicType.SOLID_BALL);

         if (graphicsListRegistry != null)
         {
            graphicsListRegistry.registerArtifact("Contact Sequence", startCoP.createArtifact());
            graphicsListRegistry.registerArtifact("Contact Sequence", endCoP.createArtifact());
         }
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

      computeContactPhasesFromStepTransitionsOther();

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
               if (feetInContact.isEmpty())
               {
                  startSide = transitionSide;
                  previousEndSide = transitionSide;
               }
               else
               {
                  startSide = transitionSide.getOppositeSide();
                  previousEndSide = transitionSide.getOppositeSide();
               }
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
         boolean isLastContact =
               (transitionNumber == numberOfTransitions - 1) || (contactSequenceInAbsoluteTime.size() >= maxCapacity && feetInContact.size() > 0);
         if (isLastContact)
            break;
      }

      previousContactPhase.getTimeInterval().setEndTime(Double.POSITIVE_INFINITY);
      for (int i = 0; i < feetInContact.size(); i++)
         previousContactPhase.addEndFoot(feetInContact.get(i), solePoses.get(feetInContact.get(i)));
      previousContactPhase.update();
   }

   private void computeContactPhasesFromStepTransitionsOther()
   {
      int numberOfTransitions = stepTransitionsInAbsoluteTime.size();

      // compute transition time and center of pressure for each time interval
      for (int transitionNumber = 0; transitionNumber < numberOfTransitions; transitionNumber++)
      {
         BipedStepTransition stepTransition = stepTransitionsInAbsoluteTime.get(transitionNumber);

         if (!isValidTransition(stepTransition))
            throw new RuntimeException("Not a valid transition.");

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

         SimpleBipedContactPhase endingContactSequence = contactSequenceInAbsoluteTime.getLast();
         SimpleBipedContactPhase newContactSequence = contactSequenceInAbsoluteTime.add();

         newContactSequence.reset();

         endingContactSequence.getTimeInterval().setEndTime(stepTransition.getTransitionTime());
         newContactSequence.getTimeInterval().setStartTime(stepTransition.getTransitionTime());

         newContactSequence.setFeetInContact(feetInContact);

         if (feetInContact.isEmpty())
         { // in flight, so the end of the previous contact phase is the foot that lifted off
            for (int i = 0; i < stepTransition.getNumberOfFeetInTransition(); i++)
            {
               if (debug)
                  assert (stepTransition.getTransitionType(i) == BipedStepTransitionType.LIFT_OFF);

               RobotSide liftOffSide = stepTransition.getTransitionSide(i);
               endingContactSequence.addEndFoot(liftOffSide, solePoses.get(liftOffSide));
            }
         }
         else if (feetInContact.size() == 1)
         { // feet in contact shouldn't be moving, one foot is in stance

            if (stepTransition.getNumberOfFeetInTransition() > 1)
               throw new RuntimeException("Currently can't handle an instant switch in support.");

            if (stepTransition.getTransitionType(0) == BipedStepTransitionType.LIFT_OFF)
            { // starting the swing phase, like in normal walking
               RobotSide transitionSide = stepTransition.getTransitionSide(0);
               RobotSide supportSide = transitionSide.getOppositeSide();
               endingContactSequence.addEndFoot(supportSide, solePoses.get(supportSide));
               newContactSequence.addStartFoot(supportSide, solePoses.get(supportSide));

               if (debug)
                  assert (supportSide == feetInContact.get(0));
            }
            else
            { // just ending a flight phase
               RobotSide transitionSide = stepTransition.getTransitionSide(0);
               newContactSequence.addStartFoot(transitionSide, solePoses.get(transitionSide));

               if (debug)
                  assert (transitionSide == feetInContact.get(0));
            }
         }
         else
         {
            if (debug)
            {
               assert (feetInContact.size() == 2);

               for (int i = 0; i < stepTransition.getNumberOfFeetInTransition(); i++)
                  assert (stepTransition.getTransitionType(i) == BipedStepTransitionType.TOUCH_DOWN);
            }

            if (stepTransition.getNumberOfFeetInTransition() == 2)
            { // just landing from a jump
               for (int i = 0; i < stepTransition.getNumberOfFeetInTransition(); i++)
               {
                  RobotSide transitionSide = stepTransition.getTransitionSide(i);
                  newContactSequence.addStartFoot(transitionSide, solePoses.get(transitionSide));
               }
            }
            else
            {
               if (debug)
                  assert (stepTransition.getNumberOfFeetInTransition() == 1);

               RobotSide transitionSide = stepTransition.getTransitionSide(0);
               RobotSide oppositeSide = transitionSide.getOppositeSide();
               endingContactSequence.addEndFoot(oppositeSide, solePoses.get(oppositeSide));
               newContactSequence.addStartFoot(oppositeSide, solePoses.get(oppositeSide));
            }
         }

         boolean isLastContact =
               (transitionNumber == numberOfTransitions - 1) || (contactSequenceInAbsoluteTime.size() >= maxCapacity && feetInContact.size() > 0);
         if (isLastContact)
            break;
      }

      SimpleBipedContactPhase contactPhase = contactSequenceInAbsoluteTime.getLast();
      contactPhase.getTimeInterval().setEndTime(Double.POSITIVE_INFINITY);
      for (int i = 0; i < feetInContact.size(); i++)
         contactPhase.addEndFoot(feetInContact.get(i), solePoses.get(feetInContact.get(i)));
      contactPhase.update();
   }

   private static boolean isValidTransition(BipedStepTransition stepTransition)
   {
      if (stepTransition.getNumberOfFeetInTransition() > 2)
      { // just started or landed from a jump
         return false;
      }

      return true;
   }
}
