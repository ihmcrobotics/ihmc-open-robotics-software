package us.ihmc.commonWalkingControlModules.capturePoint.comBasedPlanner;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.TimeIntervalTools;

import java.util.ArrayList;
import java.util.List;

public class BipedContactSequenceUpdater
{
   private static final int maxCapacity = 5;
   private final RecyclingArrayList<BipedStepTransition> stepTransitionsInAbsoluteTime = new RecyclingArrayList<>(BipedStepTransition::new);

   private final RecyclingArrayList<SimpleBipedContactPhase> contactSequenceInRelativeTime = new RecyclingArrayList<>(maxCapacity,
                                                                                                                      SimpleBipedContactPhase::new);
   private final RecyclingArrayList<SimpleBipedContactPhase> contactSequenceInAbsoluteTime = new RecyclingArrayList<>(maxCapacity,
                                                                                                                      SimpleBipedContactPhase::new);

   private final List<RobotSide> startFeetInContact = new ArrayList<>();
   private final List<RobotSide> endFeetInContact = new ArrayList<>();
   private final SideDependentList<FramePose3D> startSolePoses = new SideDependentList<>();
   private final SideDependentList<FramePose3D> endSolePoses = new SideDependentList<>();
   private final SideDependentList<MovingReferenceFrame> soleFrames;

   public BipedContactSequenceUpdater(SideDependentList<MovingReferenceFrame> soleFrames)
   {
      this.soleFrames = soleFrames;
      contactSequenceInAbsoluteTime.clear();
      contactSequenceInRelativeTime.clear();

      for (RobotSide robotSide : RobotSide.values)
      {
         startFeetInContact.add(robotSide);
         startSolePoses.set(robotSide, new FramePose3D());
         endSolePoses.set(robotSide, new FramePose3D());
      }
   }

   public void initialize()
   {
      contactSequenceInAbsoluteTime.clear();
   }

   public List<SimpleBipedContactPhase> getContactSequence()
   {
      return contactSequenceInRelativeTime;
   }

   public void update(List<? extends BipedTimedStep> stepSequence, List<RobotSide> currentFeetInContact, double currentTime)
   {
      // initialize contact state and sole positions
      for (RobotSide robotSide : RobotSide.values)
      {
         startSolePoses.get(robotSide).setToZero(soleFrames.get(robotSide));
         endSolePoses.get(robotSide).setToZero(soleFrames.get(robotSide));
      }
      startFeetInContact.clear();
      for (int footIndex = 0; footIndex < currentFeetInContact.size(); footIndex++)
         startFeetInContact.add(currentFeetInContact.get(footIndex));

      BipedContactSequenceTools.computeStepTransitionsFromStepSequence(stepTransitionsInAbsoluteTime, currentTime, stepSequence);
      BipedContactSequenceTools.trimPastContactSequences(contactSequenceInAbsoluteTime, currentTime, currentFeetInContact, startSolePoses);

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
   }

   // fixme this function is probably not correct

   private void computeContactPhasesFromStepTransitions()
   {
      int numberOfTransitions = stepTransitionsInAbsoluteTime.size();
      SimpleBipedContactPhase contactPhase = contactSequenceInAbsoluteTime.getLast();

      // compute transition time and center of pressure for each time interval
      for (int transitionNumber = 0; transitionNumber < numberOfTransitions; transitionNumber++)
      {
         BipedStepTransition stepTransition = stepTransitionsInAbsoluteTime.get(transitionNumber);
         startFeetInContact.clear();
         for (int i = 0; i < endFeetInContact.size(); i++)
         {
            RobotSide side = endFeetInContact.get(i);
            startFeetInContact.add(side);
            startSolePoses.get(side).setIncludingFrame(endSolePoses.get(side));
         }

         for (int transitioningFootNumber = 0; transitioningFootNumber < stepTransition.getNumberOfFeetInTransition(); transitioningFootNumber++)
         {
            RobotSide transitionSide = stepTransition.getTransitionSide(transitioningFootNumber);
            switch (stepTransition.getTransitionType(transitioningFootNumber))
            {
            case LIFT_OFF:
               endFeetInContact.remove(transitionSide);
               break;
            case TOUCH_DOWN:
               endFeetInContact.add(transitionSide);
               endSolePoses.get(transitionSide).setIncludingFrame(stepTransition.transitionPose(transitionSide));
               break;
            }
         }

         // end the previous phase and add a new one
         contactPhase.getTimeInterval().setEndTime(stepTransition.getTransitionTime());
         contactPhase = contactSequenceInAbsoluteTime.add();

         contactPhase.reset();
         contactPhase.setFeetInContact(endFeetInContact);
         for (int i = 0; i < startFeetInContact.size(); i++)
            contactPhase.addStartFoot(startFeetInContact.get(i), startSolePoses.get(startFeetInContact.get(i)));
         for (int i = 0; i < endFeetInContact.size(); i++)
            contactPhase.addStartFoot(endFeetInContact.get(i), startSolePoses.get(endFeetInContact.get(i)));
         contactPhase.getTimeInterval().setStartTime(stepTransition.getTransitionTime());
         contactPhase.update();

         boolean isLastContact = (transitionNumber == numberOfTransitions - 1) || (contactSequenceInAbsoluteTime.size() == maxCapacity);
         if (isLastContact)
            break;
      }

      contactPhase.getTimeInterval().setEndTime(Double.POSITIVE_INFINITY);

   }

}
