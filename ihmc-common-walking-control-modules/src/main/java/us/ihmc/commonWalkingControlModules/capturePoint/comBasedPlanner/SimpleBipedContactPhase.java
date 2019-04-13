package us.ihmc.commonWalkingControlModules.capturePoint.comBasedPlanner;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactState;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.TimeInterval;
import us.ihmc.robotics.time.TimeIntervalBasics;

import java.util.ArrayList;
import java.util.List;

/**
 * This is the an implementation fo {@link ContactStateProvider} for bipeds. It could be used to compute fancy CoP locations, if desired.
 */
public class SimpleBipedContactPhase implements ContactStateProvider
{
   private final TimeInterval timeInterval = new TimeInterval();
   private final List<RobotSide> feetInContact = new ArrayList<>();
   private ContactState contactState = ContactState.IN_CONTACT;

   private final List<RobotSide> startFeet = new ArrayList<>();
   private final List<RobotSide> endFeet = new ArrayList<>();
   private final SideDependentList<FramePose3D> startFootPoses = new SideDependentList<>();
   private final SideDependentList<FramePose3D> endFootPoses = new SideDependentList<>();

   private final FramePoint3D startCopPosition = new FramePoint3D();
   private final FramePoint3D endCopPosition = new FramePoint3D();

   private boolean isUpToDate = false;

   public SimpleBipedContactPhase()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         startFootPoses.put(robotSide, new FramePose3D());
         endFootPoses.put(robotSide, new FramePose3D());
      }
   }

   @Override
   public TimeIntervalBasics getTimeInterval()
   {
      return timeInterval;
   }

   @Override
   public FramePoint3DReadOnly getCopStartPosition()
   {
      if (!isUpToDate)
         throw new RuntimeException("The CoP positions are not up to date.");

      return startCopPosition;
   }

   @Override
   public FramePoint3DReadOnly getCopEndPosition()
   {
      if (!isUpToDate)
         throw new RuntimeException("The CoP positions are not up to date.");

      return endCopPosition;
   }

   @Override
   public ContactState getContactState()
   {
      return contactState;
   }

   public List<RobotSide> getFeetInContact()
   {
      return feetInContact;
   }

   public void reset()
   {
      feetInContact.clear();
      startFeet.clear();
      endFeet.clear();
      startCopPosition.setToNaN();
      endCopPosition.setToNaN();
      for (RobotSide robotSide : RobotSide.values)
      {
         startFootPoses.get(robotSide).setToNaN();
         endFootPoses.get(robotSide).setToNaN();
      }

      isUpToDate = false;
   }

   public void resetEnd()
   {
      endFeet.clear();
      endCopPosition.setToNaN();
      for (RobotSide robotSide : RobotSide.values)
      {
         endFootPoses.get(robotSide).setToNaN();
      }

      isUpToDate = false;
   }

   public void set(SimpleBipedContactPhase other)
   {
      reset();
      setFeetInContact(other.feetInContact);
      getTimeInterval().set(other.timeInterval);
      for (int i = 0; i < other.startFeet.size(); i++)
         addStartFoot(other.startFeet.get(i), other.startFootPoses.get(other.startFeet.get(i)));
      for (int i = 0; i < other.endFeet.size(); i++)
         addEndFoot(other.endFeet.get(i), other.endFootPoses.get(other.endFeet.get(i)));

      isUpToDate = false;

      update();
   }

   public void setFeetInContact(List<RobotSide> feetInContact)
   {
      if (feetInContact.size() > 2)
         throw new IllegalArgumentException("There can't be more than 2 feet in contact for a biped.");

      this.feetInContact.clear();
      for (int i = 0; i < feetInContact.size(); i++)
      {
         this.feetInContact.add(feetInContact.get(i));
      }

      isUpToDate = false;
   }

   public void addStartFoot(RobotSide robotSide, FramePose3DReadOnly pose)
   {
      if (startFeet.contains(robotSide))
         throw new RuntimeException("Already contains this.");
      startFeet.add(robotSide);
      startFootPoses.get(robotSide).setMatchingFrame(pose);

      isUpToDate = false;
   }

   public void addEndFoot(RobotSide robotSide, FramePose3DReadOnly pose)
   {
      if (endFeet.contains(robotSide))
         throw new RuntimeException("Already contains this.");
      endFeet.add(robotSide);
      endFootPoses.get(robotSide).setMatchingFrame(pose);

      isUpToDate = false;
   }

   public void setStartFootPoses(SideDependentList<? extends FramePose3DReadOnly> poses)
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         startFootPoses.get(robotSide).setMatchingFrame(poses.get(robotSide));
      }

      isUpToDate = false;
   }

   private final FramePoint3D tempPoint = new FramePoint3D();

   public void update()
   {
      if (feetInContact.isEmpty())
      {
         contactState = ContactState.FLIGHT;
      }
      else
      {
         contactState = ContactState.IN_CONTACT;

         startCopPosition.setToZero();
         for (int i = 0; i < startFeet.size(); i++)
         {
            tempPoint.setIncludingFrame(startFootPoses.get(startFeet.get(i)).getPosition());
            tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
            startCopPosition.add(tempPoint);
         }
         startCopPosition.scale(1.0 / startFeet.size());

         endCopPosition.setToZero();
         for (int i = 0; i < endFeet.size(); i++)
         {
            tempPoint.setIncludingFrame(endFootPoses.get(endFeet.get(i)).getPosition());
            tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
            endCopPosition.add(tempPoint);
         }
         endCopPosition.scale(1.0 / endFeet.size());
      }

      isUpToDate = true;
   }
}
