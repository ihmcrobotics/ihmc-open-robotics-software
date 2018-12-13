package us.ihmc.commonWalkingControlModules.capturePoint.comBasedPlanner;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactMotion;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactState;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.TimeInterval;
import us.ihmc.robotics.time.TimeIntervalBasics;

import java.util.ArrayList;
import java.util.List;

public class SimpleBipedContactPhase implements ContactStateProvider
{
   private final TimeInterval timeInterval = new TimeInterval();
   private final List<RobotSide> feetInContact = new ArrayList<>();
   private ContactState contactState = ContactState.IN_CONTACT;
   private final ContactMotion contactMotion = ContactMotion.LINEAR;

   private final List<RobotSide> startFeet = new ArrayList<>();
   private final List<RobotSide> endFeet = new ArrayList<>();
   private final SideDependentList<FramePose3D> startFootPoses = new SideDependentList<>();
   private final SideDependentList<FramePose3D> nextFootPoses = new SideDependentList<>();



   private final FramePoint3D startCopPosition = new FramePoint3D();
   private final FramePoint3D endCopPosition = new FramePoint3D();

   public SimpleBipedContactPhase()
   {
      for (RobotSide robotSide : RobotSide.values)
      {
         startFootPoses.put(robotSide, new FramePose3D());
         nextFootPoses.put(robotSide, new FramePose3D());
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
      return startCopPosition;
   }

   @Override
   public FramePoint3DReadOnly getCopEndPosition()
   {
      return endCopPosition;
   }

   @Override
   public ContactState getContactState()
   {
      return contactState;
   }

   @Override
   public ContactMotion getContactMotion()
   {
      return contactMotion;
   }

   public List<RobotSide> getFeetInContact()
   {
      return feetInContact;
   }

   public void reset()
   {
      feetInContact.clear();
      startCopPosition.setToNaN();
      endCopPosition.setToNaN();
      for (RobotSide robotSide : RobotSide.values)
      {
         startFootPoses.get(robotSide).setToNaN();
         nextFootPoses.get(robotSide).setToNaN();
      }
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
   }

   public void addStartFoot(RobotSide robotSide, FramePose3DReadOnly pose)
   {
      startFeet.add(robotSide);
      startFootPoses.get(robotSide).setIncludingFrame(pose);
   }

   public void addNextFoot(RobotSide robotSide, FramePose3DReadOnly pose)
   {
      endFeet.add(robotSide);
      nextFootPoses.get(robotSide).setIncludingFrame(pose);
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
         for (int i = 0; i < startFeet.size(); i++)
         {
            tempPoint.setIncludingFrame(nextFootPoses.get(endFeet.get(i)).getPosition());
            tempPoint.changeFrame(ReferenceFrame.getWorldFrame());
            endCopPosition.add(tempPoint);
         }
         endCopPosition.scale(1.0 / endFeet.size());
      }

   }
}
