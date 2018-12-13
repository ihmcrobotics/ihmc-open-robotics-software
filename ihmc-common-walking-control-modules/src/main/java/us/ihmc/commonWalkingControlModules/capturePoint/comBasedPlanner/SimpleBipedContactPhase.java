package us.ihmc.commonWalkingControlModules.capturePoint.comBasedPlanner;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactMotion;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactState;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.euclid.referenceFrame.FrameConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;
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

   private final FrameConvexPolygon2D startFootPolygon = new FrameConvexPolygon2D();
   private final FrameConvexPolygon2D nextFootPolygon = new FrameConvexPolygon2D();

   private final FramePoint3D startCopPosition = new FramePoint3D();
   private final FramePoint3D endCopPosition = new FramePoint3D();

   public SimpleBipedContactPhase()
   {
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

   public void reset()
   {
      feetInContact.clear();
      startFootPolygon.clearAndUpdate();
      nextFootPolygon.clearAndUpdate();
      startCopPosition.setToNaN();
      endCopPosition.setToNaN();
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

   public void setStartFootPolygon(FrameConvexPolygon2DReadOnly polygon)
   {
      startFootPolygon.set(polygon);
   }

   public void setNextFootPolygon(FrameConvexPolygon2DReadOnly polygon)
   {
      nextFootPolygon.set(polygon);
   }

   public void update()
   {
      if (feetInContact.isEmpty())
      {
         contactState = ContactState.FLIGHT;
      }
      else if (feetInContact.size() == 1)
      {
         contactState = ContactState.IN_CONTACT;
         FramePoint2DReadOnly startCentroid = startFootPolygon.getCentroid();
         startCopPosition.setIncludingFrame(startCentroid.getReferenceFrame(), startCentroid, 0.0);
         startCopPosition.changeFrame(ReferenceFrame.getWorldFrame());
         endCopPosition.setIncludingFrame(startCopPosition);
      }
      else
      {
         contactState = ContactState.IN_CONTACT;
         FramePoint2DReadOnly startCentroid = startFootPolygon.getCentroid();
         FramePoint2DReadOnly endCentroid = startFootPolygon.getCentroid();
         startCopPosition.setIncludingFrame(startCentroid.getReferenceFrame(), startCentroid, 0.0);
         startCopPosition.changeFrame(ReferenceFrame.getWorldFrame());
         endCopPosition.setIncludingFrame(endCentroid.getReferenceFrame(), endCentroid, 0.0);
         endCopPosition.changeFrame(ReferenceFrame.getWorldFrame());
      }
   }
}
