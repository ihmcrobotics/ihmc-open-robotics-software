package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactState;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DBasics;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.time.TimeInterval;
import us.ihmc.robotics.time.TimeIntervalBasics;
import us.ihmc.robotics.time.TimeIntervalReadOnly;

import java.util.ArrayList;
import java.util.List;

public class ContactPlaneProvider implements ContactStateProvider
{
   private int planeProviderId = -1;
   private ContactState contactState = ContactState.IN_CONTACT;
   private final FramePoint3D startCopPosition = new FramePoint3D();
   private final FramePoint3D endCopPosition = new FramePoint3D();
   private final TimeIntervalBasics timeInterval = new TimeInterval();

   private final List<ConvexPolygon2DReadOnly> contactPointsInBodyFrame = new ArrayList<>();
   private final List<FramePose3DReadOnly> contactPoses = new ArrayList<>();

   private int totalContactPoints = 0;

   public ContactPlaneProvider()
   {
      startCopPosition.setToNaN();
      endCopPosition.setToNaN();
   }

   public void reset()
   {
      planeProviderId = -1;
      startCopPosition.setToNaN();
      endCopPosition.setToNaN();
      totalContactPoints = 0;
      contactPointsInBodyFrame.clear();
      contactPoses.clear();
   }

   public void set(ContactPlaneProvider other)
   {
      reset();
      setPlaneProviderId(other.getPlaneProviderId());
      setStartCopPosition(other.getCopStartPosition());
      setEndCopPosition(other.getCopEndPosition());
      setTimeInterval(other.getTimeInterval());
      setContactState(other.getContactState());
      for (int i = 0; i < other.getNumberOfContactPlanes(); i++)
         addContact(other.getContactPose(i), other.getContactsInBodyFrame(i));
   }

   public void setPlaneProviderId(int planeProviderId)
   {
      this.planeProviderId = planeProviderId;
   }

   public int getPlaneProviderId()
   {
      return planeProviderId;
   }

   public void setStartCopPosition(FramePoint3DReadOnly startCopPosition)
   {
      this.startCopPosition.set(startCopPosition);
   }

   public void setStartCopPosition(FramePoint2DReadOnly startCopPosition)
   {
      this.startCopPosition.set(startCopPosition, 0.0);
   }

   public void setStartCopPosition(Point2DReadOnly startCopPosition)
   {
      this.startCopPosition.set(startCopPosition, 0.0);
   }

   public void setEndCopPosition(FramePoint3DReadOnly endCopPosition)
   {
      this.endCopPosition.set(endCopPosition);
   }

   public void setEndCopPosition(FramePoint2DReadOnly endCopPosition)
   {
      this.endCopPosition.set(endCopPosition, 0.0);
   }

   public void setEndCopPosition(Point2DReadOnly endCopPosition)
   {
      this.endCopPosition.set(endCopPosition, 0.0);
   }

   public void setTimeInterval(TimeIntervalReadOnly timeInterval)
   {
      this.timeInterval.set(timeInterval);
   }

   public void setContactState(ContactState contactState)
   {
      this.contactState = contactState;
   }

   public FramePoint3DReadOnly getCopStartPosition()
   {
      return startCopPosition;
   }

   public FramePoint3DReadOnly getCopEndPosition()
   {
      return endCopPosition;
   }

   public ContactState getContactState()
   {
      return contactState;
   }

   public TimeIntervalBasics getTimeInterval()
   {
      return timeInterval;
   }

   public void setDuration(double duration)
   {
      setEndTime(getTimeInterval().getStartTime() + duration);
   }

   public void setStartTime(double startTime)
   {
      getTimeInterval().setStartTime(startTime);
   }

   public void setEndTime(double endTime)
   {
      getTimeInterval().setEndTime(endTime);
   }

   public void addContact(FramePose3DReadOnly contactPose, ConvexPolygon2DReadOnly contactPointsInBodyFrame)
   {
      this.contactPoses.add(contactPose);
      this.contactPointsInBodyFrame.add(contactPointsInBodyFrame);
      totalContactPoints += contactPointsInBodyFrame.getNumberOfVertices();
   }

   public void setStartFromEnd(ContactStateProvider previousContactState)
   {
      setStartTime(previousContactState.getTimeInterval().getEndTime());
      setStartCopPosition(previousContactState.getCopEndPosition());
   }

   public int getNumberOfContactPlanes()
   {
      return contactPointsInBodyFrame.size();
   }

   public int getTotalNumberOfPointsInContact()
   {
      return totalContactPoints;
   }

   public int getNumberOfContactPointsInPlane(int contactPlane)
   {
      return contactPointsInBodyFrame.get(contactPlane).getNumberOfVertices();
   }

   public ConvexPolygon2DReadOnly getContactsInBodyFrame(int contactPlane)
   {
      return contactPointsInBodyFrame.get(contactPlane);
   }

   public FramePose3DReadOnly getContactPose(int contactPlane)
   {
      return contactPoses.get(contactPlane);
   }
}
