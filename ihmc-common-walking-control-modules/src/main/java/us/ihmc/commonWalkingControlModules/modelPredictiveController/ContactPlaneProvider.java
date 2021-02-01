package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactState;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
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
   private final FramePoint3D startECMPPosition = new FramePoint3D();
   private final FramePoint3D endECMPPosition = new FramePoint3D();
   private final FrameVector3D startECMPVelocity = new FrameVector3D();
   private final FrameVector3D endECMPVelocity = new FrameVector3D();
   private final TimeIntervalBasics timeInterval = new TimeInterval();

   private final List<ConvexPolygon2DReadOnly> contactPointsInBodyFrame = new ArrayList<>();
   private final List<FramePose3DReadOnly> contactPoses = new ArrayList<>();

   private int totalContactPoints = 0;

   public ContactPlaneProvider()
   {
      startECMPPosition.setToNaN();
      endECMPPosition.setToNaN();
      startECMPVelocity.setToNaN();
      endECMPVelocity.setToNaN();
   }

   public void reset()
   {
      planeProviderId = -1;
      startECMPPosition.setToNaN();
      endECMPPosition.setToNaN();
      startECMPVelocity.setToNaN();
      endECMPVelocity.setToNaN();
      totalContactPoints = 0;
      contactPointsInBodyFrame.clear();
      contactPoses.clear();
   }

   public void set(ContactPlaneProvider other)
   {
      reset();
      setPlaneProviderId(other.getPlaneProviderId());
      setStartECMPPosition(other.getECMPStartPosition());
      setEndECMPPosition(other.getECMPEndPosition());
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

   public void setStartECMPPosition(FramePoint3DReadOnly startECMPPosition)
   {
      this.startECMPPosition.set(startECMPPosition);
   }

   public void setStartECMPPosition(FramePoint2DReadOnly startECMPPosition, double height)
   {
      this.startECMPPosition.set(startECMPPosition, height);
   }

   public void setStartECMPPosition(Point2DReadOnly startECMPPosition, double height)
   {
      this.startECMPPosition.set(startECMPPosition, height);
   }

   public void setStartECMPVelocity(FrameVector3DReadOnly startECMPVelocity)
   {
      this.startECMPVelocity.set(startECMPVelocity);
   }

   public void setEndECMPPosition(FramePoint3DReadOnly endECMPPosition)
   {
      this.endECMPPosition.set(endECMPPosition);
   }

   public void setEndECMPPosition(FramePoint2DReadOnly endECMPPosition, double height)
   {
      this.endECMPPosition.set(endECMPPosition, height);
   }

   public void setEndECMPPosition(Point2DReadOnly endECMPPosition, double height)
   {
      this.endECMPPosition.set(endECMPPosition, height);
   }

   public void setEndECMPVelocity(FrameVector3DReadOnly endECMPVelocity)
   {
      this.endECMPVelocity.set(endECMPVelocity);
   }

   public void setLinearECMPVelocity()
   {
      startECMPVelocity.sub(getECMPEndPosition(), getECMPStartPosition());
      startECMPVelocity.scale(1.0 / Math.min(getTimeInterval().getDuration(), 10.0));
      endECMPVelocity.set(startECMPVelocity);
   }

   public void setTimeInterval(TimeIntervalReadOnly timeInterval)
   {
      this.timeInterval.set(timeInterval);
   }

   public void setContactState(ContactState contactState)
   {
      this.contactState = contactState;
   }

   public FramePoint3DReadOnly getECMPStartPosition()
   {
      return startECMPPosition;
   }

   public FramePoint3DReadOnly getECMPEndPosition()
   {
      return endECMPPosition;
   }

   public FrameVector3DReadOnly getECMPStartVelocity()
   {
      if (startECMPVelocity.containsNaN())
         setLinearECMPVelocity();
      return startECMPVelocity;
   }

   public FrameVector3DReadOnly getECMPEndVelocity()
   {
      if (endECMPVelocity.containsNaN())
         setLinearECMPVelocity();
      return endECMPVelocity;
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
      setStartECMPPosition(previousContactState.getECMPEndPosition());
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
