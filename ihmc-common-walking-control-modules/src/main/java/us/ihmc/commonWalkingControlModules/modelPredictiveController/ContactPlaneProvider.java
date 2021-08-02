package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactState;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateBasics;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
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

/**
 * This class both implements and extends the {@link ContactStateProvider} to also provide contact planes, the vertices of which are needed to compute the
 * admissible contact forces for the model predictive controller.
 */
public class ContactPlaneProvider implements ContactStateBasics<ContactPlaneProvider>
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

   private int hashCode;
   private int planeHashCode;
   private int timeHashCode;
   private int startHashCode;
   private int startVelHashCode;
   private int endHashCode;
   private int endVelHashCode;

   public ContactPlaneProvider()
   {
      reset();
   }

   /**
    * Resets this contact state, which allows it to be reused for the next contact.
    */
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

      planeHashCode = 0;
      hashCode = -1;
   }

   /**
    * Sets this contact interval from another contact interval, without creating or remembering the other data.
    * @param other
    */
   public void set(ContactPlaneProvider other)
   {
      this.set((ContactStateProvider<?>) other);

      setPlaneProviderId(other.getPlaneProviderId());
      for (int i = 0; i < other.getNumberOfContactPlanes(); i++)
         addContact(other.getContactPose(i), other.getContactsInBodyFrame(i));
   }

   /**
    * Sets the plane provider Id.
    * @param planeProviderId id
    */
   public void setPlaneProviderId(int planeProviderId)
   {
      this.planeProviderId = planeProviderId;
   }

   /**
    * Gets the potentially unique Id for this segment.
    * @return Id
    */
   public int getPlaneProviderId()
   {
      return planeProviderId;
   }

   /**
    * Sets the desired eCMP location at the beginning of this time interval.
    * @param startECMPPosition desired starting eCMP position to set.
    */
   public void setStartECMPPosition(FramePoint3DReadOnly startECMPPosition)
   {
      this.startECMPPosition.set(startECMPPosition);
      startHashCode = ContactPlaneProviderTools.computePointHashCode(this.startECMPPosition);
   }

   /**
    * Sets the desired eCMP location at the beginning of this time interval.
    * @param startECMPPosition desired starting eCMP position to set in XY
    * @param height height of the desired eCMP starting location.
    */
   public void setStartECMPPosition(FramePoint2DReadOnly startECMPPosition, double height)
   {
      this.startECMPPosition.set(startECMPPosition, height);
      startHashCode = ContactPlaneProviderTools.computePointHashCode(this.startECMPPosition);
   }

   /**
    * Sets the desired eCMP location at the beginning of this time interval.
    * @param startECMPPosition desired starting eCMP position to set in XY
    * @param height height of the desired eCMP starting location.
    */
   public void setStartECMPPosition(Point2DReadOnly startECMPPosition, double height)
   {
      this.startECMPPosition.set(startECMPPosition, height);
      startHashCode = ContactPlaneProviderTools.computePointHashCode(this.startECMPPosition);
   }

   /**
    * Sets the desired eCMP velocity at the beginning of this time interval.
    * @param startECMPVelocity desired starting eCMP velocity
    */
   public void setStartECMPVelocity(FrameVector3DReadOnly startECMPVelocity)
   {
      this.startECMPVelocity.set(startECMPVelocity);
      startVelHashCode = ContactPlaneProviderTools.computeVectorHashCode(this.startECMPVelocity);
   }

   /**
    * Sets the desired eCMP location at the end of this time interval.
    * @param endECMPPosition desired ending eCMP position to set.
    */
   public void setEndECMPPosition(FramePoint3DReadOnly endECMPPosition)
   {
      this.endECMPPosition.set(endECMPPosition);
      endHashCode = ContactPlaneProviderTools.computePointHashCode(this.endECMPPosition);
   }

   /**
    * Sets the desired eCMP location at the end of this time interval.
    * @param endECMPPosition desired ending eCMP position to set in XY
    * @param height height of the desired eCMP ending location.
    */
   public void setEndECMPPosition(FramePoint2DReadOnly endECMPPosition, double height)
   {
      this.endECMPPosition.set(endECMPPosition, height);
      endHashCode = ContactPlaneProviderTools.computePointHashCode(this.endECMPPosition);
   }

   /**
    * Sets the desired eCMP location at the end of this time interval.
    * @param endECMPPosition desired ending eCMP position to set in XY
    * @param height height of the desired eCMP ending location.
    */
   public void setEndECMPPosition(Point2DReadOnly endECMPPosition, double height)
   {
      this.endECMPPosition.set(endECMPPosition, height);
      endHashCode = ContactPlaneProviderTools.computePointHashCode(this.endECMPPosition);
   }

   /**
    * Sets the desired eCMP velocity at the end of this time interval.
    * @param endECMPVelocity desired end eCMP velocity
    */
   public void setEndECMPVelocity(FrameVector3DReadOnly endECMPVelocity)
   {
      this.endECMPVelocity.set(endECMPVelocity);
      endVelHashCode = ContactPlaneProviderTools.computeVectorHashCode(this.endECMPVelocity);
   }

   /**
    * Sets the start and end eCMP velocity so that it forms a linear function. Just takes the difference between {@link #getECMPEndPosition()} and
    * {@link #getECMPStartPosition()} and scales this by the time {@link #getTimeInterval()} duration.
    */
   public void setLinearECMPVelocity()
   {
      startECMPVelocity.sub(getECMPEndPosition(), getECMPStartPosition());
      startECMPVelocity.scale(1.0 / Math.min(getTimeInterval().getDuration(), 10.0));
      endECMPVelocity.set(startECMPVelocity);
      startVelHashCode = ContactPlaneProviderTools.computeVectorHashCode(this.startECMPVelocity);
      endVelHashCode = startVelHashCode;
   }

   /**
    * Sets the time interval for this contact phase
    * @param timeInterval interval to set
    */
   public void setTimeInterval(TimeIntervalReadOnly timeInterval)
   {
      this.timeInterval.set(timeInterval);
      timeHashCode = ContactPlaneProviderTools.computeTimeHashCode(this.timeInterval);
   }

   /**
    * Sets the {@link ContactState} for this contact phase. Can be either in contact or in flight
    * @param contactState contact state to set.
    */
   public void setContactState(ContactState contactState)
   {
      this.contactState = contactState;
   }

   /**
    * Gets the desired eCMP position at the start of this contact phase.
    */
   public FramePoint3DReadOnly getECMPStartPosition()
   {
      return startECMPPosition;
   }

   /**
    * Gets the desired eCMP position at the end of this contact phase.
    */
   public FramePoint3DReadOnly getECMPEndPosition()
   {
      return endECMPPosition;
   }

   /**
    * Gets the desired eCMP velocity at the start of this contact phase. If this velocity has not been set, this method sets it to linear and returns that value.
    */
   public FrameVector3DReadOnly getECMPStartVelocity()
   {
      if (startECMPVelocity.containsNaN())
         setLinearECMPVelocity();
      return startECMPVelocity;
   }

   /**
    * Gets the desired eCMP velocity at the end of this contact phase. If this velocity has not been set, this method sets it to linear and returns that value.
    */
   public FrameVector3DReadOnly getECMPEndVelocity()
   {
      if (endECMPVelocity.containsNaN())
         setLinearECMPVelocity();
      return endECMPVelocity;
   }

   /**
    * Gets the {@link ContactState}, either flight or support, for this phase.
    * @return contact state
    */
   public ContactState getContactState()
   {
      return contactState;
   }

   /**
    * Gets the time interval over which this contact phase occurs
    * @return time interval
    */
   public TimeIntervalBasics getTimeInterval()
   {
      return timeInterval;
   }

   /**
    * Convenience function for setting the duration of this phase, assuming the start time is set correctly.
    * @param duration desired duration
    */
   public void setDuration(double duration)
   {
      setEndTime(getTimeInterval().getStartTime() + duration);
   }

   /**
    * Sets the start time for this contact phase.
    * @param startTime start time
    */
   public void setStartTime(double startTime)
   {
      getTimeInterval().setStartTime(startTime);
      timeHashCode = ContactPlaneProviderTools.computeTimeHashCode(this.timeInterval);
   }

   /**
    * Sets the end time for this contact phase.
    * @param endTime end time
    */
   public void setEndTime(double endTime)
   {
      getTimeInterval().setEndTime(endTime);
      timeHashCode = ContactPlaneProviderTools.computeTimeHashCode(this.timeInterval);
   }

   /**
    * Adds a pose and contact point locations in the pose frame that are in contact during this contact phase
    * @param contactPose pose in which the contact points are expressed
    * @param contactPointsInBodyFrame contact points in the local frame for this contact.
    */
   public void addContact(FramePose3DReadOnly contactPose, ConvexPolygon2DReadOnly contactPointsInBodyFrame)
   {
      this.contactPoses.add(contactPose);
      this.contactPointsInBodyFrame.add(contactPointsInBodyFrame);
      totalContactPoints += contactPointsInBodyFrame.getNumberOfVertices();
      planeHashCode += ContactPlaneProviderTools.prime * ContactPlaneProviderTools.computePlaneHashCode(contactPose, contactPointsInBodyFrame);
   }

   /**
    * Sets the initial state of this contact phase from the end of the previous one. That is, it sets the desired ECMP start position and the start time.
    * @param previousContactState previous contact phase to continue.
    */
   public void setStartFromEnd(ContactStateProvider previousContactState)
   {
      setStartTime(previousContactState.getTimeInterval().getEndTime());
      setStartECMPPosition(previousContactState.getECMPEndPosition());
   }

   /**
    * Gets the total number of contacting planes for this contact phase
    * @return number of contact planes
    */
   public int getNumberOfContactPlanes()
   {
      return contactPointsInBodyFrame.size();
   }

   /**
    * Gets the total number of contact points for this contact phase. Note that these contact points do not necessarily represent the minimal set of contact
    * points needed for the same contact wrench cone. That is, the convex hull of these points may not have the same number of vertices as returned.
    * @return total number of contact points
    */
   public int getTotalNumberOfPointsInContact()
   {
      return totalContactPoints;
   }

   /**
    * Returns the number of contact points contained in the plane index queried
    * @param contactPlane plane index to query
    * @return number of contact points in that plane
    */
   public int getNumberOfContactPointsInPlane(int contactPlane)
   {
      return contactPointsInBodyFrame.get(contactPlane).getNumberOfVertices();
   }

   /**
    * Returns the vertices of the contact plane queried. The vertices are expressed relative to the contact plane.
    * @param contactPlane plane index to query.
    * @return contact points in the plane.
    */
   public ConvexPolygon2DReadOnly getContactsInBodyFrame(int contactPlane)
   {
      return contactPointsInBodyFrame.get(contactPlane);
   }

   /**
    * Gets the pose of the contact queried.
    * @param contactPlane plane index to query
    * @return pose of the plane
    */
   public FramePose3DReadOnly getContactPose(int contactPlane)
   {
      return contactPoses.get(contactPlane);
   }

   @Override
   public int hashCode()
   {
      if (hashCode == -1)
      {
         hashCode = ContactPlaneProviderTools.computePlaneProviderHashCode(planeHashCode, timeHashCode, startHashCode, startVelHashCode, endHashCode, endVelHashCode);
      }
      return hashCode;
   }

   @Override
   public boolean equals(Object obj)
   {
      if (this == obj)
         return true;
      if (obj == null)
         return false;
      if (getClass() != obj.getClass())
         return false;
      ContactPlaneProvider other = (ContactPlaneProvider) obj;

      return equals(this, other);
   }

   private static final double timeEpsilon = 1e-2;
   private static final double positionEpsilon = 5e-3;

   private static boolean equals(ContactPlaneProvider planeA, ContactPlaneProvider planeB)
   {
      return epsilonEquals(planeA, planeB, timeEpsilon, positionEpsilon);
   }

   public static boolean epsilonEquals(ContactPlaneProvider planeA, ContactPlaneProvider planeB, double timeEpsilon, double positionEpsilon)
   {
      if (planeA == null || planeB == null)
         return false;

      if (!MathTools.epsilonEquals(planeA.getTimeInterval().getEndTime(), planeB.getTimeInterval().getEndTime(), timeEpsilon))
         return false;

      if (planeA.getContactState() != planeB.getContactState())
         return false;

      if (planeA.getContactState() == ContactState.FLIGHT)
         return true;

      if (planeA.getNumberOfContactPlanes() != planeB.getNumberOfContactPlanes())
         return false;

      for (int i = 0; i < planeA.getNumberOfContactPlanes(); i++)
      {
         if (planeA.getNumberOfContactPointsInPlane(i) != planeB.getNumberOfContactPointsInPlane(i))
            return false;

         if (planeA.getContactPose(i).getPosition().distanceXYSquared(planeB.getContactPose(i).getPosition()) >  positionEpsilon * positionEpsilon)
            return false;
      }

      return true;
   }


}
