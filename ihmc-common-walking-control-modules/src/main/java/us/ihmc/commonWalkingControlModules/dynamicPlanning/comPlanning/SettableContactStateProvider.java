package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameTuple3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.robotics.time.TimeInterval;
import us.ihmc.robotics.time.TimeIntervalBasics;
import us.ihmc.robotics.time.TimeIntervalReadOnly;

/**
 * This is the most basic implementation fo a contact state provider for the {@link CoMTrajectoryPlannerInterface}. This is really useful for visualizing
 * what will happen for certain sequences, as it allows the user to directly specify where the start and end ECMP positions are and the contact time intervals.
 */
public class SettableContactStateProvider implements ContactStateBasics<SettableContactStateProvider>
{
   private ContactState contactState = ContactState.IN_CONTACT;
   private final FramePoint3D startECMPPosition = new FramePoint3D();
   private final FramePoint3D endECMPPosition = new FramePoint3D();
   private final FrameVector3D startECMPVelocity = new FrameVector3D();
   private final FrameVector3D endECMPVelocity = new FrameVector3D();
   private final TimeIntervalBasics timeInterval = new TimeInterval();

   private final FrameVector3D externalContactAccelerationStart = new FrameVector3D();
   private final FrameVector3D externalContactAccelerationEnd = new FrameVector3D();

   public SettableContactStateProvider()
   {
      reset();
   }

   public SettableContactStateProvider(SettableContactStateProvider other)
   {
      this();
      set(other);
   }

   public void reset()
   {
      startECMPPosition.setToNaN();
      startECMPVelocity.setToNaN();
      endECMPPosition.setToNaN();
      endECMPVelocity.setToNaN();

      externalContactAccelerationStart.setToNaN();
      externalContactAccelerationEnd.setToNaN();
   }

   public void set(SettableContactStateProvider other)
   {
      this.set((ContactStateProvider<?>) other);
   }

   public void setStartECMPPosition(FramePoint3DReadOnly startECMPPosition)
   {
      this.startECMPPosition.set(startECMPPosition);
   }

   public void setStartECMPVelocity(FrameVector3DReadOnly startECMPVelocity)
   {
      this.startECMPVelocity.set(startECMPVelocity);
   }

   public void setStartECMPPosition(FramePoint2DReadOnly startECMPPosition, double height)
   {
      this.startECMPPosition.checkReferenceFrameMatch(startECMPPosition);
      setStartECMPPosition((Point2DReadOnly) startECMPPosition, height);
   }

   public void setStartECMPPosition(Point2DReadOnly startECMPPosition, double height)
   {
      this.startECMPPosition.set(startECMPPosition, height);
   }

   public void setEndECMPPosition(FramePoint3DReadOnly endECMPPosition)
   {
      this.endECMPPosition.set(endECMPPosition);
   }

   public void setEndECMPVelocity(FrameVector3DReadOnly endECMPVelocity)
   {
      this.endECMPVelocity.set(endECMPVelocity);
   }

   public void setEndECMPPosition(FramePoint2DReadOnly endECMPPosition, double height)
   {
      this.endECMPPosition.checkReferenceFrameMatch(endECMPPosition);
      setEndECMPPosition((Point2DReadOnly) endECMPPosition, height);
   }

   public void setExternalContactAccelerationStart(FrameTuple3DReadOnly desiredAcceleration)
   {
      this.externalContactAccelerationStart.set(desiredAcceleration);
   }

   public void setExternalContactAccelerationEnd(FrameTuple3DReadOnly desiredAcceleration)
   {
      this.externalContactAccelerationEnd.set(desiredAcceleration);
   }

   public void setEndECMPPosition(Point2DReadOnly endECMPPosition, double height)
   {
      this.endECMPPosition.set(endECMPPosition, height);
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

   public void setStartFromEnd(ContactStateProvider<?> previousContactState)
   {
      setStartTime(previousContactState.getTimeInterval().getEndTime());
      setStartECMPPosition(previousContactState.getECMPEndPosition());
   }

   public FrameVector3DReadOnly getExternalContactAccelerationStart()
   {
      return externalContactAccelerationStart;
   }

   public FrameVector3DReadOnly getExternalContactAccelerationEnd()
   {
      return externalContactAccelerationEnd;
   }
}
