package us.ihmc.quadrupedRobotics.planning.icp;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.quadrupedBasics.gait.TimeInterval;
import us.ihmc.quadrupedBasics.gait.TimeIntervalProvider;
import us.ihmc.quadrupedBasics.gait.TimeIntervalReadOnly;
import us.ihmc.quadrupedRobotics.planning.ContactState;

public class SettableContactStateProvider implements ContactStateProvider
{
   private ContactState contactState = ContactState.IN_CONTACT;
   private final FramePoint3D copPosition = new FramePoint3D();
   private final TimeInterval timeInterval = new TimeInterval();

   public SettableContactStateProvider()
   {
      copPosition.setToNaN();
   }

   public void setCopPosition(FramePoint3DReadOnly copPosition)
   {
      this.copPosition.set(copPosition);
   }

   public void setTimeInterval(TimeIntervalReadOnly timeInterval)
   {
      this.timeInterval.set(timeInterval);
   }

   public void setContactState(ContactState contactState)
   {
      this.contactState = contactState;
   }

   public FramePoint3DReadOnly getCopPosition()
   {
      return copPosition;
   }

   public ContactState getContactState()
   {
      return contactState;
   }

   public TimeInterval getTimeInterval()
   {
      return timeInterval;
   }

}
