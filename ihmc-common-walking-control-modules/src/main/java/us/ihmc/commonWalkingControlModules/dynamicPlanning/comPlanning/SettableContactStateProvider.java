package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.robotics.time.TimeInterval;
import us.ihmc.robotics.time.TimeIntervalReadOnly;

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

   public FramePoint3DReadOnly getCopStartPosition()
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
