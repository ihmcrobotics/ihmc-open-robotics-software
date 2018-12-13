package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.robotics.time.TimeInterval;
import us.ihmc.robotics.time.TimeIntervalReadOnly;

public class SettableContactStateProvider implements ContactStateProvider
{
   private ContactState contactState = ContactState.IN_CONTACT;
   private final FramePoint3D startCopPosition = new FramePoint3D();
   private final TimeInterval timeInterval = new TimeInterval();

   public SettableContactStateProvider()
   {
      startCopPosition.setToNaN();
   }

   public void setStartCopPosition(FramePoint3DReadOnly startCopPosition)
   {
      this.startCopPosition.set(startCopPosition);
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
      return startCopPosition;
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
