package us.ihmc.quadrupedRobotics.planning.icp;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.quadrupedBasics.gait.TimeInterval;
import us.ihmc.quadrupedBasics.gait.TimeIntervalProvider;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedContactPhase implements TimeIntervalProvider
{
   private final TimeInterval timeInterval = new TimeInterval();
   private final QuadrantDependentList<ContactState> contactStates = new QuadrantDependentList<>();
   private final QuadrantDependentList<FramePoint3D> solePosition = new QuadrantDependentList<>();
   private final FramePoint3D copPosition = new FramePoint3D();
   private ContactState contactState = ContactState.IN_CONTACT;

   public QuadrupedContactPhase()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         contactStates.set(robotQuadrant, ContactState.IN_CONTACT);
         solePosition.set(robotQuadrant, new FramePoint3D());
      }
   }

   public void set(QuadrupedContactPhase other)
   {
      setTimeInterval(other.getTimeInterval());
      setContactStates(other.getContactStates());
      setSolePosition(other.getSolePosition());
      setCopPosition(other.getCopPosition());
      setContactState(other.getContactState());
   }

   @Override
   public TimeInterval getTimeInterval()
   {
      return timeInterval;
   }

   public QuadrantDependentList<ContactState> getContactStates()
   {
      return contactStates;
   }

   public QuadrantDependentList<FramePoint3D> getSolePosition()
   {
      return solePosition;
   }

   public FramePoint3DReadOnly getCopPosition()
   {
      return copPosition;
   }

   public ContactState getContactState()
   {
      return contactState;
   }

   public void setTimeInterval(TimeInterval timeInterval)
   {
      this.timeInterval.set(timeInterval);
   }

   public void setContactStates(QuadrantDependentList<ContactState> contactStates)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         this.contactStates.set(robotQuadrant, contactStates.get(robotQuadrant));
      }
   }

   public void setSolePosition(QuadrantDependentList<FramePoint3D> solePosition)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         this.solePosition.get(robotQuadrant).setIncludingFrame(solePosition.get(robotQuadrant));
         this.solePosition.get(robotQuadrant).changeFrame(ReferenceFrame.getWorldFrame());
      }
   }

   public void setCopPosition(FramePoint3DReadOnly copPosition)
   {
      this.copPosition.setIncludingFrame(copPosition);
   }

   public void setContactState(ContactState contactState)
   {
      this.contactState = contactState;
   }
}
