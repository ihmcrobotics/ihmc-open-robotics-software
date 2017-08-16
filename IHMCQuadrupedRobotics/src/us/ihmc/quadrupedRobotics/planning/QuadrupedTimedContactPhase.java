package us.ihmc.quadrupedRobotics.planning;

import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedRobotics.util.TimeInterval;
import us.ihmc.quadrupedRobotics.util.TimeIntervalProvider;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedTimedContactPhase implements TimeIntervalProvider
{
   private final TimeInterval timeInterval;
   private final QuadrantDependentList<ContactState> contactState;
   private final QuadrantDependentList<FramePoint3D> solePosition;

   public QuadrupedTimedContactPhase()
   {
      timeInterval = new TimeInterval();
      contactState = new QuadrantDependentList<>();
      solePosition = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         contactState.set(robotQuadrant, ContactState.IN_CONTACT);
         solePosition.set(robotQuadrant, new FramePoint3D());
      }
   }

   public void set(QuadrupedTimedContactPhase other)
   {
      setTimeInterval(other.getTimeInterval());
      setContactState(other.getContactState());
      setSolePosition(other.getSolePosition());
   }

   @Override
   public TimeInterval getTimeInterval()
   {
      return timeInterval;
   }

   public QuadrantDependentList<ContactState> getContactState()
   {
      return contactState;
   }

   public QuadrantDependentList<FramePoint3D> getSolePosition()
   {
      return solePosition;
   }

   public void setTimeInterval(TimeInterval timeInterval)
   {
      this.timeInterval.set(timeInterval);
   }

   public void setContactState(QuadrantDependentList<ContactState> contactState)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         this.contactState.set(robotQuadrant, contactState.get(robotQuadrant));
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
}
