package us.ihmc.quadrupedRobotics.planning.comPlanning;

import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.TimedContactInterval;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

public class QuadrupedTimedContactInterval extends TimedContactInterval
{
   private final QuadrantDependentList<ContactState> contactState;
   private final QuadrantDependentList<FramePoint3D> solePosition;

   public QuadrupedTimedContactInterval()
   {
      contactState = new QuadrantDependentList<>();
      solePosition = new QuadrantDependentList<>();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         contactState.set(robotQuadrant, ContactState.IN_CONTACT);
         solePosition.set(robotQuadrant, new FramePoint3D());
      }
   }

   public void set(QuadrupedTimedContactInterval other)
   {
      super.set(other);
      setContactState(other.getContactState());
      setSolePosition(other.getSolePosition());
   }

   public QuadrantDependentList<ContactState> getContactState()
   {
      return contactState;
   }

   public QuadrantDependentList<FramePoint3D> getSolePosition()
   {
      return solePosition;
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
