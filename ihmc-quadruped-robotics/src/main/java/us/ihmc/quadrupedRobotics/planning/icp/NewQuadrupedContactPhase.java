package us.ihmc.quadrupedRobotics.planning.icp;

import org.apache.commons.lang3.mutable.MutableDouble;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.quadrupedBasics.gait.TimeInterval;
import us.ihmc.quadrupedBasics.gait.TimeIntervalProvider;
import us.ihmc.quadrupedRobotics.planning.ContactState;
import us.ihmc.quadrupedRobotics.planning.QuadrupedCenterOfPressureTools;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;

import java.util.ArrayList;
import java.util.List;

public class NewQuadrupedContactPhase
{
   private final TimeInterval timeInterval = new TimeInterval();
   private final List<RobotQuadrant> feetInContact = new ArrayList<>();
   private final FramePoint3D copPosition = new FramePoint3D();
   private ContactState contactState = ContactState.IN_CONTACT;

   private final QuadrantDependentList<FramePoint3D> solePosition = new QuadrantDependentList<>();
   private final QuadrantDependentList<MutableDouble> normalizedContactPressures = new QuadrantDependentList<>();

   public NewQuadrupedContactPhase()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         feetInContact.add(robotQuadrant);
         solePosition.put(robotQuadrant, new FramePoint3D());
         normalizedContactPressures.put(robotQuadrant, new MutableDouble(0.0));
      }
   }

   public void setTimeInterval(TimeInterval timeInterval)
   {
      this.timeInterval.set(timeInterval);
   }

   public void setFeetInContact(List<RobotQuadrant> feetInContact)
   {
      this.feetInContact.clear();
      for (int i = 0; i < feetInContact.size(); i++)
         this.feetInContact.add(feetInContact.get(i));
   }

   public void setSolePosition(QuadrantDependentList<FramePoint3D> solePosition)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         this.solePosition.get(robotQuadrant).setIncludingFrame(solePosition.get(robotQuadrant));
         this.solePosition.get(robotQuadrant).changeFrame(ReferenceFrame.getWorldFrame());
      }
   }

   public void set(NewQuadrupedContactPhase other)
   {
      setTimeInterval(other.getTimeInterval());
      setFeetInContact(other.getFeetInContact());
      setSolePosition(other.getSolePosition());

      update();
   }

   public void update()
   {
      if (feetInContact.isEmpty())
         contactState = ContactState.NO_CONTACT;
      else
         contactState = ContactState.IN_CONTACT;

      QuadrupedCenterOfPressureTools.computeNominalNormalizedContactPressure(normalizedContactPressures, feetInContact);
      QuadrupedCenterOfPressureTools.computeCenterOfPressure(copPosition, solePosition, normalizedContactPressures);
   }

   public TimeInterval getTimeInterval()
   {
      return timeInterval;
   }

   public List<RobotQuadrant> getFeetInContact()
   {
      return feetInContact;
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


}
