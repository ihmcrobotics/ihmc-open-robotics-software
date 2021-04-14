package us.ihmc.quadrupedRobotics.planning.comPlanning;

import org.apache.commons.lang3.mutable.MutableDouble;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactState;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;
import us.ihmc.quadrupedRobotics.planning.QuadrupedCenterOfPressureTools;
import us.ihmc.robotics.robotSide.QuadrantDependentList;
import us.ihmc.robotics.robotSide.RobotQuadrant;
import us.ihmc.robotics.time.TimeInterval;

import java.util.ArrayList;
import java.util.List;

/**
 * This is the an implementation fo {@link ContactStateProvider} for quadruped. It could be used to compute fancy CoP locations, if desired.
 */
public class QuadrupedContactPhase implements ContactStateProvider<QuadrupedContactPhase>
{
   private final TimeInterval timeInterval = new TimeInterval();
   private final List<RobotQuadrant> feetInContact = new ArrayList<>();
   private final List<String> feetNamesInContact = new ArrayList<>();
   private final FramePoint3D copPosition = new FramePoint3D();
   private final FrameVector3DReadOnly zeroVector = new FrameVector3D();
   private ContactState contactState = ContactState.IN_CONTACT;

   private final QuadrantDependentList<FramePoint3D> solePositions = new QuadrantDependentList<>();
   private final QuadrantDependentList<MutableDouble> normalizedContactPressures = new QuadrantDependentList<>();

   public QuadrupedContactPhase()
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
      {
         feetInContact.add(robotQuadrant);
         feetNamesInContact.add(robotQuadrant.getShortName());
         solePositions.put(robotQuadrant, new FramePoint3D());
         normalizedContactPressures.put(robotQuadrant, new MutableDouble(0.0));
      }
   }

   public void reset()
   {
      feetInContact.clear();
      feetNamesInContact.clear();
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         solePositions.get(robotQuadrant).setToNaN();
      copPosition.setToNaN();
      timeInterval.reset();
   }

   public void setTimeInterval(TimeInterval timeInterval)
   {
      this.timeInterval.set(timeInterval);
   }

   public void setFeetInContact(List<RobotQuadrant> feetInContact)
   {
      if (feetInContact.size() > 4)
         throw new IllegalArgumentException("There can't be more than 4 feet in contact for a quadruped.");

      this.feetInContact.clear();
      this.feetNamesInContact.clear();
      for (int i = 0; i < feetInContact.size(); i++)
      {
         this.feetInContact.add(feetInContact.get(i));
         this.feetNamesInContact.add(feetInContact.get(i).getShortName());
      }
   }

   public void setSolePositions(QuadrantDependentList<? extends FramePoint3DReadOnly> solePosition)
   {
      for (RobotQuadrant robotQuadrant : RobotQuadrant.values)
         this.solePositions.get(robotQuadrant).setMatchingFrame(solePosition.get(robotQuadrant));
   }

   public void set(QuadrupedContactPhase other)
   {
      setTimeInterval(other.getTimeInterval());
      setFeetInContact(other.feetInContact);
      setSolePositions(other.solePositions);

      update();
   }

   public void update()
   {
      if (feetInContact.isEmpty())
         contactState = ContactState.FLIGHT;
      else
         contactState = ContactState.IN_CONTACT;

      QuadrupedCenterOfPressureTools.computeNominalNormalizedContactPressure(normalizedContactPressures, feetInContact);
      QuadrupedCenterOfPressureTools.computeCenterOfPressure(copPosition, solePositions, normalizedContactPressures);
   }

   @Override
   public TimeInterval getTimeInterval()
   {
      return timeInterval;
   }

   @Override
   public FramePoint3DReadOnly getECMPStartPosition()
   {
      return copPosition;
   }

   @Override
   public FramePoint3DReadOnly getECMPEndPosition()
   {
      return copPosition;
   }

   @Override
   public FrameVector3DReadOnly getECMPStartVelocity()
   {
      return zeroVector;
   }

   @Override
   public FrameVector3DReadOnly getECMPEndVelocity()
   {
      return zeroVector;
   }

   @Override
   public ContactState getContactState()
   {
      return contactState;
   }

   public List<RobotQuadrant> getFeetInContact()
   {
      return feetInContact;
   }

   public List<String> getBodiesInContact()
   {
      return feetNamesInContact;
   }

   public FramePoint3DReadOnly getSolePosition(RobotQuadrant robotQuadrant)
   {
      return solePositions.get(robotQuadrant);
   }
}
