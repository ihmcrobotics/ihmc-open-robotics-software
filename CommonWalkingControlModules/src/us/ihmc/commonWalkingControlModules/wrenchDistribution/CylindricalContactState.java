package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import us.ihmc.utilities.math.geometry.ReferenceFrame;

public interface CylindricalContactState
{
   public boolean isInContact();
   public void setInContact(boolean inContact);
   public double getCylinderRadius();
   public double getHalfHandWidth();
   public double getCoefficientOfFriction();
   public double getTensileGripForce();
   public ReferenceFrame getEndEffectorFrame();
}
