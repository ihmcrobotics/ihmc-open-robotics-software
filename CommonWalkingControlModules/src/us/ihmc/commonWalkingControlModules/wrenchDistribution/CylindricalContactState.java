package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import us.ihmc.utilities.math.geometry.ReferenceFrame;

public interface CylindricalContactState
{
   public static final double DEFAULT_WRHO = 0.002;
   public static final double DEFAULT_WPHI = 0.002;
   
   public boolean isInContact();
   public void setInContact(boolean inContact);
   public double getCylinderRadius();
   public double getHalfHandWidth();
   public double getCoefficientOfFriction();
   public double getTensileGripForce();
   public ReferenceFrame getEndEffectorFrame();
   public ReferenceFrame getCylinderFrame();
   public double getGripWeaknessFactor();
   
   // TODO: Probably get rid of that. Now, it is used for smooth unload/load transitions in the CarIngressEgressController.
   public void setRhoContactRegularization(double wRho);
   public double getRhoContactRegularization();
   public void setPhiContactRegularization(double wPhi);
   public double getPhiContactRegularization();
   public void resetContactRegularization();
}
