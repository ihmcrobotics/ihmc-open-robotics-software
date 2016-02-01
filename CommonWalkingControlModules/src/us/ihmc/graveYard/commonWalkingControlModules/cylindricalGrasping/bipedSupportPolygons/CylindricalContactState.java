package us.ihmc.graveYard.commonWalkingControlModules.cylindricalGrasping.bipedSupportPolygons;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;

public interface CylindricalContactState
{
   public boolean isInContact();
   public void setInContact(boolean inContact);
   public double getCylinderRadius();
   public double getHalfHandWidth();
   public double getCoefficientOfFriction();
   public double getTensileGripForce();
   public ReferenceFrame getEndEffectorFrame();
   public ReferenceFrame getCylinderFrame();
   public double getGripWeaknessFactor();
}
