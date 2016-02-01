package us.ihmc.graveYard.commonWalkingControlModules.cylindricalGrasping.bipedSupportPolygons;

import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;


public interface ContactableCylinderBody
{
   public abstract String getName();
   public abstract RigidBody getRigidBody();
   public abstract ReferenceFrame getBodyFrame();
   public abstract ReferenceFrame getCylinderFrame();
   public abstract double getHalfHandWidth();
   public abstract double getCylinderRadius();
   public abstract double getGripStrength();
   public abstract double getGripWeaknessFactor();
}