package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.util.List;

import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.ReferenceFrame;
import us.ihmc.robotics.screwTheory.RigidBody;

public interface PlaneContactState
{
   public abstract RigidBody getRigidBody();
   public abstract ReferenceFrame getFrameAfterParentJoint();
   public abstract ReferenceFrame getPlaneFrame();
   public abstract boolean inContact();
   public abstract FrameVector getContactNormalFrameVectorCopy();
   public abstract void getContactNormalFrameVector(FrameVector frameVectorToPack);
   public abstract List<FramePoint> getContactFramePointsInContactCopy();
   public abstract void getContactFramePointsInContact(List<FramePoint> contactPointListToPack);
   public abstract List<FramePoint2d> getContactFramePoints2dInContactCopy();
   public abstract double getCoefficientOfFriction();
   public abstract int getNumberOfContactPointsInContact();
   public abstract int getTotalNumberOfContactPoints(); 
   
   public abstract List<? extends ContactPointInterface> getContactPoints();
}
