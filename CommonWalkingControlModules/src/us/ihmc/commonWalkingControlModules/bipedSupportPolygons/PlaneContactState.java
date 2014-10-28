package us.ihmc.commonWalkingControlModules.bipedSupportPolygons;

import java.util.List;

import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.ReferenceFrame;
import us.ihmc.utilities.screwTheory.RigidBody;

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
