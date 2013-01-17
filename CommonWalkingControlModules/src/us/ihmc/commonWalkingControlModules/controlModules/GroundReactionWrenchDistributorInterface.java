package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.screwTheory.SpatialForceVector;

public interface GroundReactionWrenchDistributorInterface
{

   public abstract void reset();

   public abstract void addContact(PlaneContactState contactState, double coefficientOfFriction, double rotationalCoefficientOfFriction);

   public abstract void solve(SpatialForceVector desiredNetSpatialForceVector);

   public abstract FramePoint2d getCenterOfPressure(PlaneContactState contactState);

   public abstract double getNormalTorque(PlaneContactState contactState);

   public abstract FrameVector getForce(PlaneContactState contactState);

}