package us.ihmc.commonWalkingControlModules.controlModules;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.screwTheory.SpatialForceVector;

public interface GroundReactionWrenchDistributorInterface
{

   public abstract void reset();

   public abstract void addContact(PlaneContactState contactState, double coefficientOfFriction, double rotationalCoefficientOfFriction);

   
   //TODO: I think the desiredNetSpatialForceVector should be the total desired wrench on the body from the feet, and therefore include mg in it
   // when this is called, instead of adding mg to it. The GroundReactionWrenchDistributor should not have to be created with m and g.
   // But let's discuss more...
   public abstract void solve(SpatialForceVector desiredNetSpatialForceVector);

   //TODO: Make these be pack methods instead of get methods.
      
   public abstract FramePoint2d getCenterOfPressure(PlaneContactState contactState);

   public abstract FrameVector getForce(PlaneContactState contactState);
   
   public abstract double getNormalTorque(PlaneContactState contactState);

}