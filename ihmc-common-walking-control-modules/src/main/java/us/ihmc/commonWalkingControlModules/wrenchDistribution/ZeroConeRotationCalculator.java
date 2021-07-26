package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public class ZeroConeRotationCalculator implements FrictionConeRotationCalculator
{
   @Override
   public double computeConeRotation(ConvexPolygon2DReadOnly supportPolygonInPlaneFrame, Point3DReadOnly contactPoint)
   {
      return 0.0;
   }
}
