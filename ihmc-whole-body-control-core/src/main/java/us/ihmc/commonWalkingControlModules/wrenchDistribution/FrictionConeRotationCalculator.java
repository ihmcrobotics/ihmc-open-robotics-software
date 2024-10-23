package us.ihmc.commonWalkingControlModules.wrenchDistribution;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.euclid.geometry.interfaces.ConvexPolygon2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;

public interface FrictionConeRotationCalculator
{
   default double computeConeRotation(YoPlaneContactState yoPlaneContactState, int contactPointIndex)
   {
      return computeConeRotation(yoPlaneContactState.getSupportPolygonInPlaneFrame(), yoPlaneContactState.getContactPoints().get(contactPointIndex));
   }

   double computeConeRotation(ConvexPolygon2DReadOnly supportPolygonInPlaneFrame, Point3DReadOnly contactPoint);
}
