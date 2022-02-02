package us.ihmc.commonWalkingControlModules.capturePoint.controller;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.BipedSupportPolygons;
import us.ihmc.commons.InterpolationTools;
import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.interfaces.FrameConvexPolygon2DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint2DReadOnly;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class DistanceInsideHeuristics
{
   private final FrameConvexPolygon2DReadOnly supportPolygon;

   private final DoubleProvider maxAllowedDistanceCMPSupport;
   private final DoubleProvider safeCoPDistanceToEdge;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());

   private final DoubleProvider distanceWhenControlIsUseless = new DoubleParameter("distanceWhenCoPControlIsUseless", registry, 0.2);

   private final YoDouble cmpDistanceFromSupport = new YoDouble("cmpDistanceFromSupport", registry);
   private final YoDouble copDistanceInsideSupport = new YoDouble("copDistanceInsideSupport", registry);

   public DistanceInsideHeuristics(FrameConvexPolygon2DReadOnly supportPolygon,
                                   DoubleProvider maxAllowedDistanceCMPSupport,
                                   DoubleProvider safeCoPDistanceToEdge,
                                   YoRegistry parentRegistry)
   {
      this.supportPolygon = supportPolygon;

      this.maxAllowedDistanceCMPSupport = maxAllowedDistanceCMPSupport;
      this.safeCoPDistanceToEdge = safeCoPDistanceToEdge;

      parentRegistry.addChild(registry);
   }

   private final FramePoint2D pointOnEdge = new FramePoint2D();
   private final FrameLine2D lineToIntersect = new FrameLine2D();

   private final FramePoint2D otherPointOfIntersection = new FramePoint2D();

   private final FramePoint2D copOfLeastEffort = new FramePoint2D();

   public void updateDistanceInside(FramePoint2DReadOnly currentICP)
   {
      double distanceFromSupport = supportPolygon.signedDistance(currentICP);
      if (distanceFromSupport <= 0.0)
      {
         cmpDistanceFromSupport.set(maxAllowedDistanceCMPSupport.getValue());
         copDistanceInsideSupport.set(safeCoPDistanceToEdge.getValue());

         copOfLeastEffort.setToNaN();
         return;
      }

      supportPolygon.orthogonalProjection(currentICP, pointOnEdge);

      lineToIntersect.set(pointOnEdge, currentICP);
      int intersections = supportPolygon.intersectionWith(lineToIntersect, pointOnEdge, otherPointOfIntersection);
      double distanceFromCoPOfLeastEffortToEdge;

      if (intersections == 1)
      {
         copOfLeastEffort.set(pointOnEdge);
         distanceFromCoPOfLeastEffortToEdge = 0.0;
      }
      else if (intersections == 2)
      {
         copOfLeastEffort.interpolate(pointOnEdge, otherPointOfIntersection, 0.5);
         distanceFromCoPOfLeastEffortToEdge = 0.5 * pointOnEdge.distance(otherPointOfIntersection);
      }
      else
      {
         throw new RuntimeException("Should never get here.");
      }

      double fractionReductionOfDistance = Math.min(distanceFromSupport / distanceWhenControlIsUseless.getValue(), 1.0);

      copDistanceInsideSupport.set(InterpolationTools.linearInterpolate(safeCoPDistanceToEdge.getValue(), distanceFromCoPOfLeastEffortToEdge, fractionReductionOfDistance));
      cmpDistanceFromSupport.set(InterpolationTools.linearInterpolate(maxAllowedDistanceCMPSupport.getValue(), -distanceFromCoPOfLeastEffortToEdge, fractionReductionOfDistance));
   }

   public double getCoPDistanceInsideSupport()
   {
      return copDistanceInsideSupport.getValue();
   }

   public double getCmpDistanceFromSupport()
   {
      return cmpDistanceFromSupport.getValue();
   }

   public FramePoint2DReadOnly getCoPOfLeastEffort()
   {
      return copOfLeastEffort;
   }
}
