package us.ihmc.robotEnvironmentAwareness.planarRegion;

import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.distanceFromPoint3DToLineSegment3D;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.isPoint2DOnSideOfLine2D;
import static us.ihmc.euclid.geometry.tools.EuclidGeometryTools.signedDistanceFromPoint3DToPlane3D;

import java.util.ArrayList;
import java.util.List;

import us.ihmc.euclid.geometry.*;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullDecomposition;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullTools;
import us.ihmc.robotics.geometry.*;

public class REAPlanarRegionTools
{

   public static List<PlanarRegion> ensureClockwiseOrder(List<PlanarRegion> planarRegions)
   {
      List<PlanarRegion> copies = new ArrayList<>(planarRegions.size());

      for (PlanarRegion planarRegion : planarRegions)
      {
         PlanarRegion copy = planarRegion.copy();
         List<? extends Point2DReadOnly> concaveHullVertices = copy.getConcaveHull();
         ConcaveHullTools.ensureClockwiseOrdering(concaveHullVertices);
         copies.add(copy);
      }

      return copies;
   }

   //TODO: Test this method extensively.
   public static List<PlanarRegion> filterRegionsByTruncatingVerticesBeneathHomeRegion(List<PlanarRegion> regionsToCheck, PlanarRegion homeRegion,
                                                                                       double depthThresholdForConvexDecomposition, PlanarRegionFilter filter)
   {
      List<PlanarRegion> filteredList = new ArrayList<>();
      Point3D pointOnPlane = new Point3D();
      Vector3D planeNormal = new Vector3D();

      homeRegion.getPointInRegion(pointOnPlane);
      homeRegion.getNormal(planeNormal);

      for (PlanarRegion regionToCheck : regionsToCheck)
      {
         PlanarRegion truncatedPlanarRegion = truncatePlanarRegionIfIntersectingWithPlane(pointOnPlane, planeNormal, regionToCheck,
                                                                                          depthThresholdForConvexDecomposition, filter);
         if (truncatedPlanarRegion != null)
            filteredList.add(truncatedPlanarRegion);
      }

      return filteredList;
   }

   /**
    * Truncate the given planar region {@code planarRegionToTuncate} with the plane such that only
    * the part that is <b>above</b> the plane remains.
    *
    * @param pointOnPlane a point on the plane. Not modified.
    * @param planeNormal the normal of the plane. Not modified.
    * @param planarRegionToTruncate the original planar region to be truncated. Not modified.
    * @param depthThresholdForConvexDecomposition used to recompute the convex decomposition of the
    *           planar region when it has been truncated.
    * @param filter the filter used to determine if the truncated region is to be created.
    * @return the truncated planar region which is completely above the plane, or {@code null} if
    *         the given planar region is completely underneath the plane or if it is too small
    *         according to {@code minTruncatedSize} and {@code minTruncatedArea}.
    */
   public static PlanarRegion truncatePlanarRegionIfIntersectingWithPlane(Point3DReadOnly pointOnPlane, Vector3DReadOnly planeNormal,
                                                                          PlanarRegion planarRegionToTruncate, double depthThresholdForConvexDecomposition,
                                                                          PlanarRegionFilter filter)
   {
      Point3D pointOnRegion = new Point3D();
      Vector3D regionNormal = new Vector3D();
      planarRegionToTruncate.getPointInRegion(pointOnRegion);
      planarRegionToTruncate.getNormal(regionNormal);

      if (EuclidGeometryTools.areVector3DsParallel(planeNormal, regionNormal, Math.toRadians(3.0)))
      { // The region and the plane are parallel, check which one is above the other.
         double signedDistance = signedDistanceFromPoint3DToPlane3D(pointOnRegion, pointOnPlane, planeNormal);
         if (signedDistance < 0.0)
            return null; // The region is underneath
         else
            return planarRegionToTruncate; // The region is above
      }

      Point3D pointOnPlaneInRegionFrame = new Point3D(pointOnPlane);
      Vector3D planeNormalInRegionFrame = new Vector3D(planeNormal);

      RigidBodyTransform transformFromRegionToWorld = new RigidBodyTransform();
      planarRegionToTruncate.getTransformToWorld(transformFromRegionToWorld);
      pointOnPlaneInRegionFrame.applyInverseTransform(transformFromRegionToWorld);
      planeNormalInRegionFrame.applyInverseTransform(transformFromRegionToWorld);

      Point2DReadOnly vertex2D = planarRegionToTruncate.getConcaveHullVertex(planarRegionToTruncate.getConcaveHullSize() - 1);
      Point3D vertex3D = new Point3D(vertex2D);
      double previousSignedDistance = signedDistanceFromPoint3DToPlane3D(vertex3D, pointOnPlaneInRegionFrame, planeNormalInRegionFrame);

      Point3D previousVertex3D = vertex3D;

      List<Point2D> truncatedConcaveHullVertices = new ArrayList<>();

      boolean isRegionEntirelyAbove = true;
      double epsilonDistance = 1.0e-10;

      for (int i = 0; i < planarRegionToTruncate.getConcaveHullSize(); i++)
      {
         vertex2D = planarRegionToTruncate.getConcaveHullVertex(i);
         vertex3D = new Point3D(vertex2D);

         double signedDistance = signedDistanceFromPoint3DToPlane3D(vertex3D, pointOnPlaneInRegionFrame, planeNormalInRegionFrame);
         isRegionEntirelyAbove &= signedDistance >= -epsilonDistance;

         if (signedDistance * previousSignedDistance < 0.0)
         {
            if (Math.abs(signedDistance) <= epsilonDistance)
            {
               truncatedConcaveHullVertices.add(new Point2D(vertex2D));
            }
            else if (Math.abs(previousSignedDistance) > epsilonDistance)
            {
               Vector3D edgeDirection = new Vector3D();
               edgeDirection.sub(vertex3D, previousVertex3D);
               Point3D intersection = EuclidGeometryTools.intersectionBetweenLineSegment3DAndPlane3D(pointOnPlaneInRegionFrame, planeNormalInRegionFrame,
                                                                                                     vertex3D, previousVertex3D);

               truncatedConcaveHullVertices.add(new Point2D(intersection));
            }
         }

         if (signedDistance >= -epsilonDistance)
         {
            truncatedConcaveHullVertices.add(new Point2D(vertex2D));
         }

         previousVertex3D = vertex3D;
         previousSignedDistance = signedDistance;
      }

      if (isRegionEntirelyAbove)
         return planarRegionToTruncate;

      if (truncatedConcaveHullVertices.isEmpty())
         return null; // The region is completely underneath

      List<ConvexPolygon2D> truncatedConvexPolygons = new ArrayList<>();
      ConcaveHullDecomposition.recursiveApproximateDecomposition(new ArrayList<>(truncatedConcaveHullVertices), depthThresholdForConvexDecomposition,
                                                                 truncatedConvexPolygons);

      PlanarRegion truncatedRegion = new PlanarRegion(transformFromRegionToWorld, truncatedConcaveHullVertices, truncatedConvexPolygons);
      truncatedRegion.setRegionId(planarRegionToTruncate.getRegionId());
      if (filter == null || filter.isPlanarRegionRelevant(truncatedRegion))
         return truncatedRegion;
      else
         return null;
   }
}
