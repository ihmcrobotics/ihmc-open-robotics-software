package us.ihmc.humanoidBehaviors.tools;

import java.util.*;

import org.apache.commons.lang3.tuple.ImmutablePair;

import org.apache.commons.lang3.tuple.Pair;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.humanoidBehaviors.tools.perception.FOVPlanesCalculator;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.SpiralBasedAlgorithm;

public class SimulatedDepthCamera
{
   private final ReferenceFrame cameraFrame;
   private final double verticalFOV;
   private final double horizontalFOV;
   private final double range;
   private final HashMap<PlanarRegion, List<Point3D>> pointsInRegions = new HashMap<>();
   private final FramePose3D tempCameraPose = new FramePose3D();
   private final Point2D tempCircleOrigin = new Point2D();
   private final FOVPlanesCalculator fovPlanesCalculator;

   public SimulatedDepthCamera(ReferenceFrame cameraFrame)
   {
      this(Double.NaN, Double.NaN, Double.POSITIVE_INFINITY, cameraFrame);
   }

   public SimulatedDepthCamera(double verticalFOV, double horizontalFOV, ReferenceFrame cameraFrame)
   {
      this(verticalFOV, horizontalFOV, Double.POSITIVE_INFINITY, cameraFrame);
   }

   public SimulatedDepthCamera(double verticalFOV, double horizontalFOV, double range, ReferenceFrame cameraFrame)
   {
      this.cameraFrame = cameraFrame;
      this.range = range;
      this.verticalFOV = verticalFOV;
      this.horizontalFOV = horizontalFOV;

      fovPlanesCalculator = new FOVPlanesCalculator(verticalFOV, horizontalFOV, cameraFrame);
   }

   public synchronized Map<Pair<Point3DReadOnly, Vector3DReadOnly>, List<Point3D>> filterUsingSpherical(PlanarRegionsList map)
   {
      pointsInRegions.clear();

      for (PlanarRegion planarRegion : map.getPlanarRegionsAsList())
      {
         pointsInRegions.put(planarRegion, new ArrayList<>());
      }

      tempCameraPose.setToZero(cameraFrame);
      tempCameraPose.changeFrame(ReferenceFrame.getWorldFrame());

      int numberOfPointsToGenerate = 50000;
      double sphereRadius = 5.0;
      Point3D[] pointsOnSphere = SpiralBasedAlgorithm.generatePointsOnSphere(tempCameraPose.getPosition(), sphereRadius, numberOfPointsToGenerate);

      ArrayList<Point3D> pointsInView = new ArrayList<>();;
      if (!Double.isNaN(verticalFOV) && !Double.isNaN(horizontalFOV))
      {
         fovPlanesCalculator.update();

         for (Point3D point3D : pointsOnSphere)
         {
            if (fovPlanesCalculator.isPointInView(point3D))
            {
               pointsInView.add(point3D);
            }
         }
      }
      else
      {
         pointsInView.addAll(Arrays.asList(pointsOnSphere));
      }

      tempCircleOrigin.set(tempCameraPose.getPosition());
      List<PlanarRegion> filteredRegions = map.getPlanarRegionsAsList();
//      List<PlanarRegion> filteredRegions = PlanarRegionTools.filterPlanarRegionsWithBoundingCircle(tempCircleOrigin, range, map.getPlanarRegionsAsList());

      for (Point3D pointInView : pointsInView)
      {
         Vector3D rayDirection = new Vector3D();
         rayDirection.sub(pointInView, tempCameraPose.getPosition());
         ImmutablePair<Point3D, PlanarRegion> intersectionPair = PlanarRegionTools.intersectRegionsWithRay(filteredRegions,
                                                                                                           tempCameraPose.getPosition(),
                                                                                                           rayDirection);
         if (intersectionPair == null)
         {
            continue;
         }
//         if (intersectionPair.getLeft().distanceSquared(tempCameraPose.getPosition()) > rayLengthSquared)
//         {
//            continue;
//         }

         Point3D intersection = intersectionPair.getLeft();
         PlanarRegion region = intersectionPair.getRight();

         if (intersection.distance(tempCameraPose.getPosition()) > range)
            continue;

         Point3D pointOnPlane = new Point3D(intersection);
         region.transformFromWorldToLocal(pointOnPlane);

         pointsInRegions.get(region).add(intersection);
      }

      HashMap<Pair<Point3DReadOnly, Vector3DReadOnly>, List<Point3D>> centersNormalsAndPoints = new HashMap<>();
      for (PlanarRegion planarRegion : pointsInRegions.keySet())
      {
         Point3D center = new Point3D();
         planarRegion.getBoundingBox3dInWorld().getCenterPoint(center);
         centersNormalsAndPoints.put(Pair.of(center, new Vector3D(planarRegion.getNormal())), pointsInRegions.get(planarRegion));
      }

      return centersNormalsAndPoints;
   }
}
