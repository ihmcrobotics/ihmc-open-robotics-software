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
import us.ihmc.humanoidBehaviors.tools.perception.PointCloudPolygonizer;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.SpiralBasedAlgorithm;

/**
 * This class does CPU ray intersections onto planar regions do simulate a depth camera.
 */
public class SimulatedDepthCamera
{
   private final ReferenceFrame cameraFrame;
   private final double verticalFOV;
   private final double horizontalFOV;
   private final double range;
   private final int numberOfPointsToGenerate;
   private final HashMap<PlanarRegion, List<Point3D>> pointsInRegions = new HashMap<>();
   private final FramePose3D tempCameraPose = new FramePose3D();
   private final Point2D tempCircleOrigin = new Point2D();
   private final FOVPlanesCalculator fovPlanesCalculator;
   private final PointCloudPolygonizer polygonizer = new PointCloudPolygonizer();

   public SimulatedDepthCamera(double verticalFOV, double horizontalFOV, double range, ReferenceFrame cameraFrame)
   {
      this(verticalFOV, horizontalFOV, range, 50000, cameraFrame);
   }

   public SimulatedDepthCamera(double verticalFOV,
                               double horizontalFOV,
                               double range,
                               int numberOfPointsToGenerate,
                               ReferenceFrame cameraFrame)
   {
      this.cameraFrame = cameraFrame;
      this.range = range;
      this.numberOfPointsToGenerate = numberOfPointsToGenerate;
      this.verticalFOV = verticalFOV;
      this.horizontalFOV = horizontalFOV;

      fovPlanesCalculator = new FOVPlanesCalculator(verticalFOV, horizontalFOV, cameraFrame);
   }

   public PlanarRegionsList computeAndPolygonize(PlanarRegionsList map)
   {
      return polygonizer.polygonize(computeRegionPointMapFrame(map));
   }

   public Map<Pair<Point3DReadOnly, Vector3DReadOnly>, List<Point3D>> computeRegionPointMapFrame(PlanarRegionsList map)
   {
      compute(map);

      HashMap<Pair<Point3DReadOnly, Vector3DReadOnly>, List<Point3D>> centersNormalsAndPoints = new HashMap<>();
      for (PlanarRegion planarRegion : pointsInRegions.keySet())
      {
         Point3D center = new Point3D();
         planarRegion.getBoundingBox3dInWorld().getCenterPoint(center);
         centersNormalsAndPoints.put(Pair.of(center, new Vector3D(planarRegion.getNormal())), pointsInRegions.get(planarRegion));
      }
      return centersNormalsAndPoints;
   }

   public List<Point3DReadOnly> computePointCloudFrame(PlanarRegionsList map)
   {
      compute(map);

      ArrayList<Point3DReadOnly> pointCloud = new ArrayList<>();
      for (List<Point3D> points : pointsInRegions.values())
      {
         pointCloud.addAll(points);
      }
      return pointCloud;
   }

   private synchronized void compute(PlanarRegionsList map)
   {
      pointsInRegions.clear();

      for (PlanarRegion planarRegion : map.getPlanarRegionsAsList())
      {
         pointsInRegions.put(planarRegion, new ArrayList<>());
      }

      tempCameraPose.setToZero(cameraFrame);
      tempCameraPose.changeFrame(ReferenceFrame.getWorldFrame());

      double sphereRadius = 1.0;
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
   }
}
