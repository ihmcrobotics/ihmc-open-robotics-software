package us.ihmc.humanoidBehaviors.tools.perception;

import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionsListCutTool;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.SpiralBasedAlgorithm;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashSet;
import java.util.List;

public class PlanesCuttingDepthSensorSimulator
{
   private final ReferenceFrame cameraFrame;
   private final FOVPlanesCalculator fovPlanesCalculator;

   public PlanesCuttingDepthSensorSimulator(ReferenceFrame cameraFrame)
   {
      this(Double.NaN, Double.NaN, Double.POSITIVE_INFINITY, cameraFrame);
   }

   public PlanesCuttingDepthSensorSimulator(double verticalFOV, double horizontalFOV, ReferenceFrame cameraFrame)
   {
      this(verticalFOV, horizontalFOV, Double.POSITIVE_INFINITY, cameraFrame);
   }

   public PlanesCuttingDepthSensorSimulator(double verticalFOV, double horizontalFOV, double range, ReferenceFrame cameraFrame)
   {
      this.cameraFrame = cameraFrame;

      fovPlanesCalculator = new FOVPlanesCalculator(verticalFOV, horizontalFOV, cameraFrame);
   }

   public synchronized PlanarRegionsList filterUsingPlaneCutting(PlanarRegionsList map)
   {
      // TODO consider prefilter with view distance
      // List<PlanarRegion> filteredRegions = PlanarRegionTools
      //            .filterPlanarRegionsWithBoundingCircle(new Point2D(observer), Math.sqrt(rayLengthSquared), regions.getPlanarRegionsAsList());

      fovPlanesCalculator.update();

      map = PlanarRegionsListCutTool.cutByPlane(fovPlanesCalculator.getTopPlane(), map);
      map = PlanarRegionsListCutTool.cutByPlane(fovPlanesCalculator.getBottomPlane(), map);
      map = PlanarRegionsListCutTool.cutByPlane(fovPlanesCalculator.getLeftPlane(), map);
      map = PlanarRegionsListCutTool.cutByPlane(fovPlanesCalculator.getRightPlane(), map);


      // TODO: Remove occlusions
      // todo: extract into tool

      // filter out invisible regions
      FramePose3D cameraPose = new FramePose3D(cameraFrame);
      cameraPose.changeFrame(ReferenceFrame.getWorldFrame());
      FixedFramePoint3DBasics cameraPosition = cameraPose.getPosition();

      int rays = 2000;
      List<Point3D> pointsOnSphere = Arrays.asList(SpiralBasedAlgorithm.generatePointsOnSphere(cameraPosition, 1.0, rays));
      HashSet<PlanarRegion> filteredRegions = new HashSet<>(); // count intersections?
      Vector3D rayDirection = new Vector3D();
      for (Point3D pointOnSphere : pointsOnSphere)
      {
         rayDirection.sub(pointOnSphere, cameraPosition);
         ImmutablePair<Point3D, PlanarRegion> intersectionPair = PlanarRegionTools.intersectRegionsWithRay(map, cameraPosition, rayDirection);
         if (intersectionPair != null)
         {
            filteredRegions.add(intersectionPair.getRight());
         }
      }

      map = new PlanarRegionsList(new ArrayList<>(filteredRegions));

      // find visible points
      //      for (PlanarRegion planarRegion : visibleMap.getPlanarRegionsAsList())
      //      {
      //         planarRegion.
      //         for (ConvexPolygon2D convexPolygon : planarRegion.getConvexPolygons())
      //         {
      //            for (int i = 0; i < convexPolygon.getNumberOfVertices(); i++)
      //            {
      //               Point2DReadOnly vertex = convexPolygon.getVertex(i);
      //            }
      //         }
      //      }

      return map;
   }
}
