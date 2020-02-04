package us.ihmc.humanoidBehaviors.tools;

import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PlanarRegionsListCutTool;
import us.ihmc.robotEnvironmentAwareness.geometry.ConcaveHullFactoryParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionPolygonizer;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.geometry.SpiralBasedAlgorithm;

import java.util.*;

public class SimulatedDepthCamera
{
   enum FilterType { SPHERICAL, PLANE_CUTTING }
   private static final FilterType FILTER_TYPE = FilterType.SPHERICAL;

   private final ReferenceFrame cameraFrame;

   private final double verticalFOV;
   private final double horizontalFOV;

   private final FramePose3D tempFramePose3D = new FramePose3D();
   private final FramePoint3D tempFramePoint3D = new FramePoint3D();
   private final Vector3D tempNormal = new Vector3D();

   private final Plane3D planeTop;
   private final Plane3D planeBottom;
   private final Plane3D planeLeft;
   private final Plane3D planeRight;

   private final ConcaveHullFactoryParameters concaveHullFactoryParameters = new ConcaveHullFactoryParameters();
   private final PolygonizerParameters polygonizerParameters = new PolygonizerParameters();
   private HashMap<PlanarRegion, List<Point3D>> pointsInRegions = new HashMap<>();
   private FramePose3D tempCameraPose = new FramePose3D();
   private Point2D tempCircleOrigin = new Point2D();

   public SimulatedDepthCamera(double verticalFOV, double horizontalFOV, ReferenceFrame cameraFrame)
   {
      this.verticalFOV = verticalFOV;
      this.horizontalFOV = horizontalFOV;
      this.cameraFrame = cameraFrame;

      planeTop = new Plane3D();
      planeBottom = new Plane3D();
      planeLeft = new Plane3D();
      planeRight = new Plane3D();
   }

   public PlanarRegionsList filterMapToVisible(PlanarRegionsList map)
   {
      return FILTER_TYPE == FilterType.SPHERICAL ? filterUsingSpherical(map) : filterUsingPlaneCutting(map);
   }

   public PlanarRegionsList filterUsingSpherical(PlanarRegionsList map)
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

      double rayLength = 5.0;
      double rayLengthSquared = MathTools.square(rayLength);
      tempCircleOrigin.set(tempCameraPose.getPosition());
      List<PlanarRegion> filteredRegions = map.getPlanarRegionsAsList();
//      List<PlanarRegion> filteredRegions = PlanarRegionTools.filterPlanarRegionsWithBoundingCircle(tempCircleOrigin, rayLength, map.getPlanarRegionsAsList());

      for (Point3D pointOnSphere : pointsOnSphere)
      {
         Vector3D rayDirection = new Vector3D();
         rayDirection.sub(pointOnSphere, tempCameraPose.getPosition());
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

         Point3D pointOnPlane = new Point3D(intersection);
         region.transformFromWorldToLocal(pointOnPlane);

         pointsInRegions.get(region).add(intersection);
      }

      List<PlanarRegionSegmentationRawData> segmentationRawData = new ArrayList<>();
      for (PlanarRegion originalRegion : pointsInRegions.keySet())
      {
         List<Point3D> points = pointsInRegions.get(originalRegion);
         Point3D center = new Point3D();
         originalRegion.getBoundingBox3dInWorld().getCenterPoint(center);
         PlanarRegionSegmentationRawData rawData = new PlanarRegionSegmentationRawData(originalRegion.getRegionId(),
                                                                                       originalRegion.getNormal(),
                                                                                       center,
                                                                                       points);
         segmentationRawData.add(rawData);
      }

      return PlanarRegionPolygonizer.createPlanarRegionsList(segmentationRawData, concaveHullFactoryParameters, polygonizerParameters);
   }

   public PlanarRegionsList filterUsingPlaneCutting(PlanarRegionsList map)
   {
      // TODO consider prefilter with view distance
      // List<PlanarRegion> filteredRegions = PlanarRegionTools
      //            .filterPlanarRegionsWithBoundingCircle(new Point2D(observer), Math.sqrt(rayLengthSquared), regions.getPlanarRegionsAsList());

      updatePlaneToFrameWithParameters(planeTop, -verticalFOV / 2.0, 0.0, 0.0, -1.0);
      updatePlaneToFrameWithParameters(planeBottom, verticalFOV / 2.0, 0.0, 0.0, 1.0);
      updatePlaneToFrameWithParameters(planeLeft, 0.0, -horizontalFOV / 2.0, 1.0, 0.0);
      updatePlaneToFrameWithParameters(planeRight, 0.0, horizontalFOV / 2.0, -1.0, 0.0);

      map = PlanarRegionsListCutTool.cutByPlane(planeTop, map);
      map = PlanarRegionsListCutTool.cutByPlane(planeBottom, map);
      map = PlanarRegionsListCutTool.cutByPlane(planeLeft, map);
      map = PlanarRegionsListCutTool.cutByPlane(planeRight, map);


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

   private void updatePlaneToFrameWithParameters(Plane3D plane, double pitch, double yaw, double yFace, double zFace)
   {
      tempFramePoint3D.setToZero(cameraFrame);
      tempFramePoint3D.changeFrame(ReferenceFrame.getWorldFrame());
      plane.setPoint(tempFramePoint3D);

      tempFramePose3D.setToZero();
      tempFramePose3D.setReferenceFrame(cameraFrame);
      tempFramePose3D.appendPitchRotation(Math.toRadians(pitch));
      tempFramePose3D.appendYawRotation(Math.toRadians(yaw)); // <-- this is wrong
      tempFramePose3D.changeFrame(ReferenceFrame.getWorldFrame());
      tempNormal.set(0.0, yFace, zFace);
      tempFramePose3D.getOrientation().transform(tempNormal);
      plane.setNormal(tempNormal);
   }
}
