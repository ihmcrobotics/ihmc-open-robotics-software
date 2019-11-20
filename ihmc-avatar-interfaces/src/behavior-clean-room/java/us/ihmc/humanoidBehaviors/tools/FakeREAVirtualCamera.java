package us.ihmc.humanoidBehaviors.tools;

import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.euclid.geometry.Plane3D;
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

public class FakeREAVirtualCamera
{
   private final ReferenceFrame cameraFrame;

   private final double verticalFOV;
   private final double horizontalFOV;

   private final FramePose3D tempFramePose3D = new FramePose3D();
   private final Vector3D tempNormal = new Vector3D();

   private final Plane3D planeTop;
   private final Plane3D planeBottom;
   private final Plane3D planeLeft;
   private final Plane3D planeRight;

   public FakeREAVirtualCamera(double verticalFOV, double horizontalFOV, ReferenceFrame cameraFrame)
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
      tempFramePose3D.setToZero();
      tempFramePose3D.setReferenceFrame(cameraFrame);
      tempFramePose3D.appendPitchRotation(pitch);
      tempFramePose3D.appendYawRotation(yaw);
      tempFramePose3D.changeFrame(ReferenceFrame.getWorldFrame());
      plane.setPoint(tempFramePose3D.getPosition());
      tempNormal.set(0.0, yFace, zFace);
      tempFramePose3D.getOrientation().transform(tempNormal);
      plane.setNormal(tempNormal);
   }
}
