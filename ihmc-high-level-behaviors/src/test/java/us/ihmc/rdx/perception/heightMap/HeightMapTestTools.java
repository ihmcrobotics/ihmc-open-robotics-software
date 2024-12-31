package us.ihmc.rdx.perception.heightMap;

import org.apache.commons.lang3.tuple.ImmutablePair;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.referenceFrame.FrameLine3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class HeightMapTestTools
{
   public static PlanarRegionsList createTwoPlaneWorld(double groundElevation, double platformElevation)
   {
      ConvexPolygon2D groundPlanePolygon = new ConvexPolygon2D();
      groundPlanePolygon.addVertex(5.0, 5.0);
      groundPlanePolygon.addVertex(5.0, -5.0);
      groundPlanePolygon.addVertex(-5.0, -5.0);
      groundPlanePolygon.addVertex(-5.0, 5.0);
      groundPlanePolygon.update();

      ConvexPolygon2D platformPolygon = new ConvexPolygon2D();
      platformPolygon.addVertex(0.5, 0.5);
      platformPolygon.addVertex(-0.5, 0.5);
      platformPolygon.addVertex(-0.5, -0.5);
      platformPolygon.addVertex(0.5, -0.5);
      platformPolygon.update();

      RigidBodyTransform groundTransform = new RigidBodyTransform();
      groundTransform.getTranslation().addZ(groundElevation);
      RigidBodyTransform platformTransform = new RigidBodyTransform();
      platformTransform.getTranslation().addZ(platformElevation);

      PlanarRegion groundPlane = new PlanarRegion(groundTransform, groundPlanePolygon);
      PlanarRegion platformPlane = new PlanarRegion(platformTransform, platformPolygon);

      return new PlanarRegionsList(groundPlane, platformPlane);
   }

   public static double computePixelZ(PlanarRegionsList planarRegionsList, CPUPerspectiveCameraForTest perspectiveCamera, int u, int v)
   {
      // Get the ray for ray casting in the world frame that starts at the focal point and goes through this pixel.
      FrameLine3D ray3D = new FrameLine3D(perspectiveCamera.getCameraFrame(), computeRayForPixel(perspectiveCamera, u, v));
      ray3D.changeFrame(ReferenceFrame.getWorldFrame());

      // Compute the inersection between that ray and the "world", which is made up of planar regions.
      ImmutablePair<Point3D, PlanarRegion> intersectionPair = PlanarRegionTools.intersectRegionsWithRay(planarRegionsList, ray3D.getPoint(), new Vector3D(ray3D.getDirection()));
      if (intersectionPair == null)
         return Double.POSITIVE_INFINITY;

      Point3DReadOnly intersection = intersectionPair.getLeft();
      FramePoint3D intersectionInWorld = new FramePoint3D(ReferenceFrame.getWorldFrame(), intersection);

      // Compute the distance from that intersection to the camera frame, which gives us our depth.
      intersectionInWorld.changeFrame(perspectiveCamera.getCameraFrame());
      // u is col, v is row
      double depth = intersectionInWorld.distanceFromOrigin();

      return intersectionInWorld.getX();

   }

   // This computes the directional ray that starts at the camera origin (the focal point) and goes through the pixel indicated by (u, v) (width, height)
   public static Line3D computeRayForPixel(CPUPerspectiveCameraForTest perspectiveCamera, int u, int v)
   {
      return new Line3D(new Point3D(), perspectiveCamera.computeDirectionOfPixelFromFocalPoint(u, v));
   }
}
