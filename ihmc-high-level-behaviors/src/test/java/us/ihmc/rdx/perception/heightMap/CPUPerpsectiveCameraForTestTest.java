package us.ihmc.rdx.perception.heightMap;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import static org.junit.jupiter.api.Assertions.*;

public class CPUPerpsectiveCameraForTestTest
{
   @Test
   public void testIntrinsicsCalculation()
   {
      double horizontalFOV = Math.toRadians(90);
      double verticalFOV = Math.toRadians(70);
      CPUPerspectiveCameraForTest camera = new CPUPerspectiveCameraForTest(horizontalFOV, verticalFOV, 1024, 720);

      // Test in the middle
      int middleU = (int) camera.getCameraIntrinsics().getCx();
      int middleV = (int) camera.getCameraIntrinsics().getCy();
      Vector3DReadOnly directionAtCenter = camera.computeDirectionOfPixelFromFocalPoint(middleU, middleV);
      Vector3D expectedCenter = new Vector3D(1.0, 0, 0);

      EuclidCoreTestTools.assertEquals(expectedCenter, directionAtCenter, 1e-3);

      // Test at the far left
      Vector3DReadOnly directionAtFarLeft = camera.computeDirectionOfPixelFromFocalPoint(0, middleV);
      Vector3D expectedDirectionAtFarLeft = new Vector3D(Math.cos(horizontalFOV / 2.0), Math.sin(horizontalFOV / 2.0), 0.0);
      EuclidCoreTestTools.assertEquals(expectedDirectionAtFarLeft, directionAtFarLeft, 1e-3);

      // Test at the far right
      Vector3DReadOnly directionAtFarRight = camera.computeDirectionOfPixelFromFocalPoint(1023, middleV);
      Vector3D expectedDirectionAtFarRight = new Vector3D(Math.cos(horizontalFOV / 2.0), -Math.sin(horizontalFOV / 2.0), 0.0);
      EuclidCoreTestTools.assertEquals(expectedDirectionAtFarRight, directionAtFarRight, 1e-3);

      // Test at the top
      Vector3DReadOnly directionATop = camera.computeDirectionOfPixelFromFocalPoint(middleU, 0);
      Vector3D expectedDirectionTop = new Vector3D(Math.cos(verticalFOV / 2.0), 0.0, Math.sin(verticalFOV / 2.0));
      EuclidCoreTestTools.assertEquals(expectedDirectionTop, directionATop, 2e-3);

      // Test at the far right
      Vector3DReadOnly directionAtBottom = camera.computeDirectionOfPixelFromFocalPoint(middleU, 719);
      Vector3D expectedDirectionBottom = new Vector3D(Math.cos(verticalFOV / 2.0), 0.0, -Math.sin(verticalFOV / 2.0));
      EuclidCoreTestTools.assertEquals(expectedDirectionBottom, directionAtBottom, 2e-3);
   }

   @Test
   public void testProjection()
   {
      double horizontalFOV = Math.toRadians(90);
      double verticalFOV = Math.toRadians(70);
      int width = 1024;
      int height = 720;
      CPUPerspectiveCameraForTest camera = new CPUPerspectiveCameraForTest(horizontalFOV, verticalFOV, width, height);

      // position the camera 1.75 meter above the ground, pointed straight down. This means it should be able to view 1 meter wide of the world, and 0.5 meters tall.
      // At this position, the entire height map should be just the platform.
      FramePose3D cameraPose = new FramePose3D();
      cameraPose.appendTranslation(0.0, 0.0, 1.0);
      cameraPose.appendPitchRotation(Math.PI / 2.0);

      camera.setCameraPose(cameraPose);

      PlanarRegionsList planarRegionsList = HeightMapTestTools.createTwoPlaneWorld(-0.05, 0.25);

      for (int u = 0; u < camera.getCameraIntrinsics().getWidth(); u++)
      {
         for (int v = 0; v < camera.getCameraIntrinsics().getHeight(); v++)
         {
            double expectedDepth = HeightMapTestTools.computePixelZ(planarRegionsList, camera, u, v);

            if (Double.isFinite(expectedDepth))
            {
               FramePoint3D pointInWorld = new FramePoint3D(camera.getCameraFrame(), camera.backProject(u, v, expectedDepth));
               pointInWorld.changeFrame(ReferenceFrame.getWorldFrame());
               if (!MathTools.epsilonEquals(pointInWorld.getZ(), -0.05, 1e-4) && !MathTools.epsilonEquals(pointInWorld.getZ(), 0.25, 1e-4))
                  fail("Invalid height detected. " + pointInWorld.getZ());

               // Get the ray for ray casting in the world frame that starts at the focal point and goes through this pixel.
               FrameLine3D ray3D = new FrameLine3D(camera.getCameraFrame(), HeightMapTestTools.computeRayForPixel(camera, u, v));
               ray3D.changeFrame(ReferenceFrame.getWorldFrame());

               // Compute the inersection between that ray and the "world", which is made up of planar regions.
               ImmutablePair<Point3D, PlanarRegion> intersectionPair  = PlanarRegionTools.intersectRegionsWithRay(planarRegionsList, ray3D.getPoint(), new Vector3D(ray3D.getDirection()));

               if (intersectionPair == null)
                  fail("Failed to find an intersection, but one was expected.");

               String failureMessage = "Failed at (u,v) = (" + u + ", " + v + ")";

               Point3DReadOnly intersection = intersectionPair.getLeft();
               // Check that the intersection from this ray matches the back projected point.
               EuclidCoreTestTools.assertEquals(failureMessage, intersection, pointInWorld, 1e-3);

               // Check that, when computing the pixels that match this point in the world, things are done correctly
               pointInWorld.changeFrame(camera.getCameraFrame());
               int[] pixels = camera.project(pointInWorld);

               assertEquals(u, pixels[0], 1, failureMessage);
               assertEquals(v, pixels[1], 1, failureMessage);
            }

         }
      }
   }

}
