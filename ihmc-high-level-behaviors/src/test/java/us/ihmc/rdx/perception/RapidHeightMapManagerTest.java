package us.ihmc.rdx.perception;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.perception.RapidHeightMapManager;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.gpuHeightMap.ZeroDefaultHeightProvider;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.ZUpFrame;

import java.time.Instant;

import static org.junit.jupiter.api.Assertions.*;

public abstract class RapidHeightMapManagerTest
{
   private static final double groundElevation = -0.05;
   private static final double platformElevation = 0.25;

   // FIXME something about these intrinsics aren't correct
   private static final int width = 1024;
   private static final int height = 512;
   private static final double horizontalFOV = Math.toRadians(90);
   private static final double verticalFOV = Math.toRadians(70);

   private CPUPerspectiveCameraForTest perspectiveCameraForTest;
   private PoseReferenceFrame cameraFrame;
   private ZUpFrame cameraZUpFrame;

   public abstract boolean runUsingCUDA();

   @BeforeEach
   public void setup()
   {
      perspectiveCameraForTest = new CPUPerspectiveCameraForTest(horizontalFOV, verticalFOV, width, height);

      cameraFrame = new PoseReferenceFrame("cameraFrame", ReferenceFrame.getWorldFrame());
      cameraZUpFrame = new ZUpFrame(cameraFrame, "cameraZupFrame");
   }

   @AfterEach
   public void destroy()
   {
      cameraFrame.remove();
      cameraZUpFrame.remove();
      cameraFrame = null;
      cameraZUpFrame = null;
   }

   @Test
   public void testSimpleCompute()
   {
      RapidHeightMapManager heightMapManager = new RapidHeightMapManager("", new ZeroDefaultHeightProvider(), perspectiveCameraForTest.getCameraIntrinsics(), runUsingCUDA());

      PlanarRegionsList worldModel = createTwoPlaneWorld();


      // position the camera 1.75 meter above the ground, pointed straight down. This means it should be able to view 1 meter wide of the world, and 0.5 meters tall.
      // At this position, the entire height map should be just the platform.
      FramePose3D cameraPose = new FramePose3D();
      cameraPose.appendTranslation(0.0, 0.0, 1.75);
      cameraPose.appendPitchRotation(Math.PI / 2.0);

      cameraFrame.setPoseAndUpdate(cameraPose);

      // Compute the depth at the current camera pose.
      DMatrixRMaj depthMapAsMatrix = new DMatrixRMaj(height, width);
      samplePlanarRegionWorldFromPerspectiveCamera(depthMapAsMatrix, worldModel);
      Mat depthMat = convertMatrixToMat(depthMapAsMatrix);

      // loop through 100 times so the alpha filter on the update can bind the new data
      for (int i = 0; i < 10; i++)
         heightMapManager.update(depthMat, Instant.now(), cameraFrame, cameraZUpFrame);

      // Check the center point of the height map.
      assertEquals(platformElevation, heightMapManager.getTerrainMapData().getHeightInWorld(0.0, 0.0), 1e-2);

      // Check the entire field of view.
      for (double x = -0.2; x <= 0.2; x += 0.01)
      {
         for (double y = -0.3; y <= 0.3; y += 0.01)
         {
            String failureMessage = "Failed at (" + x + ", " + y +")";
            assertEquals(platformElevation, heightMapManager.getTerrainMapData().getHeightInWorld(x, y), 1e-2, failureMessage);
            assertEquals(platformElevation, heightMapManager.getLatestHeightMapData().getHeightAt(x, y), 1e-2, failureMessage);
         }
      }
   }

   public void samplePlanarRegionWorldFromPerspectiveCamera(DMatrixRMaj depthMapToPack, PlanarRegionsList planarRegionsList)
   {
      // u is x, v is y in pixel frame
      for (int col = 0; col < width; col++)
      {
         for (int row = 0; row < height; row++)
         {
            // Get the ray for ray casting in the world frame that starts at the focal point and goes through this pixel.
            FrameLine3D ray3D = new FrameLine3D(cameraFrame, computeRayForPixel(col, row));
            ray3D.changeFrame(ReferenceFrame.getWorldFrame());

            // Compute the inersection between that ray and the "world", which is made up of planar regions.
            Point3DReadOnly intersection = PlanarRegionTools.intersectRegionsWithRay(planarRegionsList, ray3D.getPoint(), new Vector3D(ray3D.getDirection()))
                                                            .getLeft();
            FramePoint3D intersectionInWorld = new FramePoint3D(ReferenceFrame.getWorldFrame(), intersection);

            // Compute the distance from that intersection to the camera frame, which gives us our depth.
            intersectionInWorld.changeFrame(cameraFrame);
            depthMapToPack.set(row, col, intersectionInWorld.distanceFromOrigin());
         }
      }
   }

   public static Mat convertMatrixToMat(DMatrixRMaj depthMap)
   {
      // depth map is stored x-y (col-row), mat is stored row-col
      Mat depthMat = new Mat(height, width, opencv_core.CV_32FC1);
      for (int row = 0; row < height; row++)
      {
         for (int col = 0; col < width; col++)
         {
            depthMat.ptr(row, col).putFloat((float) depthMap.get(row, col));
         }
      }

      return depthMat;
   }

   // This computes the directional ray that starts at the camera origin (the focal point) and goes through the pixel indicated by (u, v) (width, height)
   private Line3D computeRayForPixel(int u, int v)
   {
      return new Line3D(new Point3D(), perspectiveCameraForTest.computeDirectionOfPixelFromFocalPoint(u, v));
   }

   private static PlanarRegionsList createTwoPlaneWorld()
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

      PlanarRegion groundPlane = new PlanarRegion(new RigidBodyTransform(), groundPlanePolygon);
      PlanarRegion platformPlane = new PlanarRegion(platformTransform, platformPolygon);

      return new PlanarRegionsList(groundPlane, platformPlane);
   }
}
