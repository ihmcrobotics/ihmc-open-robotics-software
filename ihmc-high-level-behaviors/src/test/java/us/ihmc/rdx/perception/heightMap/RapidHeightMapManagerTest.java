package us.ihmc.rdx.perception.heightMap;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.ejml.data.DMatrixRMaj;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.perception.RapidHeightMapManager;
import us.ihmc.perception.gpuHeightMap.ZeroDefaultHeightProvider;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.time.Instant;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

public abstract class RapidHeightMapManagerTest
{
   private static final double platformWidth = 1.0;
   private static final double groundElevation = -0.05;
   private static final double platformElevation = 0.25;

   // FIXME something about these intrinsics aren't correct
   private static final int width = 1024;
   private static final int height = 512;
   private static final double horizontalFOV = Math.toRadians(90);
   private static final double verticalFOV = Math.toRadians(70);

   private CPUPerspectiveCameraForTest perspectiveCameraForTest;

   public abstract boolean runUsingCUDA();

   @BeforeEach
   public void setup()
   {
      perspectiveCameraForTest = new CPUPerspectiveCameraForTest(horizontalFOV, verticalFOV, width, height);
   }

   @AfterEach
   public void destroy()
   {
      perspectiveCameraForTest.destroy();
      perspectiveCameraForTest = null;
   }

   @Test
   public void testSimpleComputeAtDifferentLocations()
   {
      RapidHeightMapManager heightMapManager = new RapidHeightMapManager("",
                                                                         new ZeroDefaultHeightProvider(),
                                                                         perspectiveCameraForTest.getCameraIntrinsics(),
                                                                         runUsingCUDA());

      PlanarRegionsList worldModel = HeightMapTestTools.createTwoPlaneWorld(-0.05, 0.25);

      // position the camera 1.75 meter above the ground, pointed straight down. This means it should be able to view 1 meter wide of the world, and 0.5 meters tall.
      // At this position, the entire height map should be just the platform.
      FramePose3D cameraPose = new FramePose3D();
      cameraPose.appendTranslation(0.0, 0.0, 1.75);
      cameraPose.appendPitchRotation(Math.PI / 2.0);

      perspectiveCameraForTest.setCameraPose(cameraPose);

      // Compute the depth at the current camera pose.
      DMatrixRMaj depthMapAsMatrix = new DMatrixRMaj(height, width);
      samplePlanarRegionWorldFromPerspectiveCamera(depthMapAsMatrix, worldModel);
      Mat depthMat = convertMatrixToMat(depthMapAsMatrix);

      // loop through 10 times so the alpha filter on the update can bind the new data
      for (int i = 0; i < 10; i++)
         heightMapManager.update(depthMat, Instant.now(), perspectiveCameraForTest.getCameraFrame(), perspectiveCameraForTest.getCameraZUpFrame());

      // Check the center point of the height map.
      assertEquals(platformElevation, heightMapManager.getTerrainMapData().getHeightInWorld(0.0, 0.0), 1e-2);
      checkHeightMapAtCameraLocation(heightMapManager, cameraPose, worldModel, true);

      Random random = new Random(1738L);
      // Now move the camera around and get a bigger picture of the height map
      for (int iiter = 0; iiter < 100; iiter++)
      {
         cameraPose.getTranslation().setX(RandomNumbers.nextDouble(random, 0.5));
         cameraPose.getTranslation().setY(RandomNumbers.nextDouble(random, 0.5));
         perspectiveCameraForTest.setCameraPose(cameraPose);

         // Compute the depth at the current camera pose.
         depthMapAsMatrix = new DMatrixRMaj(height, width);
         samplePlanarRegionWorldFromPerspectiveCamera(depthMapAsMatrix, worldModel);
         depthMat = convertMatrixToMat(depthMapAsMatrix);

         // run the height map at the new location
         heightMapManager.requestReset();
         for (int i = 0; i < 6; i++)
            heightMapManager.update(depthMat, Instant.now(), perspectiveCameraForTest.getCameraFrame(), perspectiveCameraForTest.getCameraZUpFrame());

         checkHeightMapAtCameraLocation(heightMapManager, cameraPose, worldModel, true);
      }
   }

   @Test
   public void testSimpleComputeThenMoving()
   {
      RapidHeightMapManager heightMapManager = new RapidHeightMapManager("",
                                                                         new ZeroDefaultHeightProvider(),
                                                                         perspectiveCameraForTest.getCameraIntrinsics(),
                                                                         runUsingCUDA());

      PlanarRegionsList worldModel = HeightMapTestTools.createTwoPlaneWorld(-0.05, 0.25);

      // position the camera 1.75 meter above the ground, pointed straight down. This means it should be able to view 1 meter wide of the world, and 0.5 meters tall.
      // At this position, the entire height map should be just the platform.
      FramePose3D cameraPose = new FramePose3D();
      cameraPose.appendTranslation(0.0, 0.0, 1.75);
      cameraPose.appendPitchRotation(Math.PI / 2.0);

      perspectiveCameraForTest.setCameraPose(cameraPose);

      // Compute the depth at the current camera pose.
      DMatrixRMaj depthMapAsMatrix = new DMatrixRMaj(height, width);
      samplePlanarRegionWorldFromPerspectiveCamera(depthMapAsMatrix, worldModel);
      Mat depthMat = convertMatrixToMat(depthMapAsMatrix);

      // loop through 10 times so the alpha filter on the update can bind the new data
      for (int i = 0; i < 10; i++)
         heightMapManager.update(depthMat, Instant.now(), perspectiveCameraForTest.getCameraFrame(), perspectiveCameraForTest.getCameraZUpFrame());

      // Check the center point of the height map.
      assertEquals(platformElevation, heightMapManager.getTerrainMapData().getHeightInWorld(0.0, 0.0), 1e-2);
      checkHeightMapAtCameraLocation(heightMapManager, cameraPose, worldModel, false);

      // Now move the camera around and get a bigger picture of the height map
      Vector3D totalTranslation = new Vector3D(0.5, -0.3, 0.0);
      Vector3D stepTranslation = new Vector3D(totalTranslation);
      stepTranslation.scale(1.0 / 100);
      for (int i = 0; i < 100; i++)
      {
         cameraPose.prependTranslation(stepTranslation);
         perspectiveCameraForTest.setCameraPose(cameraPose);

         // Compute the depth at the current camera pose.
         depthMapAsMatrix = new DMatrixRMaj(height, width);
         samplePlanarRegionWorldFromPerspectiveCamera(depthMapAsMatrix, worldModel);
         depthMat = convertMatrixToMat(depthMapAsMatrix);

         // run the height map at the new location
         heightMapManager.update(depthMat, Instant.now(), perspectiveCameraForTest.getCameraFrame(), perspectiveCameraForTest.getCameraZUpFrame());

         checkHeightMapAtCameraLocation(heightMapManager, cameraPose, worldModel, false);
      }
   }

   private void checkHeightMapAtCameraLocation(RapidHeightMapManager heightMapManager, FramePose3DReadOnly cameraPose, PlanarRegionsList worldModel, boolean onlyCheckNewPoints)
   {
      // Check the platform
      for (double x = -platformWidth / 2.0; x <= platformWidth / 2.0; x += 0.01)
      {
         for (double y = -platformWidth / 2.0; y <= platformWidth / 2.0; y += 0.01)
         {
            if (onlyCheckNewPoints && !isPointCloseEnoughToGetData(heightMapManager, x, y, cameraPose))
               continue;

            // If the point is out of bounds of the height map, skip it
            if (heightMapManager.getTerrainMapData().isPointInWorldOutOfBounds(x, y))
               continue;

            // If the point is right next to the edge, it could very easily miss it and then be "invisible". It could also fail because of rounding and averaging.
            if (isPointOnEdge(x, y, 2e-2))
               continue;

            Point3D pointInWorld = new Point3D(x, y, platformElevation);

            // If the point would lie outside the field of view of the camera, don't test it.
            if (!checkIfPointIsInFOV(perspectiveCameraForTest, pointInWorld, 20))
               continue;

            String failureMessage = "Failed at (" + x + ", " + y + ") when camera is at (" + cameraPose.getX() + ", " + cameraPose.getY() + ")";
            assertEquals(platformElevation, heightMapManager.getTerrainMapData().getHeightInWorld(x, y), 1e-2, failureMessage);
         }
      }

      // Check the ground away from the platform
      for (double x = -0.75; x <= 0.75; x += 0.01)
      {
         for (double y = -0.75; y <= 0.74; y += 0.01)
         {
            if (onlyCheckNewPoints && !isPointCloseEnoughToGetData(heightMapManager, x, y, cameraPose))
               continue;

            // If the point is out of bounds of the height map, skip it
            if (heightMapManager.getTerrainMapData().isPointInWorldOutOfBounds(x, y))
               continue;
            // If it's within a certain distance of the edge, don't test the point
            if (isPointOnEdge(x, y, 0.05))
               continue;
            // If it's on a slightly larger (by epsilon) version of the platform, don't test the point
            if (isPointOnPlatform(x, y, 1e-4))
               continue;

            Point3D pointInWorld = new Point3D(x, y, groundElevation);

            // If the camera can't see the point, then there's no way to have data
            if (!checkIfPointIsDefinitelyVisible(cameraPose, pointInWorld, worldModel))
               continue;

            // If the point would lie outside the field of view of the camera, don't test it.
            if (!checkIfPointIsInFOV(perspectiveCameraForTest, pointInWorld, 10))
               continue;

            String failureMessage = "Failed at (" + x + ", " + y + ") when camera is at (" + cameraPose.getX() + ", " + cameraPose.getY() + ")";
            assertEquals(groundElevation, heightMapManager.getTerrainMapData().getHeightInWorld(x, y), 1e-2, failureMessage);

            //            assertEquals(platformElevation, heightMapManager.getLatestHeightMapData().getHeightAt(x, y), 1e-2, failureMessage);
         }
      }
   }

   private static boolean isPointCloseEnoughToGetData(RapidHeightMapManager heightMapManager, double x, double y, FramePose3DReadOnly cameraPose)
   {
      double distance = heightMapManager.getHeightMapParameters().getLocalWidthInMeters() / 2.0;
      if (x - cameraPose.getX() > distance)
         return false;
      if (cameraPose.getX() - x < distance)
         return false;

      if (y - cameraPose.getY() > distance)
         return false;
      if (cameraPose.getY() - y < distance)
         return false;

      return false;
   }

   private static boolean isPointOnPlatform(double x, double y, double onEdgeEpsilon)
   {
      if (x > -platformWidth / 2.0 - onEdgeEpsilon)
         return true;
      if (x < platformWidth / 2.0 + onEdgeEpsilon)
         return true;
      if (y > -platformWidth / 2.0 - onEdgeEpsilon)
         return true;
      if (y < platformWidth / 2.0 + onEdgeEpsilon)
         return true;

      return false;
   }

   private static boolean isPointOnEdge(double x, double y, double onEdgeEpsilon)
   {
      boolean isPointOnEdge = MathTools.epsilonEquals(x, 0.5, onEdgeEpsilon);
      isPointOnEdge |= MathTools.epsilonEquals(x, -0.5, onEdgeEpsilon);
      isPointOnEdge |= MathTools.epsilonEquals(y, -0.5, onEdgeEpsilon);
      isPointOnEdge |= MathTools.epsilonEquals(y, 0.5, onEdgeEpsilon);

      return isPointOnEdge;
   }

   private static boolean checkIfPointIsInFOV(CPUPerspectiveCameraForTest camera, Point3DReadOnly pointInWorld, int epsilon)
   {
      FramePoint3D pointInCamera = new FramePoint3D(ReferenceFrame.getWorldFrame(), pointInWorld);
      pointInCamera.changeFrame(camera.getCameraFrame());

      int[] uvPair = camera.project(pointInCamera);
      if (uvPair[0] < epsilon || uvPair[0] >= camera.getCameraIntrinsics().getWidth() - epsilon)
         return false;
      if (uvPair[1] < epsilon || uvPair[1] >= camera.getCameraIntrinsics().getHeight() - epsilon)
         return false;

      return true;
   }

   private static boolean checkIfPointIsDefinitelyVisible(FramePose3DReadOnly cameraPose, Point3DReadOnly pointInWorld, PlanarRegionsList world)
   {
      // This performs both the oirginal visibility check, as well as searches neighboring points. If any neighboring point is invisible, then this is said to be
      // invisible

      if (!checkIfPointIsVisible(cameraPose, pointInWorld, world))
         return false;

      double epsilon = 0.02;
      int offsets = 8;
      for (int offsetIdx = 0; offsetIdx < offsets; offsetIdx++)
      {
         for (double radialOffset = 0.01; radialOffset <= epsilon; radialOffset += 0.01)
         {
            // Offset the desired point by some distance
            Vector3D offset = new Vector3D(Math.sin(Math.PI * 2 / offsets), Math.cos(Math.PI * 2 / offsets), 0);
            offset.normalize();
            offset.scale(radialOffset);

            Point3D moddedPoint = new Point3D(pointInWorld);
            moddedPoint.add(offset);

            // Test whether that offset point is visible, too
            if (!checkIfPointIsVisible(cameraPose, moddedPoint, world))
               return false;
         }
      }

      return true;
   }

   private static boolean checkIfPointIsVisible(FramePose3DReadOnly cameraPose, Point3DReadOnly pointInWorld, PlanarRegionsList world)
   {
      Vector3D direction = new Vector3D(pointInWorld);
      direction.sub(cameraPose.getPosition());
      ImmutablePair<Point3D, PlanarRegion> intersectionPair = PlanarRegionTools.intersectRegionsWithRay(world, cameraPose.getPosition(), direction);
      if (intersectionPair == null)
         return true;
      double percentage = EuclidGeometryTools.percentageAlongLineSegment3D(intersectionPair.getLeft(), cameraPose.getPosition(), pointInWorld);

      // If the collision is between the point and the camera, the point is not visible
      if (percentage < 1.0 && percentage > 0.0)
         return false;

      return true;
   }

   public void samplePlanarRegionWorldFromPerspectiveCamera(DMatrixRMaj depthMapToPack, PlanarRegionsList planarRegionsList)
   {
      // u is x, v is y in pixel frame
      for (int col = 0; col < width; col++)
      {
         for (int row = 0; row < height; row++)
         {
            depthMapToPack.set(row, col, HeightMapTestTools.computePixelZ(planarRegionsList, perspectiveCameraForTest, col, row));
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
}
