package us.ihmc.perception.rapidRegions;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.jetbrains.annotations.NotNull;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Plane3D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.UnitVector3DBasics;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.robotics.geometry.FramePlanarRegionsList;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;

import java.nio.ShortBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.Random;

public class RapidPlanarRegionsExtractorTest
{
   /**
    * Tests RapidRegionsExtractor on depth data generated on a single uniform plane. No noise is added.
    * The patch feature data and planar region are checked against the expected plane.
    */
   @Test
   public void testSingleUniformPlane()
   {
      int numTests = 25;
      Random random = new Random(32890);

      CameraIntrinsics cameraIntrinsics = createGenericCameraIntrinsics();
      RapidPlanarRegionsExtractor rapidPlanarRegionsExtractor = new RapidPlanarRegionsExtractor(cameraIntrinsics);

      GpuMat depthImageDevice = new GpuMat(cameraIntrinsics.getHeight(), cameraIntrinsics.getWidth(), opencv_core.CV_16UC1);
      Mat depthImageHost = new Mat(cameraIntrinsics.getHeight(), cameraIntrinsics.getWidth(), opencv_core.CV_16UC1);

      PoseReferenceFrame cameraFrame = new PoseReferenceFrame("cameraFrame", ReferenceFrame.getWorldFrame());
      FramePlanarRegionsList planarRegionsList = new FramePlanarRegionsList();

      for (int i = 0; i < numTests; i++)
      {
         Plane3D plane = generateRandomPlane(random);
         short[] depthArray = createDepthDataForSinglePlane(cameraIntrinsics, plane);

         ShortBuffer depthDataBuffer = depthImageHost.createBuffer();
         depthDataBuffer.put(depthArray);
         depthImageDevice.upload(depthImageHost);

         rapidPlanarRegionsExtractor.update(depthImageDevice, cameraFrame, planarRegionsList);

         Vector3D[] normals = rapidPlanarRegionsExtractor.getNormals();
         Point3D[] centroids = rapidPlanarRegionsExtractor.getCentroids();

         // Normal returns in ZUp, but all math here is in z-out
         for (int j = 0; j < normals.length; j++)
         {
            toZOut(normals[j]);
            toZOut(centroids[j]);
         }

         // Test all centroids are on the plane
         for (int j = 0; j < centroids.length; j++)
         {
            Assertions.assertTrue(plane.distance(centroids[j]) < 1.0e-3);
         }

         // Test all normals match the plane
         for (int j = 0; j < normals.length; j++)
         {
            double angularThresholdDegrees = 8.0;
            double dotProduct = Math.abs(plane.getNormal().dot(normals[j]));
            Assertions.assertTrue(dotProduct > Math.cos(Math.toRadians(angularThresholdDegrees)));
         }

         // Check one region was found
         Assertions.assertEquals(1, planarRegionsList.getPlanarRegionsList().getNumberOfPlanarRegions());

         // Check region properties
         PlanarRegion planarRegion = planarRegionsList.getPlanarRegionsList().getPlanarRegion(0);
         Vector3D regionNormal = new Vector3D(planarRegion.getNormal());
         toZOut(regionNormal);
         if (regionNormal.getZ() * plane.getNormal().getZ() < 0.0)
            regionNormal.negate();
         Assertions.assertTrue(regionNormal.epsilonEquals(plane.getNormal(), 1.0e-4));
      }
   }

   /**
    * Tests RapidRegionsExtractor on depth data generated on four planes. No noise is added.
    * The four planes are grouped into quadrants of the image, with (cx, cy) dividing the quadrants.
    * The patch feature data and planar region are checked against the expected plane
    *   (0,0) ----------------------> u, x
    *     |        P1 | P2
    *     |           |
    *     |-----------+--------------
    *     |           |  (cx, cy)
    *     |           |
    *     |        P3 | P4
    *     v, y
    */
   @Test
   public void testMultipleUniformPlanes()
   {
      int numTests = 25;
      Random random = new Random(32890);

      CameraIntrinsics cameraIntrinsics = createGenericCameraIntrinsics();
      RapidPlanarRegionsExtractor rapidPlanarRegionsExtractor = new RapidPlanarRegionsExtractor(cameraIntrinsics);
      RapidRegionsExtractorParameters parameters = rapidPlanarRegionsExtractor.getRapidRegionsExtractorParameters();

      GpuMat depthImageDevice = new GpuMat(cameraIntrinsics.getHeight(), cameraIntrinsics.getWidth(), opencv_core.CV_16UC1);
      Mat depthImageHost = new Mat(cameraIntrinsics.getHeight(), cameraIntrinsics.getWidth(), opencv_core.CV_16UC1);

      PoseReferenceFrame cameraFrame = new PoseReferenceFrame("cameraFrame", ReferenceFrame.getWorldFrame());
      FramePlanarRegionsList planarRegionsList = new FramePlanarRegionsList();

      int patchSize = parameters.getPatchSize();
      int patchImageHeight = cameraIntrinsics.getHeight() / patchSize;
      int patchImageWidth = cameraIntrinsics.getWidth() / patchSize;

      int numberOfQuadrants = 4;

      for (int i = 0; i < numTests; i++)
      {
         // Generate 4 random distinct planes
         Plane3D[] planes = new Plane3D[numberOfQuadrants];
         planes[0] = generateRandomPlane(random, 1.0, 1.0);
         planes[1] = generateRandomPlane(random, 1.0, -1.0);
         planes[2] = generateRandomPlane(random, -1.0, 1.0);
         planes[3] = generateRandomPlane(random, -1.0, -1.0);

         short[] depthArray = createDepthDataForMultiplePlanes(cameraIntrinsics, planes);

         ShortBuffer depthDataBuffer = depthImageHost.createBuffer();
         depthDataBuffer.put(depthArray);
         depthImageDevice.upload(depthImageHost);

         rapidPlanarRegionsExtractor.update(depthImageDevice, cameraFrame, planarRegionsList);

         Vector3D[] normals = rapidPlanarRegionsExtractor.getNormals();
         Point3D[] centroids = rapidPlanarRegionsExtractor.getCentroids();

         // Normal returns in ZUp, but all math here is in z-out
         for (int j = 0; j < normals.length; j++)
         {
            toZOut(normals[j]);
            toZOut(centroids[j]);
         }

         // Test all centroids are on the plane
         for (int patchX = 0; patchX < patchImageWidth; patchX++)
         {
            for (int patchY = 0; patchY < patchImageHeight; patchY++)
            {
               Plane3D plane = planes[getQuadrantPlaneIndex(patchX * patchSize, patchY * patchSize, cameraIntrinsics)];
               Point3D centroid = centroids[patchX + patchY * patchImageWidth];
               Assertions.assertTrue(plane.distance(centroid) < 1.0e-3);
            }
         }

         // Test all normals match the plane
         for (int patchX = 0; patchX < patchImageWidth; patchX++)
         {
            for (int patchY = 0; patchY < patchImageHeight; patchY++)
            {
               Plane3D plane = planes[getQuadrantPlaneIndex(patchX * patchSize, patchY * patchSize, cameraIntrinsics)];
               Vector3D normal = normals[patchX + patchY * patchImageWidth];

               double angularThresholdDegrees = 8.0;
               double dotProduct = Math.abs(plane.getNormal().dot(normal));
               Assertions.assertTrue(dotProduct > Math.cos(Math.toRadians(angularThresholdDegrees)));
            }
         }

         // Check 4 regions were found
         int numberOfPlanarRegions = planarRegionsList.getPlanarRegionsList().getNumberOfPlanarRegions();
         int expectedNumberOfPlanes = 4;
         Assertions.assertEquals(expectedNumberOfPlanes, numberOfPlanarRegions);

         // Check region properties
         List<Integer> indicesToCheckForNormals = new ArrayList<>(List.of(0, 1, 2, 3));

         for (int j = 0; j < expectedNumberOfPlanes; j++)
         {
            PlanarRegion planarRegion = planarRegionsList.getPlanarRegionsList().getPlanarRegion(j);
            Vector3D regionNormal = new Vector3D(planarRegion.getNormal());
            toZOut(regionNormal);

            // Check that one of the normals matches this one
            boolean foundMatch = false;
            for (int k = 0; k < indicesToCheckForNormals.size(); k++)
            {
               Plane3D plane = planes[indicesToCheckForNormals.get(k)];

               if (regionNormal.getZ() * plane.getNormal().getZ() < 0.0)
                  regionNormal.negate();

               if (regionNormal.epsilonEquals(plane.getNormal(), 1.0e-4))
               {
                  foundMatch = true;
                  indicesToCheckForNormals.remove(k);
                  break;
               }
            }

            Assertions.assertTrue(foundMatch);
         }
      }
   }

   private static Plane3D generateRandomPlane(Random random)
   {
      double pointZ = EuclidCoreRandomTools.nextDouble(random, 1.0, 5.0);
      double normalX = EuclidCoreRandomTools.nextDouble(random, 0.5);
      double normalY = EuclidCoreRandomTools.nextDouble(random, 0.5);
      double normalZ = 1.0;

      Plane3D plane = new Plane3D();
      plane.getPoint().set(0.0, 0.0, pointZ);
      plane.getNormal().set(normalX, normalY, normalZ);
      plane.getNormal().normalize();
      return plane;
   }

   private static Plane3D generateRandomPlane(Random random, double signNormalX, double signNormalY)
   {
      double pointZ = EuclidCoreRandomTools.nextDouble(random, 1.0, 5.0);
      double normalX = signNormalX * EuclidCoreRandomTools.nextDouble(random, 0.25, 0.5);
      double normalY = signNormalY * EuclidCoreRandomTools.nextDouble(random, 0.25, 0.5);
      double normalZ = 1.0;

      Plane3D plane = new Plane3D();
      plane.getPoint().set(0.0, 0.0, pointZ);
      plane.getNormal().set(normalX, normalY, normalZ);
      plane.getNormal().normalize();
      return plane;
   }

   private static final Random random = new Random(32980);

   private static short[] createDepthDataForSinglePlane(CameraIntrinsics cameraIntrinsics, Plane3D plane)
   {
      short[] depthArray = new short[cameraIntrinsics.getWidth() * cameraIntrinsics.getHeight()];

      for (int u = 0; u < cameraIntrinsics.getWidth(); u++)
      {
         for (int v = 0; v < cameraIntrinsics.getHeight(); v++)
         {
            // compute depth in mm
            short depth = (short) (Math.round(computeDepth(cameraIntrinsics, plane, u, v) * 1000));
            depthArray[u + v * cameraIntrinsics.getWidth()] = depth;
         }
      }
      return depthArray;
   }

   private static short[] createDepthDataForMultiplePlanes(CameraIntrinsics cameraIntrinsics, Plane3D[] planes)
   {
      short[] depthArray = new short[cameraIntrinsics.getWidth() * cameraIntrinsics.getHeight()];

      for (int u = 0; u < cameraIntrinsics.getWidth(); u++)
      {
         for (int v = 0; v < cameraIntrinsics.getHeight(); v++)
         {
            // compute depth in mm
            Plane3D plane = planes[getQuadrantPlaneIndex(u, v, cameraIntrinsics)];
            short depth = (short) Math.round((computeDepth(cameraIntrinsics, plane, u, v) * 1000));
            depthArray[u + v * cameraIntrinsics.getWidth()] = depth;
         }
      }
      return depthArray;
   }

   private static int getQuadrantPlaneIndex(int u, int v, CameraIntrinsics cameraIntrinsics)
   {
      if (u < (int) cameraIntrinsics.getCx())
      {
         return v < (int) cameraIntrinsics.getCy() ? 0 : 1;
      }
      else
      {
         return v < (int) cameraIntrinsics.getCy() ? 2 : 3;
      }
   }

   @NotNull
   private static CameraIntrinsics createGenericCameraIntrinsics()
   {
      CameraIntrinsics cameraIntrinsics = new CameraIntrinsics();
      cameraIntrinsics.setWidth(1280);
      cameraIntrinsics.setHeight(720);
      cameraIntrinsics.setCx(640);
      cameraIntrinsics.setCy(360);
      cameraIntrinsics.setFx(700);
      cameraIntrinsics.setFy(700);
      return cameraIntrinsics;
   }

   private static Point3D project(CameraIntrinsics cameraIntrinsics, int u, int v, float depth)
   {
      float x = (float) ((u - cameraIntrinsics.getCx()) / cameraIntrinsics.getFx() * depth);
      float y = (float) ((v - cameraIntrinsics.getCy()) / cameraIntrinsics.getFy() * depth);
      return new Point3D(x, y, depth);
   }

   private static float computeDepth(CameraIntrinsics cameraIntrinsics, Plane3D plane, int u, int v)
   {
      Point3DBasics point = plane.getPoint();
      UnitVector3DBasics normal = plane.getNormal();

      double numerator = normal.dot(point);
      double denominatorX = normal.getX() * (u - cameraIntrinsics.getCx()) / cameraIntrinsics.getFx();
      double denominatorY = normal.getY() * (v - cameraIntrinsics.getCy()) / cameraIntrinsics.getFy();
      double denominatorZ = normal.getZ();

      return (float) Math.abs((numerator / (denominatorX + denominatorY + denominatorZ)));
   }

   /**
    * From "zOut" where z points out of the image frame to "zUp" where z points along the image plane
   */
   private static void toZUp(Tuple3DBasics zOut)
   {
      zOut.set(zOut.getZ(), -zOut.getX(), -zOut.getY());
   }

   /**
    * From "zUp" where z points along the image plane to "zOut" where z points out of the image
   */
   private static void toZOut(Tuple3DBasics zUp)
   {
      zUp.set(-zUp.getY(), -zUp.getZ(), zUp.getX());
   }
}
