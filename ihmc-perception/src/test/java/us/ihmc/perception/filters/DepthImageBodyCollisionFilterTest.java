package us.ihmc.perception.filters;

import org.bytedeco.javacpp.ShortPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameSphere3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.sensors.CameraIntrinsics;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.scs2.simulation.collision.Collidable;

import java.nio.ShortBuffer;
import java.util.Arrays;
import java.util.List;
import java.util.Random;

import static org.junit.jupiter.api.Assertions.*;

class DepthImageBodyCollisionFilterTest
{
   private static final float depthFilterTolerance = 0.05f;
   private static final double sphereRadius = 0.1;
   private static final int imageWidth = 10;
   private static final int imageHeight = 10;

   @Test
   public void testDepthPointsInsideBodyCollisionOfASphere()
   {
      ///// This test is a simple camera looking down at the ground from 1 meter high. There's a single collision sphere that's 0.1 radius sitting on the ground,
      ///// centered at (0.0, 0.0) (x,y) in the world.

      // First, set up the camera frame and object
      ReferenceFrame cameraFrame = createGodsEyeCameraFrame();

      // Now, set up the sphere collidable at (0.0, 0.0, 0.0) in the world.
      FrameSphere3D sphere3D = createCollisionSphere(cameraFrame, new Point3D());

      RobotCollisionModel robotCollisionModel = createRobotCollisionModel(sphere3D);

      // Create the filter.
      DepthImageBodyCollisionFilter depthImageBodyCollisionFilter = new DepthImageBodyCollisionFilter(robotCollisionModel,
                                                                                                      new RigidBody("dummyBody", ReferenceFrame.getWorldFrame()));
      depthImageBodyCollisionFilter.setCollisionTolerance(depthFilterTolerance);
      // Create spoofed depth values.
      short[] depthValues = createDepthMeasures(1.0f);

      CameraIntrinsics cameraIntrinsics = setupCameraIntrinsics();

      GpuMat inputDepthImage = convertDepthMeasuresToInputMat(depthValues);
      GpuMat outputFilteredDepthImage = new GpuMat(inputDepthImage.size(), inputDepthImage.type());

      printGpuMat("sensed", inputDepthImage);

      depthImageBodyCollisionFilter.process(inputDepthImage, outputFilteredDepthImage, cameraIntrinsics, cameraFrame);

      Mat result = new Mat();
      outputFilteredDepthImage.download(result);
      PerceptionDebugTools.printMat("w", result, 1);

      // Check against the Euclid objects
      checkAgainstCPUCalculation(cameraFrame, depthValues, result, depthImageBodyCollisionFilter.getRobotCollidables(), cameraIntrinsics, imageWidth);

      checkExpectedZeroValues(result, imageWidth, imageHeight);

      // Now, let's randomize the location of the sphere on the surface.
      int iters = 20;
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         FramePoint3D position = new FramePoint3D(ReferenceFrame.getWorldFrame());
         position.setX(RandomNumbers.nextDouble(random, 1.0));
         position.setY(RandomNumbers.nextDouble(random, 1.0));
         position.changeFrame(cameraFrame);

         sphere3D.getPosition().set(position);

         depthImageBodyCollisionFilter.process(inputDepthImage, outputFilteredDepthImage, cameraIntrinsics, cameraFrame);
         outputFilteredDepthImage.download(result);

         // Check against the Euclid objects
         checkAgainstCPUCalculation(cameraFrame, depthValues, result, depthImageBodyCollisionFilter.getRobotCollidables(), cameraIntrinsics, imageWidth);
      }
   }

   @Disabled
   @Test
   public void testFilteringDepthPointsInTheShadowOfTheSphere()
   {
      ReferenceFrame cameraFrame = createGodsEyeCameraFrame();

      FrameSphere3D sphere3D = createCollisionSphere(cameraFrame, new Point3D(0.0, 0.0, 0.5));

      RobotCollisionModel robotCollisionModel = createRobotCollisionModel(sphere3D);

      DepthImageBodyCollisionFilter depthImageBodyCollisionFilter = new DepthImageBodyCollisionFilter(robotCollisionModel,
                                                                                                      new RigidBody("dummyBody", ReferenceFrame.getWorldFrame()));
      depthImageBodyCollisionFilter.setCollisionTolerance(depthFilterTolerance);

      short[] depthValues = createDepthMeasures(1.0f);

      CameraIntrinsics cameraIntrinsics = setupCameraIntrinsics();

      GpuMat inputDepthImage = convertDepthMeasuresToInputMat(depthValues);
      GpuMat outputFilteredDepthImage = new GpuMat(inputDepthImage.size(), inputDepthImage.type());

      printGpuMat("sensed", inputDepthImage);

      depthImageBodyCollisionFilter.process(inputDepthImage, outputFilteredDepthImage, cameraIntrinsics, cameraFrame);

      Mat result = new Mat();
      outputFilteredDepthImage.download(result);
      PerceptionDebugTools.printMat("filtered", result, 1);

      checkAgainstCPUCalculation(cameraFrame, depthValues, result, depthImageBodyCollisionFilter.getRobotCollidables(), cameraIntrinsics, imageWidth);

      // Now, let's randomize the location, but keeping it in front of the visualized plane.
      int iters = 20;
      Random random = new Random(1738L);
      for (int iter = 0; iter < iters; iter++)
      {
         FramePoint3D position = new FramePoint3D(ReferenceFrame.getWorldFrame());
         position.setX(RandomNumbers.nextDouble(random, 1.0));
         position.setY(RandomNumbers.nextDouble(random, 1.0));
         position.setZ(RandomNumbers.nextDouble(random, 1.0));

         LogTools.info("Trying iteration {} at position " + position, iter);

         position.changeFrame(cameraFrame);

         sphere3D.getPosition().set(position);

         depthImageBodyCollisionFilter.process(inputDepthImage, outputFilteredDepthImage, cameraIntrinsics, cameraFrame);
         outputFilteredDepthImage.download(result);

         // Check against the Euclid objects
         checkAgainstCPUCalculation(cameraFrame, depthValues, result, depthImageBodyCollisionFilter.getRobotCollidables(), cameraIntrinsics, imageWidth);
      }
   }

   @Test
   public void testThatPointsInFrontOfSphereArentFiltered()
   {
      ReferenceFrame cameraFrame = createGodsEyeCameraFrame();

      FrameSphere3D sphere3D = createCollisionSphere(cameraFrame, new Point3D(0.0, 0.0, 1.5));

      RobotCollisionModel robotCollisionModel = createRobotCollisionModel(sphere3D);

      DepthImageBodyCollisionFilter depthImageBodyCollisionFilter = new DepthImageBodyCollisionFilter(robotCollisionModel,
                                                                                                      new RigidBody("dummyBody", ReferenceFrame.getWorldFrame()));
      depthImageBodyCollisionFilter.setCollisionTolerance(depthFilterTolerance);

      short[] depthValues = createDepthMeasures(1.0f);

      CameraIntrinsics cameraIntrinsics = setupCameraIntrinsics();

      GpuMat inputDepthImage = convertDepthMeasuresToInputMat(depthValues);
      GpuMat outputFilteredDepthImage = new GpuMat(inputDepthImage.size(), inputDepthImage.type());

      printGpuMat("sensed", inputDepthImage);

      depthImageBodyCollisionFilter.process(inputDepthImage, outputFilteredDepthImage, cameraIntrinsics, cameraFrame);

      Mat result = new Mat();
      outputFilteredDepthImage.download(result);
      PerceptionDebugTools.printMat("filtered", result, 1);

      // Nothing should be filtered.
      checkExpectedValues(depthValues, result);

      checkAgainstCPUCalculation(cameraFrame, depthValues, result, depthImageBodyCollisionFilter.getRobotCollidables(), cameraIntrinsics, imageWidth);
   }

   private static ReferenceFrame createGodsEyeCameraFrame()
   {
      RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
      rigidBodyTransform.getTranslation().set(0.0, 0.0, 1.0);
      // Rotate 90° around Y to point along -Z
      rigidBodyTransform.getRotation().setYawPitchRoll(0.0, Math.toRadians(90.0), 0.0);

      ReferenceFrame cameraFrame = new ReferenceFrame("cameraFrame", ReferenceFrame.getWorldFrame())
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(rigidBodyTransform);
         }
      };
      cameraFrame.update();

      return cameraFrame;
   }

   private static FrameSphere3D createCollisionSphere(ReferenceFrame cameraFrame, Point3D locationInWorld)
   {
      FrameSphere3D sphere = new FrameSphere3D(cameraFrame, sphereRadius);
      FramePoint3D position = new FramePoint3D(ReferenceFrame.getWorldFrame(), locationInWorld);
      position.changeFrame(cameraFrame);
      sphere.getPosition().set(position);

      return sphere;
   }

   private static RobotCollisionModel createRobotCollisionModel(FrameShape3DBasics shape)
   {
      return multiBodySystem ->
      {
         RigidBodyBasics dummyBody = new RigidBody("dummyBody", ReferenceFrame.getWorldFrame());

         return List.of(new Collidable(dummyBody, 0, 0, shape));
      };
   }

   private static short[] createDepthMeasures(float depth)
   {
      short[] depthValues = new short[imageWidth * imageHeight];
      Arrays.fill(depthValues, (short) (depth * 1000));
      return depthValues;
   }

   private static GpuMat convertDepthMeasuresToInputMat(short[] depth)
   {
      Mat depthMat = new Mat(imageHeight, imageWidth, opencv_core.CV_16UC1);
      ShortPointer sp = new ShortPointer(depth);
      depthMat.data().put(sp);
      GpuMat inputDepthImage = new GpuMat();
      inputDepthImage.upload(depthMat);

      return inputDepthImage;
   }

   private static void printGpuMat(String prefix, GpuMat matToPrint)
   {
      Mat test = new Mat();
      matToPrint.download(test);
      PerceptionDebugTools.printMat(prefix, test, 1);
   }


   private static Point3D[] convertDepthValuesToPoints(short[] depthValues, CameraIntrinsics cameraIntrinsics, int width)
   {
      Point3D[] points = new Point3D[depthValues.length];

      int x_index = 0;
      int y_index = 0;
      for (int i = 0; i < depthValues.length; i++, x_index++)
      {
         if (x_index == width)
         {
            x_index = 0;
            y_index++;
         }
         double depthInMeters = depthValues[i] / 1000.0;
         double y = -(x_index - cameraIntrinsics.getCx()) / cameraIntrinsics.getFx() * depthInMeters;
         double z = -(y_index - cameraIntrinsics.getCy()) / cameraIntrinsics.getFy() * depthInMeters;
         points[i] = new Point3D(depthInMeters, y, z);
      }

      return points;
   }

   private static void checkAgainstCPUCalculation(ReferenceFrame cameraFrame, short[] depthValues, Mat resultToCheck,
                                                  List<Collidable> collidables, CameraIntrinsics cameraIntrinsics, int imageWidth)
   {
      Point3D[] pointCloud = convertDepthValuesToPoints(depthValues, cameraIntrinsics, imageWidth);
      short[] expected = new short[depthValues.length];
      System.arraycopy(depthValues, 0, expected, 0, depthValues.length);

      for (Collidable collidable : collidables)
      {
         FrameShape3DReadOnly collisionShape = collidable.getShape();
         if (collisionShape instanceof FrameSphere3D sphere)
         {
            // Create a copy of the sphere, and then inflate it by the tolerance.
            FrameSphere3D sphereToCheck = new FrameSphere3D(sphere);
            sphereToCheck.checkReferenceFrameMatch(cameraFrame);
            sphereToCheck.setRadius(sphere.getRadius() + depthFilterTolerance);
            for (int i = 0; i < depthValues.length; i++)
            {
               Line3DReadOnly line = new Line3D(new Point3D(), pointCloud[i]);
               Point3D intersection1 = new Point3D();
               Point3D intersection2 = new Point3D();

               int intersections = sphereToCheck.intersectionWith(line, intersection1, intersection2);
               if (intersections > 0)
               {
                  if (MathTools.intervalContains(EuclidGeometryTools.percentageAlongLineSegment3D(intersection1, new Point3D(), pointCloud[i]), 0.0, 1.0))
                     expected[i] = 0;
               }
               if (intersections > 1)
               {
                  if (MathTools.intervalContains(EuclidGeometryTools.percentageAlongLineSegment3D(intersection2, new Point3D(), pointCloud[i]), 0.0, 1.0))
                     expected[i] = 0;
               }
            }
         }
         else
         {
            throw new RuntimeException("Haven't set this shape up yet");
         }
      }

      checkExpectedValues(expected, resultToCheck);
   }

   private static void checkExpectedValues(short[] expected, Mat result)
   {
      ShortBuffer resultBuf = result.createBuffer();

      // Check pixels against expected
      for (int i = 0; i < expected.length; i++)
      {
         short val = resultBuf.get(i);
         assertEquals(expected[i], val, "Pixel at index " + i + " should be " + expected[i]);
      }
   }


   private static void checkExpectedZeroValues(Mat result, int width, int height)
   {
      if (depthFilterTolerance != 0.1f)
         return;

      // These are the expected indices that will be zero based on the collision
      int[] zeroIndices = {35, 44, 45, 46, 53, 54, 55, 56, 57, 64, 65, 66, 75};

      ShortBuffer resultBuf = result.createBuffer();

      // Check zeroed pixels
      for (int idx : zeroIndices)
      {
         short val = resultBuf.get(idx);
         assertEquals(0, val, "Pixel at index " + idx + " should be zeroed due to collision");
      }

      // Check non-zero pixels remain unchanged
      for (int i = 0; i < width * height; i++)
      {
         boolean isZeroed = false;
         for (int idx : zeroIndices) if (i == idx) isZeroed = true;
         if (isZeroed) continue;

         short val = resultBuf.get(i);
         assertEquals(1000, val, "Pixel at index " + i + " should remain unchanged");
      }
   }

   /**
    * These are the camera intrinsics.
    */
   private static CameraIntrinsics setupCameraIntrinsics()
   {
      CameraIntrinsics cameraIntrinsics = new CameraIntrinsics();
      cameraIntrinsics.setFx(10.0);
      cameraIntrinsics.setFy(10.0);
      cameraIntrinsics.setCx(imageWidth / 2.0);
      cameraIntrinsics.setCy(imageHeight / 2.0);
      return cameraIntrinsics;
   }
}