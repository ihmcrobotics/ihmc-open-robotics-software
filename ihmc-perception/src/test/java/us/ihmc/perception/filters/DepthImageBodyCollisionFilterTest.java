package us.ihmc.perception.filters;

import org.bytedeco.javacpp.ShortPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.interfaces.Line3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameSphere3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.FrameShape3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.robotics.physics.RobotCollisionModel;
import us.ihmc.scs2.simulation.collision.Collidable;

import java.nio.ShortBuffer;
import java.util.List;

import static org.junit.jupiter.api.Assertions.*;

class DepthImageBodyCollisionFilterTest
{
   private static final double depthFilterTolerance = 0.1;

   private final CameraIntrinsics cameraIntrinsics = new CameraIntrinsics();

   @Test
   public void testDepthPointsInsideBodyCollision()
   {
      ///// This test is a simple camera looking down at the ground from 1 meter high. There's a single collision sphere that's 0.1 radius sitting on the ground,
      ///// centered at (0.0, 0.0) (x,y) in the world.

      // First, set up the camera frame and object
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

      // Now, set up the sphere collidable at (0.0, 0.0, 0.0) in the world.
      RobotCollisionModel robotCollisionModel = multiBodySystem ->
      {
         RigidBodyBasics dummyBody = new RigidBody("dummyBody", ReferenceFrame.getWorldFrame());
         FrameSphere3D sphere3D = new FrameSphere3D(cameraFrame, 0.1);

         FramePoint3D position = new FramePoint3D(ReferenceFrame.getWorldFrame());
         position.changeFrame(cameraFrame);

         sphere3D.getPosition().set(position);


         return List.of(new Collidable(dummyBody, 0, 0, sphere3D));
      };

      // Create the filter.
      RigidBody dummyBody = new RigidBody("dummyBody", ReferenceFrame.getWorldFrame());
      DepthImageBodyCollisionFilter depthImageBodyCollisionFilter = new DepthImageBodyCollisionFilter(robotCollisionModel, dummyBody);

      // Create spoofed depth values.
      short[] depthValues = new short[]
            {
                  1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000,
                  1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000,
                  1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000,
                  1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000,
                  1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000,
                  1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000,
                  1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000,
                  1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000,
                  1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000,
                  1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000
            };

      int width = 10;
      int height = 10;
      setupCameraIntrinsics(width, height);

      Mat depthMat = new Mat(height, width, opencv_core.CV_16UC1);
      ShortPointer sp = new ShortPointer(depthValues);
      depthMat.data().put(sp);
      GpuMat inputDepthImage = new GpuMat();
      inputDepthImage.upload(depthMat);
      GpuMat outputFilteredDepthImage = new GpuMat(inputDepthImage.size(), inputDepthImage.type());

      Mat test = new Mat();
      inputDepthImage.download(test);
      PerceptionDebugTools.printMat("s", test, 1);

      depthImageBodyCollisionFilter.process(inputDepthImage, outputFilteredDepthImage, cameraIntrinsics, cameraFrame);

      Mat result = new Mat();
      outputFilteredDepthImage.download(result);

      PerceptionDebugTools.printMat("w", result, 1);

      // Check against the Euclid objects
      checkAgainstCPUCalculation(cameraFrame, depthValues, result, robotCollisionModel.getRobotCollidables(dummyBody), cameraIntrinsics, width);

//      checkExpectedZeroValues(result, width, height);
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
               int intersections = sphereToCheck.intersectionWith(line, null, null);
               if (intersections > 0)
               {
                  expected[i] = 0;
               }
            }

         }
         else
         {
            throw new RuntimeException("Haven't set this shape up yet");
         }
      }

      ShortBuffer resultBuf = resultToCheck.createBuffer();

      // Check zeroed pixels
      for (int idx = 0; idx < depthValues.length; idx++)
      {
         short val = resultBuf.get(idx);
         assertEquals(expected[idx], val, "Pixel at index " + idx + " should be " + expected[idx]);
      }

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

   @Test
   public void testDepthPointsPastBodyCollisionButOnRay()
   {
      RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
      rigidBodyTransform.getTranslation().set(0.0, 0.0, 1.0);
      rigidBodyTransform.getRotation().setYawPitchRoll(0.0, Math.toRadians(90.0), 0.0);

      ReferenceFrame cameraFrame = new ReferenceFrame("cameraFrame", ReferenceFrame.getWorldFrame())
      {
         @Override
         protected void updateTransformToParent(RigidBodyTransform transformToParent)
         {
            transformToParent.set(rigidBodyTransform);
         }
      };

      RobotCollisionModel robotCollisionModel = multiBodySystem ->
      {
         RigidBodyBasics dummyBody = new RigidBody("dummyBody", ReferenceFrame.getWorldFrame());
         FrameSphere3D sphere3D = new FrameSphere3D(cameraFrame, 0.1);
         sphere3D.getPosition().set(0.5, 0.0, 0.0); // same units as depth (10–16)


         return List.of(new Collidable(dummyBody, 0, 0, sphere3D));
      };

      RigidBody dummyBody = new RigidBody("dummyBody", ReferenceFrame.getWorldFrame());
      DepthImageBodyCollisionFilter depthImageBodyCollisionFilter = new DepthImageBodyCollisionFilter(robotCollisionModel, dummyBody);

      short[] depthValues = new short[]
            {
                  1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000,
                  1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000,
                  1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000,
                  1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000,
                  1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000,
                  1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000,
                  1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000,
                  1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000,
                  1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000,
                  1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000, 1000
            };

      int width = 10;
      int height = 10;
      setupCameraIntrinsics(width, height);

      Mat depthMat = new Mat(height, width, opencv_core.CV_16UC1);
      ShortPointer sp = new ShortPointer(depthValues);
      depthMat.data().put(sp);
      GpuMat inputDepthImage = new GpuMat();
      inputDepthImage.upload(depthMat);
      GpuMat outputFilteredDepthImage = new GpuMat(inputDepthImage.size(), inputDepthImage.type());

      Mat test = new Mat();
      inputDepthImage.download(test);
      PerceptionDebugTools.printMat("s", test, 1);

      depthImageBodyCollisionFilter.process(inputDepthImage, outputFilteredDepthImage, cameraIntrinsics, cameraFrame);

      Mat result = new Mat();
      outputFilteredDepthImage.download(result);

      PerceptionDebugTools.printMat("w", result, 1);

      checkAgainstCPUCalculation(cameraFrame, depthValues, result, robotCollisionModel.getRobotCollidables(dummyBody), cameraIntrinsics, width);

      checkExpectedZeroValues(result, width, height);
   }


   private static void checkExpectedZeroValues(Mat result, int width, int height)
   {
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
   private void setupCameraIntrinsics(int uSize, int vSize)
   {
      cameraIntrinsics.setFx(10.0);
      cameraIntrinsics.setFy(10.0);
      cameraIntrinsics.setCx(uSize/2.0);
      cameraIntrinsics.setCy(vSize/2.0);
   }
}