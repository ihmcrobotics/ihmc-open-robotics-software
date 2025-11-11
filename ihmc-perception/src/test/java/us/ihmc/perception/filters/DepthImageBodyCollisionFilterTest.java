package us.ihmc.perception.filters;

import org.bytedeco.javacpp.ShortPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.referenceFrame.FrameSphere3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
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
   private final CameraIntrinsics cameraIntrinsics = new CameraIntrinsics();

   @Test
   public void testDepthPointsInsideBodyCollision()
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

      RobotCollisionModel robotCollisionModel = multiBodySystem ->
      {
         RigidBodyBasics dummyBody = new RigidBody("dummyBody", ReferenceFrame.getWorldFrame());
         FrameSphere3D sphere3D = new FrameSphere3D(cameraFrame, 0.1);
         sphere3D.getPosition().set(1.0, 0.0, 0.0);


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

      checkExpectedZeroValues(result, width, height);
   }

   @Test
   public void testDephtPointsPastBodyCollisionButOnRay()
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