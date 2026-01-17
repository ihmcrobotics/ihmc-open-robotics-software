package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.gpuMapping.VoxelMapExtractor;

public class VoxelMapExtractorTest
{
   /**
    * Fake test to see if we are creating any memory leaks on the GPU or the CPU.
    * This runs the update over and over again to ensure we don't run out of memory.
    * Manually check the memory management with htop and nvidia-smi
    */
   @Test
   public void testRapidVoxelMapExtractor()
   {
      VoxelMapExtractor voxelMapExtractor = new VoxelMapExtractor();

      // Create fake data for the test
      GpuMat gpuMat = new GpuMat(1280, 720, opencv_core.CV_16UC1);
      gpuMat.setTo(new Scalar(3000));
      CameraIntrinsics cameraIntrinsics = new CameraIntrinsics();
      // Need to set these to the size of the image
      cameraIntrinsics.setWidth(1280);
      cameraIntrinsics.setHeight(720);
      double[] transformArray = getDoubles();

      RigidBodyTransform rigidBodyTransform = new RigidBodyTransform(transformArray);

      for (int i = 0; i < 1000; i++)
      {
         voxelMapExtractor.update(gpuMat, cameraIntrinsics, rigidBodyTransform, rigidBodyTransform, rigidBodyTransform, new Point3D(1.0, 2.0, 3.0));
      }

      voxelMapExtractor.destroy();
   }
   /**
    * Create a fake transformation matrix just for the test
    */
   private static double[] getDoubles()
   {
      double[] transformArray = new double[12];
      // Identity rotation matrix
      transformArray[0] = 1.0;  // R00
      transformArray[1] = 0.0;  // R01
      transformArray[2] = 0.0;  // R02
      transformArray[3] = 1.0;  // Tx

      transformArray[4] = 0.0;  // R10
      transformArray[5] = 1.0;  // R11
      transformArray[6] = 0.0;  // R12
      transformArray[7] = 2.0;  // Ty

      transformArray[8] = 0.0; // R20
      transformArray[9] = 0.0; // R21
      transformArray[10] = 1.0; // R22
      transformArray[11] = 3.0; // Tz
      return transformArray;
   }


}
