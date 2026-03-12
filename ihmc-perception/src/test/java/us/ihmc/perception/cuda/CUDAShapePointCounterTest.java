package us.ihmc.perception.cuda;

import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Scalar;
import org.junit.jupiter.api.Test;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.perception.RawImage;
import us.ihmc.sensors.CameraIntrinsics;

import java.time.Instant;

import static org.bytedeco.opencv.global.opencv_core.CV_16U;
import static org.junit.jupiter.api.Assertions.*;

public class CUDAShapePointCounterTest
{
   @Test
   public void testCountPointsInSphere()
   {
      CUDAShapePointCounter counter = new CUDAShapePointCounter();

      GpuMat gpuMat = new GpuMat(720, 1280, CV_16U, new Scalar(1000));
      CameraIntrinsics cameraIntrinsics = new CameraIntrinsics(720, 1280, 2000.0, 2000.0, 0.5 * 1280, 0.5 * 720);
      RawImage depthImage = RawImage.createWith16BitDepth(gpuMat, cameraIntrinsics, new RigidBodyTransform(), Instant.now(), 0, 0.001f);

      long points = counter.countPointsInSphere(depthImage, new Vector3D(1.0, 0.0, 0.0), 5.0f);
      assertEquals(1280 * 720, points);

      counter.close();
   }

   @Test
   public void testCountPointsInCapsule()
   {
      CUDAShapePointCounter counter = new CUDAShapePointCounter();

      GpuMat gpuMat = new GpuMat(720, 1280, CV_16U, new Scalar(1000));
      CameraIntrinsics cameraIntrinsics = new CameraIntrinsics(720, 1280, 2000.0, 2000.0, 0.5 * 1280, 0.5 * 720);
      RawImage depthImage = RawImage.createWith16BitDepth(gpuMat, cameraIntrinsics, new RigidBodyTransform(), Instant.now(), 0, 0.001f);

      long points = counter.countPointsInCapsule(depthImage, new Vector3D(1.0, 0.0, -0.5), new Vector3D(1.0, 0.0, 0.5), 5.0f);
      assertEquals(1280 * 720, points);

      counter.close();
   }
}
