package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.euclid.referenceFrame.FrameCapsule3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.tools.PerceptionDebugTools;
import us.ihmc.scs2.simulation.collision.Collidable;

import java.net.URL;
import java.util.List;

import static org.bytedeco.cuda.global.cudart.cudaFreeAsync;
import static org.bytedeco.cuda.global.cudart.cudaStreamSynchronize;
import static org.bytedeco.opencv.global.opencv_core.CV_16UC1;

public class CUDABodyCollisionFilter
{
   private static final int BLOCK_SIZE_XY = 16;

   private final CUDAProgram program;
   private final CUDAKernel kernel;
   private final CUstream_st stream;

   private GpuMat collisionResults; // Results from the kernel
   private int error;
   private int numCollidables;

   public CUDABodyCollisionFilter()
   {
      URL bodyCollisionCheckURL = getClass().getResource("BodyCollisionCheck.cu");
      URL utilsURL = getClass().getResource("Utils.cu");
      URL perceptionUtilsURL = getClass().getResource("PerceptionUtils.cu");
      URL mathUtilsURL = getClass().getResource("MathUtils.cuh");

      try
      {
         program = new CUDAProgram(bodyCollisionCheckURL, utilsURL, perceptionUtilsURL, mathUtilsURL);
         kernel = program.loadKernel("checkBodyCollision");
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }

      stream = CUDAStreamManager.getStream();
      if (stream == null)
      {
         throw new RuntimeException("CUDA stream could not be initialized!");
      }
   }

   public void process(GpuMat latestDepthImage,
                       CameraIntrinsics cameraIntrinsics,
                       List<Collidable> robotCollidables,
                       ReferenceFrame cameraFrame)
   {
      if (collisionResults != null)
      {
         collisionResults.release();
         collisionResults = null;
      }

      numCollidables = countCapsules(robotCollidables);

      if (numCollidables == 0)
      {
         System.out.println("No collidables detected. Skipping processing.");
         return;
      }

      FloatPointer collidableGeometryPointer = getCollidablesPointer(robotCollidables, cameraFrame);
      int dataSize = collidableGeometryPointer.sizeof() * Float.BYTES;
      FloatPointer deviceCollidableGeometryPointer = new FloatPointer();
      CUDATools.mallocAsync(deviceCollidableGeometryPointer, dataSize, stream);
      CUDATools.memcpyAsync(deviceCollidableGeometryPointer, collidableGeometryPointer, dataSize, stream);

      collisionResults = new GpuMat(latestDepthImage.rows(), latestDepthImage.cols(), CV_16UC1);

      dim3 blockSize = new dim3(BLOCK_SIZE_XY, BLOCK_SIZE_XY, 1);

      int gridSizeX = (latestDepthImage.cols() + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
      int gridSizeY = (latestDepthImage.rows() + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
      dim3 gridSize = new dim3(gridSizeX, gridSizeY, 1);

      kernel.withPointer(latestDepthImage.data())
            .withLong(latestDepthImage.step())
            .withInt(latestDepthImage.cols())
            .withInt(latestDepthImage.rows())
            .withFloat((float) cameraIntrinsics.getFx())
            .withFloat((float) cameraIntrinsics.getFy())
            .withFloat((float) cameraIntrinsics.getCx())
            .withFloat((float) cameraIntrinsics.getCy())
            .withPointer(collisionResults.data()) // changed to collisionResults.data()
            .withLong(collisionResults.step())
            .withPointer(deviceCollidableGeometryPointer)
            .withInt(numCollidables)
            .run(stream, gridSize, blockSize, 0);

      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      Mat hostResults = new Mat();
      collisionResults.download(hostResults);
      PerceptionDebugTools.displayDepth("mask", hostResults, 1);

      blockSize.close();
      gridSize.close();
      collidableGeometryPointer.close();
      deviceCollidableGeometryPointer.close();
   }

   private int countCapsules(List<Collidable> robotCollidables)
   {
      int count = 0;
      if (robotCollidables != null)
      {
         for (Collidable collidable : robotCollidables)
         {
            if (collidable.getShape() instanceof FrameCapsule3D)
            {
               count++;
            }
         }
      }
      return count;
   }

   public FloatPointer getCollidablesPointer(List<Collidable> robotCollidables, ReferenceFrame cameraFrame)
   {
      if (numCollidables == 0)
      {
         return new FloatPointer(0);
      }

      FloatPointer geometryPointer = new FloatPointer(numCollidables * 7);

      int index = 0;
      for (Collidable collidable : robotCollidables)
      {
         if (collidable.getShape() instanceof FrameCapsule3D capsule)
         {
            FrameCapsule3D bodypart = new FrameCapsule3D(capsule);
            bodypart.changeFrame(cameraFrame);
            geometryPointer.put(index++, (float) bodypart.getTopCenter().getX());
            geometryPointer.put(index++, (float) bodypart.getTopCenter().getY());
            geometryPointer.put(index++, (float) bodypart.getTopCenter().getZ());
            geometryPointer.put(index++, (float) bodypart.getBottomCenter().getX());
            geometryPointer.put(index++, (float) bodypart.getBottomCenter().getY());
            geometryPointer.put(index++, (float) bodypart.getBottomCenter().getZ());
            geometryPointer.put(index++, (float) bodypart.getRadius());
         }
      }
      return geometryPointer;
   }

   public void close()
   {
      if (collisionResults != null)
      {
         CUDATools.checkCUDAError(cudaFreeAsync(collisionResults.data(), stream));
         collisionResults.release();
      }
      kernel.close();
      program.close();
      CUDAStreamManager.releaseStream(stream);
   }
}