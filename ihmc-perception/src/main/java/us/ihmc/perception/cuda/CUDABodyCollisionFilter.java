package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.opencv.opencv_core.GpuMat;
import us.ihmc.euclid.referenceFrame.FrameCapsule3D;
import us.ihmc.euclid.referenceFrame.FrameSphere3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.scs2.simulation.collision.Collidable;

import java.net.URL;
import java.util.List;

import static org.bytedeco.cuda.global.cudart.cudaFree;
import static org.bytedeco.cuda.global.cudart.cudaStreamSynchronize;

public class CUDABodyCollisionFilter
{
   private static final int BLOCK_SIZE_XY = 32;
   private final int NUMBER_OF_ATTRIBUTES = 7;

   private final CUDAProgram program;
   private final CUDAKernel kernel;
   private final CUstream_st stream;
   private final List<Collidable> robotCollidables;
   private final int dataSize;

   private final int numberOfCollidables;
   private final FloatPointer deviceCollidableGeometryPointer;
   private final FloatPointer collidableGeometryPointer;

   public CUDABodyCollisionFilter(List<Collidable> robotCollidables)
   {
      this.robotCollidables = robotCollidables;
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

      // Note: We get the expected number of collidables and allocate that memory amount on the GPU
      // By allocating once we don't have to free the memory in each call, just at the end
      numberOfCollidables = countSupportedCollidables(robotCollidables);
      dataSize = numberOfCollidables * NUMBER_OF_ATTRIBUTES;
      deviceCollidableGeometryPointer = new FloatPointer();
      CUDATools.mallocAsync(deviceCollidableGeometryPointer, dataSize, stream);

      collidableGeometryPointer = new FloatPointer((long) numberOfCollidables * NUMBER_OF_ATTRIBUTES);
   }

   public void process(GpuMat deviceInputDepthImage, GpuMat deviceOutputImageToPack, CameraIntrinsics cameraIntrinsics, ReferenceFrame cameraFrame)
   {
      int error;

      if (numberOfCollidables == 0)
      {
         // We have no collidables to process, return the original mat
         deviceInputDepthImage.copyTo(deviceOutputImageToPack);
         return;
      }

      getCollidablesPointer(robotCollidables, cameraFrame);

      CUDATools.memcpyAsync(deviceCollidableGeometryPointer, collidableGeometryPointer, dataSize, stream);

      int gridSizeX = (deviceInputDepthImage.cols() + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
      int gridSizeY = (deviceInputDepthImage.rows() + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
      dim3 gridSize = new dim3(gridSizeX, gridSizeY, 1);
      dim3 blockSize = new dim3(BLOCK_SIZE_XY, BLOCK_SIZE_XY, 1);

      kernel.withPointer(deviceInputDepthImage.data());
      kernel.withLong(deviceInputDepthImage.step());
      kernel.withInt(deviceInputDepthImage.cols());
      kernel.withInt(deviceInputDepthImage.rows());
      kernel.withFloat((float) cameraIntrinsics.getFx());
      kernel.withFloat((float) cameraIntrinsics.getFy());
      kernel.withFloat((float) cameraIntrinsics.getCx());
      kernel.withFloat((float) cameraIntrinsics.getCy());
      kernel.withPointer(deviceOutputImageToPack.data());
      kernel.withLong(deviceOutputImageToPack.step());
      kernel.withPointer(deviceCollidableGeometryPointer);
      kernel.withInt(numberOfCollidables);
      kernel.withInt(NUMBER_OF_ATTRIBUTES);

      kernel.run(stream, gridSize, blockSize, 0);
      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      blockSize.close();
      gridSize.close();
   }

   private int countSupportedCollidables(List<Collidable> robotCollidables)
   {
      if (robotCollidables == null)
         return 0;

      int count = 0;
      for (Collidable collidable : robotCollidables)
      {
         if (isCollidableShapeSupported(collidable))
         {
            count++;
         }
      }
      return count;
   }

   public void getCollidablesPointer(List<Collidable> robotCollidables, ReferenceFrame cameraFrame)
   {
      int index = 0;

      for (Collidable collidable : robotCollidables)
      {
         if (!isCollidableShapeSupported(collidable))
            continue;

         if (collidable.getShape() instanceof FrameSphere3D sphere)
         {
            FrameSphere3D bodypart = new FrameSphere3D(sphere);
            bodypart.changeFrame(cameraFrame);
            collidableGeometryPointer.put(index++, bodypart.getPosition().getX32());
            collidableGeometryPointer.put(index++, bodypart.getPosition().getY32());
            collidableGeometryPointer.put(index++, bodypart.getPosition().getZ32());
            collidableGeometryPointer.put(index++, bodypart.getPosition().getX32());
            collidableGeometryPointer.put(index++, bodypart.getPosition().getY32());
            collidableGeometryPointer.put(index++, bodypart.getPosition().getZ32());
            collidableGeometryPointer.put(index++, (float) bodypart.getRadius());
         }
         else if (collidable.getShape() instanceof FrameCapsule3D capsule)
         {
            FrameCapsule3D bodypart = new FrameCapsule3D(capsule);
            bodypart.changeFrame(cameraFrame);
            collidableGeometryPointer.put(index++, bodypart.getTopCenter().getX32());
            collidableGeometryPointer.put(index++, bodypart.getTopCenter().getY32());
            collidableGeometryPointer.put(index++, bodypart.getTopCenter().getZ32());
            collidableGeometryPointer.put(index++, bodypart.getBottomCenter().getX32());
            collidableGeometryPointer.put(index++, bodypart.getBottomCenter().getY32());
            collidableGeometryPointer.put(index++, bodypart.getBottomCenter().getZ32());
            collidableGeometryPointer.put(index++, (float) bodypart.getRadius());
         }
      }
   }

   private boolean isCollidableShapeSupported(Collidable collidable)
   {
      return (collidable.getShape() instanceof FrameCapsule3D || collidable.getShape() instanceof FrameSphere3D) && !collidable.getRigidBody()
                                                                                                                               .getName()
                                                                                                                               .matches("TORSO_LINK|PELVIS_LINK");
   }

   public void close()
   {
      // Need to free this memory on the GPU to shut down correctly
      cudaFree(deviceCollidableGeometryPointer);

      deviceCollidableGeometryPointer.close();
      collidableGeometryPointer.close();
      kernel.close();
      program.close();
      CUDAStreamManager.releaseStream(stream);
   }
}