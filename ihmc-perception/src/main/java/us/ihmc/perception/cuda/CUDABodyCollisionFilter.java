package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import us.ihmc.euclid.referenceFrame.FrameCapsule3D;
import us.ihmc.euclid.referenceFrame.FrameSphere3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.scs2.simulation.collision.Collidable;

import java.net.URL;
import java.util.List;

import static org.bytedeco.cuda.global.cudart.cudaStreamSynchronize;

public class CUDABodyCollisionFilter
{
   private static final int BLOCK_SIZE_XY = 32;
   private final int NUMBER_OF_ATTRIBUTES = 7;

   private final CUDAProgram program;
   private final CUDAKernel kernel;
   private final CUstream_st stream;
   private final List<Collidable> robotCollidables;

   private int numberOfCollidables;

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
   }

   public GpuMat process(GpuMat depthImage,
                         CameraIntrinsics cameraIntrinsics,
                         ReferenceFrame cameraFrame)
   {
      numberOfCollidables = countCapsules(robotCollidables);

      if (numberOfCollidables == 0)
      {
         return depthImage.clone();
      }

      GpuMat depthImageWithoutRobot = new GpuMat(depthImage.size(), opencv_core.CV_16UC1);

      FloatPointer collidableGeometryPointer = getCollidablesPointer(robotCollidables, cameraFrame);

      int dataSize = numberOfCollidables * NUMBER_OF_ATTRIBUTES * Float.BYTES;

      FloatPointer deviceCollidableGeometryPointer = new FloatPointer();
      CUDATools.mallocAsync(deviceCollidableGeometryPointer, dataSize, stream);
      CUDATools.memcpyAsync(deviceCollidableGeometryPointer, collidableGeometryPointer, dataSize, stream);

      dim3 blockSize = new dim3(BLOCK_SIZE_XY, BLOCK_SIZE_XY, 1);

      int gridSizeX = (depthImage.cols() + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
      int gridSizeY = (depthImage.rows() + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
      dim3 gridSize = new dim3(gridSizeX, gridSizeY, 1);

      kernel.withPointer(depthImage.data());
      kernel.withLong(depthImage.step());
      kernel.withInt(depthImage.cols());
      kernel.withInt(depthImage.rows());
      kernel.withFloat((float) cameraIntrinsics.getFx());
      kernel.withFloat((float) cameraIntrinsics.getFy());
      kernel.withFloat((float) cameraIntrinsics.getCx());
      kernel.withFloat((float) cameraIntrinsics.getCy());
      kernel.withPointer(depthImageWithoutRobot.data());
      kernel.withLong(depthImageWithoutRobot.step());
      kernel.withPointer(deviceCollidableGeometryPointer);
      kernel.withInt(numberOfCollidables);
      kernel.withInt(NUMBER_OF_ATTRIBUTES);

      kernel.run(stream, gridSize, blockSize, 0);
      int error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      blockSize.close();
      gridSize.close();
      collidableGeometryPointer.close();
      deviceCollidableGeometryPointer.close();

      return depthImageWithoutRobot;
   }

   private int countCapsules(List<Collidable> robotCollidables)
   {
      if (robotCollidables == null)
         return 0;

      int count = 0;
      for (Collidable collidable : robotCollidables)

      {
         if ((collidable.getShape() instanceof FrameCapsule3D || collidable.getShape() instanceof FrameSphere3D) && !collidable.getRigidBody()
                                                                                                                               .getName()
                                                                                                                               .matches("TORSO_LINK|PELVIS_LINK"))
         {
            count++;
         }
      }
      return Math.max(count, 0); // Avoid negative count
   }

   public FloatPointer getCollidablesPointer(List<Collidable> robotCollidables, ReferenceFrame cameraFrame)
   {
      if (numberOfCollidables == 0)
         return new FloatPointer(0);

      FloatPointer geometryPointer = new FloatPointer((long) numberOfCollidables * NUMBER_OF_ATTRIBUTES);
      int index = 0;

      for (Collidable collidable : robotCollidables)
      {
         if (collidable.getShape() instanceof FrameSphere3D sphere)
         {
            FrameSphere3D bodypart = new FrameSphere3D(sphere);
            bodypart.changeFrame(cameraFrame);
            geometryPointer.put(index++, bodypart.getPosition().getX32());
            geometryPointer.put(index++, bodypart.getPosition().getY32());
            geometryPointer.put(index++, bodypart.getPosition().getZ32());
            geometryPointer.put(index++, bodypart.getPosition().getX32());
            geometryPointer.put(index++, bodypart.getPosition().getY32());
            geometryPointer.put(index++, bodypart.getPosition().getZ32());
            geometryPointer.put(index++, (float) bodypart.getRadius());
         }
         if (collidable.getShape() instanceof FrameCapsule3D capsule && !collidable.getRigidBody().getName().matches("TORSO_LINK|PELVIS_LINK"))
         {
            FrameCapsule3D bodypart = new FrameCapsule3D(capsule);
            bodypart.changeFrame(cameraFrame);
            geometryPointer.put(index++, bodypart.getTopCenter().getX32());
            geometryPointer.put(index++, bodypart.getTopCenter().getY32());
            geometryPointer.put(index++, bodypart.getTopCenter().getZ32());
            geometryPointer.put(index++, bodypart.getBottomCenter().getX32());
            geometryPointer.put(index++, bodypart.getBottomCenter().getY32());
            geometryPointer.put(index++, bodypart.getBottomCenter().getZ32());
            geometryPointer.put(index++, (float) bodypart.getRadius());
         }
      }
      return geometryPointer;
   }

   public void close()
   {
      kernel.close();
      program.close();
      CUDAStreamManager.releaseStream(stream);
   }
}