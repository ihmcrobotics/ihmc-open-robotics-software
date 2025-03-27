package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.euclid.referenceFrame.FrameCapsule3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.scs2.simulation.collision.Collidable;

import java.net.URL;
import java.util.List;

import static org.bytedeco.cuda.global.cudart.*;

public class CUDABodyCollisionFilter {
   private static final int BLOCK_SIZE_XY = 16;

   // CUDA stuff
   private final CUDAProgram program;
   private final CUDAKernel kernel;
   private final CUstream_st stream;

   // Input data
   private List<Collidable> robotCollidables;
   private ReferenceFrame cameraFrame;
   private CameraIntrinsics cameraIntrinsics;
   private Mat latestDepthImage;

   // Output data
   private IntPointer collisionResults; // Results from the kernel
   private int error;

   public CUDABodyCollisionFilter() {
      // Get URLs to the CUDA files
      URL bodyCollisionCheckURL = getClass().getResource("BodyCollisionCheck.cu");
      URL utilsURL = getClass().getResource("Utils.cu");
      URL perceptionUtilsURL = getClass().getResource("PerceptionUtils.cu");
      URL mathUtilsURL = getClass().getResource("MathUtils.cuh");

      // Compile the program, and get the kernel
      try {
         program = new CUDAProgram(bodyCollisionCheckURL, utilsURL, perceptionUtilsURL, mathUtilsURL);
         kernel = program.loadKernel("checkBodyCollision");
      } catch (Exception e) {
         throw new RuntimeException(e);
      }

      // Get a stream
      stream = CUDAStreamManager.getStream();
   }

   public void process(Mat depthImage, CameraIntrinsics cameraIntrinsics, List<Collidable> robotCollidables, ReferenceFrame cameraFrame) {
      this.latestDepthImage = depthImage;
      this.cameraIntrinsics = cameraIntrinsics;
      this.robotCollidables = robotCollidables;
      this.cameraFrame = cameraFrame;

      // Prepare data for the kernel
      double[][] collidableGeometry = getCollidables();
      if(collidableGeometry.length == 0) return;
      FloatPointer collidableGeometryPointer = new FloatPointer(collidableGeometry.length * 7); // 7 floats per collidable
      for (int i = 0; i < collidableGeometry.length; i++) {
         for (int j = 0; j < 7; j++) {
            collidableGeometryPointer.put(i * 7 + j, (float) collidableGeometry[i][j]);
         }
      }

      // Prepare result pointer
      int numPixels = depthImage.getWidth() * depthImage.getHeight();
      if (collisionResults == null || collisionResults.limit() < numPixels) {
         if (collisionResults != null) {
            error = cudaFreeAsync(collisionResults, stream);
            CUDATools.checkCUDAError(error);
         }
         CUDATools.mallocAsync(collisionResults, numPixels, stream);
         collisionResults.limit(numPixels);
      }

      // Calculate block size and grid size of the kernel launch
      dim3 blockSize = new dim3(BLOCK_SIZE_XY, BLOCK_SIZE_XY, 1);

      int gridSizeX = (depthImage.getWidth() + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
      int gridSizeY = (depthImage.getHeight() + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
      dim3 gridSize = new dim3(gridSizeX, gridSizeY, 1);

      // Run the kernel
      kernel.withPointer(latestDepthImage.data())
            .withLong(latestDepthImage.step())
            .withInt(depthImage.getWidth())
            .withInt(depthImage.getHeight())
            .withFloat((float) cameraIntrinsics.getFx())
            .withFloat((float) cameraIntrinsics.getFy())
            .withFloat((float) cameraIntrinsics.getCx())
            .withFloat((float) cameraIntrinsics.getCy())
            .withPointer(collidableGeometryPointer)
            .withInt(collidableGeometry.length)
            .withPointer(collisionResults)
            .run(stream, gridSize, blockSize, 0);

      // Synchronize the stream to ensure we can read the results
      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      // Release pointers
      blockSize.close();
      gridSize.close();
      collidableGeometryPointer.close();
   }

   public IntPointer getCollisionResults() {
      return collisionResults;
   }

   public double[][] getCollidables() {
      if (robotCollidables == null || robotCollidables.isEmpty()) {
         return new double[0][0];
      }

      int numCollidables = 0;
      for (Collidable collidable : robotCollidables) {
         if (collidable.getShape() instanceof FrameCapsule3D) {
            numCollidables++;
         }
      }

      double[][] collidableData = new double[numCollidables][7];
      int rowIndex = 0;

      for (Collidable collidable : robotCollidables) {
         if (collidable.getShape() instanceof FrameCapsule3D capsule) {
            FrameCapsule3D bodypart = new FrameCapsule3D(capsule);
            bodypart.changeFrame(cameraFrame);

            collidableData[rowIndex][0] = bodypart.getTopCenter().getX();
            collidableData[rowIndex][1] = bodypart.getTopCenter().getY();
            collidableData[rowIndex][2] = bodypart.getTopCenter().getZ();
            collidableData[rowIndex][3] = bodypart.getBottomCenter().getX();
            collidableData[rowIndex][4] = bodypart.getBottomCenter().getY();
            collidableData[rowIndex][5] = bodypart.getBottomCenter().getZ();
            collidableData[rowIndex][6] = bodypart.getRadius();

            rowIndex++;
         }
      }

      return collidableData;
   }

   public void close() {
      if (collisionResults != null) {
         CUDATools.checkCUDAError(cudaFreeAsync(collisionResults, stream));
      }
      kernel.close();
      program.close();
      CUDAStreamManager.releaseStream(stream);
   }
}