package us.ihmc.perception.cuda;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.ShortPointer;
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

public class CUDABodyCollisionFilter {
   private static final int BLOCK_SIZE_XY = 16;

   private final CUDAProgram program;
   private final CUDAKernel kernel;
   private final CUstream_st stream;

   private List<Collidable> robotCollidables;
   private ReferenceFrame cameraFrame;
   private CameraIntrinsics cameraIntrinsics;
   private GpuMat latestDepthImage;

   private GpuMat collisionResults; // Results from the kernel
   private int error;

   public CUDABodyCollisionFilter() {
      URL bodyCollisionCheckURL = getClass().getResource("BodyCollisionCheck.cu");
      URL utilsURL = getClass().getResource("Utils.cu");
      URL perceptionUtilsURL = getClass().getResource("PerceptionUtils.cu");
      URL mathUtilsURL = getClass().getResource("MathUtils.cuh");

      try {
         program = new CUDAProgram(bodyCollisionCheckURL, utilsURL, perceptionUtilsURL, mathUtilsURL);
         kernel = program.loadKernel("checkBodyCollision");
      } catch (Exception e) {
         throw new RuntimeException(e);
      }

      stream = CUDAStreamManager.getStream();
   }

   public void process(GpuMat depthImage,
                       CameraIntrinsics cameraIntrinsics,
                       List<Collidable> robotCollidables,
                       ReferenceFrame cameraFrame) {
      this.latestDepthImage = depthImage;
      this.cameraIntrinsics = cameraIntrinsics;
      this.robotCollidables = robotCollidables;
      this.cameraFrame = cameraFrame;

      double[][] collidableGeometry = getCollidables();
      if (collidableGeometry.length == 0)
         return;
      FloatPointer collidableGeometryPointer = new FloatPointer(
            collidableGeometry.length * 7); // 7 floats per collidable
      for (int i = 0; i < collidableGeometry.length; i++) {
         for (int j = 0; j < 7; j++) {
            collidableGeometryPointer.put(i * 7 + j, (float) collidableGeometry[i][j]);
         }
      }

      // Prepare result pointer. changed to CV_16UC1 to match the kernel.
      collisionResults = new GpuMat(depthImage.rows(), depthImage.cols(), CV_16UC1);

      // Calculate block size and grid size of the kernel launch
      dim3 blockSize = new dim3(BLOCK_SIZE_XY, BLOCK_SIZE_XY, 1);

      int gridSizeX = (depthImage.cols() + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
      int gridSizeY = (depthImage.rows() + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
      dim3 gridSize = new dim3(gridSizeX, gridSizeY, 1);

      // Run the kernel
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
            .withPointer(collidableGeometryPointer)
            .withInt(collidableGeometry.length)
            .run(stream, gridSize, blockSize, 0);

      // Synchronize the stream to ensure we can read the results
      error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      Mat hostResults = new Mat(latestDepthImage.rows(), latestDepthImage.cols(), CV_16UC1);
      collisionResults.download(hostResults);
      PerceptionDebugTools.display("mask", hostResults, 10);

      // Release pointers
      blockSize.close();
      gridSize.close();
      collidableGeometryPointer.close();
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
         CUDATools.checkCUDAError(cudaFreeAsync(collisionResults.data(), stream)); // changed to collisionResults.data()
      }
      kernel.close();
      program.close();
      CUDAStreamManager.releaseStream(stream);
   }
}