package us.ihmc.perception.gpuMapping;

import org.bytedeco.cuda.cudart.CUstream_st;
import org.bytedeco.cuda.cudart.dim3;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.GpuMat;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.cuda.CUDAKernel;
import us.ihmc.perception.cuda.CUDAProgram;
import us.ihmc.perception.cuda.CUDAStreamManager;
import us.ihmc.perception.cuda.CUDATools;
import us.ihmc.perception.tools.PerceptionDebugTools;

import java.net.URL;

import static org.bytedeco.cuda.global.cudart.cudaMemset2DAsync;
import static org.bytedeco.cuda.global.cudart.cudaStreamSynchronize;

public class VoxelMapExtractor
{
   private static final boolean PRINT_TIMING_FOR_KERNELS = false;
   /**
    * The choice of 16 here is to utilize more SMs (Multi Processors) on the GPU.
    * This was chosen based on GPU profiling and significantly effects performance.
    */
   private static final int BLOCK_SIZE_XY = 8;

   private final CUstream_st stream;
   private final dim3 blockSize;
   private final FloatPointer sensorToWorldAlignedGroundTransformHostPointer;
   private final FloatPointer sensorToWorldAlignedGroundTransformDevicePointer;

   private final FloatPointer parametersHostPointer;
   private final FloatPointer parametersDevicePointer;
   private final CUDAProgram voxelMapProgram;
   private final GpuMat voxelMap;

   private final CUDAKernel voxelMapKernel;
   private final float[] sensorToWorldAlignedGroundTransformArray = new float[16];
   int cellsPerAxis = 10;
   private final float cellSize = 0.4f;

   public VoxelMapExtractor()
   {
      stream = CUDAStreamManager.getStream();

      // Load header and main file
      URL mathUtilsHeaderPath = getClass().getResource("/us/ihmc/perception/cuda/MathUtils.cuh");
      URL kernelPath = getClass().getResource("VoxelMapExtractor.cu");

      blockSize = new dim3(BLOCK_SIZE_XY, BLOCK_SIZE_XY, 1);

      try
      {
         voxelMapProgram = new CUDAProgram(kernelPath, mathUtilsHeaderPath);

         voxelMapKernel = voxelMapProgram.loadKernel("voxelMapKernel");
         voxelMapKernel.enableKernelTimings(PRINT_TIMING_FOR_KERNELS);

         voxelMap = new GpuMat(1, cellsPerAxis * cellsPerAxis * cellsPerAxis, opencv_core.CV_32SC1);

         // Initialize transformation pointers
         sensorToWorldAlignedGroundTransformHostPointer = new FloatPointer(16);
         sensorToWorldAlignedGroundTransformDevicePointer = new FloatPointer();

         parametersHostPointer = new FloatPointer(8);
         parametersDevicePointer = new FloatPointer();
      }
      catch (Exception e)
      {
         throw new RuntimeException(e);
      }
   }

   public void update(GpuMat latestDepthImageGPU,
                      CameraIntrinsics cameraIntrinsics,
                      RigidBodyTransform sensorToWorldTransform,
                      RigidBodyTransform sensorToGroundTransform,
                      RigidBodyTransform groundToWorldTransform,
                      Point3D voxelMapCenter)
   {

      // Populate parameter buffers with the necessary values
      float[] parametersArray = populateParameterArray(cameraIntrinsics);
      parametersHostPointer.put(parametersArray);
      CUDATools.mallocAsync(parametersDevicePointer, parametersArray.length, stream);
      CUDATools.memcpyAsync(parametersDevicePointer, parametersHostPointer, parametersArray.length, stream);

      // Remove rotation from transformation
      RigidBodyTransform groundToWorldNoRotation = new RigidBodyTransform(groundToWorldTransform);
      groundToWorldNoRotation.getRotation().setIdentity();

      // Invert translation-only transform
      RigidBodyTransform worldToGroundNoRotation = new RigidBodyTransform(groundToWorldNoRotation);
      worldToGroundNoRotation.invert();

      // This transformation only has rotation
      RigidBodyTransform groundToWorldAlignedGround = new RigidBodyTransform(worldToGroundNoRotation);
      groundToWorldAlignedGround.multiply(groundToWorldTransform);

      // Step 3: Multiply with full ground->world to keep rotation, giving aligned ground
      RigidBodyTransform sensorToWorldAlignedGround = new RigidBodyTransform(worldToGroundNoRotation);
      sensorToWorldAlignedGround.multiply(groundToWorldTransform);

      // Step 4: Apply sensor->ground transform
      sensorToWorldAlignedGround.multiply(sensorToGroundTransform);

      sensorToWorldAlignedGround.get(this.sensorToWorldAlignedGroundTransformArray);
      sensorToWorldAlignedGroundTransformHostPointer.put(this.sensorToWorldAlignedGroundTransformArray);
      CUDATools.mallocAsync(sensorToWorldAlignedGroundTransformDevicePointer, this.sensorToWorldAlignedGroundTransformArray.length, stream);
      CUDATools.memcpyAsync(sensorToWorldAlignedGroundTransformDevicePointer,
                            sensorToWorldAlignedGroundTransformHostPointer,
                            this.sensorToWorldAlignedGroundTransformArray.length,
                            stream);

      cudaMemset2DAsync(voxelMap.data(), voxelMap.step(), 0, (long) voxelMap.cols() * Integer.BYTES, voxelMap.rows());

      int gridDimX = (latestDepthImageGPU.cols() + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
      int gridDimY = (latestDepthImageGPU.rows() + BLOCK_SIZE_XY - 1) / BLOCK_SIZE_XY;
      dim3 registerKernelGridDim = new dim3(gridDimX, gridDimY, 1);

      voxelMapKernel.withPointer(latestDepthImageGPU.data()).withLong(latestDepthImageGPU.step());
      voxelMapKernel.withPointer(voxelMap.data()).withLong(voxelMap.step());
      voxelMapKernel.withPointer(sensorToWorldAlignedGroundTransformDevicePointer);
      voxelMapKernel.withPointer(parametersDevicePointer);

      voxelMapKernel.run(stream, registerKernelGridDim, blockSize, 0);

      registerKernelGridDim.close();

      int error = cudaStreamSynchronize(stream);
      CUDATools.checkCUDAError(error);

      Mat what = new Mat();
      voxelMap.download(what);
      PerceptionDebugTools.printMat("s", what, 1);
   }

   public float[] populateParameterArray(CameraIntrinsics cameraIntrinsics)
   {
      return new float[] {(float) cameraIntrinsics.getHeight(),
                          (float) cameraIntrinsics.getWidth(),
                          (float) cameraIntrinsics.getCx(),
                          (float) cameraIntrinsics.getCy(),
                          (float) cameraIntrinsics.getFx(),
                          (float) cameraIntrinsics.getFy(),
                          (float) cellSize,
                          (float) cellsPerAxis};
   }

   public float getCellSize()
   {
      return cellSize;
   }

   public int getCellsPerAxis()
   {
      return cellsPerAxis;
   }

   public GpuMat getVoxelMap()
   {
      return voxelMap;
   }

   public void destroy()
   {
      voxelMap.close();
      blockSize.close();
      voxelMapProgram.close();
      voxelMapKernel.close();
   }
}
