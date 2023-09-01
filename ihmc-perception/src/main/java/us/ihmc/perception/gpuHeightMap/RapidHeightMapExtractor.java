package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.opencl.OpenCLFloatBuffer;
import us.ihmc.perception.opencl.OpenCLFloatParameters;
import us.ihmc.perception.opencl.OpenCLManager;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;

public class RapidHeightMapExtractor
{
   private float gridWidthInMeters = 8.0f;
   private float cellSizeXYInMeters = 0.02f;

   private int centerIndex;
   private int cellsPerAxis;

   private int mode = 0; // 0 -> Ouster, 1 -> Realsense

   private OpenCLManager openCLManager;
   private OpenCLFloatParameters parametersBuffer;

   private OpenCLFloatBuffer groundToSensorTransformBuffer;
   private float[] groundToSensorTransformArray = new float[16];

   private OpenCLFloatBuffer sensorToGroundTransformBuffer;
   private float[] sensorToGroundTransformArray = new float[16];

   private OpenCLFloatBuffer groundPlaneBuffer;
   private _cl_program rapidHeightMapUpdaterProgram;
   private _cl_kernel heightMapUpdateKernel;
   private BytedecoImage inputDepthImage;
   private BytedecoImage outputHeightMapImage;

   private final CameraIntrinsics cameraIntrinsics = new CameraIntrinsics();

   private boolean firstRun = true;
   private boolean patchSizeChanged = true;
   private boolean modified = true;
   private boolean processing = false;

   private HeightMapData latestHeightMapData;

   public void create(OpenCLManager openCLManager,  BytedecoImage depthImage, int mode)
   {
      this.mode = mode;
      this.inputDepthImage = depthImage;
      this.openCLManager = openCLManager;
      rapidHeightMapUpdaterProgram = openCLManager.loadProgram("RapidHeightMapExtractor", "HeightMapUtils.cl");

      centerIndex = HeightMapTools.computeCenterIndex(gridWidthInMeters, cellSizeXYInMeters);
      cellsPerAxis = 2 * centerIndex + 1;

      parametersBuffer = new OpenCLFloatParameters();

      groundToSensorTransformBuffer = new OpenCLFloatBuffer(16);
      groundToSensorTransformBuffer.createOpenCLBufferObject(openCLManager);

      sensorToGroundTransformBuffer = new OpenCLFloatBuffer(16);
      sensorToGroundTransformBuffer.createOpenCLBufferObject(openCLManager);

      groundPlaneBuffer = new OpenCLFloatBuffer(4);
      groundPlaneBuffer.createOpenCLBufferObject(openCLManager);

      outputHeightMapImage = new BytedecoImage(cellsPerAxis, cellsPerAxis, opencv_core.CV_16UC1);
      outputHeightMapImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);

      heightMapUpdateKernel = openCLManager.createKernel(rapidHeightMapUpdaterProgram, "heightMapUpdateKernel");
   }

   private void populateParameterBuffer(Tuple3DReadOnly gridCenter)
   {
      //// Fill parameters buffer
      parametersBuffer.setParameter(cellSizeXYInMeters);
      parametersBuffer.setParameter(centerIndex);
      parametersBuffer.setParameter((float) inputDepthImage.getImageHeight());
      parametersBuffer.setParameter((float) inputDepthImage.getImageWidth());
      parametersBuffer.setParameter((float) gridCenter.getX());
      parametersBuffer.setParameter((float) gridCenter.getY());
      parametersBuffer.setParameter((float) mode);
      parametersBuffer.setParameter((float) cameraIntrinsics.getCx());
      parametersBuffer.setParameter((float) cameraIntrinsics.getCy());
      parametersBuffer.setParameter((float) cameraIntrinsics.getFx());
      parametersBuffer.setParameter((float) cameraIntrinsics.getFy());

      parametersBuffer.writeOpenCLBufferObject(openCLManager);
   }


   public void update(RigidBodyTransform sensorToGroundTransform, float planeHeight)
   {
      if (!processing)
      {
         // Upload input depth image
         inputDepthImage.writeOpenCLImage(openCLManager);

         // Fill ground plane buffer
         RigidBodyTransform groundToSensorTransform = new RigidBodyTransform(sensorToGroundTransform);
         groundToSensorTransform.invert();

         Point3D gridCenter = new Point3D(sensorToGroundTransform.getTranslation());

         populateParameterBuffer(gridCenter);

         // Fill world-to-sensor transform buffer
         groundToSensorTransform.get(groundToSensorTransformArray);
         groundToSensorTransformBuffer.getBytedecoFloatBufferPointer().asBuffer().put(groundToSensorTransformArray);
         groundToSensorTransformBuffer.writeOpenCLBufferObject(openCLManager);

         // Fill sensor-to-world transform buffer
         sensorToGroundTransform.get(sensorToGroundTransformArray);
         sensorToGroundTransformBuffer.getBytedecoFloatBufferPointer().asBuffer().put(sensorToGroundTransformArray);
         sensorToGroundTransformBuffer.writeOpenCLBufferObject(openCLManager);

         // Generate a +Z vector in world frame
         Vector3D groundNormalSensorFrame = new Vector3D(0.0, 0.0, 1.0);
         groundToSensorTransform.transform(groundNormalSensorFrame);

         //LogTools.info("Ground normal in sensor frame: " + groundNormalSensorFrame);

         groundPlaneBuffer.getBytedecoFloatBufferPointer().asBuffer().put(new float[] {groundNormalSensorFrame.getX32(), groundNormalSensorFrame.getY32(),
                                                                                       groundNormalSensorFrame.getZ32(),
                                                                                       (float) (planeHeight - sensorToGroundTransform.getTranslationZ())});
         groundPlaneBuffer.writeOpenCLBufferObject(openCLManager);

         // Set kernel arguments for the height map kernel
         openCLManager.setKernelArgument(heightMapUpdateKernel, 0, inputDepthImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(heightMapUpdateKernel, 1, outputHeightMapImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(heightMapUpdateKernel, 2, parametersBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(heightMapUpdateKernel, 3, sensorToGroundTransformBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(heightMapUpdateKernel, 4, groundToSensorTransformBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(heightMapUpdateKernel, 5, groundPlaneBuffer.getOpenCLBufferObject());

         // Execute kernel with length and width parameters
         openCLManager.execute2D(heightMapUpdateKernel, cellsPerAxis, cellsPerAxis);

         // Read height map image into CPU memory
         outputHeightMapImage.readOpenCLImage(openCLManager);

         latestHeightMapData = convertToHeightMapData(gridCenter);
      }
   }

   private HeightMapData convertToHeightMapData(Tuple3DReadOnly center)
   {
      HeightMapData heightMapData = new HeightMapData(cellSizeXYInMeters, gridWidthInMeters, center.getX(), center.getY());
      BytePointer heightMapPointer = outputHeightMapImage.getBytedecoByteBufferPointer();

      float maxHeight = 0.7f;
      float minHeight = 0.0f;


      for (int xIndex = 0; xIndex < cellsPerAxis; xIndex++)
      {
         for (int yIndex = 0; yIndex < cellsPerAxis; yIndex++)
         {
            int heightIndex = xIndex * cellsPerAxis + yIndex;
            float cellHeight = (float) (heightMapPointer.getShort(heightIndex * 2L)) / 10000.0f;
            cellHeight = (float) MathTools.clamp(cellHeight, minHeight, maxHeight);
            if (cellHeight > maxHeight - 0.01f)
               cellHeight = 0.0f;

            int key = HeightMapTools.indicesToKey(xIndex, yIndex, centerIndex);
            heightMapData.setHeightAt(key, cellHeight);
         }
      }

      return heightMapData;
   }

   public boolean isProcessing()
   {
      return processing;
   }

   public void setProcessing(boolean processing)
   {
      this.processing = processing;
   }

   public boolean isModified()
   {
      return modified;
   }

   public void setModified(boolean modified)
   {
      this.modified = modified;
   }

   public float getCellSizeXYInMeters()
   {
      return cellSizeXYInMeters;
   }

   public BytedecoImage getOutputHeightMapImage()
   {
      return outputHeightMapImage;
   }

   public int getCellsPerAxis()
   {
      return cellsPerAxis;
   }

   public int getCenterIndex()
   {
      return centerIndex;
   }

   public HeightMapData getLatestHeightMapData()
   {
      return latestHeightMapData;
   }

   public void setHeightMapResolution(float widthInMeters, float cellSizeXYInMeters)
   {
      this.gridWidthInMeters = widthInMeters;
      this.cellSizeXYInMeters = cellSizeXYInMeters;

      centerIndex = HeightMapTools.computeCenterIndex(gridWidthInMeters, cellSizeXYInMeters);
      cellsPerAxis = 2 * centerIndex + 1;
   }

   public void setDepthIntrinsics(double fx, double fy, double cx, double cy)
   {
      cameraIntrinsics.setFx(fx);
      cameraIntrinsics.setFy(fy);
      cameraIntrinsics.setCx(cx);
      cameraIntrinsics.setCy(cy);
   }
}
