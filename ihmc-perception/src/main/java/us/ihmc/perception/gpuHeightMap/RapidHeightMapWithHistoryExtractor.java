package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Tuple3DReadOnly;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.OpenCLFloatBuffer;
import us.ihmc.perception.OpenCLIntBuffer;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.perception.opencl.OpenCLFloatParameters;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapTools;

public class RapidHeightMapWithHistoryExtractor
{
   private static final float gridWidthInMeters = 8.0f;
   private static final float cellSizeXYInMeters = 0.05f;

   private static final int bufferLengthPerCell = 10;

   private int centerIndex;
   private int cellsPerAxis;

   private OpenCLManager openCLManager;
   private OpenCLFloatParameters parametersBuffer;

   private OpenCLFloatBuffer worldToSensorTransformBuffer;
   private float[] worldToSensorTransformArray = new float[16];

   private OpenCLFloatBuffer sensorToWorldTransformBuffer;
   private float[] sensorToWorldTransformArray = new float[16];

   private OpenCLFloatBuffer groundPlaneBuffer;
   private _cl_program rapidHeightMapUpdaterProgram;
   private _cl_kernel heightMapUpdateKernel;
   private _cl_kernel initializeDataStructureKernel;
   private _cl_kernel translateHeightMapKernel;
   private _cl_kernel heightMapUpdateDataKernel;
   private _cl_kernel computeHeightMapOutputValuesKernel;
   private BytedecoImage inputDepthImage;
   private BytedecoImage dataKeyImage1;
   private BytedecoImage dataKeyImage2;
   private BytedecoImage activeDataKeyImage;
   private OpenCLFloatBuffer heightSamplesBuffer;
   private OpenCLFloatBuffer varianceSamplesBuffer;
   private OpenCLIntBuffer samplesPerBufferedValueBuffer;
   private OpenCLIntBuffer bufferWriteKeysBuffer;
   private OpenCLIntBuffer entriesInBufferBuffer;

   private BytedecoImage outputHeightMapImage;
   private BytedecoImage outputVarianceImage;

   private boolean firstRun = true;
   private boolean patchSizeChanged = true;
   private boolean modified = true;
   private boolean processing = false;

   private final Point2D previousOrigin = new Point2D();
   private HeightMapData latestHeightMapData;

   public void create(OpenCLManager openCLManager,  BytedecoImage depthImage)
   {
      this.inputDepthImage = depthImage;
      this.openCLManager = openCLManager;
      rapidHeightMapUpdaterProgram = openCLManager.loadProgram("RapidHeightMapWithHistoryExtractor", "HeightMapUtils.cl");

      previousOrigin.setToNaN();

      centerIndex = HeightMapTools.computeCenterIndex(gridWidthInMeters, cellSizeXYInMeters);
      cellsPerAxis = 2 * centerIndex + 1;

      parametersBuffer = new OpenCLFloatParameters();

      worldToSensorTransformBuffer = new OpenCLFloatBuffer(16);
      worldToSensorTransformBuffer.createOpenCLBufferObject(openCLManager);

      sensorToWorldTransformBuffer = new OpenCLFloatBuffer(16);
      sensorToWorldTransformBuffer.createOpenCLBufferObject(openCLManager);

      groundPlaneBuffer = new OpenCLFloatBuffer(4);
      groundPlaneBuffer.createOpenCLBufferObject(openCLManager);

      dataKeyImage1 = new BytedecoImage(cellsPerAxis, cellsPerAxis, opencv_core.CV_16UC1);
      dataKeyImage2 = new BytedecoImage(cellsPerAxis, cellsPerAxis, opencv_core.CV_16UC1);
      dataKeyImage1.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
      dataKeyImage2.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);

      outputHeightMapImage = new BytedecoImage(cellsPerAxis, cellsPerAxis, opencv_core.CV_16UC1);
      outputHeightMapImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
      outputVarianceImage = new BytedecoImage(cellsPerAxis, cellsPerAxis, opencv_core.CV_16UC1);
      outputVarianceImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);

      activeDataKeyImage = dataKeyImage1;

      int totalCells = cellsPerAxis * cellsPerAxis;
      heightSamplesBuffer = new OpenCLFloatBuffer(totalCells * bufferLengthPerCell);
      varianceSamplesBuffer = new OpenCLFloatBuffer(totalCells * bufferLengthPerCell);
      samplesPerBufferedValueBuffer = new OpenCLIntBuffer(totalCells * bufferLengthPerCell);
      bufferWriteKeysBuffer = new OpenCLIntBuffer(totalCells);
      entriesInBufferBuffer = new OpenCLIntBuffer(totalCells);
      heightSamplesBuffer.createOpenCLBufferObject(openCLManager);
      varianceSamplesBuffer.createOpenCLBufferObject(openCLManager);
      samplesPerBufferedValueBuffer.createOpenCLBufferObject(openCLManager);
      bufferWriteKeysBuffer.createOpenCLBufferObject(openCLManager);
      entriesInBufferBuffer.createOpenCLBufferObject(openCLManager);

      heightMapUpdateKernel = openCLManager.createKernel(rapidHeightMapUpdaterProgram, "heightMapUpdateKernel");
      initializeDataStructureKernel = openCLManager.createKernel(rapidHeightMapUpdaterProgram, "initializeDataStructureKernel");
      translateHeightMapKernel = openCLManager.createKernel(rapidHeightMapUpdaterProgram, "translateHeightMapKernel");
      heightMapUpdateDataKernel = openCLManager.createKernel(rapidHeightMapUpdaterProgram, "heightMapUpdateDataKernel");
      computeHeightMapOutputValuesKernel = openCLManager.createKernel(rapidHeightMapUpdaterProgram, "computeHeightMapOutputValuesKernel");
   }

   private void populateParameterBuffer(Tuple3DReadOnly gridCenter, Point2DReadOnly previousCenter)
   {
      //// Fill parameters buffer
      parametersBuffer.setParameter(cellSizeXYInMeters);
      parametersBuffer.setParameter(centerIndex);
      parametersBuffer.setParameter((float) inputDepthImage.getImageHeight());
      parametersBuffer.setParameter((float) inputDepthImage.getImageWidth());
      parametersBuffer.setParameter(gridCenter.getX32());
      parametersBuffer.setParameter(gridCenter.getY32());
      parametersBuffer.setParameter((float) bufferLengthPerCell);
      if (!previousCenter.containsNaN())
      {
         parametersBuffer.setParameter(previousCenter.getX32());
         parametersBuffer.setParameter(previousCenter.getY32());
      }
      else
      {
         parametersBuffer.setParameter(0.0f);
         parametersBuffer.setParameter(0.0f);
      }

      parametersBuffer.writeOpenCLBufferObject(openCLManager);
   }


   public void update(RigidBodyTransform sensorToWorldTransform, float planeHeight)
   {
      if (!processing)
      {
         // Upload input depth image
         inputDepthImage.writeOpenCLImage(openCLManager);

         // Fill ground plane buffer
         RigidBodyTransform worldToSensorTransform = new RigidBodyTransform(sensorToWorldTransform);
         worldToSensorTransform.invert();

         Point3D gridCenter = new Point3D(sensorToWorldTransform.getTranslation());

         populateParameterBuffer(gridCenter, previousOrigin);

         // Fill world-to-sensor transform buffer
         worldToSensorTransform.get(worldToSensorTransformArray);
         worldToSensorTransformBuffer.getBytedecoFloatBufferPointer().asBuffer().put(worldToSensorTransformArray);
         worldToSensorTransformBuffer.writeOpenCLBufferObject(openCLManager);

         // Fill sensor-to-world transform buffer
         sensorToWorldTransform.get(sensorToWorldTransformArray);
         sensorToWorldTransformBuffer.getBytedecoFloatBufferPointer().asBuffer().put(sensorToWorldTransformArray);
         sensorToWorldTransformBuffer.writeOpenCLBufferObject(openCLManager);

         // Generate a +Z vector in world frame
         Vector3D groundNormalSensorFrame = new Vector3D(0.0, 0.0, 1.0);
         worldToSensorTransform.transform(groundNormalSensorFrame);

         LogTools.info("Ground normal in sensor frame: " + groundNormalSensorFrame);

         groundPlaneBuffer.getBytedecoFloatBufferPointer().asBuffer().put(new float[] {groundNormalSensorFrame.getX32(), groundNormalSensorFrame.getY32(),
                                                                                       groundNormalSensorFrame.getZ32(),
                                                                                       (float) (planeHeight - sensorToWorldTransform.getTranslationZ())});
         groundPlaneBuffer.writeOpenCLBufferObject(openCLManager);

         Vector2D translation = new Vector2D(gridCenter);
         translation.sub(previousOrigin);

         LogTools.info("New origin, " + gridCenter);
         LogTools.info("Old origin, " + previousOrigin);

         if (firstRun || previousOrigin.containsNaN() || translation.norm() > 0.5)
         {
            activeDataKeyImage = dataKeyImage1;
            openCLManager.setKernelArgument(initializeDataStructureKernel, 0, parametersBuffer.getOpenCLBufferObject());
            openCLManager.setKernelArgument(initializeDataStructureKernel, 1, activeDataKeyImage.getOpenCLImageObject());
            openCLManager.setKernelArgument(initializeDataStructureKernel, 2, heightSamplesBuffer.getOpenCLBufferObject());
            openCLManager.setKernelArgument(initializeDataStructureKernel, 3, varianceSamplesBuffer.getOpenCLBufferObject());
            openCLManager.setKernelArgument(initializeDataStructureKernel, 4, samplesPerBufferedValueBuffer.getOpenCLBufferObject());
            openCLManager.setKernelArgument(initializeDataStructureKernel, 5, bufferWriteKeysBuffer.getOpenCLBufferObject());
            openCLManager.setKernelArgument(initializeDataStructureKernel, 6, entriesInBufferBuffer.getOpenCLBufferObject());

            openCLManager.execute2D(initializeDataStructureKernel, cellsPerAxis, cellsPerAxis);
         }
         else
         {
            BytedecoImage newDataKeyImage;
            if (activeDataKeyImage == dataKeyImage1)
               newDataKeyImage = dataKeyImage2;
            else
               newDataKeyImage = dataKeyImage1;



            openCLManager.setKernelArgument(translateHeightMapKernel, 0, parametersBuffer.getOpenCLBufferObject());
            openCLManager.setKernelArgument(translateHeightMapKernel, 1, activeDataKeyImage.getOpenCLImageObject());
            openCLManager.setKernelArgument(translateHeightMapKernel, 2, newDataKeyImage.getOpenCLImageObject());
            openCLManager.setKernelArgument(translateHeightMapKernel, 3, heightSamplesBuffer.getOpenCLBufferObject());
            openCLManager.setKernelArgument(translateHeightMapKernel, 4, varianceSamplesBuffer.getOpenCLBufferObject());
            openCLManager.setKernelArgument(translateHeightMapKernel, 5, samplesPerBufferedValueBuffer.getOpenCLBufferObject());
            openCLManager.setKernelArgument(translateHeightMapKernel, 6, bufferWriteKeysBuffer.getOpenCLBufferObject());
            openCLManager.setKernelArgument(translateHeightMapKernel, 7, entriesInBufferBuffer.getOpenCLBufferObject());

            openCLManager.execute2D(translateHeightMapKernel, cellsPerAxis, cellsPerAxis);
            activeDataKeyImage = newDataKeyImage;
         }

         // Set kernel arguments for the height map kernel
         openCLManager.setKernelArgument(heightMapUpdateDataKernel, 0, inputDepthImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(heightMapUpdateDataKernel, 1, parametersBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(heightMapUpdateDataKernel, 2, sensorToWorldTransformBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(heightMapUpdateDataKernel, 3, worldToSensorTransformBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(heightMapUpdateDataKernel, 4, groundPlaneBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(heightMapUpdateDataKernel, 5, activeDataKeyImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(heightMapUpdateDataKernel, 6, heightSamplesBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(heightMapUpdateDataKernel, 7, varianceSamplesBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(heightMapUpdateDataKernel, 8, samplesPerBufferedValueBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(heightMapUpdateDataKernel, 9, bufferWriteKeysBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(heightMapUpdateDataKernel, 10, entriesInBufferBuffer.getOpenCLBufferObject());

         // Execute kernel with length and width parameters
         openCLManager.execute2D(heightMapUpdateDataKernel, cellsPerAxis, cellsPerAxis);

         // Set kernel arguments for the height map kernel
         openCLManager.setKernelArgument(computeHeightMapOutputValuesKernel, 0, parametersBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(computeHeightMapOutputValuesKernel, 1, activeDataKeyImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(computeHeightMapOutputValuesKernel, 2, heightSamplesBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(computeHeightMapOutputValuesKernel, 3, varianceSamplesBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(computeHeightMapOutputValuesKernel, 4, samplesPerBufferedValueBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(computeHeightMapOutputValuesKernel, 5, entriesInBufferBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(computeHeightMapOutputValuesKernel, 6, outputHeightMapImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(computeHeightMapOutputValuesKernel, 7, outputVarianceImage.getOpenCLImageObject());

         openCLManager.execute2D(computeHeightMapOutputValuesKernel, cellsPerAxis, cellsPerAxis);

         // Read height map image into CPU memory
         outputHeightMapImage.readOpenCLImage(openCLManager);
         outputVarianceImage.readOpenCLImage(openCLManager);

         latestHeightMapData = convertToHeightMapData(gridCenter);


         firstRun = false;
         previousOrigin.set(gridCenter);
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
}
