package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_mem;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.OpenCLFloatBuffer;
import us.ihmc.perception.OpenCLManager;

import java.nio.ByteBuffer;
import java.util.List;

public class SimpleImageGPUHeightMapUpdater
{
   private final SimpleGPUHeightMapParameters parameters;
   private final int numberOfCells;

   private final OpenCLManager openCLManager;

   private final OpenCLFloatBuffer localizationBuffer = new OpenCLFloatBuffer(14);
   private final OpenCLFloatBuffer parametersBuffer = new OpenCLFloatBuffer(11);
   private final OpenCLFloatBuffer intrinsicsBuffer = new OpenCLFloatBuffer(4);

   private final OpenCLFloatBuffer elevationMapData;
   //   private final OpenCLFloatBuffer updatedMapData;

   private final BytedecoImage depthImage;
   private final int imageWidth;
   private final int imageHeight;

   private final _cl_program heightMapProgram;
   private final _cl_kernel zeroValuesKernel;
   private final _cl_kernel addPointsFromImageKernel;
   private final _cl_kernel averageMapKernel;

   private final SimpleGPUHeightMap simpleGPUHeightMap;

   private float fx;
   private float fy;
   private float cx;
   private float cy;

   public SimpleImageGPUHeightMapUpdater(int imageWidth, int imageHeight, ByteBuffer sourceData, SimpleGPUHeightMapParameters parameters)
   {
      this.depthImage = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_16UC1, sourceData);
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;

      this.openCLManager = new OpenCLManager();
      this.parameters = parameters;
      openCLManager.create();

      // the added two are for the borders
      numberOfCells = ((int) Math.round(parameters.mapLength / parameters.resolution)) + 2;
      simpleGPUHeightMap = new SimpleGPUHeightMap();

      int floatsPerLayer = numberOfCells * numberOfCells;
      elevationMapData = new OpenCLFloatBuffer(3 * floatsPerLayer);
      //      updatedMapData = new OpenCLFloatBuffer(3 * floatsPerLayer);

      heightMapProgram = openCLManager.loadProgram("SimpleGPUHeightMap");
      zeroValuesKernel = openCLManager.createKernel(heightMapProgram, "zeroValuesKernel");
      addPointsFromImageKernel = openCLManager.createKernel(heightMapProgram, "addPointsFromImageKernel");
      averageMapKernel = openCLManager.createKernel(heightMapProgram, "averageMapKernel");
   }

   public void setCameraIntrinsics(double fx, double fy, double cx, double cy)
   {
      this.fx = (float) fx;
      this.fy = (float) fy;
      this.cx = (float) cx;
      this.cy = (float) cy;
   }

   // Fixme the transform is wrong
   public void inputFromImage(_cl_mem image, int imageWidth, int imageHeight, RigidBodyTransformReadOnly transformToWorld)
   {
      populateLocalizaitonBuffer(transformToWorld.getTranslation().getX32(), transformToWorld.getTranslation().getY32(), transformToWorld);
      populateParametersBuffer();
      populateIntrinsicsBuffer();

      updateMapWithKernel();

      // TODO set a real cneter
      updateMapObject(0.0, 0.0);
   }

   public SimpleGPUHeightMap getHeightMap()
   {
      return simpleGPUHeightMap;
   }

   private final RotationMatrixBasics rotation = new RotationMatrix();

   private void populateLocalizaitonBuffer(float centerX, float centerY, RigidBodyTransformReadOnly transformToDesiredFrame)
   {
      rotation.set(transformToDesiredFrame.getRotation());

      int index = 0;
      localizationBuffer.getBytedecoFloatBufferPointer().put(index++, centerX);
      localizationBuffer.getBytedecoFloatBufferPointer().put(index++, centerY);
      for (int i = 0; i < 3; i++)
      {
         for (int j = 0; j < 3; j++)
            localizationBuffer.getBytedecoFloatBufferPointer().put(index++, (float) rotation.getElement(i, j));
      }
      for (int i = 0; i < 3; i++)
         localizationBuffer.getBytedecoFloatBufferPointer().put(index++, (float) transformToDesiredFrame.getTranslation().getElement(i));
   }

   private void populateIntrinsicsBuffer()
   {
      intrinsicsBuffer.getBytedecoFloatBufferPointer().put(0, cx);
      intrinsicsBuffer.getBytedecoFloatBufferPointer().put(1, cy);
      intrinsicsBuffer.getBytedecoFloatBufferPointer().put(2, fx);
      intrinsicsBuffer.getBytedecoFloatBufferPointer().put(3, fy);
   }

   private void populateParametersBuffer()
   {
      parametersBuffer.getBytedecoFloatBufferPointer().put(0, (float) numberOfCells);
      parametersBuffer.getBytedecoFloatBufferPointer().put(1, (float) numberOfCells);
      parametersBuffer.getBytedecoFloatBufferPointer().put(2, (float) parameters.resolution);
      parametersBuffer.getBytedecoFloatBufferPointer().put(3, (float) parameters.minValidDistance);
      parametersBuffer.getBytedecoFloatBufferPointer().put(4, (float) parameters.maxHeightRange);
      parametersBuffer.getBytedecoFloatBufferPointer().put(5, (float) parameters.rampedHeightRangeA);
      parametersBuffer.getBytedecoFloatBufferPointer().put(6, (float) parameters.rampedHeightRangeB);
      parametersBuffer.getBytedecoFloatBufferPointer().put(7, (float) parameters.rampedHeightRangeC);
      parametersBuffer.getBytedecoFloatBufferPointer().put(8, (float) parameters.sensorNoiseFactor);
      parametersBuffer.getBytedecoFloatBufferPointer().put(9, (float) parameters.initialVariance);
      parametersBuffer.getBytedecoFloatBufferPointer().put(10, (float) parameters.maxVariance);
   }

   boolean firstRun = true;

   private void updateMapWithKernel()
   {
      // TODO reshape height map
      if (firstRun)
      {
         firstRun = false;
         localizationBuffer.createOpenCLBufferObject(openCLManager);
         parametersBuffer.createOpenCLBufferObject(openCLManager);
         elevationMapData.createOpenCLBufferObject(openCLManager);
         intrinsicsBuffer.createOpenCLBufferObject(openCLManager);
         depthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);


         openCLManager.setKernelArgument(zeroValuesKernel, 0, parametersBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(zeroValuesKernel, 1, elevationMapData.getOpenCLBufferObject());

         openCLManager.setKernelArgument(addPointsFromImageKernel, 0, depthImage.getOpenCLImageObject());
         openCLManager.setKernelArgument(addPointsFromImageKernel, 1, localizationBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(addPointsFromImageKernel, 2, parametersBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(addPointsFromImageKernel, 3, intrinsicsBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(addPointsFromImageKernel, 4, elevationMapData.getOpenCLBufferObject());

         openCLManager.setKernelArgument(averageMapKernel, 0, elevationMapData.getOpenCLBufferObject());
         openCLManager.setKernelArgument(averageMapKernel, 1, parametersBuffer.getOpenCLBufferObject());
      }
      else
      {
         depthImage.writeOpenCLImage(openCLManager);

         localizationBuffer.writeOpenCLBufferObject(openCLManager);
         parametersBuffer.writeOpenCLBufferObject(openCLManager);
         intrinsicsBuffer.writeOpenCLBufferObject(openCLManager);
      }

      openCLManager.execute2D(zeroValuesKernel, numberOfCells, numberOfCells);
      openCLManager.execute2D(addPointsFromImageKernel, imageWidth, imageHeight);
      openCLManager.execute2D(averageMapKernel, numberOfCells, numberOfCells);

      elevationMapData.readOpenCLBufferObject(openCLManager);

      openCLManager.finish();
   }

   private void updateMapObject(double centerX, double centerY)
   {
      simpleGPUHeightMap.setCenter(centerX, centerY);
      simpleGPUHeightMap.setResolution(parameters.resolution);

      simpleGPUHeightMap.updateFromFloatBuffer(elevationMapData.getBackingDirectFloatBuffer(), numberOfCells);
   }
}
