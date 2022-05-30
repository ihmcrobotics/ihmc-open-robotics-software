package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_mem;
import org.bytedeco.opencl._cl_program;
import org.bytedeco.opencl.global.OpenCL;
import org.bytedeco.opencv.global.opencv_core;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.OpenCLFloatBuffer;
import us.ihmc.perception.OpenCLManager;
import us.ihmc.robotics.heightMap.HeightMapTools;

import java.nio.ByteBuffer;

public class SimpleGPUHeightMapUpdater
{
   private final SimpleGPUHeightMapParameters parameters;
   private final int numberOfCells;

   private final OpenCLManager openCLManager;

   private final OpenCLFloatBuffer localizationBuffer = new OpenCLFloatBuffer(14);
   private final OpenCLFloatBuffer parametersBuffer = new OpenCLFloatBuffer(8);
   private final OpenCLFloatBuffer intrinsicsBuffer = new OpenCLFloatBuffer(4);

   private _cl_mem varianceData;
   private _cl_mem counterData;
   private _cl_mem centroidData;

   private BytedecoImage depthImage;
   private BytedecoImage centroidImage;
   private BytedecoImage varianceZImage;
   private BytedecoImage normalImage;
   private BytedecoImage countImage;
   private int imageWidth;
   private int imageHeight;

   private _cl_program heightMapProgram;
   private _cl_kernel zeroValuesKernel;
   private _cl_kernel addPointsFromImageKernel;
   private _cl_kernel averageMapKernel;
   private _cl_kernel computeNormalsKernel;

   private final SimpleGPUHeightMap simpleGPUHeightMap = new SimpleGPUHeightMap();

   private float fx;
   private float fy;
   private float cx;
   private float cy;

   private ByteBuffer backingByteBuffer;
   private int centerIndex;

   public SimpleGPUHeightMapUpdater(SimpleGPUHeightMapParameters parameters)
   {
      this.openCLManager = new OpenCLManager();
      this.parameters = parameters;

      // the added two are for the borders
      centerIndex = HeightMapTools.computeCenterIndex(parameters.mapLength, parameters.resolution);
      numberOfCells = 2 * centerIndex + 1;
   }

   public void create(int imageWidth, int imageHeight, ByteBuffer sourceData, double fx, double fy, double cx, double cy)
   {
      this.backingByteBuffer = sourceData;
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;

      this.fx = (float) fx;
      this.fy = (float) fy;
      this.cx = (float) cx;
      this.cy = (float) cy;

      // todo this depth image probably doesn't need to be created.
      this.depthImage = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_16UC1, sourceData);

      // these are the outputs structure of the map
      this.centroidImage = new BytedecoImage(numberOfCells, numberOfCells, opencv_core.CV_32FC3);
      this.varianceZImage = new BytedecoImage(numberOfCells, numberOfCells, opencv_core.CV_32FC1);
      this.normalImage = new BytedecoImage(numberOfCells, numberOfCells, opencv_core.CV_32FC3);
      this.countImage = new BytedecoImage(numberOfCells, numberOfCells, opencv_core.CV_8UC1);

      openCLManager.create();
      heightMapProgram = openCLManager.loadProgram("SimpleGPUHeightMap");
      zeroValuesKernel = openCLManager.createKernel(heightMapProgram, "zeroValuesKernel");
      addPointsFromImageKernel = openCLManager.createKernel(heightMapProgram, "addPointsFromImageKernel");
      averageMapKernel = openCLManager.createKernel(heightMapProgram, "averageMapImagesKernel");
      computeNormalsKernel = openCLManager.createKernel(heightMapProgram, "computeNormalsKernel");
   }

   public void destroy()
   {
      heightMapProgram.close();
      zeroValuesKernel.close();
      addPointsFromImageKernel.close();
      averageMapKernel.close();
      computeNormalsKernel.close();

      localizationBuffer.destroy(openCLManager);
      parametersBuffer.destroy(openCLManager);
      intrinsicsBuffer.destroy(openCLManager);
      openCLManager.releaseBufferObject(centroidData);
      openCLManager.releaseBufferObject(varianceData);
      openCLManager.releaseBufferObject(counterData);
      centroidData.releaseReference();
      varianceData.releaseReference();
      counterData.releaseReference();

      depthImage.destroy(openCLManager);
      centroidImage.destroy(openCLManager);
      varianceZImage.destroy(openCLManager);
      normalImage.destroy(openCLManager);
      countImage.destroy(openCLManager);

      openCLManager.destroy();
   }

   public void computeFromDepthMap(RigidBodyTransformReadOnly transformToWorld)
   {
      populateLocalizaitonBuffer(transformToWorld.getTranslation().getX32(), transformToWorld.getTranslation().getY32(), transformToWorld);
      populateParametersBuffer();
      populateIntrinsicsBuffer();

      updateMapWithKernel();

      updateMapObject(transformToWorld.getTranslation().getX32(), transformToWorld.getTranslation().getY32());
   }

   public SimpleGPUHeightMap getHeightMap()
   {
      return simpleGPUHeightMap;
   }

   private final RotationMatrixBasics rotation = new RotationMatrix();

   private void populateLocalizaitonBuffer(float centerX, float centerY, RigidBodyTransformReadOnly transformToDesiredFrame)
   {
      rotation.set(transformToDesiredFrame.getRotation());

      localizationBuffer.getBytedecoFloatBufferPointer().put(0, centerX);
      localizationBuffer.getBytedecoFloatBufferPointer().put(1, centerY);
      localizationBuffer.getBytedecoFloatBufferPointer().put(2, (float) rotation.getM00());
      localizationBuffer.getBytedecoFloatBufferPointer().put(3, (float) rotation.getM01());
      localizationBuffer.getBytedecoFloatBufferPointer().put(4, (float) rotation.getM02());
      localizationBuffer.getBytedecoFloatBufferPointer().put(5, (float) rotation.getM10());
      localizationBuffer.getBytedecoFloatBufferPointer().put(6, (float) rotation.getM11());
      localizationBuffer.getBytedecoFloatBufferPointer().put(7, (float) rotation.getM12());
      localizationBuffer.getBytedecoFloatBufferPointer().put(8, (float) rotation.getM20());
      localizationBuffer.getBytedecoFloatBufferPointer().put(9, (float) rotation.getM21());
      localizationBuffer.getBytedecoFloatBufferPointer().put(10, (float) rotation.getM22());
      localizationBuffer.getBytedecoFloatBufferPointer().put(11, transformToDesiredFrame.getTranslation().getX32());
      localizationBuffer.getBytedecoFloatBufferPointer().put(12, transformToDesiredFrame.getTranslation().getY32());
      localizationBuffer.getBytedecoFloatBufferPointer().put(13, transformToDesiredFrame.getTranslation().getZ32());
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
      parametersBuffer.getBytedecoFloatBufferPointer().put(0, (float) parameters.resolution);
      parametersBuffer.getBytedecoFloatBufferPointer().put(1, (float) parameters.minValidDistance);
      parametersBuffer.getBytedecoFloatBufferPointer().put(2, (float) parameters.maxHeightRange);
      parametersBuffer.getBytedecoFloatBufferPointer().put(3, (float) parameters.rampedHeightRangeA);
      parametersBuffer.getBytedecoFloatBufferPointer().put(4, (float) parameters.rampedHeightRangeB);
      parametersBuffer.getBytedecoFloatBufferPointer().put(5, (float) parameters.rampedHeightRangeC);
      parametersBuffer.getBytedecoFloatBufferPointer().put(6, (float) centerIndex);
      parametersBuffer.getBytedecoFloatBufferPointer().put(7, (float) 1);
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
         long cellsSize = (long) numberOfCells * numberOfCells * Integer.BYTES;
         centroidData = openCLManager.createBufferObject(3 * cellsSize, null);
         varianceData = openCLManager.createBufferObject(cellsSize, null);
         counterData = openCLManager.createBufferObject(cellsSize, null);

         intrinsicsBuffer.createOpenCLBufferObject(openCLManager);
         depthImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);
         centroidImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
         normalImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_WRITE_ONLY);
         varianceZImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_WRITE_ONLY);
         countImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
      }
      else
      {
         depthImage.writeOpenCLImage(openCLManager);

         localizationBuffer.writeOpenCLBufferObject(openCLManager);
         parametersBuffer.writeOpenCLBufferObject(openCLManager);
         intrinsicsBuffer.writeOpenCLBufferObject(openCLManager);
      }

      openCLManager.setKernelArgument(zeroValuesKernel, 0, parametersBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(zeroValuesKernel, 1, centroidData);
      openCLManager.setKernelArgument(zeroValuesKernel, 2, varianceData);
      openCLManager.setKernelArgument(zeroValuesKernel, 3, counterData);

      openCLManager.setKernelArgument(addPointsFromImageKernel, 0, depthImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(addPointsFromImageKernel, 1, localizationBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(addPointsFromImageKernel, 2, parametersBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(addPointsFromImageKernel, 3, intrinsicsBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(addPointsFromImageKernel, 4, centroidData);
      openCLManager.setKernelArgument(addPointsFromImageKernel, 5, varianceData);
      openCLManager.setKernelArgument(addPointsFromImageKernel, 6, counterData);

      openCLManager.setKernelArgument(averageMapKernel, 0, centroidData);
      openCLManager.setKernelArgument(averageMapKernel, 1, varianceData);
      openCLManager.setKernelArgument(averageMapKernel, 2, counterData);
      openCLManager.setKernelArgument(averageMapKernel, 3, parametersBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(averageMapKernel, 4, centroidImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(averageMapKernel, 5, varianceZImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(averageMapKernel, 6, countImage.getOpenCLImageObject());

      openCLManager.setKernelArgument(computeNormalsKernel, 0, centroidImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(computeNormalsKernel, 1, countImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(computeNormalsKernel, 2, parametersBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeNormalsKernel, 3, normalImage.getOpenCLImageObject());

      openCLManager.execute2D(zeroValuesKernel, numberOfCells, numberOfCells);
      openCLManager.execute2D(addPointsFromImageKernel, imageWidth, imageHeight);
      openCLManager.execute2D(averageMapKernel, numberOfCells, numberOfCells);
      openCLManager.execute2D(computeNormalsKernel, numberOfCells, numberOfCells);

      openCLManager.enqueueReadImage(centroidImage.getOpenCLImageObject(), numberOfCells, numberOfCells, centroidImage.getBytedecoByteBufferPointer());
      openCLManager.enqueueReadImage(varianceZImage.getOpenCLImageObject(), numberOfCells, numberOfCells, varianceZImage.getBytedecoByteBufferPointer());
      openCLManager.enqueueReadImage(countImage.getOpenCLImageObject(), numberOfCells, numberOfCells, countImage.getBytedecoByteBufferPointer());
      openCLManager.enqueueReadImage(normalImage.getOpenCLImageObject(), numberOfCells, numberOfCells, normalImage.getBytedecoByteBufferPointer());

      openCLManager.finish();
   }

   private void updateMapObject(double centerX, double centerY)
   {
      simpleGPUHeightMap.reshape(parameters.resolution, parameters.mapLength, centerX, centerY);
      simpleGPUHeightMap.updateFromFloatBufferImage(centroidImage.getBytedecoOpenCVMat(),
                                                    varianceZImage.getBytedecoOpenCVMat(),
                                                    normalImage.getBytedecoOpenCVMat(),
                                                    countImage.getBytedecoOpenCVMat());
   }
}
