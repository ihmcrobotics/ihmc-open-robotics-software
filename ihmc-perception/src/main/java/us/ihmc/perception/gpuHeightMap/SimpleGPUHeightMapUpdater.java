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
import us.ihmc.perception.opencl.OpenCLFloatBuffer;
import us.ihmc.perception.opencl.OpenCLManager;

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

   private BytedecoImage depthImageMeters;
   private BytedecoImage centroidXImage;
   private BytedecoImage centroidYImage;
   private BytedecoImage centroidZImage;
   private BytedecoImage varianceZImage;
   private BytedecoImage normalXImage;
   private BytedecoImage normalYImage;
   private BytedecoImage normalZImage;
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

   private int centerIndex;

   public SimpleGPUHeightMapUpdater(SimpleGPUHeightMapParameters parameters)
   {
      this.openCLManager = new OpenCLManager();
      this.parameters = parameters;

      // the added two are for the borders
      centerIndex = HeightMapTools.computeCenterIndex(parameters.getMapLength(), parameters.getResolution());
      numberOfCells = 2 * centerIndex + 1;
   }

   public void create(int imageWidth, int imageHeight, ByteBuffer source16UC1DepthImageByteBuffer, double fx, double fy, double cx, double cy)
   {
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;

      this.fx = (float) fx;
      this.fy = (float) fy;
      this.cx = (float) cx;
      this.cy = (float) cy;

      // todo this depth image probably doesn't need to be created.
      this.depthImageMeters = new BytedecoImage(imageWidth, imageHeight, opencv_core.CV_32FC1, source16UC1DepthImageByteBuffer);

      // these are the outputs structure of the map
      this.centroidXImage = new BytedecoImage(numberOfCells, numberOfCells, opencv_core.CV_32FC1);
      this.centroidYImage = new BytedecoImage(numberOfCells, numberOfCells, opencv_core.CV_32FC1);
      this.centroidZImage = new BytedecoImage(numberOfCells, numberOfCells, opencv_core.CV_32FC1);
      this.varianceZImage = new BytedecoImage(numberOfCells, numberOfCells, opencv_core.CV_32FC1);
      this.normalXImage = new BytedecoImage(numberOfCells, numberOfCells, opencv_core.CV_32FC1);
      this.normalYImage = new BytedecoImage(numberOfCells, numberOfCells, opencv_core.CV_32FC1);
      this.normalZImage = new BytedecoImage(numberOfCells, numberOfCells, opencv_core.CV_32FC1);
      this.countImage = new BytedecoImage(numberOfCells, numberOfCells, opencv_core.CV_8UC1);

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
      if (centroidData != null)
      {
         openCLManager.releaseBufferObject(centroidData);
         centroidData.releaseReference();
      }

      if (varianceData != null)
      {
         openCLManager.releaseBufferObject(varianceData);
         varianceData.releaseReference();
      }

      if (counterData != null)
      {
         openCLManager.releaseBufferObject(counterData);
         counterData.releaseReference();
      }

      depthImageMeters.destroy(openCLManager);
      centroidXImage.destroy(openCLManager);
      centroidYImage.destroy(openCLManager);
      centroidZImage.destroy(openCLManager);
      varianceZImage.destroy(openCLManager);
      normalXImage.destroy(openCLManager);
      normalYImage.destroy(openCLManager);
      normalZImage.destroy(openCLManager);
      countImage.destroy(openCLManager);

      openCLManager.destroy();
   }

   public void computeFromDepthMap(RigidBodyTransformReadOnly transformToWorld)
   {
      computeFromDepthMap(transformToWorld.getTranslation().getX32(), transformToWorld.getTranslation().getY32(), transformToWorld);
   }

   public void computeFromDepthMap(float centerX, float centerY, RigidBodyTransformReadOnly transformToWorld)
   {
      populateLocalizationBuffer(centerX, centerY, transformToWorld);
      populateParametersBuffer();
      populateIntrinsicsBuffer();

      updateMapWithKernel();

      updateMapObject(centerX, centerY);
   }

   public SimpleGPUHeightMap getHeightMap()
   {
      return simpleGPUHeightMap;
   }

   private final RotationMatrixBasics rotation = new RotationMatrix();

   private void populateLocalizationBuffer(float centerX, float centerY, RigidBodyTransformReadOnly transformToDesiredFrame)
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
      parametersBuffer.getBytedecoFloatBufferPointer().put(0, (float) parameters.getResolution());
      parametersBuffer.getBytedecoFloatBufferPointer().put(1, (float) parameters.getMinValidDistance());
      parametersBuffer.getBytedecoFloatBufferPointer().put(2, (float) parameters.getMaxHeightRange());
      parametersBuffer.getBytedecoFloatBufferPointer().put(3, (float) parameters.getRampedHeightRangeA());
      parametersBuffer.getBytedecoFloatBufferPointer().put(4, (float) parameters.getRampedHeightRangeB());
      parametersBuffer.getBytedecoFloatBufferPointer().put(5, (float) parameters.getRampedHeightRangeC());
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
         depthImageMeters.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_ONLY);
         centroidXImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
         centroidYImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
         centroidZImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
         normalXImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_WRITE_ONLY);
         normalYImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_WRITE_ONLY);
         normalZImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_WRITE_ONLY);
         varianceZImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_WRITE_ONLY);
         countImage.createOpenCLImage(openCLManager, OpenCL.CL_MEM_READ_WRITE);
      }
      else
      {
         depthImageMeters.writeOpenCLImage(openCLManager);

         localizationBuffer.writeOpenCLBufferObject(openCLManager);
         parametersBuffer.writeOpenCLBufferObject(openCLManager);
         intrinsicsBuffer.writeOpenCLBufferObject(openCLManager);
      }

      openCLManager.setKernelArgument(zeroValuesKernel, 0, parametersBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(zeroValuesKernel, 1, centroidData);
      openCLManager.setKernelArgument(zeroValuesKernel, 2, varianceData);
      openCLManager.setKernelArgument(zeroValuesKernel, 3, counterData);

      openCLManager.setKernelArgument(addPointsFromImageKernel, 0, depthImageMeters.getOpenCLImageObject());
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
      openCLManager.setKernelArgument(averageMapKernel, 4, centroidXImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(averageMapKernel, 5, centroidYImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(averageMapKernel, 6, centroidZImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(averageMapKernel, 7, varianceZImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(averageMapKernel, 8, countImage.getOpenCLImageObject());

      openCLManager.setKernelArgument(computeNormalsKernel, 0, centroidXImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(computeNormalsKernel, 1, centroidYImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(computeNormalsKernel, 2, centroidZImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(computeNormalsKernel, 3, countImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(computeNormalsKernel, 4, parametersBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(computeNormalsKernel, 5, normalXImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(computeNormalsKernel, 6, normalYImage.getOpenCLImageObject());
      openCLManager.setKernelArgument(computeNormalsKernel, 7, normalZImage.getOpenCLImageObject());

      openCLManager.execute2D(zeroValuesKernel, numberOfCells, numberOfCells);
      openCLManager.execute2D(addPointsFromImageKernel, imageWidth, imageHeight);
      openCLManager.execute2D(averageMapKernel, numberOfCells, numberOfCells);
      openCLManager.execute2D(computeNormalsKernel, numberOfCells, numberOfCells);

      openCLManager.enqueueReadImage(centroidXImage.getOpenCLImageObject(), numberOfCells, numberOfCells, centroidXImage.getBytedecoByteBufferPointer());
      openCLManager.enqueueReadImage(centroidYImage.getOpenCLImageObject(), numberOfCells, numberOfCells, centroidYImage.getBytedecoByteBufferPointer());
      openCLManager.enqueueReadImage(centroidZImage.getOpenCLImageObject(), numberOfCells, numberOfCells, centroidZImage.getBytedecoByteBufferPointer());
      openCLManager.enqueueReadImage(varianceZImage.getOpenCLImageObject(), numberOfCells, numberOfCells, varianceZImage.getBytedecoByteBufferPointer());
      openCLManager.enqueueReadImage(countImage.getOpenCLImageObject(), numberOfCells, numberOfCells, countImage.getBytedecoByteBufferPointer());
      openCLManager.enqueueReadImage(normalXImage.getOpenCLImageObject(), numberOfCells, numberOfCells, normalXImage.getBytedecoByteBufferPointer());
      openCLManager.enqueueReadImage(normalYImage.getOpenCLImageObject(), numberOfCells, numberOfCells, normalYImage.getBytedecoByteBufferPointer());
      openCLManager.enqueueReadImage(normalZImage.getOpenCLImageObject(), numberOfCells, numberOfCells, normalZImage.getBytedecoByteBufferPointer());
   }

   private void updateMapObject(double centerX, double centerY)
   {
      simpleGPUHeightMap.reshape(parameters.getResolution(), parameters.getMapLength(), centerX, centerY);
      simpleGPUHeightMap.updateFromFloatBufferImage(centroidXImage.getBytedecoOpenCVMat(),
                                                    centroidYImage.getBytedecoOpenCVMat(),
                                                    centroidZImage.getBytedecoOpenCVMat(),
                                                    varianceZImage.getBytedecoOpenCVMat(),
                                                    normalXImage.getBytedecoOpenCVMat(),
                                                    normalYImage.getBytedecoOpenCVMat(),
                                                    normalZImage.getBytedecoOpenCVMat(),
                                                    countImage.getBytedecoOpenCVMat());
   }
}
