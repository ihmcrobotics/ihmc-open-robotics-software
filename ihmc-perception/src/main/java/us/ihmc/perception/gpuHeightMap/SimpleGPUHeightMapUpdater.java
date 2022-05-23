package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import us.ihmc.commons.time.Stopwatch;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.log.LogTools;
import us.ihmc.perception.OpenCLFloatBuffer;
import us.ihmc.perception.OpenCLManager;

import java.util.List;

public class SimpleGPUHeightMapUpdater
{
   private final SimpleGPUHeightMapParameters parameters;
   private final int numberOfCells;

   private final OpenCLManager openCLManager = new OpenCLManager();

   private final OpenCLFloatBuffer inputPointCloudBuffer = new OpenCLFloatBuffer(0);
   private final OpenCLFloatBuffer localizationBuffer = new OpenCLFloatBuffer(14);
   private final OpenCLFloatBuffer parametersBuffer = new OpenCLFloatBuffer(11);

   private final OpenCLFloatBuffer elevationMapData;
//   private final OpenCLFloatBuffer updatedMapData;

   private final _cl_program heightMapProgram;
   private final _cl_kernel zeroValuesKernel;
   private final _cl_kernel addPointsKernel;
   private final _cl_kernel averageMapKernel;

   private final SimpleGPUHeightMap simpleGPUHeightMap;

   private final Stopwatch pointcloudPacking = new Stopwatch();
   private final Stopwatch bufferWriting = new Stopwatch();
   private final Stopwatch processing = new Stopwatch();
   private final Stopwatch bufferReading = new Stopwatch();

   public SimpleGPUHeightMapUpdater()
   {
      this(new SimpleGPUHeightMapParameters());
   }

   public SimpleGPUHeightMapUpdater(SimpleGPUHeightMapParameters parameters)
   {
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
      addPointsKernel = openCLManager.createKernel(heightMapProgram, "addPointsKernel");
      averageMapKernel = openCLManager.createKernel(heightMapProgram, "averageMapKernel");
   }

   public void input(List<Point3D> rawPoints, RigidBodyTransformReadOnly transformToWorld)
   {
      inputPointCloudBuffer.resize(3 * rawPoints.size(), openCLManager);
      packPointCloudIntoFloatBUffer(rawPoints);

      // TODO set a real center
      populateLocalizaitonBuffer((float) 0.0, (float) 0.0, transformToWorld);
      populateParametersBuffer();

      updateMapWithKernel(rawPoints.size());

      // TODO set a real cneter
      updateMapObject(0.0, 0.0);
   }

   public SimpleGPUHeightMap getHeightMap()
   {
      return simpleGPUHeightMap;
   }

   private void packPointCloudIntoFloatBUffer(List<Point3D> points)
   {
      pointcloudPacking.resetLap();

      FloatPointer floatBuffer = inputPointCloudBuffer.getBytedecoFloatBufferPointer();

      int index = 0;
      for (int i = 0; i < points.size(); i++)
      {
         floatBuffer.put(index++, points.get(i).getX32());
         floatBuffer.put(index++, points.get(i).getY32());
         floatBuffer.put(index++, points.get(i).getZ32());
      }

      pointcloudPacking.lap();
      pointcloudPacking.suspend();
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

   private void updateMapWithKernel(int pointsSize)
   {
      bufferWriting.resetLap();

      // TODO reshape height map
      if (firstRun)
      {
         firstRun = false;
         localizationBuffer.createOpenCLBufferObject(openCLManager);
         parametersBuffer.createOpenCLBufferObject(openCLManager);
         inputPointCloudBuffer.createOpenCLBufferObject(openCLManager);
         elevationMapData.createOpenCLBufferObject(openCLManager);

         openCLManager.setKernelArgument(zeroValuesKernel, 0, parametersBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(zeroValuesKernel, 1, elevationMapData.getOpenCLBufferObject());

         openCLManager.setKernelArgument(addPointsKernel, 0, inputPointCloudBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(addPointsKernel, 1, localizationBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(addPointsKernel, 2, parametersBuffer.getOpenCLBufferObject());
         openCLManager.setKernelArgument(addPointsKernel, 3, elevationMapData.getOpenCLBufferObject());

         openCLManager.setKernelArgument(averageMapKernel, 0, elevationMapData.getOpenCLBufferObject());
         openCLManager.setKernelArgument(averageMapKernel, 1, parametersBuffer.getOpenCLBufferObject());
      }
      else
      {
         inputPointCloudBuffer.writeOpenCLBufferObject(openCLManager);
         localizationBuffer.writeOpenCLBufferObject(openCLManager);
         parametersBuffer.writeOpenCLBufferObject(openCLManager);
      }
      bufferWriting.lap();
      bufferWriting.suspend();

      processing.resetLap();

      openCLManager.execute2D(zeroValuesKernel, numberOfCells, numberOfCells);
      openCLManager.execute1D(addPointsKernel, pointsSize);
      openCLManager.execute2D(averageMapKernel, numberOfCells, numberOfCells);

      processing.lap();
      processing.suspend();

      bufferReading.resetLap();

      elevationMapData.readOpenCLBufferObject(openCLManager);

      bufferReading.lap();
      bufferReading.suspend();

      openCLManager.finish();
   }

   private void updateMapObject(double centerX, double centerY)
   {
      simpleGPUHeightMap.setCenter(centerX, centerY);
      simpleGPUHeightMap.setResolution(parameters.resolution);

      simpleGPUHeightMap.updateFromFloatBuffer(elevationMapData.getBackingDirectFloatBuffer(), numberOfCells);
   }

   public void printStopWatches()
   {
      LogTools.info("Point cloud packing : " + pointcloudPacking.averageLap());
      LogTools.info("Buffer writing : " + bufferWriting.averageLap());
      LogTools.info("Processing : " + processing.averageLap());
      LogTools.info("Buffer reading : " + bufferReading.averageLap());
   }



}
