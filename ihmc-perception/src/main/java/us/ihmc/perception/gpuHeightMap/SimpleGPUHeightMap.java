package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.OpenCLFloatBuffer;
import us.ihmc.perception.OpenCLManager;

import java.util.List;

public class SimpleGPUHeightMap
{
   private final SimpleGPUHeightMapParameters parameters;
   private final int numberOfCells;
   private final int floatsPerLayer;

   private final OpenCLManager openCLManager = new OpenCLManager();

   private final OpenCLFloatBuffer inputPointCloudBuffer = new OpenCLFloatBuffer(0);
   private final OpenCLFloatBuffer localizationBuffer = new OpenCLFloatBuffer(14);
   private final OpenCLFloatBuffer parametersBuffer = new OpenCLFloatBuffer(14);

   private final OpenCLFloatBuffer elevationMapData;
   private final OpenCLFloatBuffer updatedMapData;

   private final _cl_program heightMapProgram;
   private final _cl_kernel addPointsKernel;
   private final _cl_kernel averageMapKernel;

   public SimpleGPUHeightMap()
   {
      this(new SimpleGPUHeightMapParameters());
   }

   public SimpleGPUHeightMap(SimpleGPUHeightMapParameters parameters)
   {
      this.parameters = parameters;
      openCLManager.create();

      // the added two are for the borders
      numberOfCells = ((int) Math.round(parameters.mapLength / parameters.resolution)) + 2;

      floatsPerLayer = numberOfCells * numberOfCells;
      elevationMapData = new OpenCLFloatBuffer(2 * floatsPerLayer);
      updatedMapData = new OpenCLFloatBuffer(3 * floatsPerLayer);

      heightMapProgram = openCLManager.loadProgram("SimpleGPUHeightMap2");
      addPointsKernel = openCLManager.createKernel(heightMapProgram, "addPointsKernel");
      averageMapKernel = openCLManager.createKernel(heightMapProgram, "averageMapKernel");
   }

   public void input(List<Point3D> rawPoints, RigidBodyTransformReadOnly transformToWorld)
   {
      inputPointCloudBuffer.resize(3 * rawPoints.size(), openCLManager);
      packPointCloudIntoFloatBUffer(rawPoints);

      populateLocalizaitonBuffer((float) 0.0, (float) 0.0, transformToWorld);
      populateParametersBuffer();

      updateMapWithKernel(rawPoints.size());
   }

   private void packPointCloudIntoFloatBUffer(List<Point3D> points)
   {
      FloatPointer floatBuffer = inputPointCloudBuffer.getBytedecoFloatBufferPointer();

      int index = 0;
      for (int i = 0; i < points.size(); i++)
      {
         floatBuffer.put(index++, points.get(i).getX32());
         floatBuffer.put(index++, points.get(i).getY32());
         floatBuffer.put(index++, points.get(i).getZ32());
      }
   }

   private final RotationMatrixBasics rotation = new RotationMatrix();

   private void populateLocalizaitonBuffer(float centerX, float centerY, RigidBodyTransformReadOnly desiredTransform)
   {
      rotation.set(desiredTransform.getRotation());

      int index = 0;
      localizationBuffer.getBytedecoFloatBufferPointer().put(index++, centerX);
      localizationBuffer.getBytedecoFloatBufferPointer().put(index++, centerY);
      for (int i = 0; i < 3; i++)
      {
         for (int j = 0; j < 3; j++)
            localizationBuffer.getBytedecoFloatBufferPointer().put(index++, (float) rotation.getElement(i, j));
      }
      for (int i = 0; i < 3; i++)
         localizationBuffer.getBytedecoFloatBufferPointer().put(index++, (float) desiredTransform.getTranslation().getElement(i));
   }

   private void populateParametersBuffer()
   {
      int index = 0;
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
      if (firstRun)
      {
         firstRun = false;
         localizationBuffer.createOpenCLBufferObject(openCLManager);
         parametersBuffer.createOpenCLBufferObject(openCLManager);
         inputPointCloudBuffer.createOpenCLBufferObject(openCLManager);
         elevationMapData.createOpenCLBufferObject(openCLManager);
         updatedMapData.createOpenCLBufferObject(openCLManager);
      }
      else
      {
         inputPointCloudBuffer.writeOpenCLBufferObject(openCLManager);
         localizationBuffer.writeOpenCLBufferObject(openCLManager);
         parametersBuffer.writeOpenCLBufferObject(openCLManager);
      }

      openCLManager.setKernelArgument(addPointsKernel, 0, inputPointCloudBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(addPointsKernel, 1, localizationBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(addPointsKernel, 2, parametersBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(addPointsKernel, 3, elevationMapData.getOpenCLBufferObject());
      openCLManager.setKernelArgument(addPointsKernel, 4, updatedMapData.getOpenCLBufferObject());

      openCLManager.setKernelArgument(averageMapKernel, 0, updatedMapData.getOpenCLBufferObject());
      openCLManager.setKernelArgument(averageMapKernel, 1, elevationMapData.getOpenCLBufferObject());
      openCLManager.setKernelArgument(averageMapKernel, 2, parametersBuffer.getOpenCLBufferObject());

      openCLManager.execute1D(addPointsKernel, pointsSize);
      openCLManager.execute2D(averageMapKernel, numberOfCells, numberOfCells);

      updatedMapData.readOpenCLBufferObject(openCLManager);

      openCLManager.finish();
   }
}
