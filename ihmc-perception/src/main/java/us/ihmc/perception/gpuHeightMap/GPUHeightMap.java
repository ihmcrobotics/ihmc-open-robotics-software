package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.matrix.interfaces.RotationMatrixBasics;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.perception.OpenCLFloatBuffer;
import us.ihmc.perception.OpenCLManager;

import java.nio.FloatBuffer;
import java.util.List;

public class GPUHeightMap
{
   private final GPUHeightMapParameters parameters;
   private final int numberOfCells;
   private final int bytesPerLayer;

   private final OpenCLManager openCLManager = new OpenCLManager();

   private final OpenCLFloatBuffer localizationBuffer = new OpenCLFloatBuffer(14);
   private final OpenCLFloatBuffer parametersBuffer = new OpenCLFloatBuffer(12);

   private final OpenCLFloatBuffer elevationMapData;
   private final OpenCLFloatBuffer updatedMapData;
   private final OpenCLFloatBuffer normalMapData;
   private final OpenCLFloatBuffer errorBuffer = new OpenCLFloatBuffer(2);

   private final _cl_program heightMapProgram;
//   private final _cl_kernel addPointsKernel;
   private final _cl_kernel errorCountingKernel;

   public GPUHeightMap()
   {
      this(new GPUHeightMapParameters());
   }

   public GPUHeightMap(GPUHeightMapParameters parameters)
   {
      this.parameters = parameters;
      openCLManager.create();

      // the added two are for the borders
      numberOfCells = ((int) Math.round(parameters.mapLength / parameters.resolution)) + 2;

      bytesPerLayer = numberOfCells * numberOfCells * Float.BYTES;
      elevationMapData = new OpenCLFloatBuffer(7 * bytesPerLayer);
      updatedMapData = new OpenCLFloatBuffer(7 * bytesPerLayer);
      normalMapData = new OpenCLFloatBuffer(3 * bytesPerLayer);

      localizationBuffer.createOpenCLBufferObject(openCLManager);
      errorBuffer.createOpenCLBufferObject(openCLManager);
      parametersBuffer.createOpenCLBufferObject(openCLManager);

      elevationMapData.createOpenCLBufferObject(openCLManager);
      updatedMapData.createOpenCLBufferObject(openCLManager);
      normalMapData.createOpenCLBufferObject(openCLManager);

      heightMapProgram = openCLManager.loadProgram("GPUHeightMap2");
//      addPointsKernel = openCLManager.createKernel(heightMapProgram, "addPointsKernel");
      errorCountingKernel = openCLManager.createKernel(heightMapProgram, "errorCountingKernel");
   }

   public void input(List<Point3D> rawPoints, RigidBodyTransformReadOnly transformToWorld)
   {
      OpenCLFloatBuffer pointsBuffer = packPointCloudIntoFloatBUffer(rawPoints);

      populateLocalizaitonBuffer((float) 0.0, (float) 0.0, transformToWorld);
      populateParametersBuffer();

      updateMapWithKernel(pointsBuffer, rawPoints.size());
   }

   private OpenCLFloatBuffer packPointCloudIntoFloatBUffer(List<Point3D> points)
   {
      OpenCLFloatBuffer pointCloudBuffer = new OpenCLFloatBuffer(points.size() * Float.BYTES);
      pointCloudBuffer.createOpenCLBufferObject(openCLManager);

      FloatBuffer floatBuffer = pointCloudBuffer.getBackingDirectFloatBuffer();

      int index = 0;
      for (int i = 0; i < points.size(); i++)
      {
         floatBuffer.put(index++, points.get(i).getX32());
         floatBuffer.put(index++, points.get(i).getY32());
         floatBuffer.put(index++, points.get(i).getZ32());
      }

      pointCloudBuffer.writeOpenCLBufferObject(openCLManager);

      return pointCloudBuffer;
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

      localizationBuffer.writeOpenCLBufferObject(openCLManager);
   }

   private void populateParametersBuffer()
   {
      int index = 0;
      parametersBuffer.getBytedecoFloatBufferPointer().put(index++, numberOfCells);
      parametersBuffer.getBytedecoFloatBufferPointer().put(index++, numberOfCells);
      parametersBuffer.getBytedecoFloatBufferPointer().put(index++, (float) parameters.minValidDistance);
      parametersBuffer.getBytedecoFloatBufferPointer().put(index++, (float) parameters.rampedHeightRangeA);
      parametersBuffer.getBytedecoFloatBufferPointer().put(index++, (float) parameters.rampedHeightRangeB);
      parametersBuffer.getBytedecoFloatBufferPointer().put(index++, (float) parameters.rampedHeightRangeC);
      parametersBuffer.getBytedecoFloatBufferPointer().put(index++, (float) parameters.maxHeightRange);
      parametersBuffer.getBytedecoFloatBufferPointer().put(index++, (float) parameters.mahalanobisThreshold);
      parametersBuffer.getBytedecoFloatBufferPointer().put(index++, (float) parameters.outlierVariance);
      parametersBuffer.getBytedecoFloatBufferPointer().put(index++, (float) parameters.traversabilityInlier);
      parametersBuffer.getBytedecoFloatBufferPointer().put(index++, (float) parameters.sensorNoiseFactor);
      parametersBuffer.getBytedecoFloatBufferPointer().put(index++, (float) parameters.resolution);

      parametersBuffer.writeOpenCLBufferObject(openCLManager);
   }



   private void updateMapWithKernel(OpenCLFloatBuffer rawPointsBuffer, int pointsSize)
   {
//      updatedMapData.getBackingDirectFloatBuffer(); // need to fill this withzero

      openCLManager.setKernelArgument(errorCountingKernel, 0, elevationMapData.getOpenCLBufferObject());
      openCLManager.setKernelArgument(errorCountingKernel, 1, rawPointsBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(errorCountingKernel, 2, localizationBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(errorCountingKernel, 3, parametersBuffer.getOpenCLBufferObject());
      openCLManager.setKernelArgument(errorCountingKernel, 4, updatedMapData.getOpenCLBufferObject());
      openCLManager.setKernelArgument(errorCountingKernel, 5, errorBuffer.getOpenCLBufferObject());

      openCLManager.execute1D(errorCountingKernel, pointsSize);

      // TODO MODIFY the translation to be relative to the center
   }
}
