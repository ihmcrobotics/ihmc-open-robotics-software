package us.ihmc.perception.gpuHeightMap;

import org.bytedeco.opencl._cl_kernel;
import org.bytedeco.opencl._cl_program;
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

   private final OpenCLFloatBuffer elevationMapData;
   private final OpenCLFloatBuffer updatedMapData;
   private final OpenCLFloatBuffer normalMapData;

   private final _cl_program heightMapProgram;
   private final _cl_kernel addPointsKernel;
   private final _cl_kernel errorCountingKernel;

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

      elevationMapData.createOpenCLBufferObject(openCLManager);
      updatedMapData.createOpenCLBufferObject(openCLManager);
      normalMapData.createOpenCLBufferObject(openCLManager);
      localizationBuffer.createOpenCLBufferObject(openCLManager);

      heightMapProgram = openCLManager.loadProgram("GPUHeightMap");
      addPointsKernel = openCLManager.createKernel(heightMapProgram, "addPointsKernel");
      errorCountingKernel = openCLManager.createKernel(heightMapProgram, "errorCountingKernel");
   }

   public void input(List<Point3D> rawPoints, RigidBodyTransformReadOnly transformToWorld)
   {
      OpenCLFloatBuffer pointsBuffer = packPointCloudIntoFloatBUffer(rawPoints);
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

      return pointCloudBuffer;
   }

   private void populateLocalizaitonBuffer(float centerX, float centerY, ReferenceFrame pointCloudFrame)
   {
      int index = 0;
      localizationBuffer.getBytedecoFloatBufferPointer().put(index++, centerX);
      localizationBuffer.getBytedecoFloatBufferPointer().put(index++, centerY);
      for (int i = 0; i < 3; i++)
      {
         for (int j = 0; j < 3; j++)
            localizationBuffer.getBytedecoFloatBufferPointer().put(index++, (float) pointCloudFrame.getTransformToWorldFrame().getRotation().getElement(i, j));
      }
      for (int i = 0; i < 3; i++)
         localizationBuffer.getBytedecoFloatBufferPointer().put(index++, (float) pointCloudFrame.getTransformToWorldFrame().getTranslation().getElement(i));

      localizationBuffer.writeOpenCLBufferObject(openCLManager);
   }

   private void updateMapWithKernel(OpenCLFloatBuffer rawPointsBuffer, RigidBodyTransformReadOnly transformToWorld, int pointsSize)
   {
      updatedMapData.getBackingDirectFloatBuffer(); // need to fill this withzero

      openCLManager.execute1D(errorCountingKernel, pointsSize);

      // TODO MODIFY the translation to be relative to the center
   }
}
