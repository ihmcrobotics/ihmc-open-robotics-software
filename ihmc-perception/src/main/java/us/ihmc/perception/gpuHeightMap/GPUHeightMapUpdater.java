package us.ihmc.perception.gpuHeightMap;

import sensor_msgs.msg.dds.PointCloud;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.idl.IDLSequence;
import us.ihmc.perception.OpenCLFloatBuffer;
import us.ihmc.perception.OpenCLManager;

import java.nio.FloatBuffer;
import java.util.List;

public class GPUHeightMapUpdater
{
   private final OpenCLManager openCLManager = new OpenCLManager();

   private final OpenCLFloatBuffer localizationBuffer = new OpenCLFloatBuffer(14);
   private OpenCLFloatBuffer pointCloudBuffer;

   public GPUHeightMapUpdater()
   {
      openCLManager.create();

      localizationBuffer.createOpenCLBufferObject(openCLManager);
   }

   // point cloud is expected to be in sensor frame
   public void updateMap(PointCloud pointCloud, ReferenceFrame pointCloudFrame)
   {
      pointCloudBuffer = new OpenCLFloatBuffer(pointCloud.getPoints().size() * Float.BYTES);
      pointCloudBuffer.createOpenCLBufferObject(openCLManager);

      packPointCloudIntoFloatBUffer(pointCloud.getPoints(), pointCloudBuffer.getBackingDirectFloatBuffer());
      populateParametersBuffer((float) 0.0, (float) 0.0, pointCloudFrame);
   }


   private void packPointCloudIntoFloatBUffer(List<Point3D32> points, FloatBuffer floatBufferToPack)
   {
      int index = 0;
      for (int i = 0; i < points.size(); i++)
      {
         floatBufferToPack.put(index++, points.get(i).getX32());
         floatBufferToPack.put(index++, points.get(i).getY32());
         floatBufferToPack.put(index++, points.get(i).getZ32());
      }
   }

   private void populateParametersBuffer(float centerX, float centerY, ReferenceFrame pointCloudFrame)
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

}
