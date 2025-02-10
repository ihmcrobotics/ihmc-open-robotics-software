package us.ihmc.perception.gpuHeightMap;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapData;

public interface RapidHeightMapExtractorInterface
{
   Point3D getSensorOrigin();

   int getSequenceNumber();

   TerrainMapData getTerrainMapData();

   void update(RigidBodyTransform sensorToWorldTransform, RigidBodyTransform sensorToGroundTransform, RigidBodyTransform groundToWorldTransform);

   default void updateHeightOffset(float z)
   {
      //Do Nothing
   }
   void reset();

   void destroy();

   HeightMapData getHeightMapData();
}
