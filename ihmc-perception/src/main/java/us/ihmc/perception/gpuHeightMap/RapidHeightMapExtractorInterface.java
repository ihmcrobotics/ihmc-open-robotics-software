package us.ihmc.perception.gpuHeightMap;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.heightMap.TerrainMapData;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;

public interface RapidHeightMapExtractorInterface
{
   void setDepthIntrinsics(CameraIntrinsics cameraIntrinsics);

   Point3D getSensorOrigin();

   HeightMapParameters getParameters();

   int getSequenceNumber();

   TerrainMapData getTerrainMapData();

   void update(RigidBodyTransform sensorToWorldTransform, RigidBodyTransform sensorToGroundTransform, RigidBodyTransform groundToWorldTransform);

   void reset();

   void destroy();

}
