package us.ihmc.avatar.heightMap;

import sensor_msgs.PointCloud2;
import us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher.PointCloudData;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.messager.Messager;
import us.ihmc.sensorProcessing.heightMap.HeightMap;
import us.ihmc.sensorProcessing.heightMap.HeightMapParameters;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;
import java.util.concurrent.atomic.AtomicBoolean;

public class HeightMapUpdater
{
   private final Messager messager;
   private final ExecutorService heightMapUpdater = Executors.newSingleThreadExecutor(ThreadTools.createNamedThreadFactory(getClass().getSimpleName()));
   private final AtomicBoolean processing = new AtomicBoolean();

   private final RigidBodyTransform transform = new RigidBodyTransform();

   private final HeightMap heightMap;
   private final HeightMapDataToVisualize dataToVisualize = new HeightMapDataToVisualize();

   public HeightMapUpdater(Messager messager)
   {
      this.messager = messager;

      HeightMapParameters parameters = new HeightMapParameters();
      heightMap = new HeightMap(parameters.getGridResolutionXY(), parameters.getGridSizeXY());

      messager.registerTopicListener(HeightMapMessagerAPI.PointCloudData, pointCloud ->
      {
         if (!processing.getAndSet(true))
         {
            heightMapUpdater.execute(() -> update(pointCloud));
         }
      });

      transform.getRotation().setToPitchOrientation(Math.toRadians(45.0));
   }

   private void update(PointCloud2 pointCloud)
   {
      PointCloudData pointCloudData = new PointCloudData(pointCloud, 1000000, false);

      // Transform ouster data
      for (int i = 0; i < pointCloudData.getPointCloud().length; i++)
      {
         pointCloudData.getPointCloud()[i].applyTransform(transform);
      }

      // Update height map
      heightMap.update(pointCloudData.getPointCloud());

      // Copy and report over messager
      dataToVisualize.clear();
      dataToVisualize.getXCells().addAll(heightMap.getXCells());
      dataToVisualize.getYCells().addAll(heightMap.getYCells());
      for (int i = 0; i < heightMap.getXCells().size(); i++)
      {
         dataToVisualize.getHeights().add(heightMap.getHeightAt(heightMap.getXCells().get(i), heightMap.getYCells().get(i)));
      }
      messager.submitMessage(HeightMapMessagerAPI.HeightMapData, dataToVisualize);

      processing.set(false);
   }
}
