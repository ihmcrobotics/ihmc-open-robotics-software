package us.ihmc.perception.voxelMap;

import perception_msgs.VoxelMapMessage;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.perception.RawImage;
import us.ihmc.sensors.ImageSensor;

import java.util.Map;
import java.util.function.Supplier;

public class VoxelMappingThread extends RepeatingTaskThread
{
   private final Map<ImageSensor, Integer> sensorDepthImageKeyMap;

   private final Supplier<RigidBodyTransformReadOnly> mapOriginSupplier;

   private final VoxelMapExtractor extractor;

   private final ROS2Node ros2Node;
   private final ROS2Publisher<VoxelMapMessage> publisher;
   private final VoxelMapMessage voxelMapMessage;

   public VoxelMappingThread(ROS2Node ros2Node,
                             ROS2Topic<VoxelMapMessage> ros2Topic,
                             int mapSize,
                             float voxelSize,
                             Supplier<RigidBodyTransformReadOnly> mapOriginSupplier,
                             Map<ImageSensor, Integer> sensorDepthImageKeyMap)
   {
      super(VoxelMappingThread.class.getSimpleName());

      this.ros2Node = ros2Node;
      publisher = ros2Node.createPublisher(ros2Topic);
      voxelMapMessage = new VoxelMapMessage();

      this.mapOriginSupplier = mapOriginSupplier;
      this.sensorDepthImageKeyMap = sensorDepthImageKeyMap;

      extractor = new VoxelMapExtractor(mapSize, voxelSize);
   }

   @Override
   protected void runTask()
   {
      RawImage[] depthImages = new RawImage[sensorDepthImageKeyMap.size()];
      int arrayIndex = 0;
      for (ImageSensor imageSensor : sensorDepthImageKeyMap.keySet())
      {
         RawImage depthImage = imageSensor.getImage(sensorDepthImageKeyMap.get(imageSensor));
         if (depthImage != null)
            depthImages[arrayIndex++] = depthImage;
      }

      if (arrayIndex == 0)
         return;

      VoxelMap map = extractor.getVoxelMap(mapOriginSupplier.get(), depthImages);

      map.toMessage(voxelMapMessage);
      publisher.publish(voxelMapMessage);

      map.close();

      for (RawImage image : depthImages)
         if (image != null)
            image.release();
   }

   @Override
   public void kill()
   {
      super.kill();

      extractor.close();
      if (publisher != null)
         ros2Node.destroyPublisher(publisher);
   }
}
