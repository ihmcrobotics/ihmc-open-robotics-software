package us.ihmc.perception;

import perception_msgs.msg.dds.Float32MultiArrayHack;
import std_msgs.msg.dds.MultiArrayDimension;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.gpuMapping.HeightMapData;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Publisher;
import us.ihmc.sensors.ImageSensor;

public class RLPerceptionPublicationManager implements AutoCloseable
{
   private static final float RL_DEPTH_OBSERVATION_SCALE = 0.05f;
   private static final float RL_HEIGHT_SCAN_OBSERVATION_WIDTH = 1.5f;
   private static final float RL_HEIGHT_SCAN_OBSERVATION_HEIGHT = 0.8f;
   private static final float RL_HEIGHT_SCAN_RESOLUTION = 0.05f;
   private static final float RL_HEIGHT_SCAN_OBSERVATION_OFFSET_X = 0.6f;

   // Depth image publishing section
   private final ImageSensor imageSensor;
   private final ImageSensorPublishThread depthPublishThread;

   // Height scan publishing section
   private final GpuMappingThread heightMapThread;
   private final RepeatingTaskThread heightScanPublishThread;
   private final ROS2Publisher<Float32MultiArrayHack> heightScanPublisher;

   public RLPerceptionPublicationManager(ROS2Node ros2Node, ImageSensor imageSensor, int depthImageKey, GpuMappingThread heightMapThread)
   {
      this.imageSensor = imageSensor;
      depthPublishThread = new ImageSensorPublishThread(ros2Node, imageSensor);
      depthPublishThread.addTopic(PerceptionAPI.RL_DEPTH_IMAGE, depthImageKey, RL_DEPTH_OBSERVATION_SCALE);

      this.heightMapThread = heightMapThread;
      heightScanPublisher = ros2Node.createPublisher(PerceptionAPI.RL_HEIGHT_SCAN);
      heightScanPublishThread = new RepeatingTaskThread("RLHeightScanPublishThread", this::publishHeightMap);
   }

   public void start()
   {
      if (imageSensor != null)
         depthPublishThread.start();

      if (heightMapThread != null && imageSensor != null)
         heightScanPublishThread.start();
   }

   @Override
   public void close()
   {
      depthPublishThread.kill();
      heightScanPublishThread.kill();
      heightScanPublisher.remove();
   }

   private void publishHeightMap()
   {
      if (!heightMapThread.blockUntilNextTaskCompletion())
         return;

      HeightMapData heightMapData = heightMapThread.getLatestHeightMapData();

      // FIXME: Assumes height map and depth image comes from same sensor
      RigidBodyTransform heightScanCenter = new RigidBodyTransform(imageSensor.getSensorFrame().getTransformToRoot());
      heightScanCenter.appendTranslation(RL_HEIGHT_SCAN_OBSERVATION_OFFSET_X, 0.0, 0.0);

      int rayCount = Math.round(
            (RL_HEIGHT_SCAN_OBSERVATION_WIDTH * RL_HEIGHT_SCAN_OBSERVATION_HEIGHT) / (RL_HEIGHT_SCAN_RESOLUTION * RL_HEIGHT_SCAN_RESOLUTION));
      float[] rlHeightScanData = new float[rayCount];

      int i = 0;
      for (float y = -0.5f * RL_HEIGHT_SCAN_OBSERVATION_HEIGHT; y < 0.5f * RL_HEIGHT_SCAN_OBSERVATION_HEIGHT; y += RL_HEIGHT_SCAN_RESOLUTION)
      {
         for (float x = -0.5f * RL_HEIGHT_SCAN_OBSERVATION_WIDTH; x < 0.5f * RL_HEIGHT_SCAN_OBSERVATION_WIDTH; x += RL_HEIGHT_SCAN_RESOLUTION)
         {
            RigidBodyTransform rayTransform = new RigidBodyTransform(heightScanCenter);
            rayTransform.appendTranslation(x, y, 0.0);
            float height = (float) heightMapData.getHeight(rayTransform.getTranslationX(), rayTransform.getTranslationY());
            if (i < rayCount)
               rlHeightScanData[i++] = rayTransform.getTranslation().getZ32() - height - 0.5f;
         }
      }

      Float32MultiArrayHack rlHeightScanDataMessage = new Float32MultiArrayHack();
      rlHeightScanDataMessage.getLayout().setDataOffset(0);
      MultiArrayDimension dimension = rlHeightScanDataMessage.getLayout().getDim().add();
      dimension.setSize(rayCount);
      dimension.setStride(rayCount);
      rlHeightScanDataMessage.getData().addAll(rlHeightScanData);
      heightScanPublisher.publish(rlHeightScanDataMessage);
   }
}
