package us.ihmc.perception;

import org.bytedeco.opencv.global.opencv_imgproc;
import perception_msgs.msg.dds.Float32MultiArrayHack;
import perception_msgs.msg.dds.ImageMessage;
import std_msgs.msg.dds.MultiArrayDimension;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.gpuMapping.HeightMapData;
import us.ihmc.perception.imageMessage.CompressionType;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.perception.tools.RawImageTools;
import us.ihmc.robotics.referenceFrames.ZUpFrame;
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

   private final ImageSensor imageSensor;

   // Depth image publishing section
   private final int depthImageKey;
   private final ROS2Publisher<ImageMessage> depthImagePublisher;
   private final ImageMessage depthImageMessage;
   private final RepeatingTaskThread depthImagePublishThread;

   // Height scan publishing section
   private final GpuMappingThread heightMapThread;
   private final ZUpFrame sensorZUpFrame;
   private final ROS2Publisher<Float32MultiArrayHack> heightScanPublisher;
   private final Float32MultiArrayHack rlHeightScanDataMessage = new Float32MultiArrayHack();
   private final RepeatingTaskThread heightScanPublishThread;

   public RLPerceptionPublicationManager(ROS2Node ros2Node, ImageSensor imageSensor, int depthImageKey, GpuMappingThread heightMapThread)
   {
      this.imageSensor = imageSensor;
      this.depthImageKey = depthImageKey;
      this.heightMapThread = heightMapThread;

      depthImagePublisher = ros2Node.createPublisher(PerceptionAPI.RL_DEPTH_IMAGE);
      depthImageMessage = new ImageMessage();
      depthImagePublishThread = new RepeatingTaskThread("RLDepthImagePublishThread", this::publishDepthImage);

      // Assumes height map and depth image come from same sensor
      sensorZUpFrame = new ZUpFrame(imageSensor.getSensorFrame(), imageSensor.getSensorFrame().getName() + "ZUp");
      heightScanPublisher = ros2Node.createPublisher(PerceptionAPI.RL_HEIGHT_SCAN);
      heightScanPublishThread = new RepeatingTaskThread("RLHeightScanPublishThread", this::publishHeightMap);
   }

   public void start()
   {
      if (imageSensor != null)
         depthImagePublishThread.startRepeating();

      if (heightMapThread != null && imageSensor != null)
         heightScanPublishThread.startRepeating();
   }

   @Override
   public void close()
   {
      depthImagePublishThread.kill();
      heightScanPublishThread.kill();
      heightScanPublisher.remove();
   }

   private void publishDepthImage() throws InterruptedException
   {
      imageSensor.waitForGrab();

      RawImage depthImage = imageSensor.getImage(depthImageKey);
      if (depthImage == null)
         return;

      RawImage downScaledImage = RawImageTools.scale(depthImage, RL_DEPTH_OBSERVATION_SCALE, opencv_imgproc.INTER_NEAREST);
      PerceptionMessageTools.packImageMessage(downScaledImage, downScaledImage.getCpuImageMat().data(), CompressionType.UNCOMPRESSED, depthImageMessage);
      depthImagePublisher.publish(depthImageMessage);

      depthImage.release();
   }

   private void publishHeightMap()
   {
      if (!heightMapThread.blockUntilNextTaskCompletion())
         return;

      HeightMapData heightMapData = heightMapThread.getLatestHeightMapData();

      sensorZUpFrame.update();
      RigidBodyTransform heightScanCenter = new RigidBodyTransform(sensorZUpFrame.getTransformToRoot());
      heightScanCenter.appendTranslation(RL_HEIGHT_SCAN_OBSERVATION_OFFSET_X, 0.0, 0.0);

      int rayCount = Math.round(
            (RL_HEIGHT_SCAN_OBSERVATION_WIDTH * RL_HEIGHT_SCAN_OBSERVATION_HEIGHT) / (RL_HEIGHT_SCAN_RESOLUTION * RL_HEIGHT_SCAN_RESOLUTION));
      float[] rlHeightScanData = new float[rayCount];

      int i = 0;
      for (float y = 0.5f * RL_HEIGHT_SCAN_OBSERVATION_WIDTH; y > -0.5f * RL_HEIGHT_SCAN_OBSERVATION_WIDTH && i < rayCount; y -= RL_HEIGHT_SCAN_RESOLUTION)
      {
         for (float x = -0.5f * RL_HEIGHT_SCAN_OBSERVATION_HEIGHT; x < 0.5f * RL_HEIGHT_SCAN_OBSERVATION_HEIGHT && i < rayCount; x += RL_HEIGHT_SCAN_RESOLUTION)
         {
            RigidBodyTransform rayTransform = new RigidBodyTransform(heightScanCenter);
            rayTransform.appendTranslation(x, y, 0.0);
            float height = (float) heightMapData.getHeight(rayTransform.getTranslationX(), rayTransform.getTranslationY());
            rlHeightScanData[i++] = rayTransform.getTranslation().getZ32() - height - 0.5f;
         }
      }

      rlHeightScanDataMessage.getLayout().setDataOffset(0);
      rlHeightScanDataMessage.getLayout().getDim().clear();
      MultiArrayDimension dimension = rlHeightScanDataMessage.getLayout().getDim().add();
      dimension.setSize(rayCount);
      dimension.setStride(rayCount);
      rlHeightScanDataMessage.getData().clear(rayCount);
      rlHeightScanDataMessage.getData().addAll(rlHeightScanData);
      heightScanPublisher.publish(rlHeightScanDataMessage);
   }
}
