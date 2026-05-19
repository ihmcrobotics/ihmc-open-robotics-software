package us.ihmc.perception;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.GpuMat;
import perception_msgs.msg.dds.Float32MultiArrayHack;
import perception_msgs.msg.dds.ImageMessage;
import std_msgs.msg.dds.MultiArrayDimension;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.gpuMapping.HeightMapData;
import us.ihmc.perception.imageMessage.CompressionType;
import us.ihmc.perception.imageMessage.PixelFormat;
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

      GpuMat float32DepthMat = new GpuMat();
      downScaledImage.getGpuImageMat().convertTo(float32DepthMat, opencv_core.CV_32FC1, downScaledImage.getDepthDiscretization());
      RawImage float32DepthImage = downScaledImage.replaceImage(float32DepthMat, PixelFormat.GRAY_F32);

      PerceptionMessageTools.packImageMessage(float32DepthImage, float32DepthImage.getCpuImageMat().data(), CompressionType.UNCOMPRESSED, depthImageMessage);
      depthImagePublisher.publish(depthImageMessage);

      float32DepthImage.release();
      downScaledImage.release();
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

      // Sample on grid vertices (inclusive endpoints) to match IsaacLab's GridPatternCfg,
      // which places rays at corners: count = (size/res + 1) per axis.
      int nx = Math.round(RL_HEIGHT_SCAN_OBSERVATION_HEIGHT / RL_HEIGHT_SCAN_RESOLUTION) + 1;
      int ny = Math.round(RL_HEIGHT_SCAN_OBSERVATION_WIDTH / RL_HEIGHT_SCAN_RESOLUTION) + 1;
      int rayCount = nx * ny;
      float[] rlHeightScanData = new float[rayCount];

      int i = 0;
      for (int iy = 0; iy < ny; iy++)
      {
         float y = -0.5f * RL_HEIGHT_SCAN_OBSERVATION_WIDTH + iy * RL_HEIGHT_SCAN_RESOLUTION;
         for (int ix = 0; ix < nx; ix++)
         {
            float x = -0.5f * RL_HEIGHT_SCAN_OBSERVATION_HEIGHT + ix * RL_HEIGHT_SCAN_RESOLUTION;
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
