package us.ihmc.sensors;

import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.perception.CameraModel;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.imageMessage.ImageMessageDecoder;
import us.ihmc.robotics.referenceFrames.MutableReferenceFrame;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2NodeBuilder;
import us.ihmc.ros2.ROS2Subscription;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.sensors.realsense.RealSenseImageSensor;

import java.time.Instant;
import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;
import java.util.concurrent.CountDownLatch;

public class ROS2RelayImageSensor extends ImageSensor
{
   private final ROS2Node ros2Node;
   private final Map<Integer, ImageReceiver> receivers;
   private final CountDownLatch receiveCountdown;

   private final int[] imageKeys;
   private final Map<Integer, RawImage> receivedImages;
   private final Map<Integer, MutableReferenceFrame> imageFrames;
   private final ReferenceFrame[] imageFrameArray;

   private volatile boolean running = false;

   public ROS2RelayImageSensor(String sensorName, Map<Integer, ROS2Topic<ImageMessage>> imageTopics)
   {
      super(sensorName);

      ros2Node = new ROS2NodeBuilder().build("ros2_" + sensorName.toLowerCase().replaceAll(" ", "_") + "_image_sensor_node");
      receivers = new HashMap<>();
      receiveCountdown = new CountDownLatch(imageTopics.size());

      imageKeys = new int[imageTopics.size()];
      receivedImages = new HashMap<>();
      imageFrames = new HashMap<>();
      imageFrameArray = new ReferenceFrame[imageTopics.size()];

      int i = 0;
      for (Map.Entry<Integer, ROS2Topic<ImageMessage>> entry : imageTopics.entrySet())
      {
         int imageKey = entry.getKey();
         imageKeys[i] = imageKey;

         receivers.put(imageKey, new ImageReceiver(ros2Node, entry.getValue(), receiveCountdown));
         imageFrames.put(imageKey, new MutableReferenceFrame("ROS2_" + sensorName + "_Image_" + imageKey + "+_Frame"));
         imageFrameArray[i] = imageFrames.get(imageKey).getReferenceFrame();

         ++i;
      }
   }

   @Override
   protected boolean startSensor()
   {
      for (ImageReceiver receiver : receivers.values())
         receiver.start();

      running = true;
      return true;
   }

   @Override
   public boolean isSensorRunning()
   {
      return running;
   }

   @Override
   protected boolean grab()
   {
      if (!running)
         return false;

      try
      {
         receiveCountdown.await();
         for (Entry<Integer, ImageReceiver> entry : receivers.entrySet())
         {
            int imageKey = entry.getKey();
            ImageReceiver receiver = entry.getValue();

            synchronized (receivedImages)
            {
               if (receivedImages.get(imageKey) != null)
                  receivedImages.get(imageKey).release();

               receivedImages.put(imageKey, receiver.getReceivedImage());
               imageFrames.get(imageKey).update(transformToWorld -> transformToWorld.set(receivedImages.get(imageKey).getTransformToWorld()));
            }
            receiver.armCountdown();
         }
      }
      catch (InterruptedException interruptedException)
      {
         return false;
      }

      return true;
   }

   @Override
   public int[] getImageKeys()
   {
      return imageKeys;
   }

   @Override
   public RawImage getImage(int imageKey)
   {
      synchronized (receivedImages)
      {
         return receivedImages.get(imageKey).get();
      }
   }

   @Override
   public ReferenceFrame getImageFrame(int imageKey)
   {
      synchronized (receivedImages)
      {
         return imageFrames.get(imageKey).getReferenceFrame();
      }
   }

   @Override
   public ReferenceFrame[] getImageFrames()
   {
      return imageFrameArray;
   }

   @Override
   public void close()
   {
      running = false;

      super.close();

      for (ImageReceiver receiver : receivers.values())
         receiver.close();

      ros2Node.destroy();
   }

   private static class ImageReceiver implements AutoCloseable
   {
      private final CountDownLatch latch;
      private volatile boolean countDown = false;

      private RawImage receivedImage;
      private final Mat decodeMat;

      private final ROS2Subscription<ImageMessage> subscription;
      private final ImageMessageDecoder decoder;

      private volatile boolean running = false;

      private ImageReceiver(ROS2Node ros2Node, ROS2Topic<ImageMessage> imageTopic, CountDownLatch latch)
      {
         this.latch = latch;

         subscription = ros2Node.createSubscription2(imageTopic, this::receiveImage);
         decodeMat = new Mat();
         decoder = new ImageMessageDecoder();
      }

      private void receiveImage(ImageMessage imageMessage)
      {
         if (!running)
            return;

         // Get some metadata
         CameraIntrinsics cameraIntrinsics = new CameraIntrinsics(imageMessage.getImageHeight(),
                                                                  imageMessage.getImageWidth(),
                                                                  imageMessage.getFocalLengthXPixels(),
                                                                  imageMessage.getFocalLengthYPixels(),
                                                                  imageMessage.getPrincipalPointXPixels(),
                                                                  imageMessage.getPrincipalPointYPixels());

         CameraModel cameraModel = CameraModel.fromByte(imageMessage.getCameraModel());

         RigidBodyTransform transformToWorld = new RigidBodyTransform(imageMessage.getOrientation(), imageMessage.getPosition());

         Instant imageAcquisitionTime = MessageTools.toInstant(imageMessage.getAcquisitionTime());

         long sequenceNumber = imageMessage.getSequenceNumber();

         float depthDiscretization = imageMessage.getDepthDiscretization();

         decoder.decodeMessage(imageMessage, decodeMat);

         synchronized (decodeMat)
         {
            if (receivedImage != null)
               receivedImage.release();

            receivedImage = new RawImage(decodeMat,
                                         null,
                                         decoder.getDecodedImagePixelFormat(),
                                         cameraIntrinsics,
                                         cameraModel,
                                         transformToWorld,
                                         imageAcquisitionTime,
                                         sequenceNumber,
                                         depthDiscretization);
         }

         if (countDown)
         {
            latch.countDown();
            countDown = false;
         }
      }

      private void armCountdown()
      {
         countDown = true;
      }

      private RawImage getReceivedImage()
      {
         synchronized (decodeMat)
         {
            return receivedImage.get();
         }
      }

      private void start()
      {
         running = true;
      }

      @Override
      public void close()
      {
         running = false;
         subscription.remove();
      }
   }

   public static ROS2RelayImageSensor createRealSenseRelay()
   {
      return new ROS2RelayImageSensor("RealSense",
                                      Map.of(RealSenseImageSensor.COLOR_IMAGE_KEY,
                                             PerceptionAPI.D455_COLOR_IMAGE,
                                             RealSenseImageSensor.DEPTH_IMAGE_KEY,
                                             PerceptionAPI.D455_DEPTH_IMAGE));
   }
}
