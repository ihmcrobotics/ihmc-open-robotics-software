package us.ihmc.perception;

import sensor_msgs.msg.dds.CameraInfo;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.packets.Packet;
import us.ihmc.communication.ros2.tf2.ROS2FollowingFrame;
import us.ihmc.communication.ros2.tf2.ROS2StaticFrame;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.yawPitchRoll.YawPitchRoll;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.sensors.ImageSensor;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;

public class ImageSensorPublishThread extends RepeatingTaskThread
{
   // Read about optical frames here: https://ros.org/reps/rep-0103.html
   private static final RigidBodyTransformReadOnly CAMERA_TO_OPTICAL_TRANSFORM = new RigidBodyTransform(new YawPitchRoll(-0.5 * Math.PI, 0.0, -0.5 * Math.PI),
                                                                                                        new Vector3D());

   private final ROS2Node ros2Node;

   private final Map<ROS2Topic<? extends Packet<?>>, Integer> topicToImageKeyMap = new HashMap<>();
   private final Map<ROS2Topic<? extends Packet<?>>, Long> lastPublishedSequenceNumbers = new HashMap<>();
   private final RawImagePublisher publisher;
   private boolean publishingIsThrottled = false;

   private final ImageSensor imageSensor;

   private final Map<Integer, ROS2FollowingFrame> ros2CameraFrames = new HashMap<>();
   private final Map<Integer, ROS2StaticFrame> ros2OpticalFrames = new HashMap<>();

   private boolean ros2FramesEnabled = false;
   private int cameraInfoPublishModulus = 1;

   public ImageSensorPublishThread(ROS2Node ros2Node, ImageSensor sensorToPublish)
   {
      super(sensorToPublish.getSensorName() + "PublishThread");

      this.ros2Node = ros2Node;

      imageSensor = sensorToPublish;
      publisher = new RawImagePublisher(ros2Node);
   }

   public void addTopic(ROS2Topic<? extends Packet<?>> topicToPublishOn, int imageKey)
   {
      topicToImageKeyMap.put(topicToPublishOn, imageKey);
   }

   @Override
   public ImageSensorPublishThread setFrequencyLimit(double publishFrequencyLimit)
   {
      super.setFrequencyLimit(publishFrequencyLimit);
      publishingIsThrottled = publishFrequencyLimit > 0.0;
      return this;
   }

   /**
    * If enabled, this thread will manage a ROS 2 frame for each image frame,
    * and a corresponding optical frame.
    * <p>
    * For more info on optical frames, see
    * <a href="https://ros.org/reps/rep-0103.html">ROS REP 103 - Standard Units of Measure and Coordinate Conventions</a>.
    *
    * @param enable Whether to enable adding ROS 2 frames for the image frames.
    */
   public void enableROS2Frames(boolean enable)
   {
      ros2FramesEnabled = enable;
   }

   /**
    * Set the number of grabs to skip between publishing {@link CameraInfo} messages.
    * If set to zero, a message will be published every grab (default).
    * If set to non-zero, {@code skip} number of grabs will be skipped after publishing a message.
    * Do not set to a negative value.
    *
    * @param skip Number of grabs to skip between publishing {@link CameraInfo} messages.
    */
   public void setCameraInfoPublishGrabSkipCount(int skip)
   {
      cameraInfoPublishModulus = skip + 1;
   }

   @Override
   protected void runTask()
   {
      try
      {  // If publisher frequency is not set, run at sensor's frequency
         if (!publishingIsThrottled)
            imageSensor.waitForGrab();

         for (Entry<ROS2Topic<? extends Packet<?>>, Integer> imageEntry : topicToImageKeyMap.entrySet())
         {
            ROS2Topic<? extends Packet<?>> imageTopic = imageEntry.getKey();

            // If the topic is a camera info topic, check whether we should skip this publish
            if (imageTopic.getType().equals(CameraInfo.class) && getCompleted() % cameraInfoPublishModulus != 0)
               continue;

            // Get the image to publish
            int imageKey = imageEntry.getValue();
            RawImage imageToPublish = imageSensor.getImage(imageKey);

            // Skip if the image is null
            if (imageToPublish == null)
               continue;

            // Skip if this image was already published on the topic
            if (lastPublishedSequenceNumbers.containsKey(imageTopic) && imageToPublish.getSequenceNumber() <= lastPublishedSequenceNumbers.get(imageTopic))
               continue;

            // Store the sequence number to avoid re-publishing this image
            lastPublishedSequenceNumbers.put(imageTopic, imageToPublish.getSequenceNumber());

            // Update sensor frames
            ReferenceFrame imageFrame = imageSensor.getImageFrame(imageKey);
            if (ros2FramesEnabled)
            {
               updateROS2Frames(imageKey);
               imageFrame = ros2OpticalFrames.get(imageKey);
            }

            // Publish the image
            publisher.publishImage(imageTopic, imageToPublish, imageFrame);
            imageToPublish.release();
         }
      }
      catch (InterruptedException ignored) {}
   }

   private void updateROS2Frames(int imageKey)
   {
      ReferenceFrame imageFrame = imageSensor.getImageFrame(imageKey);

      ROS2FollowingFrame cameraFrame = ros2CameraFrames.get(imageKey);
      ROS2StaticFrame opticalFrame = ros2OpticalFrames.get(imageKey);

      // Ensure we've added these frames
      if (cameraFrame == null)
      {
         cameraFrame = new ROS2FollowingFrame(ros2Node, "ros2_" + imageFrame.getName(), imageSensor.getSensorFrame(), imageFrame);
         ros2CameraFrames.put(imageKey, cameraFrame);
      }

      if (opticalFrame == null)
      {
         opticalFrame = new ROS2StaticFrame(ros2Node, cameraFrame.getFrameId() + "_optical", cameraFrame, CAMERA_TO_OPTICAL_TRANSFORM);
         ros2OpticalFrames.put(imageKey, opticalFrame);
      }

      // If sensor frame changed, update our frames as well
      if (!cameraFrame.getParent().equals(imageSensor.getSensorFrame()))
      {
         cameraFrame.remove();
         opticalFrame.remove();

         cameraFrame = new ROS2FollowingFrame(ros2Node, "ros2_" + imageFrame.getName(), imageSensor.getSensorFrame(), imageFrame);
         opticalFrame = new ROS2StaticFrame(ros2Node, cameraFrame.getFrameId() + "_optical", cameraFrame, CAMERA_TO_OPTICAL_TRANSFORM);

         ros2CameraFrames.replace(imageKey, cameraFrame);
         ros2OpticalFrames.replace(imageKey, opticalFrame);
      }

      cameraFrame.update();
      opticalFrame.update();
   }

   @Override
   public void kill()
   {
      super.kill();
      interrupt();

      ros2CameraFrames.values().forEach(ReferenceFrame::remove);
      ros2OpticalFrames.values().forEach(ReferenceFrame::remove);
      publisher.close();
   }
}
