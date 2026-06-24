package us.ihmc.perception.detections.foundationPose;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.FoundationPoseRequest;
import perception_msgs.FoundationPoseResult;
import perception_msgs.ImageMessage;
import us.ihmc.commons.thread.RepeatingTaskThread;
import us.ihmc.communication.HumanoidROS2Topic;
import us.ihmc.jros2.ROS2Node;
import us.ihmc.jros2.ROS2Publisher;
import us.ihmc.jros2.ROS2Topic;
import us.ihmc.perception.RawImage;
import us.ihmc.perception.imageMessage.CompressionType;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.sensors.ImageSensor;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Consumer;

public class ROS2FoundationPoseCommunicator
{
   private static final BytePointer JPG = new BytePointer(".jpg");
   private static final BytePointer PNG = new BytePointer(".png");

   private static final HumanoidROS2Topic<?> FOUNDATION_POSE_TOPIC = new HumanoidROS2Topic<>().withPrefix("foundation_pose");
   private static final ROS2Topic<ImageMessage> COLOR_TOPIC = FOUNDATION_POSE_TOPIC.withModule("color_rgb8").withType(ImageMessage.class);
   private static final ROS2Topic<ImageMessage> DEPTH_TOPIC = FOUNDATION_POSE_TOPIC.withModule("depth_mono16").withType(ImageMessage.class);
   private static final ROS2Topic<FoundationPoseRequest> REQUEST_TOPIC = FOUNDATION_POSE_TOPIC.withModule("request").withType(FoundationPoseRequest.class);
   private static final ROS2Topic<std_msgs.String_> REMOVE_TOPIC = FOUNDATION_POSE_TOPIC.withModule("remove").withType(std_msgs.String_.class);
   private static final ROS2Topic<FoundationPoseResult> RESULT_TOPIC = FOUNDATION_POSE_TOPIC.withModule("result").withType(FoundationPoseResult.class);

   private final ROS2Node ros2Node;
   private final ROS2Publisher<FoundationPoseRequest> requestPublisher;
   private final FoundationPoseRequest requestMessage;

   private final ROS2Publisher<std_msgs.String_> removePublisher;
   private final std_msgs.String_ removeMessage;

   private final ROS2Publisher<ImageMessage> colorPublisher;
   private final ROS2Publisher<ImageMessage> depthPublisher;
   private final ImageMessage colorMessage;
   private final ImageMessage depthMessage;

   private final ImageSensor imageSensor;
   private final int colorKey;
   private final int depthKey;
   private final RepeatingTaskThread sensorPublishThread;

   private final List<Consumer<FoundationPoseResult>> resultCallbacks;

   public ROS2FoundationPoseCommunicator(ROS2Node ros2Node, ImageSensor imageSensor, int colorKey, int depthKey)
   {
      this.ros2Node = ros2Node;
      requestPublisher = ros2Node.createPublisher(REQUEST_TOPIC);
      requestMessage = new FoundationPoseRequest();

      removePublisher = ros2Node.createPublisher(REMOVE_TOPIC);
      removeMessage = new std_msgs.String_();

      colorPublisher = ros2Node.createPublisher(COLOR_TOPIC);
      depthPublisher = ros2Node.createPublisher(DEPTH_TOPIC);
      colorMessage = new ImageMessage();
      depthMessage = new ImageMessage();

      ros2Node.createSubscription(RESULT_TOPIC, reader ->
      {
         FoundationPoseResult result = reader.read();
         if (result != null)
            this.onResultReceived(result);
      });

      this.imageSensor = imageSensor;
      this.colorKey = colorKey;
      this.depthKey = depthKey;
      sensorPublishThread = new RepeatingTaskThread(getClass().getSimpleName() + "SensorPublishThread", this::publishSensor);
      sensorPublishThread.startRepeating();

      resultCallbacks = new ArrayList<>();
   }

   public void addResultCallback(Consumer<FoundationPoseResult> callback)
   {
      resultCallbacks.add(callback);
   }

   public void track(String objectId, String meshFileName, RawImage objectMask, RawImage color, RawImage depth)
   {
      // Ensure we get all images
      if (color.get() == null)
         return;

      if (depth.get() == null)
      {
         color.release();
         return;
      }

      if (objectMask.get() == null)
      {
         color.release();
         depth.release();
         return;
      }

      // Get color in rgb
      Mat rgbColor = new Mat();
      color.getPixelFormat().convertToPixelFormat(color.getCpuImageMat(), rgbColor, PixelFormat.RGB8);

      // Compress images
      BytePointer encodedColor = new BytePointer();
      BytePointer encodedDepth = new BytePointer();
      BytePointer encodedMask = new BytePointer();

      opencv_imgcodecs.imencode(JPG, rgbColor, encodedColor);
      opencv_imgcodecs.imencode(PNG, depth.getCpuImageMat(), encodedDepth);
      opencv_imgcodecs.imencode(PNG, objectMask.getCpuImageMat(), encodedMask);

      // Pack and publish request
      requestMessage.setObjectId(objectId);
      requestMessage.setMeshFile(meshFileName);
      PerceptionMessageTools.packImageMessage(color, encodedColor, CompressionType.JPEG, requestMessage.getColor());
      PerceptionMessageTools.packImageMessage(depth, encodedDepth, CompressionType.PNG, requestMessage.getDepth());
      PerceptionMessageTools.packImageMessage(objectMask, encodedMask, CompressionType.PNG, requestMessage.getObjectMask());
      requestPublisher.publish(requestMessage);

      // Release pointers
      rgbColor.close();
      encodedColor.close();
      encodedDepth.close();
      encodedMask.close();
   }

   public void remove(String objectId)
   {
      removeMessage.setData(objectId);
      removePublisher.publish(removeMessage);
   }

   public void destroy()
   {
      sensorPublishThread.kill();
      sensorPublishThread.interrupt();

      ros2Node.destroyPublisher(requestPublisher);
      ros2Node.destroyPublisher(removePublisher);
      ros2Node.destroyPublisher(colorPublisher);
      ros2Node.destroyPublisher(depthPublisher);
   }

   private void onResultReceived(FoundationPoseResult result)
   {
      for (Consumer<FoundationPoseResult> resultCallback : resultCallbacks)
         resultCallback.accept(result);
   }

   private void publishSensor()
   {
      try
      {
         imageSensor.waitForGrab();

         RawImage color = imageSensor.getImage(colorKey);
         RawImage depth = imageSensor.getImage(depthKey);

         // Get color in RGB
         Mat rgbColor = new Mat();
         color.getPixelFormat().convertToPixelFormat(color.getCpuImageMat(), rgbColor, PixelFormat.RGB8);

         // Compress images
         BytePointer encodedColor = new BytePointer();
         BytePointer encodedDepth = new BytePointer();

         opencv_imgcodecs.imencode(JPG, rgbColor, encodedColor);
         opencv_imgcodecs.imencode(PNG, depth.getCpuImageMat(), encodedDepth);

         // Publish compressed images
         PerceptionMessageTools.packImageMessage(color, encodedColor, CompressionType.JPEG, colorMessage);
         PerceptionMessageTools.packImageMessage(depth, encodedDepth, CompressionType.PNG, depthMessage);

         colorPublisher.publish(colorMessage);
         depthPublisher.publish(depthMessage);

         rgbColor.close();
         encodedColor.close();
         encodedDepth.close();
         color.release();
         depth.release();
      }
      catch (InterruptedException ignored) {}
   }
}