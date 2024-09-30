package us.ihmc.perception.streaming;

import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.communication.ros2.ROS2SRTStreamTopicPair;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.perception.CameraModel;
import us.ihmc.perception.camera.CameraIntrinsics;
import us.ihmc.perception.ffmpeg.FFmpegTools;
import us.ihmc.perception.imageMessage.CompressionType;
import us.ihmc.perception.imageMessage.PixelFormat;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2PublisherBasics;

import java.time.Instant;

public class ROS2SRTVideoStreamImageMessageRelayWorker
{
   private final ROS2PublisherBasics<ImageMessage> publisher;
   private final ROS2SRTVideoSubscriber subscriber;

   private final ImageMessage imageMessage;
   private long frameSequenceNumber = 0L;

   public ROS2SRTVideoStreamImageMessageRelayWorker(ROS2Node publisherNode, ROS2Node subscriberNode, ROS2SRTStreamTopicPair streamTopicPair)
   {
      imageMessage = new ImageMessage();
      imageMessage.setPixelFormat(PixelFormat.fromImageType(streamTopicPair.imageType()).toByte());
      imageMessage.setCompressionType(CompressionType.UNCOMPRESSED.toByte());
      imageMessage.setCameraModel(CameraModel.PINHOLE.toByte());

      // Create publisher and subscriber using two separate nodes as publisher should ideally only publish on loopback.
      publisher = publisherNode.createPublisher(streamTopicPair.imageMessageTopic());
      subscriber = new ROS2SRTVideoSubscriber(new ROS2Helper(subscriberNode),
                                              streamTopicPair.streamStatusTopic(),
                                              FFmpegTools.imageTypeToAVPixelFormat(streamTopicPair.imageType()));
      subscriber.addNewFrameConsumer(this::republishFrameAsImageMessage);
      subscriber.subscribe();
   }

   public void destroy()
   {
      subscriber.destroy();
      publisher.remove();
   }

   private void republishFrameAsImageMessage(Mat frame)
   {
      // Set acquisition time as now... this isn't super accurate though
      MessageTools.toMessage(Instant.now(), imageMessage.getAcquisitionTime());

      // Pack the uncompressed data
      PerceptionMessageTools.packImageMessageData(imageMessage, frame.data().limit(OpenCVTools.dataSize(frame)));

      // Set the camera intrinsics
      CameraIntrinsics cameraIntrinsics = subscriber.getCameraIntrinsics();
      imageMessage.setImageWidth(cameraIntrinsics.getWidth());
      imageMessage.setImageHeight(cameraIntrinsics.getHeight());
      imageMessage.setFocalLengthXPixels((float) cameraIntrinsics.getFx());
      imageMessage.setFocalLengthYPixels((float) cameraIntrinsics.getFy());
      imageMessage.setPrincipalPointXPixels((float) cameraIntrinsics.getCx());
      imageMessage.setPrincipalPointYPixels((float) cameraIntrinsics.getCy());
      imageMessage.setDepthDiscretization(subscriber.getDepthDiscretization());

      // Set the sensor pose
      FramePose3DReadOnly sensorPose = subscriber.getSensorPose();
      imageMessage.getPosition().set(sensorPose.getPosition());
      imageMessage.getOrientation().set(sensorPose.getOrientation());

      // Set and increment the sequence number
      imageMessage.setSequenceNumber(frameSequenceNumber++);

      // Send the message
      publisher.publish(imageMessage);
   }
}
