package us.ihmc.perception.ouster;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import perception_msgs.msg.dds.ImageMessage;
import perception_msgs.msg.dds.LidarScanMessage;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.LidarPointCloudCompression;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.perception.CameraModel;
import us.ihmc.perception.comms.ImageMessageFormat;
import us.ihmc.perception.tools.NativeMemoryTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;

import java.nio.ByteBuffer;
import java.time.Instant;
import java.util.function.Supplier;

/**
 * This class publishes a PNG compressed depth image from the Ouster as fast as the frames come in.
 */
public class OusterDepthPublisher
{
   private final RealtimeROS2Node realtimeROS2Node;
   private final IHMCRealtimeROS2Publisher<ImageMessage> imagePublisher;
   private final IHMCRealtimeROS2Publisher<LidarScanMessage> lidarScanPublisher;

   private final FramePose3D cameraPose = new FramePose3D();
   private IntPointer compressionParameters;
   private ByteBuffer pngImageBuffer;
   private BytePointer pngImageBytePointer;
   private long sequenceNumber = 0;
   private final ImageMessage outputImageMessage = new ImageMessage();
   private final Supplier<Boolean> publishLidarScan;
   private final LidarScanMessage lidarScanMessage = new LidarScanMessage();
   private int depthHeight;
   private int depthWidth;
   private int numberOfPointsPerFullScan;

   public OusterDepthPublisher(ROS2Topic<ImageMessage> imageMessageTopic,
                               ROS2Topic<LidarScanMessage> lidarScanTopic,
                               Supplier<Boolean> publishLidarScan)
   {
      this.publishLidarScan = publishLidarScan;

      realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(PubSubImplementation.FAST_RTPS, "ouster_depth_publisher");

      LogTools.info("Publishing ROS 2 ImageMessage: {}", imageMessageTopic);
      imagePublisher = ROS2Tools.createPublisher(realtimeROS2Node, imageMessageTopic);

      if (lidarScanTopic != null)
      {
         LogTools.info("Publishing ROS 2 LidarScanMessage: {}", lidarScanTopic);
         lidarScanPublisher = ROS2Tools.createPublisher(realtimeROS2Node, lidarScanTopic);
      }
      else
      {
         lidarScanPublisher = null;
      }

      realtimeROS2Node.spin();
   }

   public void initialize(int depthWidth, int depthHeight)
   {
      this.depthHeight = depthHeight;
      this.depthWidth = depthWidth;
      numberOfPointsPerFullScan = depthWidth * depthHeight;

      pngImageBuffer = NativeMemoryTools.allocate(depthWidth * depthHeight * 2);
      pngImageBytePointer = new BytePointer(pngImageBuffer);

      compressionParameters = new IntPointer(opencv_imgcodecs.IMWRITE_PNG_COMPRESSION, 1);
   }

   public void extractCompressAndPublish(ReferenceFrame ousterSensorFrame,
                                         OusterDepthExtractionKernel depthExtractionKernel,
                                         Instant acquisitionInstant,
                                         ByteBuffer beamAltitudeAnglesBuffer,
                                         ByteBuffer beamAzimuthAnglesBuffer)
   {
      // Important not to store as a field, as update() needs to be called each frame
      cameraPose.setToZero(ousterSensorFrame);
      cameraPose.changeFrame(ReferenceFrame.getWorldFrame());

      depthExtractionKernel.runKernel(ousterSensorFrame.getTransformToRoot());
      // Encode as PNG which is lossless and handles single channel images.
      opencv_imgcodecs.imencode(".png",
                                depthExtractionKernel.getExtractedDepthImage().getBytedecoOpenCVMat(),
                                pngImageBytePointer,
                                compressionParameters);

      outputImageMessage.getPosition().set(cameraPose.getPosition());
      outputImageMessage.getOrientation().set(cameraPose.getOrientation());
      MessageTools.toMessage(acquisitionInstant, outputImageMessage.getAcquisitionTime());
      // Sadly, Pub Sub makes us go through a TByteArrayList. If we rewrite our DDS layer, we should allow a memcpy to native DDS buffer.
      outputImageMessage.getData().resetQuick();
      for (int i = 0; i < pngImageBytePointer.limit(); i++)
      {
         outputImageMessage.getData().add(pngImageBytePointer.get(i));
      }
      ImageMessageFormat.DEPTH_PNG_16UC1.packMessageFormat(outputImageMessage);
      outputImageMessage.setSequenceNumber(sequenceNumber++);
      outputImageMessage.setImageWidth(depthWidth);
      outputImageMessage.setImageHeight(depthHeight);
      outputImageMessage.setFocalLengthXPixels(depthWidth / (2.0f * (float) Math.PI)); // These are nominal values approximated by Duncan & Tomasz
      outputImageMessage.setFocalLengthYPixels(depthHeight / ((float) Math.PI / 2.0f));
      CameraModel.OUSTER.packMessageFormat(outputImageMessage);
      MessageTools.packIDLSequence(beamAltitudeAnglesBuffer, outputImageMessage.getOusterBeamAltitudeAngles());
      MessageTools.packIDLSequence(beamAzimuthAnglesBuffer, outputImageMessage.getOusterBeamAzimuthAngles());
      imagePublisher.publish(outputImageMessage);

      if (lidarScanPublisher != null && publishLidarScan.get())
      {
         lidarScanMessage.setUniqueId(sequenceNumber);
         lidarScanMessage.getLidarPosition().set(cameraPose.getPosition());
         lidarScanMessage.getLidarOrientation().set(cameraPose.getOrientation());
         lidarScanMessage.setRobotTimestamp(Conversions.secondsToNanoseconds(acquisitionInstant.getEpochSecond()) + acquisitionInstant.getNano());
         lidarScanMessage.getScan().reset();
         LidarPointCloudCompression.compressPointCloud(numberOfPointsPerFullScan,
                                                       lidarScanMessage,
                                                       (i, j) -> depthExtractionKernel.getPointCloudInWorldFrame().get(3 * i + j));
         lidarScanPublisher.publish(lidarScanMessage);
      }
   }

   public void destroy()
   {
      realtimeROS2Node.destroy();
   }
}
