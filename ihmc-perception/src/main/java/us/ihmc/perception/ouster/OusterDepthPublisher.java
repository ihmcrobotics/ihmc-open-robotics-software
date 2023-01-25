package us.ihmc.perception.ouster;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import perception_msgs.msg.dds.ImageMessage;
import perception_msgs.msg.dds.LidarScanMessage;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.IHMCRealtimeROS2Publisher;
import us.ihmc.communication.packets.LidarPointCloudCompression;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.OpenCVImageFormat;
import us.ihmc.perception.tools.NativeMemoryTools;
import us.ihmc.ros2.ROS2Topic;

import java.nio.ByteBuffer;
import java.time.Instant;
import java.util.HashMap;

/**
 * This class publishes a PNG compressed depth image from the Ouster as fast as the frames come in.
 */
public class OusterDepthPublisher
{
   private final ROS2Topic<?>[] outputTopics;
   private final HashMap<ROS2Topic<?>, IHMCRealtimeROS2Publisher> publisherMap;

   private final FramePose3D cameraPose = new FramePose3D();

   private final IntPointer compressionParameters;
   private final ByteBuffer pngImageBuffer;
   private final BytePointer pngImageBytePointer;

   private long sequenceNumber = 0;

   private final ImageMessage outputImageMessage = new ImageMessage();
   private final LidarScanMessage lidarScanMessage = new LidarScanMessage();
   private final int depthHeight;
   private final int depthWidth;
   private final int numberOfPointsPerFullScan;

   public OusterDepthPublisher(HashMap<ROS2Topic<?>, IHMCRealtimeROS2Publisher> publisherMap, int depthWidth, int depthHeight, ROS2Topic<?>... outputTopics)
   {
      this.outputTopics = outputTopics;
      this.publisherMap = publisherMap;
      this.depthHeight = depthHeight;
      this.depthWidth = depthWidth;
      numberOfPointsPerFullScan = depthWidth * depthHeight;

      pngImageBuffer = NativeMemoryTools.allocate(depthWidth * depthHeight * 2);
      pngImageBytePointer = new BytePointer(pngImageBuffer);

      compressionParameters = new IntPointer(opencv_imgcodecs.IMWRITE_PNG_COMPRESSION, 1);
   }

   /**
    * Synchronized to make sure it's only running ever once at a time.
    * This should also be guaranteed by the ResettableExceptionHandlingExecutorService.
    */
   public synchronized void extractCompressAndPublish(ReferenceFrame sensorFrame, OusterDepthExtractionKernel depthExtractionKernel, Instant acquisitionInstant)
   {
      for (ROS2Topic<?> topic : outputTopics)
      {
         if (topic.getType().equals(ImageMessage.class))
         {
            // Important not to store as a field, as update() needs to be called each frame
            cameraPose.setToZero(sensorFrame);
            cameraPose.changeFrame(ReferenceFrame.getWorldFrame());

            depthExtractionKernel.runKernel(cameraPose);
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
            outputImageMessage.setFormat(OpenCVImageFormat.PNG.ordinal());
            outputImageMessage.setSequenceNumber(sequenceNumber++);
            outputImageMessage.setImageWidth(depthWidth);
            outputImageMessage.setImageHeight(depthHeight);

            publisherMap.get(topic).publish(outputImageMessage);
         }
         else if (topic.getType().equals(LidarScanMessage.class))
         {
            lidarScanMessage.setUniqueId(sequenceNumber);
            lidarScanMessage.getLidarPosition().set(cameraPose.getPosition());
            lidarScanMessage.getLidarOrientation().set(cameraPose.getOrientation());
            lidarScanMessage.setRobotTimestamp(Conversions.secondsToNanoseconds(acquisitionInstant.getEpochSecond()) + acquisitionInstant.getNano());
            lidarScanMessage.getScan().reset();
            //            compressedPointCloudBuffer.rewind();
            LidarPointCloudCompression.compressPointCloud(numberOfPointsPerFullScan,
                                                          lidarScanMessage,
                                                          (i, j) -> depthExtractionKernel.getPointCloudInSensorFrame().get(3 * i + j));

            publisherMap.get(topic).publish(lidarScanMessage);
         }
         else
         {
            throw new RuntimeException("We don't have the ability to publish this type of message");
         }
      }
   }
}
