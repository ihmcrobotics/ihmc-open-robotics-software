package us.ihmc.sensors;

import org.bytedeco.javacpp.BytePointer;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.perception.logging.PerceptionDataLogger;
import us.ihmc.perception.logging.PerceptionLoggerConstants;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.common.SampleInfo;
import us.ihmc.pubsub.subscriber.Subscriber;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.IHMCCommonPaths;
import us.ihmc.tools.string.StringTools;

import java.text.SimpleDateFormat;
import java.time.Instant;
import java.util.Date;

public class ZEDColorAndDepthLogger
{
   protected final ImageMessage imageMessage = new ImageMessage();
   private final SampleInfo sampleInfo = new SampleInfo();
   private final String colorChannelName = PerceptionLoggerConstants.ZED2_COLOR_NAME;
   private final String depthChannelName = PerceptionLoggerConstants.ZED2_DEPTH_NAME;
   private final String timeChannelName = PerceptionLoggerConstants.ZED2_TIME_NAME;
   private final PerceptionDataLogger zedDepthDataLogger;
   private final Object colorBytePointerSyncObject = new Object();
   private BytePointer colorBytePointer;

   public ZEDColorAndDepthLogger(String title,
                                 DomainFactory.PubSubImplementation pubSubImplementation,
                                 ROS2Topic<ImageMessage> depthTopic,
                                 ROS2Topic<ImageMessage> colorTopic)
   {
      zedDepthDataLogger = new PerceptionDataLogger();
      SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
      String depthLogFileName = dateFormat.format(new Date()) + "_" + "ZEDPerceptionDepthLog.hdf5";
      zedDepthDataLogger.openLogFile(IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.resolve(depthLogFileName).toString());

      zedDepthDataLogger.addImageChannel(this.depthChannelName);
      zedDepthDataLogger.setChannelEnabled(this.depthChannelName, true);

      zedDepthDataLogger.addImageChannel(this.colorChannelName);
      zedDepthDataLogger.setChannelEnabled(this.colorChannelName, true);

      zedDepthDataLogger.addLongChannel(this.timeChannelName, 1, PerceptionLoggerConstants.DEFAULT_BLOCK_SIZE);
      zedDepthDataLogger.setChannelEnabled(this.timeChannelName, true);

      RealtimeROS2Node realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(pubSubImplementation, StringTools.titleToSnakeCase(title));
      realtimeROS2Node.createSubscription(depthTopic, this::receiveAndLogDepthImagesCallback);
      realtimeROS2Node.createSubscription(colorTopic, this::receiveAndLogColorImagesCallback);
      realtimeROS2Node.spin();

      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, getClass().getName() + "-Shutdown"));
   }

   private void receiveAndLogDepthImagesCallback(Subscriber<ImageMessage> subscriber)
   {
      Instant current_time = Instant.now();
      subscriber.takeNextData(imageMessage, sampleInfo);

      byte[] heapArray = new byte[PerceptionLoggerConstants.COMPRESSED_IMAGE_BUFFER_SIZE];
      System.arraycopy(imageMessage.getData().toArray(), 0, heapArray, 0, imageMessage.getData().size());

      BytePointer bytePointer = new BytePointer(PerceptionLoggerConstants.COMPRESSED_IMAGE_BUFFER_SIZE);
      bytePointer.put(heapArray, 0, imageMessage.getData().size());
      bytePointer.limit(imageMessage.getData().size());

      zedDepthDataLogger.storeBytesFromPointer(depthChannelName, bytePointer);
      synchronized (colorBytePointerSyncObject)
      {
         zedDepthDataLogger.storeBytesFromPointer(colorChannelName, colorBytePointer);
      }
      zedDepthDataLogger.storeLongs(timeChannelName, current_time.getEpochSecond());
   }

   private void receiveAndLogColorImagesCallback(Subscriber<ImageMessage> subscriber)
   {
      subscriber.takeNextData(imageMessage, sampleInfo);

      byte[] heapArray = new byte[PerceptionLoggerConstants.COMPRESSED_IMAGE_BUFFER_SIZE];
      System.arraycopy(imageMessage.getData().toArray(), 0, heapArray, 0, imageMessage.getData().size());

      synchronized (colorBytePointerSyncObject)
      {
         colorBytePointer = new BytePointer(PerceptionLoggerConstants.COMPRESSED_IMAGE_BUFFER_SIZE);
         colorBytePointer.put(heapArray, 0, imageMessage.getData().size());
         colorBytePointer.limit(imageMessage.getData().size());
      }
   }

   private void destroy()
   {
      zedDepthDataLogger.closeLogFile();
   }

   public static void main(String[] args)
   {
      new ZEDColorAndDepthLogger("ZED 2 Depth",
                                 DomainFactory.PubSubImplementation.FAST_RTPS,
                                 PerceptionAPI.ZED2_DEPTH,
                                 PerceptionAPI.ZED2_COLOR_IMAGES.get(RobotSide.LEFT));
   }
}