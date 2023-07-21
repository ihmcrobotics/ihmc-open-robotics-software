package us.ihmc.perception.realsense;

import boofcv.struct.calib.CameraPinholeBrown;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.message.Time;
import perception_msgs.msg.dds.VideoPacket;
import sensor_msgs.Image;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.MutableBytePointer;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.PausablePeriodicThread;
import us.ihmc.utilities.ros.ROS1Helper;
import us.ihmc.utilities.ros.RosTools;
import us.ihmc.utilities.ros.publisher.RosCameraInfoPublisher;
import us.ihmc.utilities.ros.publisher.RosImagePublisher;

public class RealsenseL515ROSNode
{
   private static final String SERIAL_NUMBER = System.getProperty("l515.serial.number", "F0000000");

   private final PausablePeriodicThread thread;
   private final ROS1Helper ros1Helper;
   private final ROS2Helper ros2Helper;
   private RealSenseHardwareManager realSenseHardwareManager;
   private BytedecoRealsense l515;
   private RosImagePublisher ros1DepthPublisher;
   private RosCameraInfoPublisher ros1DepthCameraInfoPublisher;
   private ChannelBuffer ros1DepthChannelBuffer;
   private Mat depthU16C1Image;
   private BytedecoImage depth32FC1Image;
   private int depthWidth;
   private int depthHeight;
   private CameraPinholeBrown depthCameraIntrinsics;
   private static final boolean publishROS1 = true;

   public RealsenseL515ROSNode()
   {
      ros1Helper = new ROS1Helper("l515_node");

      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "l515_node");
      ros2Helper = new ROS2Helper(ros2Node);

      thread = new PausablePeriodicThread("L515Node", UnitConversions.hertzToSeconds(31.0), false, this::update);
      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, "L515Shutdown"));
      thread.start();

      realSenseHardwareManager = new RealSenseHardwareManager();
      l515 = realSenseHardwareManager.createFullFeaturedL515(SERIAL_NUMBER);
      if (l515.getDevice() == null)
      {
         thread.stop();
         throw new RuntimeException("Device not found. Set -Dl515.serial.number=F0000000");
      }
      l515.initialize();

      depthWidth = l515.getDepthWidth();
      depthHeight = l515.getDepthHeight();

      if (publishROS1)
      {
         String ros1DepthImageTopic = RosTools.L515_DEPTH;
         String ros1DepthCameraInfoTopic = RosTools.L515_DEPTH_CAMERA_INFO;
         LogTools.info("Publishing ROS 1 depth: {} {}", ros1DepthImageTopic, ros1DepthCameraInfoTopic);
         ros1DepthPublisher = new RosImagePublisher();
         ros1DepthCameraInfoPublisher = new RosCameraInfoPublisher();
         ros1Helper.attachPublisher(ros1DepthCameraInfoTopic, ros1DepthCameraInfoPublisher);
         ros1Helper.attachPublisher(ros1DepthImageTopic, ros1DepthPublisher);
         ros1DepthChannelBuffer = ros1DepthPublisher.getChannelBufferFactory().getBuffer(2 * depthWidth * depthHeight);
      }

      depthCameraIntrinsics = new CameraPinholeBrown();
   }

   private void update()
   {
      if (l515.readFrameData())
      {
         l515.updateDataBytePointers();

         if (depthU16C1Image == null)
         {
            MutableBytePointer depthFrameData = l515.getDepthFrameData();
            depthU16C1Image = new Mat(l515.getDepthHeight(), l515.getDepthWidth(), opencv_core.CV_16UC1, depthFrameData);
            depth32FC1Image = new BytedecoImage(l515.getDepthWidth(), l515.getDepthHeight(), opencv_core.CV_32FC1);

            depthCameraIntrinsics.setFx(l515.getDepthFocalLengthPixelsX());
            depthCameraIntrinsics.setFy(l515.getDepthFocalLengthPixelsY());
            depthCameraIntrinsics.setSkew(0.0);
            depthCameraIntrinsics.setCx(l515.getDepthPrincipalOffsetXPixels());
            depthCameraIntrinsics.setCy(l515.getDepthPrincipalOffsetYPixels());
         }

         VideoPacket videoPacket = new VideoPacket();
         videoPacket.setImageHeight(depthHeight);
         videoPacket.setImageWidth(depthWidth);
         BytePointer dataPointer = depthU16C1Image.ptr();
         int depthFrameDataSize = l515.getDepthFrameDataSize();
         for (int i = 0; i < depthFrameDataSize; i++)
         {
            videoPacket.getData().add(dataPointer.get(i));
         }
         ros2Helper.publish(ROS2Tools.IHMC_ROOT.withModule("l515").withType(VideoPacket.class).withSuffix("depth"), videoPacket);

         if (publishROS1 && ros1DepthPublisher.isConnected() && ros1DepthCameraInfoPublisher.isConnected())
         {
            ros1DepthChannelBuffer.clear();
            for (int i = 0; i < depthFrameDataSize; i++)
            {
               ros1DepthChannelBuffer.writeByte(dataPointer.get(i));
            }

            ros1DepthChannelBuffer.readerIndex(0);
            ros1DepthChannelBuffer.writerIndex(depthFrameDataSize);

            double now = Conversions.nanosecondsToSeconds(System.nanoTime());
            ros1DepthCameraInfoPublisher.publish("camera_depth_optical_frame", depthCameraIntrinsics, new Time(now));
            // maybe need to copy here if there are errors
            int bytesPerValue = 2;
            Image message = ros1DepthPublisher.createMessage(depthWidth, depthHeight, bytesPerValue, "16UC1", ros1DepthChannelBuffer);
            //               if(timestampSupplier != null)
            //                  message.getHeader().setStamp(new Time(Conversions.nanosecondsToSeconds(timestampSupplier.getAsLong())));
            message.getHeader().setStamp(new Time(now));

            ros1DepthPublisher.publish(message);
         }
      }
   }

   private void destroy()
   {
      l515.deleteDevice();
      realSenseHardwareManager.deleteContext();
   }

   public static void main(String[] args)
   {
      new RealsenseL515ROSNode();
   }
}
