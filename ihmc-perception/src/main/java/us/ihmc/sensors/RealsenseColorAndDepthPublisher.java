package us.ihmc.sensors;

import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.MutableBytePointer;
import us.ihmc.perception.comms.ImageMessageFormat;
import us.ihmc.perception.realsense.BytedecoRealsense;
import us.ihmc.perception.realsense.RealSenseHardwareManager;
import us.ihmc.perception.realsense.RealsenseSettingsProfile;
import us.ihmc.perception.tools.OpenCVJPEGCompression;
import us.ihmc.perception.tools.OpenCVPNGCompression;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.thread.Throttler;

import java.time.Instant;
import java.util.function.Supplier;

/**
 * Publishes color and depth images from Realsense devices.
 */
public class RealsenseColorAndDepthPublisher
{
   private final String serialNumber;
   private final Activator nativesLoadedActivator;
   private final ROS2Helper ros2Helper;
   private final Supplier<ReferenceFrame> sensorFrameUpdater;
   private final FramePose3D cameraPose = new FramePose3D();
   private final ROS2Topic<ImageMessage> colorTopic;
   private final ROS2Topic<ImageMessage> depthTopic;
   private volatile boolean running = true;
   private final double outputPeriod = UnitConversions.hertzToSeconds(30.0);
   private final Throttler throttler = new Throttler();

   private final RealsenseSettingsProfile settingsProfile;
   private RealSenseHardwareManager realSenseHardwareManager;
   private BytedecoRealsense realsense;
   private Mat depth16UC1Image;
   private Mat color8UC3Image;

   private final OpenCVPNGCompression pngCompression = new OpenCVPNGCompression();
   private final OpenCVJPEGCompression jpegCompression = new OpenCVJPEGCompression(80);
   private final ImageMessage colorImageMessage = new ImageMessage();
   private final ImageMessage depthImageMessage = new ImageMessage();
   private long depthSequenceNumber = 0;
   private long colorSequenceNumber = 0;

   public RealsenseColorAndDepthPublisher(String serialNumber,
                                          RealsenseSettingsProfile settingsProfile,
                                          ROS2Topic<ImageMessage> depthTopic,
                                          ROS2Topic<ImageMessage> colorTopic,
                                          Supplier<ReferenceFrame> sensorFrameUpdater)
   {
      this.serialNumber = serialNumber;
      this.settingsProfile = settingsProfile;
      this.colorTopic = colorTopic;
      this.depthTopic = depthTopic;
      this.sensorFrameUpdater = sensorFrameUpdater;

      nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();

      ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "realsense_color_and_depth_publisher");
      ros2Helper = new ROS2Helper(ros2Node);
   }

   public void run()
   {
      while (running)
      {
         update();
         throttler.waitAndRun(outputPeriod); // do the waiting after we send to remove unecessary latency
      }
   }

   private void update()
   {
      if (nativesLoadedActivator.poll())
      {
         if (nativesLoadedActivator.isNewlyActivated())
         {
            realSenseHardwareManager = new RealSenseHardwareManager();
            realsense = realSenseHardwareManager.createBytedecoRealsenseDevice(serialNumber, settingsProfile);

            if (realsense.getDevice() == null)
            {
               running = false;
               throw new RuntimeException("Device not found. Set -Dd435.serial.number=00000000000");
            }
            realsense.enableColor(settingsProfile);
            realsense.initialize();

            pngCompression.allocate(realsense.getDepthWidth() * realsense.getDepthHeight() * ImageMessageFormat.DEPTH_PNG_16UC1.getBytesPerPixel());
            jpegCompression.allocate(realsense.getColorWidth() * realsense.getColorHeight() * ImageMessageFormat.COLOR_JPEG_YUVI420.getBytesPerPixel());
         }

         if (realsense.readFrameData())
         {
            realsense.updateDataBytePointers();

            Instant aquisitionTime = Instant.now();

            MutableBytePointer depthFrameData = realsense.getDepthFrameData();
            depth16UC1Image = new Mat(realsense.getDepthHeight(), realsense.getDepthWidth(), opencv_core.CV_16UC1, depthFrameData);
            PerceptionMessageTools.setDepthExtrinsicsFromRealsense(realsense, depthImageMessage);

            MutableBytePointer colorFrameData = realsense.getColorFrameData();
            color8UC3Image = new Mat(realsense.getColorHeight(), realsense.getColorWidth(), opencv_core.CV_8UC3, colorFrameData);
            PerceptionMessageTools.setColorExtrinsicsFromRealsense(realsense, colorImageMessage);

            // Important not to store as a field, as update() needs to be called each frame
            ReferenceFrame cameraFrame = sensorFrameUpdater.get();
            cameraPose.setToZero(cameraFrame);
            cameraPose.changeFrame(ReferenceFrame.getWorldFrame());

            pngCompression.compress(depth16UC1Image);
            jpegCompression.compressRGB(color8UC3Image);

            PerceptionMessageTools.packImageMessageData(pngCompression.getCompressedData(), depthImageMessage);
            ImageMessageFormat.DEPTH_PNG_16UC1.packMessageFormat(depthImageMessage);
            depthImageMessage.setImageHeight(realsense.getDepthHeight());
            depthImageMessage.setImageWidth(realsense.getDepthWidth());
            depthImageMessage.getPosition().set(cameraPose.getPosition());
            depthImageMessage.getOrientation().set(cameraPose.getOrientation());
            depthImageMessage.setSequenceNumber(depthSequenceNumber++);
            MessageTools.toMessage(aquisitionTime, depthImageMessage.getAcquisitionTime());
            ros2Helper.publish(depthTopic, depthImageMessage);

            PerceptionMessageTools.packImageMessageData(jpegCompression.getCompressedData(), colorImageMessage);
            ImageMessageFormat.COLOR_JPEG_YUVI420.packMessageFormat(colorImageMessage);
            colorImageMessage.setImageHeight(realsense.getColorHeight());
            colorImageMessage.setImageWidth(realsense.getColorWidth());
            colorImageMessage.getPosition().set(cameraPose.getPosition());
            colorImageMessage.getOrientation().set(cameraPose.getOrientation());
            colorImageMessage.setSequenceNumber(colorSequenceNumber++);
            MessageTools.toMessage(aquisitionTime, colorImageMessage.getAcquisitionTime());
            ros2Helper.publish(colorTopic, colorImageMessage);
         }
      }
   }

   public void destroy()
   {
      running = false;
      realsense.deleteDevice();
      realSenseHardwareManager.deleteContext();
   }

   public static void main(String[] args)
   {
      /*
         Color: [fx:901.3026, fy:901.8400, cx:635.2337, cy:350.9427, h:720, w:1280]
         Depth: [fx:730.7891, fy:731.0859, cx:528.6094, cy:408.1602, h:768, w:1024]
      */

      // Tripod: F1121365, F0245563
      String l515SerialNumber = System.getProperty("l515.serial.number", "F1121365");
      RealsenseColorAndDepthPublisher realsensePublisher = new RealsenseColorAndDepthPublisher(l515SerialNumber,
                                                                                               RealsenseSettingsProfile.L515_COLOR_720P_DEPTH_768P_30HZ,
                                                                                               ROS2Tools.L515_DEPTH_IMAGE,
                                                                                               ROS2Tools.L515_COLOR_IMAGE,
                                                                                               ReferenceFrame::getWorldFrame);
      Runtime.getRuntime().addShutdownHook(new Thread(realsensePublisher::destroy, "Shutdown"));
      realsensePublisher.run();
   }
}