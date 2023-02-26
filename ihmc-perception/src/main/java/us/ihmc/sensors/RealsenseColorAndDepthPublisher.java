package us.ihmc.sensors;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoOpenCVTools;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.MutableBytePointer;
import us.ihmc.perception.realsense.BytedecoRealsense;
import us.ihmc.perception.realsense.RealSenseHardwareManager;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.thread.Throttler;

import java.time.Instant;
import java.util.function.Supplier;

import static org.bytedeco.opencv.global.opencv_highgui.imshow;
import static org.bytedeco.opencv.global.opencv_highgui.waitKey;

/**
 * Publishes color and depth from Realsense D435
 * ----+ L515 Device Configuration: Serial Number: F0245563, Depth Height 768, Depth Width: 1024, Depth FPS: 15, Color Height 720, Color Width: 1280, Color FPS:
 * 15
 * ----+ D435: Serial Number: 752112070330, Depth Width: 848, Depth Height: 480, Depth FPS: 30, Color Width: 848, Color Height: 480, Color FPS: 30
 */
public class RealsenseColorAndDepthPublisher
{
   private final Activator nativesLoadedActivator;
   private final ROS2Helper ros2Helper;
   private final Supplier<ReferenceFrame> sensorFrameUpdater;
   private final FramePose3D cameraPose = new FramePose3D();
   private final ROS2Topic<ImageMessage> colorTopic;
   private final ROS2Topic<ImageMessage> depthTopic;
   private final ImageMessage colorImageMessage = new ImageMessage();
   private final ImageMessage depthImageMessage = new ImageMessage();
   private final Mat yuvColorImage = new Mat();
   private final Throttler throttler = new Throttler();

   private RealSenseHardwareManager realSenseHardwareManager;
   private BytedecoRealsense sensor;
   private Mat depth16UC1Image;
   private Mat color8UC3Image;

   private volatile boolean running = true;
   private final double outputPeriod = UnitConversions.hertzToSeconds(30.0);

   private BytePointer compressedColorPointer;
   private BytePointer compressedDepthPointer;

   private final String serialNumber;
   private int depthHeight;
   private int depthWidth;
   private final int colorHeight;
   private final int colorWidth;
   private final int colorFPS;
   private final int depthFPS;
   private long depthSequenceNumber = 0;
   private long colorSequenceNumber = 0;

   public RealsenseColorAndDepthPublisher(String serialNumber,
                                          int depthWidth,
                                          int depthHeight,
                                          int depthFPS,
                                          int colorWidth,
                                          int colorHeight,
                                          int colorFPS,
                                          ROS2Topic<ImageMessage> depthTopic,
                                          ROS2Topic<ImageMessage> colorTopic,
                                          Supplier<ReferenceFrame> sensorFrameUpdater)
   {
      this.serialNumber = serialNumber;
      this.depthWidth = depthWidth;
      this.depthHeight = depthHeight;
      this.colorWidth = colorWidth;
      this.colorHeight = colorHeight;
      this.depthFPS = depthFPS;
      this.colorFPS = colorFPS;
      this.colorTopic = colorTopic;
      this.depthTopic = depthTopic;
      this.sensorFrameUpdater = sensorFrameUpdater;

      nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();

      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "realsense_color_depth_node");
      ros2Helper = new ROS2Helper(ros2Node);

      Runtime.getRuntime().addShutdownHook(new Thread(() ->
      {
         ThreadTools.sleepSeconds(0.5);
         destroy();
      }, getClass().getSimpleName() + "Shutdown"));

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
            sensor = realSenseHardwareManager.createBytedecoRealsenseDevice(this.serialNumber, this.depthWidth, this.depthHeight, this.depthFPS);

            if (sensor.getDevice() == null)
            {
               running = false;
               throw new RuntimeException("Device not found. Set -D<model>.serial.number=00000000000");
            }
            sensor.enableColor(this.colorWidth, this.colorHeight, this.colorFPS);
            sensor.initialize();

            depthWidth = sensor.getDepthWidth();
            depthHeight = sensor.getDepthHeight();

            LogTools.info(String.format("Color: [fx:%.4f, fy:%.4f, cx:%.4f, cy:%.4f, h:%d, w:%d]",
                                        sensor.getColorFocalLengthPixelsX(),
                                        sensor.getColorFocalLengthPixelsY(),
                                        sensor.getColorPrincipalOffsetXPixels(),
                                        sensor.getColorPrincipalOffsetYPixels(),
                                        colorHeight,
                                        colorWidth));

            LogTools.info(String.format("Depth: [fx:%.4f, fy:%.4f, cx:%.4f, cy:%.4f, h:%d, w:%d]",
                                        sensor.getDepthFocalLengthPixelsX(),
                                        sensor.getDepthFocalLengthPixelsY(),
                                        sensor.getDepthPrincipalOffsetXPixels(),
                                        sensor.getDepthPrincipalOffsetYPixels(),
                                        depthHeight,
                                        depthWidth));
         }

         if (sensor.readFrameData())
         {
            sensor.updateDataBytePointers();

            Instant now = Instant.now();

            MutableBytePointer depthFrameData = sensor.getDepthFrameData();
            depth16UC1Image = new Mat(depthHeight, depthWidth, opencv_core.CV_16UC1, depthFrameData);
            PerceptionMessageTools.setDepthExtrinsicsFromRealsense(sensor, depthImageMessage.getIntrinsicParameters());

            MutableBytePointer colorFrameData = sensor.getColorFrameData();
            color8UC3Image = new Mat(this.colorHeight, this.colorWidth, opencv_core.CV_8UC3, colorFrameData);
            PerceptionMessageTools.setColorExtrinsicsFromRealsense(sensor, colorImageMessage.getIntrinsicParameters());

            // Important not to store as a field, as update() needs to be called each frame
            ReferenceFrame cameraFrame = sensorFrameUpdater.get();
            cameraPose.setToZero(cameraFrame);
            cameraPose.changeFrame(ReferenceFrame.getWorldFrame());

            PerceptionMessageTools.publishPNGCompressedDepthImage(depth16UC1Image, depthTopic, depthImageMessage, ros2Helper, cameraPose, now, depthSequenceNumber++,
                                                           sensor.getDepthHeight(), sensor.getDepthWidth());

            PerceptionMessageTools.publishJPGCompressedColorImage(color8UC3Image, yuvColorImage, colorTopic, colorImageMessage, ros2Helper, cameraPose, now,
                                                           colorSequenceNumber++, sensor.getColorHeight(), sensor.getColorWidth());

         }
      }
   }

   private void destroy()
   {
      running = false;
      sensor.deleteDevice();
      realSenseHardwareManager.deleteContext();
   }

   public static void main(String[] args)
   {
      /*
         Color: [fx:901.3026, fy:901.8400, cx:635.2337, cy:350.9427, h:720, w:1280]
         Depth: [fx:730.7891, fy:731.0859, cx:528.6094, cy:408.1602, h:768, w:1024]
      */

      // L515: [F1121365, F0245563], D455: [215122254074]
      String l515SerialNumber = System.getProperty("l515.serial.number", "F1121365");
      new RealsenseColorAndDepthPublisher(l515SerialNumber,
                                          1024,
                                          768,
                                          30,
                                          1280,
                                          720,
                                          30,
                                          ROS2Tools.L515_DEPTH_IMAGE,
                                          ROS2Tools.L515_COLOR_IMAGE,
                                          ReferenceFrame::getWorldFrame);
   }
}