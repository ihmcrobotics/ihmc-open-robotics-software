package us.ihmc.sensors;

import boofcv.struct.calib.CameraPinholeBrown;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.VideoPacket;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoOpenCVTools;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.MutableBytePointer;
import us.ihmc.perception.realsense.BytedecoRealsense;
import us.ihmc.perception.realsense.RealSenseHardwareManager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.thread.Throttler;

import java.time.Instant;

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
   private RealSenseHardwareManager realSenseHardwareManager;
   private BytedecoRealsense sensor;

   private final byte[] heapByteArrayData = new byte[1000000];

   private ROS2Topic<VideoPacket> colorTopic;
   private ROS2Topic<VideoPacket> depthTopic;

   private Mat depthU16C1Image;

   private final Mat yuvColorImage = new Mat();

   private Mat color8UC3Image;
   private Mat depth8UC3Image;

   private volatile boolean running = true;
   private final double outputPeriod = UnitConversions.hertzToSeconds(30.0);

   private CameraPinholeBrown depthCameraIntrinsics;
   private CameraPinholeBrown colorCameraIntrinsics;

   private final VideoPacket colorVideoPacket = new VideoPacket();
   private final VideoPacket depthVideoPacket = new VideoPacket();

   private final Throttler throttler = new Throttler();

   private BytePointer compressedColorPointer;
   private BytePointer compressedDepthPointer;

   private String serialNumber;
   private int depthHeight;
   private int depthWidth;
   private int colorHeight;
   private int colorWidth;
   private int colorFPS;
   private int depthFPS;

   public RealsenseColorAndDepthPublisher(String serialNumber, int depthWidth, int depthHeight, int depthFPS, int colorWidth, int colorHeight, int colorFPS, ROS2Topic<VideoPacket> depthTopic, ROS2Topic<VideoPacket> colorTopic)
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

      nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();

      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "realsense_color_depth_node");
      ros2Helper = new ROS2Helper(ros2Node);

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
               throw new RuntimeException("Device not found. Set -Dd435.serial.number=00000000000");
            }
            sensor.enableColor(this.colorWidth, this.colorHeight, this.colorFPS);
            sensor.initialize();

            depthWidth = sensor.getDepthWidth();
            depthHeight = sensor.getDepthHeight();

            depthCameraIntrinsics = new CameraPinholeBrown();
            colorCameraIntrinsics = new CameraPinholeBrown();
         }

         if (sensor.readFrameData())
         {
            Instant now = Instant.now();

            sensor.updateDataBytePointers();

            long begin_acquire = System.nanoTime();

            MutableBytePointer depthFrameData = sensor.getDepthFrameData();
            depthU16C1Image = new Mat(depthHeight, depthWidth, opencv_core.CV_16UC1, depthFrameData);
            setDepthExtrinsics(sensor, depthVideoPacket);

            MutableBytePointer colorFrameData = sensor.getColorFrameData();
            color8UC3Image = new Mat(this.colorHeight, this.colorWidth, opencv_core.CV_8UC3, colorFrameData);
            setColorExtrinsics(sensor, colorVideoPacket);

            long end_acquire = System.nanoTime();

            long begin_depth = System.nanoTime();

            compressedDepthPointer = new BytePointer();
            BytedecoOpenCVTools.compressImagePNG(depthU16C1Image, compressedDepthPointer);
            BytedecoOpenCVTools.fillVideoPacket(compressedDepthPointer, heapByteArrayData, depthVideoPacket, sensor.getDepthHeight(), sensor.getDepthWidth());
            ros2Helper.publish(this.depthTopic, depthVideoPacket);

            long end_depth = System.nanoTime();

            compressedColorPointer = new BytePointer();
            BytedecoOpenCVTools.compressRGBImageJPG(color8UC3Image, yuvColorImage, compressedColorPointer);
            BytedecoOpenCVTools.fillVideoPacket(compressedColorPointer, heapByteArrayData, colorVideoPacket, sensor.getColorHeight(), sensor.getColorWidth());
            ros2Helper.publish(this.colorTopic, colorVideoPacket);

            long end_color = System.nanoTime();

            LogTools.debug("Acquisition Time: {} ms", (end_acquire - begin_acquire) / 1e6);

            LogTools.debug(String.format("Depth Raw: %d, Final:%d, Ratio: %.3f, Time: %.3f ms",
                          depthU16C1Image.rows() * depthU16C1Image.cols() * 2, compressedDepthPointer.capacity(),
                          (float) (depthU16C1Image.rows() * depthU16C1Image.cols() * 2) / (float) compressedDepthPointer.capacity(), (end_depth - begin_depth) / 1e6));

            LogTools.debug(String.format("Color Raw: %d, Final:%d, Ratio: %.3f, Time: %.3f ms",
                          color8UC3Image.rows() * color8UC3Image.cols() * 3,
                          compressedColorPointer.capacity(),
                          (float) (color8UC3Image.rows() * color8UC3Image.cols() * 3) / (float) compressedColorPointer.capacity(), (end_color - end_depth) / 1e6));

            /* Debug Only: Show depth and color for quick sensor testing on systems with display */
            //            display(depthU16C1Image, color8UC3Image);
         }
      }
   }

   private void setDepthExtrinsics(BytedecoRealsense sensor, VideoPacket depthPacket)
   {
      depthPacket.getIntrinsicParameters().setFx(sensor.getDepthFocalLengthPixelsX());
      depthPacket.getIntrinsicParameters().setFy(sensor.getDepthFocalLengthPixelsY());
      depthPacket.getIntrinsicParameters().setSkew(0.0);
      depthPacket.getIntrinsicParameters().setCx(sensor.getDepthPrincipalOffsetXPixels());
      depthPacket.getIntrinsicParameters().setCy(sensor.getDepthPrincipalOffsetYPixels());
   }

   private void setColorExtrinsics(BytedecoRealsense sensor, VideoPacket depthPacket)
   {
      depthPacket.getIntrinsicParameters().setFx(sensor.getColorFocalLengthPixelsX());
      depthPacket.getIntrinsicParameters().setFy(sensor.getColorFocalLengthPixelsY());
      depthPacket.getIntrinsicParameters().setSkew(0.0);
      depthPacket.getIntrinsicParameters().setCx(sensor.getColorPrincipalOffsetXPixels());
      depthPacket.getIntrinsicParameters().setCy(sensor.getColorPrincipalOffsetYPixels());
   }

   private void display(Mat depth, Mat color)
   {
      Mat depthDisplay = new Mat();
      BytedecoOpenCVTools.clampTo8BitUnsignedChar(depth, depthDisplay, 0.0, 255.0);
      BytedecoOpenCVTools.convert8BitGrayTo8BitRGBA(depthDisplay, depthDisplay);

      imshow("Depth", depthDisplay);
      imshow("Color", color);
      waitKey(1);
   }

   private void destroy()
   {
      running = false;
      sensor.deleteDevice();
      realSenseHardwareManager.deleteContext();
   }
}
