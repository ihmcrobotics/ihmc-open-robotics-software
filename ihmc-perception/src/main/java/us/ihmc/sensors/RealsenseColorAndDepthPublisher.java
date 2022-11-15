package us.ihmc.sensors;

import boofcv.struct.calib.CameraPinholeBrown;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.VideoPacket;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.perception.BytedecoOpenCVTools;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.MutableBytePointer;
import us.ihmc.perception.realsense.BytedecoRealsense;
import us.ihmc.perception.realsense.RealSenseHardwareManager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.thread.Throttler;

import java.time.Instant;

import static org.bytedeco.opencv.global.opencv_highgui.imshow;
import static org.bytedeco.opencv.global.opencv_highgui.waitKey;

/**
 * Publishes color and depth from Realsense D435
 *       ----+ L515 Device Configuration: Serial Number: F0245563, Depth Height 768, Depth Width: 1024, Depth FPS: 15, Color Height 720, Color Width: 1280, Color FPS: 15
 *       ----+ D435: Serial Number: 752112070330, Depth Width: 848, Depth Height: 480, Depth FPS: 30, Color Width: 848, Color Height: 480, Color FPS: 30
 */
public class RealsenseColorAndDepthPublisher
{
   private final Activator nativesLoadedActivator;
   private final ROS2Helper ros2Helper;
   private RealSenseHardwareManager realSenseHardwareManager;
   private BytedecoRealsense sensor;

   private Mat depthU16C1Image;

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

   private final IntPointer compressionParameters = new IntPointer(opencv_imgcodecs.IMWRITE_JPEG_QUALITY, 75);;

   private String serialNumber;
   private int depthHeight;
   private int depthWidth;
   private int colorHeight;
   private int colorWidth;
   private int colorFPS;
   private int depthFPS;

   public RealsenseColorAndDepthPublisher(String serialNumber, int depthWidth, int depthHeight, int depthFPS, int colorWidth, int colorHeight, int colorFPS)
   {
      this.serialNumber = serialNumber;
      this.depthWidth = depthWidth;
      this.depthHeight = depthHeight;
      this.colorWidth = colorWidth;
      this.colorHeight = colorHeight;
      this.depthFPS = depthFPS;
      this.colorFPS = colorFPS;

      nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();

      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "d435_color_depth_node");
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
            sensor = realSenseHardwareManager.createBytedecoRealsense(this.serialNumber, this.depthWidth, this.depthHeight, this.depthFPS);

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

            long begin = System.currentTimeMillis();
            sensor.updateDataBytePointers();

            MutableBytePointer depthFrameData = sensor.getDepthFrameData();
            depthU16C1Image = new Mat(depthHeight, depthWidth, opencv_core.CV_16UC1, depthFrameData);
            setDepthExtrinsics(sensor);

            MutableBytePointer colorFrameData = sensor.getColorFrameData();
            color8UC3Image = new Mat(depthHeight, depthWidth, opencv_core.CV_8UC3, colorFrameData);
            setColorExtrinsics(sensor);

            depth8UC3Image = new Mat(depthU16C1Image.rows(), depthU16C1Image.cols(), opencv_core.CV_8UC3);
            BytedecoOpenCVTools.transferDepth16UC1ToLower8UC3(depthU16C1Image, depth8UC3Image);


            compressedDepthPointer = new BytePointer();
            BytedecoOpenCVTools.compressRGBImageJPG(depth8UC3Image, compressedDepthPointer, compressionParameters);
            fillVideoPacket(compressedDepthPointer, depthVideoPacket, sensor.getDepthHeight(), sensor.getDepthWidth());
            ros2Helper.publish(ROS2Tools.D435_DEPTH, depthVideoPacket);

            compressedColorPointer = new BytePointer();
            BytedecoOpenCVTools.compressRGBImageJPG(color8UC3Image, compressedColorPointer, compressionParameters);
            fillVideoPacket(compressedColorPointer, colorVideoPacket, sensor.getColorHeight(), sensor.getColorWidth());
            ros2Helper.publish(ROS2Tools.D435_VIDEO, colorVideoPacket);

            long end = System.currentTimeMillis();

//            LogTools.info("Uncompressed Bytes: {}", depthU16C1Image.rows() * depthU16C1Image.cols() * 2 + color8UC3Image.rows() * color8UC3Image.cols() * 3);
//            LogTools.info("Compressed Bytes: {}", depthVideoPacket.getData().size() + colorVideoPacket.getData().size());
//            LogTools.info("Time Taken: {} ms", end - begin);

            /* Debug Only: Show depth and color for quick sensor testing on systems with display */
//            display(depthU16C1Image, color8UC3Image);
         }
      }
   }

   private void fillVideoPacket(BytePointer compressedBytes, VideoPacket packet, int height, int width)
   {
      byte[] heapByteArrayData = new byte[compressedBytes.asBuffer().remaining()];
      compressedBytes.asBuffer().get(heapByteArrayData);
      packet.getData().resetQuick();
      packet.getData().add(heapByteArrayData);
      packet.setImageHeight(height);
      packet.setImageWidth(width);
      packet.setVideoSource(VideoSource.MULTISENSE_LEFT_EYE.toByte());
   }



   private void setDepthExtrinsics(BytedecoRealsense d435)
   {
      depthCameraIntrinsics.setFx(d435.getDepthFocalLengthPixelsX());
      depthCameraIntrinsics.setFy(d435.getDepthFocalLengthPixelsY());
      depthCameraIntrinsics.setSkew(0.0);
      depthCameraIntrinsics.setCx(d435.getDepthPrincipalOffsetXPixels());
      depthCameraIntrinsics.setCy(d435.getDepthPrincipalOffsetYPixels());
   }

   private void setColorExtrinsics(BytedecoRealsense d435)
   {
      colorCameraIntrinsics.setFx(d435.getColorFocalLengthPixelsX());
      colorCameraIntrinsics.setFy(d435.getColorFocalLengthPixelsY());
      colorCameraIntrinsics.setSkew(0.0);
      colorCameraIntrinsics.setCx(d435.getColorPrincipalOffsetXPixels());
      colorCameraIntrinsics.setCy(d435.getColorPrincipalOffsetYPixels());
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
