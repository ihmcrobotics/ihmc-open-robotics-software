package us.ihmc.sensors;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;
import perception_msgs.msg.dds.IntrinsicParametersMessage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
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
   private Mat depthU16C1Image;
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

         }

         if (sensor.readFrameData())
         {
            sensor.updateDataBytePointers();

            Instant now = Instant.now();

            MutableBytePointer depthFrameData = sensor.getDepthFrameData();
            depthU16C1Image = new Mat(depthHeight, depthWidth, opencv_core.CV_16UC1, depthFrameData);
            setDepthExtrinsics(sensor, depthImageMessage.getIntrinsicParameters());

            MutableBytePointer colorFrameData = sensor.getColorFrameData();
            color8UC3Image = new Mat(this.colorHeight, this.colorWidth, opencv_core.CV_8UC3, colorFrameData);
            setColorExtrinsics(sensor, colorImageMessage.getIntrinsicParameters());

            // Important not to store as a field, as update() needs to be called each frame
            ReferenceFrame cameraFrame = sensorFrameUpdater.get();
            cameraPose.setToZero(cameraFrame);
            cameraPose.changeFrame(ReferenceFrame.getWorldFrame());
            depthImageMessage.getPosition().set(cameraPose.getPosition());
            depthImageMessage.getOrientation().set(cameraPose.getOrientation());
            colorImageMessage.getPosition().set(cameraPose.getPosition());
            colorImageMessage.getOrientation().set(cameraPose.getOrientation());
            colorImageMessage.setSequenceNumber(colorSequenceNumber++);
            depthImageMessage.setSequenceNumber(depthSequenceNumber++);

            MessageTools.toMessage(now, depthImageMessage.getAcquisitionTime());
            MessageTools.toMessage(now, colorImageMessage.getAcquisitionTime());

            compressedDepthPointer = new BytePointer();
            BytedecoOpenCVTools.compressImagePNG(depthU16C1Image, compressedDepthPointer);
            BytedecoOpenCVTools.fillImageMessage(compressedDepthPointer, depthImageMessage, sensor.getDepthHeight(), sensor.getDepthWidth());
            ros2Helper.publish(this.depthTopic, depthImageMessage);

            compressedColorPointer = new BytePointer();
            BytedecoOpenCVTools.compressRGBImageJPG(color8UC3Image, yuvColorImage, compressedColorPointer);
            BytedecoOpenCVTools.fillImageMessage(compressedColorPointer, colorImageMessage, sensor.getColorHeight(), sensor.getColorWidth());
            ros2Helper.publish(this.colorTopic, colorImageMessage);

            /* Debug Only: Show depth and color for quick sensor testing on systems with display */
            //display(depthU16C1Image, color8UC3Image);

            //LogTools.info(String.format("Depth: [fx:%.4f, fy:%.4f, cx:%.4f, cy:%.4f, h:%d, w:%d]",
            //                            sensor.getDepthFocalLengthPixelsX(),
            //                            sensor.getDepthFocalLengthPixelsY(),
            //                            sensor.getDepthPrincipalOffsetXPixels(),
            //                            sensor.getDepthPrincipalOffsetYPixels(),
            //                            depthHeight,
            //                            depthWidth));
            //
            //LogTools.info(String.format("Color: [fx:%.4f, fy:%.4f, cx:%.4f, cy:%.4f, h:%d, w:%d]",
            //                            sensor.getColorFocalLengthPixelsX(),
            //                            sensor.getColorFocalLengthPixelsY(),
            //                            sensor.getColorPrincipalOffsetXPixels(),
            //                            sensor.getColorPrincipalOffsetYPixels(),
            //                            colorHeight,
            //                            colorWidth));
         }
      }
   }

   private void setDepthExtrinsics(BytedecoRealsense sensor, IntrinsicParametersMessage intrinsicParametersMessage)
   {
      intrinsicParametersMessage.setFx(sensor.getDepthFocalLengthPixelsX());
      intrinsicParametersMessage.setFy(sensor.getDepthFocalLengthPixelsY());
      intrinsicParametersMessage.setSkew(0.0);
      intrinsicParametersMessage.setCx(sensor.getDepthPrincipalOffsetXPixels());
      intrinsicParametersMessage.setCy(sensor.getDepthPrincipalOffsetYPixels());
   }

   private void setColorExtrinsics(BytedecoRealsense sensor, IntrinsicParametersMessage intrinsicParametersMessage)
   {
      intrinsicParametersMessage.setFx(sensor.getColorFocalLengthPixelsX());
      intrinsicParametersMessage.setFy(sensor.getColorFocalLengthPixelsY());
      intrinsicParametersMessage.setSkew(0.0);
      intrinsicParametersMessage.setCx(sensor.getColorPrincipalOffsetXPixels());
      intrinsicParametersMessage.setCy(sensor.getColorPrincipalOffsetYPixels());
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

   public static void main(String[] args)
   {
      /*
         Color: [fx:901.3026, fy:901.8400, cx:635.2337, cy:350.9427, h:720, w:1280]
         Depth: [fx:730.7891, fy:731.0859, cx:528.6094, cy:408.1602, h:768, w:1024]
      */

      //String l515SerialNumber = System.getProperty("l515.serial.number", "F1121365");
      String l515SerialNumber = System.getProperty("l515.serial.number", "F0245563");
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