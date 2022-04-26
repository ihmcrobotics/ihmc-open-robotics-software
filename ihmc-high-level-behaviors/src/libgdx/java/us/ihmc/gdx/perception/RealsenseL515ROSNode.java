package us.ihmc.gdx.perception;

import boofcv.struct.calib.CameraPinholeBrown;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import org.jboss.netty.buffer.ChannelBuffer;
import org.ros.message.Time;
import sensor_msgs.Image;
import us.ihmc.commons.Conversions;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoImage;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.MutableBytePointer;
import us.ihmc.perception.realsense.BytedecoRealsenseL515;
import us.ihmc.perception.realsense.RealSenseHardwareManager;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.thread.PausablePeriodicThread;
import us.ihmc.utilities.ros.ROS1Helper;
import us.ihmc.utilities.ros.RosTools;
import us.ihmc.utilities.ros.publisher.RosCameraInfoPublisher;
import us.ihmc.utilities.ros.publisher.RosImagePublisher;

import java.nio.ByteBuffer;

public class RealsenseL515ROSNode
{
   private static final String SERIAL_NUMBER = System.getProperty("l515.serial.number", "F0000000");

   private final PausablePeriodicThread thread;
   private final Activator nativesLoadedActivator;
   private final ROS1Helper ros1Helper;
   private RealSenseHardwareManager realSenseHardwareManager;
   private BytedecoRealsenseL515 l515;
   private RosImagePublisher ros1DepthPublisher;
   private RosCameraInfoPublisher ros1DepthCameraInfoPublisher;
   private ChannelBuffer ros1DepthChannelBuffer;
   private Mat depthU16C1Image;
   private BytedecoImage depth32FC1Image;
   private int depthWidth;
   private int depthHeight;
   private CameraPinholeBrown depthCameraIntrinsics;

   public RealsenseL515ROSNode()
   {
      nativesLoadedActivator = BytedecoTools.loadNativesOnAThread();

      ros1Helper = new ROS1Helper("l515_node");

      thread = new PausablePeriodicThread("L515Node", UnitConversions.hertzToSeconds(31.0), false, this::update);
      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, "L515Shutdown"));
      thread.start();
   }

   private void update()
   {
      if (nativesLoadedActivator.poll())
      {
         if (nativesLoadedActivator.isNewlyActivated())
         {
            realSenseHardwareManager = new RealSenseHardwareManager();
            l515 = realSenseHardwareManager.createFullFeaturedL515(SERIAL_NUMBER);
            l515.initialize();

            depthWidth = l515.getDepthWidth();
            depthHeight = l515.getDepthHeight();

            String ros1DepthImageTopic = RosTools.L515_DEPTH;
            String ros1DepthCameraInfoTopic = RosTools.L515_DEPTH_CAMERA_INFO;
            LogTools.info("Publishing ROS 1 depth: {} {}", ros1DepthImageTopic, ros1DepthCameraInfoTopic);
            ros1DepthPublisher = new RosImagePublisher();
            ros1DepthCameraInfoPublisher = new RosCameraInfoPublisher();
            ros1Helper.attachPublisher(ros1DepthCameraInfoTopic, ros1DepthCameraInfoPublisher);
            ros1Helper.attachPublisher(ros1DepthImageTopic, ros1DepthPublisher);
            ros1DepthChannelBuffer = ros1DepthPublisher.getChannelBufferFactory().getBuffer(2 * depthWidth * depthHeight);

            depthCameraIntrinsics = new CameraPinholeBrown();
         }

         if (l515.readFrameData())
         {
            l515.updateDataBytePointers();

            if (depthU16C1Image == null)
            {
               MutableBytePointer depthFrameData = l515.getDepthFrameData();
               depthU16C1Image = new Mat(l515.getDepthHeight(), l515.getDepthWidth(), opencv_core.CV_16UC1, depthFrameData);
               depth32FC1Image = new BytedecoImage(l515.getDepthWidth(), l515.getDepthHeight(), opencv_core.CV_32FC1);

               depthCameraIntrinsics.setFx(l515.getFocalLengthPixelsX());
               depthCameraIntrinsics.setFy(l515.getFocalLengthPixelsY());
               depthCameraIntrinsics.setSkew(0.0);
               depthCameraIntrinsics.setCx(l515.getPrincipalOffsetXPixels());
               depthCameraIntrinsics.setCy(l515.getPrincipalOffsetYPixels());
            }

            if (ros1DepthPublisher.isConnected() && ros1DepthCameraInfoPublisher.isConnected())
            {
               depthU16C1Image.convertTo(depth32FC1Image.getBytedecoOpenCVMat(), opencv_core.CV_32FC1, l515.getDepthToMeterConversion(), 0.0);
               depth32FC1Image.rewind();
               ByteBuffer depthFloatBuffer = depth32FC1Image.getBackingDirectByteBuffer();

               ros1DepthChannelBuffer.clear();
               int size = 2 * depthWidth * depthHeight;
               for (int y = 0; y < depthHeight; y++)
               {
                  for (int x = 0; x < depthWidth; x++)
                  {
                     float eyeDepthMeters = depthFloatBuffer.getFloat();

                     int row = y + 1;
                     int backForY = row * depthWidth * 2;
                     int forwardForX = x * 2;
                     int index = size - backForY + forwardForX;
                     char depthChar16 = (char) Math.round(eyeDepthMeters * 1000.0f); // 1000 is 1 meter
                     ros1DepthChannelBuffer.setChar(index, depthChar16);
                  }
               }

               ros1DepthChannelBuffer.readerIndex(0);
               ros1DepthChannelBuffer.writerIndex(size);

               ros1DepthCameraInfoPublisher.publish("camera_depth_optical_frame", depthCameraIntrinsics, new Time());
               Image message = ros1DepthPublisher.createMessage(depthWidth, depthHeight, 2, "16UC1", ros1DepthChannelBuffer); // maybe need to copy here if there are errors

//               if(timestampSupplier != null)
//                  message.getHeader().setStamp(new Time(Conversions.nanosecondsToSeconds(timestampSupplier.getAsLong())));
               message.getHeader().setStamp(new Time(Conversions.nanosecondsToSeconds(System.nanoTime())));

               ros1DepthPublisher.publish(message);
            }
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
