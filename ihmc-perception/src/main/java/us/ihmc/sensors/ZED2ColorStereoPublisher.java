package us.ihmc.sensors;

import boofcv.struct.calib.CameraPinholeBrown;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_videoio.VideoCapture;
import perception_msgs.msg.dds.VideoPacket;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.perception.BytedecoOpenCVTools;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.thread.Throttler;

import static org.opencv.videoio.Videoio.*;

/*
 * Supported Stereo Resolutions: {'1344.0x376.0': 'OK', '2560.0x720.0': 'OK', '3840.0x1080.0': 'OK', '4416.0x1242.0': 'OK'}
 * Find all possible resolutions with this Python script:
 *
 *
 *   import pandas as pd
 *   import cv2
 *
 *   url = "https://en.wikipedia.org/wiki/List_of_common_resolutions"
 *   table = pd.read_html(url)[0]
 *   table.columns = table.columns.droplevel()
 *   cap = cv2.VideoCapture(2, cv2.CAP_V4L2)
 *   resolutions = {}
 *   for index, row in table[["W", "H"]].iterrows():
 *       cap.set(cv2.CAP_PROP_FRAME_WIDTH, row["W"])
 *       cap.set(cv2.CAP_PROP_FRAME_HEIGHT, row["H"])
 *       width = cap.get(cv2.CAP_PROP_FRAME_WIDTH)
 *       height = cap.get(cv2.CAP_PROP_FRAME_HEIGHT)
 *       resolutions[str(width)+"x"+str(height)] = "OK"
 *   print(resolutions)
 *
 * */

public class ZED2ColorStereoPublisher
{
   private final Activator nativesLoadedActivator;
   private final ROS2Helper ros2Helper;

   private final byte[] heapByteArrayData = new byte[1000000];

   private final Mat yuvImageLeft = new Mat();
   private final Mat yuvImageRight = new Mat();

   private final Mat color8UC3ImageLeft = new Mat();
   private final Mat color8UC3ImageRight = new Mat();

   private final VideoPacket colorVideoPacketLeft = new VideoPacket();
   private final VideoPacket colorVideoPacketRight = new VideoPacket();

   private final BytePointer compressedColorPointerLeft = new BytePointer();
   private final BytePointer compressedColorPointerRight = new BytePointer();

   private ROS2Topic<VideoPacket> colorTopicLeft;
   private ROS2Topic<VideoPacket> depthTopicRight;

   private CameraPinholeBrown leftColorIntrinsics;
   private CameraPinholeBrown rightColorIntrinsics;

   private final double outputPeriod = UnitConversions.hertzToSeconds(30.0);
   private final Throttler throttler = new Throttler();

   private VideoCapture capStereo;

   private volatile boolean running = true;

   public ZED2ColorStereoPublisher(String cameraId)
   {
      capStereo = new VideoCapture(cameraId);
      setVideoCaptureProperties(capStereo);

      nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();

      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "realsense_color_depth_node");
      ros2Helper = new ROS2Helper(ros2Node);

      while (running)
      {
         update();
         throttler.waitAndRun(outputPeriod);
      }
   }

   public void update()
   {
      if (nativesLoadedActivator.poll())
      {
         if (nativesLoadedActivator.isNewlyActivated())
         {
            leftColorIntrinsics = new CameraPinholeBrown();
            rightColorIntrinsics = new CameraPinholeBrown();
         }
      }

      readImage(color8UC3ImageLeft);
      BytedecoOpenCVTools.compressRGBImageJPG(color8UC3ImageLeft, yuvImageLeft, compressedColorPointerLeft);
      BytedecoOpenCVTools.fillVideoPacket(compressedColorPointerLeft, heapByteArrayData, colorVideoPacketLeft, 720, 2560);
      ros2Helper.publish(ROS2Tools.ZED2_STEREO_COLOR, colorVideoPacketLeft);
   }

   public void setVideoCaptureProperties(VideoCapture capture)
   {
      //capture.set(CAP_PROP_BUFFERSIZE, 3);
      capture.set(CAP_PROP_FRAME_WIDTH, 4416.0);
      capture.set(CAP_PROP_FRAME_HEIGHT, 1242.0);
      capture.set(CAP_PROP_FPS, 30);
      //capture.set(CAP_PROP_FOURCC, VideoWriter.fourcc((byte)'M', (byte)'J', (byte)'P', (byte)'G'));
      //capture.set(CAP_PROP_AUTO_EXPOSURE, 0.25);
      //capture.set(CAP_PROP_AUTO_WB, 0.25);
      //capture.set(CAP_PROP_AUTOFOCUS, 0.25);
      //capture.set(CAP_PROP_MODE, CAP_OPENCV_MJPEG);
   }

   public void readImage(Mat mat)
   {
      boolean status = capStereo.read(mat);
   }
}
