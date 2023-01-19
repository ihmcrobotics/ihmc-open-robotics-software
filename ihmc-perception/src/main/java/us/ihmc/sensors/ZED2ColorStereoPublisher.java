package us.ihmc.sensors;

import boofcv.struct.calib.CameraPinholeBrown;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.opencv_core.Mat;
import org.bytedeco.opencv.opencv_videoio.VideoCapture;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.packets.MessageTools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoOpenCVTools;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.comms.ImageMessageFormat;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.thread.Throttler;

import java.time.Instant;
import java.util.function.Supplier;

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
   private final Supplier<ReferenceFrame> sensorFrameUpdater;
   private final FramePose3D cameraPose = new FramePose3D();

   private final Mat yuvCombinedImage = new Mat();
   private final Mat color8UC3CombinedImage = new Mat();

   private final ImageMessage colorImageMessage = new ImageMessage();

   private final BytePointer compressedColorPointer = new BytePointer();
   private final String cameraId;
   private final int fps;
   private final ROS2Topic<ImageMessage> colorTopic;

   private CameraPinholeBrown leftColorIntrinsics;
   private CameraPinholeBrown rightColorIntrinsics;

   private int colorSequenceNumber = 0;
   private int imageHeight = 720;
   private int imageWidth = 2560;

   private final double outputPeriod = UnitConversions.hertzToSeconds(30.0);
   private final Throttler throttler = new Throttler();

   private VideoCapture capStereo;

   private volatile boolean running = true;

   public ZED2ColorStereoPublisher(String cameraId,
                                   int imageHeight,
                                   int imageWidth,
                                   int fps,
                                   ROS2Topic<ImageMessage> colorTopic,
                                   Supplier<ReferenceFrame> sensorFrameUpdater)
   {
      this.cameraId = cameraId;
      this.imageWidth = imageWidth;
      this.imageHeight = imageHeight;
      this.fps = fps;
      this.colorTopic = colorTopic;
      this.sensorFrameUpdater = sensorFrameUpdater;

      capStereo = new VideoCapture(cameraId);
      setVideoCaptureProperties(capStereo);

      nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();

      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "zed2_combined_publisher_node");
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

         Instant now = Instant.now();

         // Important not to store as a field, as update() needs to be called each frame
         ReferenceFrame cameraFrame = sensorFrameUpdater.get();
         cameraPose.setToZero(cameraFrame);
         cameraPose.changeFrame(ReferenceFrame.getWorldFrame());

         readImage(color8UC3CombinedImage);

         PerceptionMessageTools.publishJPGCompressedColorImage(color8UC3CombinedImage, yuvCombinedImage, colorTopic, colorImageMessage, ros2Helper,
                                                               cameraPose, now, colorSequenceNumber++, imageHeight, imageWidth);
      }

   }

   public void setVideoCaptureProperties(VideoCapture capture)
   {
      /* Supported Stereo Resolutions: {'1344.0x376.0': 'OK', '2560.0x720.0': 'OK', '3840.0x1080.0': 'OK', '4416.0x1242.0': 'OK'} */

      LogTools.warn("Setting Camera Parameters for ZED2.");
      capture.set(CAP_PROP_FRAME_WIDTH, 2560);
      capture.set(CAP_PROP_FRAME_HEIGHT, 720);
      LogTools.warn("Frame Rate Available: {}", capture.get(CAP_PROP_FPS));

      // TODO: Find more optimal parameters for the following.
      //capture.set(CAP_PROP_BUFFERSIZE, 3);
      //capture.set(CAP_PROP_FPS, 30);
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

   public static void main(String[] args)
   {
      String cameraId = System.getProperty("zed2.camera.id", "/dev/v4l/by-id/usb-Technologies__Inc._ZED_2-video-index0");
      ZED2ColorStereoPublisher zed = new ZED2ColorStereoPublisher(cameraId, 720, 2560, 30, ROS2Tools.ZED2_STEREO_COLOR, ReferenceFrame::getWorldFrame);
   }
}
