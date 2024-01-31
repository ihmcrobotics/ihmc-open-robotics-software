package us.ihmc.sensors;

import boofcv.struct.calib.CameraPinholeBrown;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.CameraModel;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.perception.zedDriver.ZEDOpenDriver;
import us.ihmc.perception.zedDriver.ZedDriverNativeLibrary;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Throttler;

import java.time.Instant;
import java.util.function.Supplier;

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
   static
   {
      ZedDriverNativeLibrary.load();
   }

   private ROS2Helper ros2Helper;
   private FramePose3D cameraPose = new FramePose3D();

   private final Supplier<ReferenceFrame> sensorFrameUpdater;
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

   private byte[] imageYUVBytes;
   private int[] dims;

   private ZEDOpenDriver.ZEDOpenDriverExternal zed;

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

      zed = new ZEDOpenDriver.ZEDOpenDriverExternal(imageHeight, fps);
      dims = new int[] {0, 0, 0};
      zed.getFrameDimensions(dims);

      imageYUVBytes = new byte[dims[0] * dims[1] * dims[2]];

      Runtime.getRuntime().addShutdownHook(new Thread(this::destroy, "Shutdown"));
   }

   /**
    * Must be called from the sensor-specific calling class, after the sensor and logger initialization have succeeded.
    * We run in a daemon thread, because otherwise it will get killed on Ctrl+C before the shutdown hooks are finished running.
    * See {@link Runtime#addShutdownHook(Thread)} for details.
    */
   public void run()
   {
      ThreadTools.startAsDaemon(this::updateThread, getClass().getSimpleName() + "UpdateThread");
   }

   private void updateThread()
   {
      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "zed2_combined_publisher_node");
      ros2Helper = new ROS2Helper(ros2Node);

      leftColorIntrinsics = new CameraPinholeBrown();
      rightColorIntrinsics = new CameraPinholeBrown();

      while (running)
      {
         update();
         throttler.waitAndRun(outputPeriod); // do the waiting after we send to remove unnecessary latency
      }

      // Make sure the Realsense
      ThreadTools.sleep(100);
   }

   public void update()
   {
      leftColorIntrinsics = new CameraPinholeBrown();
      rightColorIntrinsics = new CameraPinholeBrown();

      Instant now = Instant.now();

      // Important not to store as a field, as update() needs to be called each frame
      ReferenceFrame cameraFrame = sensorFrameUpdater.get();
      cameraPose.setToZero(cameraFrame);
      cameraPose.changeFrame(ReferenceFrame.getWorldFrame());

      boolean valid = readImage(color8UC3CombinedImage);

      if (valid)
      {
         OpenCVTools.compressRGBImageJPG(color8UC3CombinedImage, yuvCombinedImage, compressedColorPointer);
         CameraModel.PINHOLE.packMessageFormat(colorImageMessage);
         PerceptionMessageTools.publishJPGCompressedColorImage(compressedColorPointer,
                                                               colorTopic,
                                                               colorImageMessage,
                                                               ros2Helper,
                                                               cameraPose,
                                                               now,
                                                               colorSequenceNumber++,
                                                               imageHeight,
                                                               imageWidth,
                                                               0.0f);
      }
   }

   public boolean readImage(Mat mat)
   {
      boolean status = zed.getFrameStereoYUVExternal(imageYUVBytes, dims);

      if (status)
      {
         BytePointer yuvBytePointer = new BytePointer(imageYUVBytes);
         Mat yuvImage = new Mat(dims[0], dims[1], opencv_core.CV_8UC2, yuvBytePointer);
         opencv_imgproc.cvtColor(yuvImage, mat, opencv_imgproc.COLOR_YUV2BGR_YUYV);

         //PerceptionDebugTools.display("Image", mat, 1);
      }

      return status;
   }

   /**
    * Must be called in the shutdown hook from the sensor-specific calling class. Handles Ctrl + C based closing gracefully.
    */
   public void destroy()
   {
      running = false;
   }

   public static void main(String[] args)
   {
      String cameraId = "";
      ZED2ColorStereoPublisher zed = new ZED2ColorStereoPublisher(cameraId, 1080, 3840, 30, PerceptionAPI.ZED2_STEREO_COLOR, ReferenceFrame::getWorldFrame);
      zed.run();
      ThreadTools.sleepForever();
   }
}