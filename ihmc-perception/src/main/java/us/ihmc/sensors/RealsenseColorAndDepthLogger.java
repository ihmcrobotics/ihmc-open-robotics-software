package us.ihmc.sensors;


import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoOpenCVTools;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.MutableBytePointer;
import us.ihmc.perception.logging.PerceptionDataLogger;
import us.ihmc.perception.logging.PerceptionLoggerConstants;
import us.ihmc.perception.realsense.BytedecoRealsense;
import us.ihmc.perception.realsense.RealSenseHardwareManager;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.tools.IHMCCommonPaths;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.thread.Throttler;

import java.nio.file.Paths;
import java.text.SimpleDateFormat;
import java.time.Instant;
import java.util.Date;

/**
 * Logs color and depth from Realsense sensors into HDf5 log file locally
 */
public class RealsenseColorAndDepthLogger
{
   private final Activator nativesLoadedActivator;
   private final FramePose3D cameraPose = new FramePose3D();
   private final String colorChannelName;
   private final String depthChannelName;
   private final ImageMessage colorImageMessage = new ImageMessage();
   private final ImageMessage depthImageMessage = new ImageMessage();
   private final Throttler throttler = new Throttler();
   private final Mat yuvColorImage = new Mat();

   private final PerceptionDataLogger perceptionDataLogger = new PerceptionDataLogger();

   private RealSenseHardwareManager realSenseHardwareManager;
   private BytedecoRealsense sensor;
   private Mat depth16UC1Image;
   private Mat color8UC3Image;

   private final BytePointer compressedColorBytePointer = new BytePointer();
   private final BytePointer compressedDepthBytePointer = new BytePointer();

   private volatile boolean running = true;
   private final double outputPeriod = UnitConversions.hertzToSeconds(30.0);

   private final String serialNumber;
   private int depthHeight;
   private int depthWidth;
   private final int colorHeight;
   private final int colorWidth;
   private final int colorFPS;
   private final int depthFPS;
   private long depthSequenceNumber = 0;
   private long colorSequenceNumber = 0;

   public RealsenseColorAndDepthLogger(String serialNumber,
                                          int depthWidth,
                                          int depthHeight,
                                          int depthFPS,
                                          int colorWidth,
                                          int colorHeight,
                                          int colorFPS,
                                          String depthChannelName,
                                          String colorChannelName)
   {
      this.serialNumber = serialNumber;
      this.depthWidth = depthWidth;
      this.depthHeight = depthHeight;
      this.colorWidth = colorWidth;
      this.colorHeight = colorHeight;
      this.depthFPS = depthFPS;
      this.colorFPS = colorFPS;
      this.colorChannelName = colorChannelName;
      this.depthChannelName = depthChannelName;

      nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();

      SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
      String logFileName = dateFormat.format(new Date()) + "_" + "PerceptionLog.hdf5";
      FileTools.ensureDirectoryExists(Paths.get(IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY_NAME), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);

      perceptionDataLogger.openLogFile(IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.resolve(logFileName).toString());
      perceptionDataLogger.addChannel(depthChannelName);
      perceptionDataLogger.setChannelEnabled(depthChannelName, true);

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
               throw new RuntimeException("Device not found. Set -Dd435.serial.number=00000000000");
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
            ReferenceFrame cameraFrame = ReferenceFrame.getWorldFrame();
            cameraPose.setToZero(cameraFrame);
            cameraPose.changeFrame(ReferenceFrame.getWorldFrame());

            BytedecoOpenCVTools.compressImagePNG(depth16UC1Image, compressedDepthBytePointer);
            perceptionDataLogger.storeBytesFromPointer(depthChannelName, compressedDepthBytePointer);

            BytedecoOpenCVTools.displayDepth("Depth", depth16UC1Image, 1);

            //BytedecoOpenCVTools.compressRGBImageJPG(color8UC3Image, yuvColorImage, compressedColorBytePointer);
            //perceptionDataLogger.storeBytesFromPointer(colorChannelName, compressedDepthBytePointer);
         }
      }
   }

   private void destroy()
   {
      running = false;
      sensor.deleteDevice();
      realSenseHardwareManager.deleteContext();
      perceptionDataLogger.closeLogFile();
   }

   public static void main(String[] args)
   {
      /*
         Color: [fx:901.3026, fy:901.8400, cx:635.2337, cy:350.9427, h:720, w:1280]
         Depth: [fx:730.7891, fy:731.0859, cx:528.6094, cy:408.1602, h:768, w:1024]
      */

      // L515: [F1121365, F0245563], D455: [215122254074]
      String l515SerialNumber = System.getProperty("l515.serial.number", "F1121365");
      new RealsenseColorAndDepthLogger(l515SerialNumber,
                                          1024,
                                          768,
                                          30,
                                          1280,
                                          720,
                                          30,
                                          PerceptionLoggerConstants.L515_DEPTH_NAME,
                                          PerceptionLoggerConstants.L515_DEPTH_NAME);
   }
}
