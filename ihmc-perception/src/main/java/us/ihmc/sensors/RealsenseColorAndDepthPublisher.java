package us.ihmc.sensors;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.property.ROS2StoredPropertySetGroup;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoOpenCVTools;
import us.ihmc.perception.BytedecoTools;
import us.ihmc.perception.MutableBytePointer;
import us.ihmc.perception.comms.PerceptionComms;
import us.ihmc.perception.logging.PerceptionDataLogger;
import us.ihmc.perception.logging.PerceptionLoggerConstants;
import us.ihmc.perception.parameters.PerceptionConfigurationParameters;
import us.ihmc.perception.realsense.BytedecoRealsense;
import us.ihmc.perception.realsense.RealSenseHardwareManager;
import us.ihmc.perception.realsense.RealsenseConfiguration;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.IHMCCommonPaths;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Throttler;

import java.nio.file.Paths;
import java.text.SimpleDateFormat;
import java.time.Instant;
import java.util.Date;
import java.util.function.Supplier;

/**
 * Publishes color and depth from Realsense D435
 * ----+ L515 Device Configuration: Serial Number: F0245563, Depth Height 768, Depth Width: 1024, Depth FPS: 15, Color Height 720, Color Width: 1280, Color FPS:
 * 15
 * ----+ D435: Serial Number: 752112070330, Depth Width: 848, Depth Height: 480, Depth FPS: 30, Color Width: 848, Color Height: 480, Color FPS: 30
 * <p>
 * Use this to retrieve files from ihmc-mini-2
 * rsync -aP ihmc-mini-2:/home/ihmc/.ihmc/logs/perception/20230226_172530_PerceptionLog.hdf5 /home/robotlab/.ihmc/logs/perception/
 */
public class RealsenseColorAndDepthPublisher
{
   private String serialNumber;
   private final ROS2Helper ros2Helper;
   private final Supplier<ReferenceFrame> sensorFrameUpdater;
   private final FramePose3D cameraPose = new FramePose3D();
   private final FramePose3D colorPoseInDepthFrame = new FramePose3D();

   private final Point3D cameraPosition = new Point3D();
   private final Quaternion cameraQuaternion = new Quaternion();

   private final ROS2Topic<ImageMessage> colorTopic;
   private final ROS2Topic<ImageMessage> depthTopic;
   private volatile boolean running = true;
   private final Throttler throttler = new Throttler();

   private PerceptionConfigurationParameters parameters = new PerceptionConfigurationParameters();
   private ROS2StoredPropertySetGroup ros2PropertySetGroup;
   private final PerceptionDataLogger perceptionDataLogger = new PerceptionDataLogger();

   private boolean sensorInitialized = false;
   private boolean loggerInitialized = false;
   private boolean previousLoggerEnabledState = false;

   private final RealsenseConfiguration realsenseConfiguration;
   private RealSenseHardwareManager realSenseHardwareManager;
   private BytedecoRealsense realsense;
   private Mat depth16UC1Image;
   private Mat color8UC3Image;
   private Mat yuvColorImage;

   private final double outputPeriod;

   private BytePointer compressedColorPointer;
   private BytePointer compressedDepthPointer;

   private final ImageMessage colorImageMessage = new ImageMessage();
   private final ImageMessage depthImageMessage = new ImageMessage();
   private long depthSequenceNumber = 0;
   private long colorSequenceNumber = 0;

   public RealsenseColorAndDepthPublisher(String serialNumber,
                                          RealsenseConfiguration realsenseConfiguration,
                                          ROS2Topic<ImageMessage> depthTopic,
                                          ROS2Topic<ImageMessage> colorTopic,
                                          Supplier<ReferenceFrame> sensorFrameUpdater)
   {
      this.serialNumber = serialNumber;
      this.realsenseConfiguration = realsenseConfiguration;
      this.colorTopic = colorTopic;
      this.depthTopic = depthTopic;
      this.sensorFrameUpdater = sensorFrameUpdater;

      outputPeriod = UnitConversions.hertzToSeconds(20.0);

      BytedecoTools.loadOpenCV();

      initializeSensor();

      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "realsense_color_and_depth_publisher");
      ros2Helper = new ROS2Helper(ros2Node);

      LogTools.info("Setting Up ROS2 Property Set Group");
      ros2PropertySetGroup = new ROS2StoredPropertySetGroup(ros2Helper);
      ros2PropertySetGroup.registerStoredPropertySet(PerceptionComms.PERCEPTION_CONFIGURATION_PARAMETERS, parameters);
   }

   /**
    * Must be called from the sensor-specific calling class, after the sensor and logger initialization have succeeded.
    */
   public void run()
   {
      while (running)
      {
         update();
         throttler.waitAndRun(outputPeriod); // do the waiting after we send to remove unecessary latency
      }
   }

   private void initializeSensor()
   {
      LogTools.info("Creating Realsense Hardware Manager");
      realSenseHardwareManager = new RealSenseHardwareManager();

      LogTools.info("Creating Bytedeco Realsense Device");
      realsense = realSenseHardwareManager.createBytedecoRealsenseDevice(serialNumber, realsenseConfiguration);

      if (realsense == null)
      {
         running = false;
         throw new RuntimeException("Device could not be initialized.");
      }

      if (realsense.getDevice() == null)
      {
         running = false;
         throw new RuntimeException("RealSense device not found. Set -D<model>.serial.number=00000000000");
      }

      realsense.enableColor(realsenseConfiguration);
      realsense.initialize();
   }

   private void update()
   {
      if (realsense.readFrameData())
      {
         realsense.updateDataBytePointers();

         Instant acquisitionTime = Instant.now();

         MutableBytePointer depthFrameData = realsense.getDepthFrameData();
         depth16UC1Image = new Mat(realsense.getDepthHeight(), realsense.getDepthWidth(), opencv_core.CV_16UC1, depthFrameData);
         PerceptionMessageTools.setDepthIntrinsicsFromRealsense(realsense, depthImageMessage);
         depthImageMessage.setIsPinholeCameraModel(true);

         MutableBytePointer colorFrameData = realsense.getColorFrameData();
         color8UC3Image = new Mat(realsense.getColorHeight(), realsense.getColorWidth(), opencv_core.CV_8UC3, colorFrameData);
         PerceptionMessageTools.setColorIntrinsicsFromRealsense(realsense, colorImageMessage);
         colorImageMessage.setIsPinholeCameraModel(true);

         yuvColorImage = new Mat();

         // Important not to store as a field, as update() needs to be called each frame
         ReferenceFrame cameraFrame = sensorFrameUpdater.get();
         cameraPose.setToZero(cameraFrame);
         cameraPose.changeFrame(ReferenceFrame.getWorldFrame());

         cameraPosition.set(cameraPose.getPosition());
         cameraQuaternion.set(cameraPose.getOrientation());

         colorPoseInDepthFrame.set(realsense.getDepthToColorTranslation(), realsense.getDepthToColorRotation());

         compressedDepthPointer = new BytePointer();
         BytedecoOpenCVTools.compressImagePNG(depth16UC1Image, compressedDepthPointer);

         if (parameters.getPublishDepth())
         {
            PerceptionMessageTools.publishCompressedDepthImage(compressedDepthPointer,
                                                               depthTopic,
                                                               depthImageMessage,
                                                               ros2Helper,
                                                               cameraPose,
                                                               acquisitionTime,
                                                               depthSequenceNumber++,
                                                               realsense.getDepthHeight(),
                                                               realsense.getDepthWidth(),
                                                               (float) realsense.getDepthDiscretization());
         }

         if (parameters.getPublishColor())
         {
            PerceptionMessageTools.publishJPGCompressedColorImage(color8UC3Image,
                                                                  yuvColorImage,
                                                                  colorTopic,
                                                                  colorImageMessage,
                                                                  ros2Helper,
                                                                  colorPoseInDepthFrame,
                                                                  acquisitionTime,
                                                                  colorSequenceNumber++,
                                                                  realsense.getColorHeight(),
                                                                  realsense.getColorWidth(),
                                                                  (float) realsense.getDepthDiscretization());
         }

         if (parameters.getLoggingEnabled())
         {
            if (!loggerInitialized)
            {
               SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
               String logFileName = dateFormat.format(new Date()) + "_" + "PerceptionLog.hdf5";
               FileTools.ensureDirectoryExists(Paths.get(IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY_NAME), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);

               perceptionDataLogger.openLogFile(IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.resolve(logFileName).toString());
               perceptionDataLogger.addImageChannel(PerceptionLoggerConstants.L515_DEPTH_NAME);
               perceptionDataLogger.setChannelEnabled(PerceptionLoggerConstants.L515_DEPTH_NAME, true);

               loggerInitialized = true;
            }

            long timestamp = Conversions.secondsToNanoseconds(acquisitionTime.getEpochSecond()) + acquisitionTime.getNano();

            perceptionDataLogger.storeLongs(PerceptionLoggerConstants.L515_SENSOR_TIME, timestamp);
            perceptionDataLogger.storeBytesFromPointer(PerceptionLoggerConstants.L515_DEPTH_NAME, compressedDepthPointer);

            perceptionDataLogger.storeFloats(PerceptionLoggerConstants.L515_SENSOR_POSITION, cameraPosition);
            perceptionDataLogger.storeFloats(PerceptionLoggerConstants.L515_SENSOR_ORIENTATION, cameraQuaternion);

            previousLoggerEnabledState = true;
         }
         else
         {
            if (previousLoggerEnabledState)
            {
               perceptionDataLogger.closeLogFile();
               previousLoggerEnabledState = false;
               loggerInitialized = false;
            }
         }

         ros2PropertySetGroup.update();
      }
   }

   /**
    * Must be called in the shutdown hook from the sensor-specific calling class. Handles Ctrl + C based closing gracefully.
    */
   public void destroy()
   {
      running = false;
      realsense.deleteDevice();
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
      RealsenseColorAndDepthPublisher realsensePublisher = new RealsenseColorAndDepthPublisher(l515SerialNumber,
                                                                                               RealsenseConfiguration.L515_COLOR_720P_DEPTH_768P_30HZ,
                                                                                               ROS2Tools.L515_DEPTH_IMAGE,
                                                                                               ROS2Tools.L515_COLOR_IMAGE,
                                                                                               ReferenceFrame::getWorldFrame);
      Runtime.getRuntime().addShutdownHook(new Thread(realsensePublisher::destroy, "Shutdown"));
      realsensePublisher.run();
   }
}
