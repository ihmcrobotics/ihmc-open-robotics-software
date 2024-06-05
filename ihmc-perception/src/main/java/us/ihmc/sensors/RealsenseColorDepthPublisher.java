package us.ihmc.sensors;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.thread.Notification;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.property.ROS2StoredPropertySetGroup;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.log.LogTools;
import us.ihmc.perception.CameraModel;
import us.ihmc.perception.comms.PerceptionComms;
import us.ihmc.perception.logging.HDF5Tools;
import us.ihmc.perception.logging.PerceptionDataLogger;
import us.ihmc.perception.logging.PerceptionLoggerConstants;
import us.ihmc.perception.opencv.OpenCVTools;
import us.ihmc.perception.parameters.PerceptionConfigurationParameters;
import us.ihmc.perception.realsense.RealsenseConfiguration;
import us.ihmc.perception.realsense.RealsenseDevice;
import us.ihmc.perception.realsense.RealsenseDeviceManager;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.IHMCCommonPaths;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Throttler;

import java.nio.file.Paths;
import java.time.Instant;
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
public class RealsenseColorDepthPublisher
{
   private static final double OUTPUT_PERIOD = UnitConversions.hertzToSeconds(20.0);

   private final ROS2StoredPropertySetGroup ros2PropertySetGroup;
   private final Supplier<ReferenceFrame> sensorFrameUpdater;
   private final ROS2Topic<ImageMessage> colorTopic;
   private final ROS2Topic<ImageMessage> depthTopic;
   private final ROS2Node ros2Node;
   private final ROS2Helper ros2Helper;
   private final PerceptionConfigurationParameters parameters = new PerceptionConfigurationParameters();
   private final PerceptionDataLogger perceptionDataLogger = new PerceptionDataLogger();
   private final RealsenseDeviceManager realsenseDeviceManager;
   private final RealsenseDevice realsense;

   // Pre-allocations for update loop
   private final ImageMessage colorImageMessage = new ImageMessage();
   private final ImageMessage depthImageMessage = new ImageMessage();
   private final FramePose3D colorPoseInDepthFrame = new FramePose3D();
   private final Quaternion cameraQuaternion = new Quaternion();
   private final FramePose3D depthPose = new FramePose3D();
   private final FramePose3D colorPose = new FramePose3D();
   private final Point3D cameraPosition = new Point3D();
   private final Throttler throttler = new Throttler();

   private long depthSequenceNumber = 0;
   private long colorSequenceNumber = 0;

   private boolean previousLoggerEnabledState = false;
   private boolean loggerInitialized = false;
   private volatile boolean running = true;
   private final Notification destroyedNotification = new Notification();

   public RealsenseColorDepthPublisher(String serialNumber,
                                       RealsenseConfiguration realsenseConfiguration,
                                       ROS2Topic<ImageMessage> depthTopic,
                                       ROS2Topic<ImageMessage> colorTopic,
                                       Supplier<ReferenceFrame> sensorFrameUpdater)
   {
      this.colorTopic = colorTopic;
      this.depthTopic = depthTopic;
      this.sensorFrameUpdater = sensorFrameUpdater;

      realsenseDeviceManager = new RealsenseDeviceManager();
      realsense = realsenseDeviceManager.createBytedecoRealsenseDevice(serialNumber, realsenseConfiguration);
      if (realsense.getDevice() == null)
      {
         destroy();
         throw new RuntimeException("Realsense device not found. Set -D<model>.serial.number=00000000000");
      }
      realsense.enableColor(realsenseConfiguration);
      realsense.initialize();

      ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "realsense_color_and_depth_publisher");
      ros2Helper = new ROS2Helper(ros2Node);

      LogTools.info("Setting up ROS2StoredPropertySetGroup");
      ros2PropertySetGroup = new ROS2StoredPropertySetGroup(ros2Helper);
      ros2PropertySetGroup.registerStoredPropertySet(PerceptionComms.PERCEPTION_CONFIGURATION_PARAMETERS, parameters);

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
      while (running)
      {
         update();
         throttler.waitAndRun(OUTPUT_PERIOD); // do the waiting after we send to remove unnecessary latency
      }

      if (realsense != null)
         realsense.deleteDevice();
      realsenseDeviceManager.deleteContext();
      perceptionDataLogger.closeLogFile();

      destroyedNotification.set();
   }

   /**
    * Get depth and color data from the sensor, compress, send over ROS 2, and log to hdf5
    */
   private void update()
   {
      if (realsense.readFrameData())
      {
         realsense.updateDataBytePointers();

         Instant acquisitionTime = Instant.now();

         // Important not to store as a field, as update() needs to be called each frame
         ReferenceFrame cameraFrame = sensorFrameUpdater.get();
         depthPose.setToZero(cameraFrame);
         depthPose.changeFrame(ReferenceFrame.getWorldFrame());

         colorPose.setIncludingFrame(cameraFrame, realsense.getDepthToColorTranslation(), realsense.getDepthToColorRotation());
         colorPose.invert();
         colorPose.changeFrame(ReferenceFrame.getWorldFrame());

         BytePointer compressedDepthPointer = new BytePointer(); // deallocate later
         BytePointer compressedColorPointer = new BytePointer(); // deallocate later

         // Compress and publish depth image
         // TODO: speed this up
         if (parameters.getPublishDepth())
         {
            Mat depth16UC1Image = new Mat(realsense.getDepthHeight(),
                                          realsense.getDepthWidth(),
                                          opencv_core.CV_16UC1,
                                          realsense.getDepthFrameData()); // deallocate later

            OpenCVTools.compressImagePNG(depth16UC1Image, compressedDepthPointer);

            PerceptionMessageTools.setDepthIntrinsicsFromRealsense(realsense, depthImageMessage);
            CameraModel.PINHOLE.packMessageFormat(depthImageMessage);
            PerceptionMessageTools.publishCompressedDepthImage(compressedDepthPointer,
                                                               depthTopic,
                                                               depthImageMessage,
                                                               ros2Helper,
                                                               depthPose,
                                                               acquisitionTime,
                                                               depthSequenceNumber++,
                                                               realsense.getDepthHeight(),
                                                               realsense.getDepthWidth(),
                                                               (float) realsense.getDepthDiscretization());

            depth16UC1Image.deallocate();
         }

         // Compress and publish color image
         // TODO: speed this up
         if (parameters.getPublishColor())
         {
            colorPoseInDepthFrame.set(realsense.getDepthToColorTranslation(), realsense.getDepthToColorRotation());

            Mat color8UC3Image = new Mat(realsense.getColorHeight(),
                                         realsense.getColorWidth(),
                                         opencv_core.CV_8UC3,
                                         realsense.getColorFrameData()); // deallocate later
            // YUV I420 has 1.5 times the height of the image
            Mat yuvColorImage = new Mat(realsense.getColorHeight() * 1.5, realsense.getColorWidth(), opencv_core.CV_8UC1); // deallocate later

            OpenCVTools.compressRGBImageJPG(color8UC3Image, yuvColorImage, compressedColorPointer);

            PerceptionMessageTools.setColorIntrinsicsFromRealsense(realsense, colorImageMessage);
            CameraModel.PINHOLE.packMessageFormat(colorImageMessage);
            PerceptionMessageTools.publishJPGCompressedColorImage(compressedColorPointer,
                                                                  colorTopic,
                                                                  colorImageMessage,
                                                                  ros2Helper,
                                                                  colorPose,
                                                                  acquisitionTime,
                                                                  colorSequenceNumber++,
                                                                  realsense.getColorHeight(),
                                                                  realsense.getColorWidth(),
                                                                  (float) realsense.getDepthDiscretization());

            color8UC3Image.deallocate();
            yuvColorImage.deallocate();
         }

         if (parameters.getLoggingEnabled())
         {
            cameraPosition.set(depthPose.getPosition());
            cameraQuaternion.set(depthPose.getOrientation());

            if (!loggerInitialized)
            {
               initializeLogger();
            }

            long timestamp = Conversions.secondsToNanoseconds(acquisitionTime.getEpochSecond()) + acquisitionTime.getNano();

            perceptionDataLogger.storeLongs(PerceptionLoggerConstants.L515_SENSOR_TIME, timestamp);
            perceptionDataLogger.storeFloats(PerceptionLoggerConstants.L515_SENSOR_POSITION, cameraPosition);
            perceptionDataLogger.storeFloats(PerceptionLoggerConstants.L515_SENSOR_ORIENTATION, cameraQuaternion);
            perceptionDataLogger.storeBytesFromPointer(PerceptionLoggerConstants.L515_DEPTH_NAME, compressedDepthPointer);

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

         compressedDepthPointer.deallocate();
         compressedColorPointer.deallocate();

         ros2PropertySetGroup.update();
      }
   }

   /**
    * Setup everything needed for the perception logger to run and collect data for various sensor signals
    */
   private void initializeLogger()
   {
      String logFileName = HDF5Tools.generateFileName();
      FileTools.ensureDirectoryExists(Paths.get(IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY_NAME), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);

      perceptionDataLogger.openLogFile(IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.resolve(logFileName).toString());
      perceptionDataLogger.addLongChannel(PerceptionLoggerConstants.L515_SENSOR_TIME, 1, PerceptionLoggerConstants.DEFAULT_BLOCK_SIZE);
      perceptionDataLogger.addFloatChannel(PerceptionLoggerConstants.L515_SENSOR_POSITION, 3, PerceptionLoggerConstants.DEFAULT_BLOCK_SIZE);
      perceptionDataLogger.addFloatChannel(PerceptionLoggerConstants.L515_SENSOR_ORIENTATION, 4, PerceptionLoggerConstants.DEFAULT_BLOCK_SIZE);
      perceptionDataLogger.addImageChannel(PerceptionLoggerConstants.L515_DEPTH_NAME);

      loggerInitialized = true;
   }

   /**
    * Must be called in the shutdown hook from the sensor-specific calling class. Handles Ctrl + C based closing gracefully.
    */
   public void destroy()
   {
      running = false;
      ros2Node.destroy();
      destroyedNotification.blockingPoll();
   }

   public static void main(String[] args)
   {
      /*
         Color: [fx:901.3026, fy:901.8400, cx:635.2337, cy:350.9427, h:720, w:1280]
         Depth: [fx:730.7891, fy:731.0859, cx:528.6094, cy:408.1602, h:768, w:1024]
      */

      // L515: [F1121365, F0245563], D455: [215122254074]
      String realsenseSerialNumber = System.getProperty("d455.serial.number", "213522252883");
      RealsenseColorDepthPublisher realsensePublisher = new RealsenseColorDepthPublisher(realsenseSerialNumber,
                                                                                         RealsenseConfiguration.D455_COLOR_720P_DEPTH_720P_30HZ,
                                                                                         PerceptionAPI.D455_DEPTH_IMAGE,
                                                                                         PerceptionAPI.D455_COLOR_IMAGE,
                                                                                         ReferenceFrame::getWorldFrame);
      realsensePublisher.run();
      ThreadTools.sleepForever();
   }
}
