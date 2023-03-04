package us.ihmc.sensors;

import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.ImageMessage;
import us.ihmc.commons.Conversions;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.nio.FileTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.property.ROS2StoredPropertySetGroup;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
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
import us.ihmc.perception.comms.ImageMessageFormat;
import us.ihmc.perception.tools.OpenCVJPEGCompression;
import us.ihmc.perception.tools.OpenCVPNGCompression;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.pubsub.DomainFactory.PubSubImplementation;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.tools.IHMCCommonPaths;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.Activator;
import us.ihmc.tools.thread.Throttler;

import java.nio.file.Paths;
import java.text.SimpleDateFormat;
import java.time.Instant;
import java.util.Date;
import java.util.function.Supplier;

/**
 * <<<<<<< HEAD
 * Publishes color and depth from Realsense D435
 * ----+ L515 Device Configuration: Serial Number: F0245563, Depth Height 768, Depth Width: 1024, Depth FPS: 15, Color Height 720, Color Width: 1280, Color FPS:
 * 15
 * ----+ D435: Serial Number: 752112070330, Depth Width: 848, Depth Height: 480, Depth FPS: 30, Color Width: 848, Color Height: 480, Color FPS: 30
 * <p>
 * Use this to retrieve files from ihmc-mini-2
 * rsync -aP ihmc-mini-2:/home/ihmc/.ihmc/logs/perception/20230226_172530_PerceptionLog.hdf5 /home/robotlab/.ihmc/logs/perception/
 * =======
 * Publishes color and depth images from Realsense devices.
 * >>>>>>> develop
 */
public class RealsenseColorAndDepthPublisher
{
   private final String serialNumber;
   private final Activator nativesLoadedActivator;
   private final ROS2Helper ros2Helper;
   private final Supplier<ReferenceFrame> sensorFrameUpdater;
   private final FramePose3D cameraPose = new FramePose3D();

   private final Point3D cameraPosition = new Point3D();
   private final Quaternion cameraQuaternion = new Quaternion();

   private final ROS2Topic<ImageMessage> colorTopic;
   private final ROS2Topic<ImageMessage> depthTopic;
   private volatile boolean running = true;
   private final Throttler throttler = new Throttler();

   private PerceptionConfigurationParameters parameters = new PerceptionConfigurationParameters();
   private ROS2StoredPropertySetGroup ros2PropertySetGroup;
   private final PerceptionDataLogger perceptionDataLogger = new PerceptionDataLogger();
   private boolean loggerInitialized = false;
   private boolean previousLoggerEnabledState = false;

   private final RealsenseConfiguration settingsProfile;
   private RealSenseHardwareManager realSenseHardwareManager;
   private BytedecoRealsense realsense;
   private Mat depth16UC1Image;
   private Mat color8UC3Image;
   private Mat yuvColorImage;

   private final double outputPeriod;

   private BytePointer compressedColorPointer;
   private BytePointer compressedDepthPointer;

   private final Pose3D mocapPose = new Pose3D();

   private int depthHeight;
   private int depthWidth;
   private final OpenCVPNGCompression pngCompression = new OpenCVPNGCompression();
   private final OpenCVJPEGCompression jpegCompression = new OpenCVJPEGCompression(80);
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
      this.settingsProfile = realsenseConfiguration;
      this.colorTopic = colorTopic;
      this.depthTopic = depthTopic;
      this.sensorFrameUpdater = sensorFrameUpdater;

      outputPeriod = UnitConversions.hertzToSeconds(20.0);

      nativesLoadedActivator = BytedecoTools.loadOpenCVNativesOnAThread();

      ROS2Node ros2Node = ROS2Tools.createROS2Node(PubSubImplementation.FAST_RTPS, "realsense_color_and_depth_publisher");
      ros2Helper = new ROS2Helper(ros2Node);

      new IHMCROS2Callback<>(ros2Node, ROS2Tools.MOCAP_RIGID_BODY, (message) ->
      {
         message.get(mocapPose);
      });

      Runtime.getRuntime().addShutdownHook(new Thread(() ->
                                                      {
                                                         ThreadTools.sleepSeconds(0.5);
                                                         destroy();
                                                      }, getClass().getSimpleName() + "Shutdown"));
   }

   public void run()
   {
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
            realsense = realSenseHardwareManager.createBytedecoRealsenseDevice(serialNumber, settingsProfile);

            if (realsense.getDevice() == null)
            {
               running = false;
               throw new RuntimeException("Device not found. Set -D<model>.serial.number=00000000000");
            }
            realsense.enableColor(settingsProfile);
            realsense.initialize();

            ros2PropertySetGroup = new ROS2StoredPropertySetGroup(ros2Helper);
            ros2PropertySetGroup.registerStoredPropertySet(PerceptionComms.PERCEPTION_CONFIGURATION_PARAMETERS, parameters);

            pngCompression.allocate(realsense.getDepthWidth() * realsense.getDepthHeight() * ImageMessageFormat.DEPTH_PNG_16UC1.getBytesPerPixel());
            jpegCompression.allocate(realsense.getColorWidth() * realsense.getColorHeight() * ImageMessageFormat.COLOR_JPEG_YUVI420.getBytesPerPixel());
         }

         if (realsense.readFrameData())
         {
            realsense.updateDataBytePointers();

            Instant acquisitionTime = Instant.now();

            MutableBytePointer depthFrameData = realsense.getDepthFrameData();
            depth16UC1Image = new Mat(realsense.getDepthHeight(), realsense.getDepthWidth(), opencv_core.CV_16UC1, depthFrameData);
            PerceptionMessageTools.setDepthIntrinsicsFromRealsense(realsense, depthImageMessage);

            MutableBytePointer colorFrameData = realsense.getColorFrameData();
            color8UC3Image = new Mat(realsense.getColorHeight(), realsense.getColorWidth(), opencv_core.CV_8UC3, colorFrameData);
            PerceptionMessageTools.setColorIntrinsicsFromRealsense(realsense, colorImageMessage);

            yuvColorImage = new Mat();

            // Important not to store as a field, as update() needs to be called each frame
            ReferenceFrame cameraFrame = sensorFrameUpdater.get();
            cameraPose.setToZero(cameraFrame);
            cameraPose.changeFrame(ReferenceFrame.getWorldFrame());

            cameraPosition.set(cameraPose.getPosition());
            cameraQuaternion.set(cameraPose.getOrientation());

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
                                                                  realsense.getDepthWidth());
            }

            if (parameters.getPublishColor())
            {
               PerceptionMessageTools.publishJPGCompressedColorImage(color8UC3Image,
                                                                     yuvColorImage,
                                                                     colorTopic,
                                                                     colorImageMessage,
                                                                     ros2Helper,
                                                                     cameraPose,
                                                                     acquisitionTime,
                                                                     colorSequenceNumber++,
                                                                     realsense.getColorHeight(), realsense.getColorWidth());
            }

            if (parameters.getLoggingEnabled())
            {
               if (!loggerInitialized)
               {
                  SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");
                  String logFileName = dateFormat.format(new Date()) + "_" + "PerceptionLog.hdf5";
                  FileTools.ensureDirectoryExists(Paths.get(IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY_NAME), DefaultExceptionHandler.MESSAGE_AND_STACKTRACE);

                  perceptionDataLogger.openLogFile(IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.resolve(logFileName).toString());
                  perceptionDataLogger.addChannel(PerceptionLoggerConstants.L515_DEPTH_NAME);
                  perceptionDataLogger.setChannelEnabled(PerceptionLoggerConstants.L515_DEPTH_NAME, true);

                  loggerInitialized = true;
               }

               long timestamp = Conversions.secondsToNanoseconds(acquisitionTime.getEpochSecond()) + acquisitionTime.getNano();

               perceptionDataLogger.storeLongArray(PerceptionLoggerConstants.L515_SENSOR_TIME, timestamp);
               perceptionDataLogger.storeBytesFromPointer(PerceptionLoggerConstants.L515_DEPTH_NAME, compressedDepthPointer);

               perceptionDataLogger.storeFloatArray(PerceptionLoggerConstants.L515_SENSOR_POSITION, cameraPosition);
               perceptionDataLogger.storeFloatArray(PerceptionLoggerConstants.L515_SENSOR_ORIENTATION, cameraQuaternion);

               perceptionDataLogger.storeFloatArray(PerceptionLoggerConstants.MOCAP_RIGID_BODY_POSITION, mocapPose.getPosition());
               perceptionDataLogger.storeFloatArray(PerceptionLoggerConstants.MOCAP_RIGID_BODY_ORIENTATION, mocapPose.getOrientation());

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
            //=======
            //            pngCompression.compress(depth16UC1Image);
            //            jpegCompression.compressRGB(color8UC3Image);
            //
            //            PerceptionMessageTools.packImageMessageData(pngCompression.getCompressedData(), depthImageMessage);
            //            ImageMessageFormat.DEPTH_PNG_16UC1.packMessageFormat(depthImageMessage);
            //            depthImageMessage.setImageHeight(realsense.getDepthHeight());
            //            depthImageMessage.setImageWidth(realsense.getDepthWidth());
            //            depthImageMessage.getPosition().set(cameraPose.getPosition());
            //            depthImageMessage.getOrientation().set(cameraPose.getOrientation());
            //            depthImageMessage.setSequenceNumber(depthSequenceNumber++);
            //            MessageTools.toMessage(acquisitionTime, depthImageMessage.getAcquisitionTime());
            //            depthImageMessage.setFocalLengthXPixels(realsense.getDepthIntrinsicParameters().fx());
            //            depthImageMessage.setFocalLengthYPixels(realsense.getDepthIntrinsicParameters().fy());
            //            depthImageMessage.setPrincipalPointXPixels(realsense.getDepthIntrinsicParameters().ppx());
            //            depthImageMessage.setPrincipalPointYPixels(realsense.getDepthIntrinsicParameters().ppy());
            //            depthImageMessage.setDepthDiscretization((float) realsense.getDepthDiscretization());
            //            depthImageMessage.setIsPinholeCameraModel(true);
            //            ros2Helper.publish(depthTopic, depthImageMessage);
            //
            //            PerceptionMessageTools.packImageMessageData(jpegCompression.getCompressedData(), colorImageMessage);
            //            ImageMessageFormat.COLOR_JPEG_YUVI420.packMessageFormat(colorImageMessage);
            //            colorImageMessage.setImageHeight(realsense.getColorHeight());
            //            colorImageMessage.setImageWidth(realsense.getColorWidth());
            //            colorImageMessage.getPosition().set(realsense.getDepthToColorTranslation());
            //            colorImageMessage.getOrientation().set(realsense.getDepthToColorRotation());
            //            colorImageMessage.setSequenceNumber(colorSequenceNumber++);
            //            MessageTools.toMessage(acquisitionTime, colorImageMessage.getAcquisitionTime());
            //            colorImageMessage.setFocalLengthXPixels(realsense.getColorIntrinsicParameters().fx());
            //            colorImageMessage.setFocalLengthYPixels(realsense.getColorIntrinsicParameters().fy());
            //            colorImageMessage.setPrincipalPointXPixels(realsense.getColorIntrinsicParameters().ppx());
            //            colorImageMessage.setPrincipalPointYPixels(realsense.getColorIntrinsicParameters().ppy());
            //            colorImageMessage.setIsPinholeCameraModel(true);
            //            ros2Helper.publish(colorTopic, colorImageMessage);
            //>>>>>>> develop
         }
      }
   }

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