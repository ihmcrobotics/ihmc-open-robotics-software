package us.ihmc.perception.logging;

import controller_msgs.msg.dds.RobotConfigurationData;
import gnu.trove.list.array.TFloatArrayList;
import org.apache.commons.lang.ArrayUtils;
import org.bytedeco.hdf5.Group;
import org.bytedeco.hdf5.global.hdf5;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.BigVideoPacket;
import perception_msgs.msg.dds.FusedSensorHeadPointCloudMessage;
import perception_msgs.msg.dds.LidarScanMessage;
import perception_msgs.msg.dds.VideoPacket;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.idl.IDLSequence;
import us.ihmc.log.LogTools;
import us.ihmc.perception.BytedecoOpenCVTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.common.SampleInfo;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.thread.ExecutorServiceTools;

import java.io.File;
import java.text.SimpleDateFormat;
import java.util.*;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import static org.bytedeco.opencv.global.opencv_highgui.imshow;
import static org.bytedeco.opencv.global.opencv_highgui.waitKeyEx;

public class PerceptionDataLogger
{
   /* TODO:
   *     Remove or fix commented parts of the code.
   *     Improve multi-threading
   *     Convert buffer sizes to final constant values
   *
   * */
   private static final int BUFFER_SIZE = 25000000;

   private final String ROOT_POSITION_NAME = "/robot/root/position/";
   private final String ROOT_ORIENTATION_NAME = "/robot/root/orientation/";
   private final String JOINT_ANGLES_NAME = "/robot/joint_angles/";
   private final String JOINT_VELOCITIES_NAME = "/robot/joint_velocities/";
   private final String JOINT_TORQUES_NAME = "/robot/joint_torques/";

   private final String OUSTER_CLOUD_NAME = "/os_cloud_node/points/";
   private final String OUSTER_DEPTH_NAME = "/ouster/depth/";

   private final String D435_DEPTH_NAME = "/d435/depth/";
   private final String D435_COLOR_NAME = "/d435/color/";

   private final String ZED2_COLOR_NAME = "/zed2/color/";

   private final String L515_DEPTH_NAME = "/l515/depth/";
   private final String L515_COLOR_NAME = "/l515/color/";

   private final byte[] messageDepthDataArray = new byte[BUFFER_SIZE];
   private final Mat inputJPEGMat = new Mat(1, 1, opencv_core.CV_8UC1);
   private final Mat inputYUVI420Mat = new Mat(1, 1, opencv_core.CV_8UC1);
   private final BytePointer messageEncodedBytePointer = new BytePointer(BUFFER_SIZE);

   private final HashMap<String, byte[]> buffers = new HashMap<>();
   private final HashMap<String, Integer> counts = new HashMap<>();

   private final CommunicationMode communicationMode;
   private final ArrayDeque<Runnable> runnablesToStopLogging = new ArrayDeque<>();
   private final FusedSensorHeadPointCloudMessage ousterCloudPacket = new FusedSensorHeadPointCloudMessage();
   private final ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(1,
                                                                                                        getClass(),
                                                                                                        ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);

   private ROS2Node ros2Node;
   private ROS2Helper ros2Helper;
   private HDF5Manager hdf5Manager;
   private RealtimeROS2Node realtimeROS2Node;

   private int pointCloudCount = 0;
   private int depthMapCount = 0;


   public PerceptionDataLogger()
   {
      communicationMode = CommunicationMode.INTERPROCESS;
   }

   public void startLogging(String logFileName, String simpleRobotName)
   {
      File f = new File(logFileName);
      if (!f.exists() && !f.isDirectory())
      {

         LogTools.info("Creating HDF5 File: " + logFileName);
         hdf5Manager = new HDF5Manager(logFileName, hdf5.H5F_ACC_TRUNC);
         hdf5Manager.getFile().openFile(logFileName, hdf5.H5F_ACC_RDWR);
      }
      else
      {
         LogTools.info("Opening Existing HDF5 File: " + logFileName);
         hdf5Manager = new HDF5Manager(logFileName, hdf5.H5F_ACC_RDWR);
      }

      // Use both regular and real-time ROS2 nodes to assign callbacks to different message types
      ros2Node = ROS2Tools.createROS2Node(communicationMode.getPubSubImplementation(), "perception_logger_node");
      ros2Helper = new ROS2Helper(ros2Node);
      realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "perception_logger_realtime_node");

      // Add callback for Ouster scans
      var ousterSubscription = ros2Helper.subscribe(ROS2Tools.OUSTER_POINT_CLOUD);
      ousterSubscription.addCallback(this::logLidarScanMessage);
      runnablesToStopLogging.addLast(ousterSubscription::destroy);

      // Add callback for Robot Configuration Data
      var robotConfigurationDataSubscription = ros2Helper.subscribe(ROS2Tools.getRobotConfigurationDataTopic(simpleRobotName));
      robotConfigurationDataSubscription.addCallback(this::logRobotConfigurationData);
      runnablesToStopLogging.addLast(robotConfigurationDataSubscription::destroy);

      // Add callback for D435 Color images
      var d435VideoSubscription = ros2Helper.subscribe(ROS2Tools.D435_VIDEO);
      d435VideoSubscription.addCallback(this::logColorD435);
      runnablesToStopLogging.addLast(d435VideoSubscription::destroy);
      buffers.put(D435_COLOR_NAME, new byte[BUFFER_SIZE]);
      counts.put(D435_COLOR_NAME, 0);

      // Add callback for D435 Depth images
      var d435DepthSubscription = ros2Helper.subscribe(ROS2Tools.D435_DEPTH);
      d435DepthSubscription.addCallback(this::logDepthD435);
      runnablesToStopLogging.addLast(d435DepthSubscription::destroy);
      buffers.put(D435_DEPTH_NAME, new byte[BUFFER_SIZE]);
      counts.put(D435_DEPTH_NAME, 0);

      // Add callback for L515 Depth maps
      var l515DepthSubscription = ros2Helper.subscribe(ROS2Tools.L515_DEPTH);
      l515DepthSubscription.addCallback(this::logDepthL515);
      runnablesToStopLogging.addLast(l515DepthSubscription::destroy);
      buffers.put(L515_DEPTH_NAME, new byte[BUFFER_SIZE]);
      counts.put(L515_DEPTH_NAME, 0);

      // Add callback for L515 Depth maps
      var l515ColorSubscription = ros2Helper.subscribe(ROS2Tools.L515_VIDEO);
      l515ColorSubscription.addCallback(this::logColorL515);
      runnablesToStopLogging.addLast(l515ColorSubscription::destroy);
      buffers.put(L515_COLOR_NAME, new byte[BUFFER_SIZE]);
      counts.put(L515_COLOR_NAME, 0);

      // Add callback for D435 Color images
      var zed2StereoSubscription = ros2Helper.subscribe(ROS2Tools.ZED2_STEREO_COLOR);
      zed2StereoSubscription.addCallback(this::logColorZED2);
      runnablesToStopLogging.addLast(zed2StereoSubscription::destroy);
      buffers.put(ZED2_COLOR_NAME, new byte[BUFFER_SIZE]);
      counts.put(ZED2_COLOR_NAME, 0);

      // Add callback for L515 Depth maps
//      var l515ColorSubscription = ros2Helper.subscribe(ROS2Tools.L515_VIDEO);
//      l515ColorSubscription.addCallback(this::logColorL515);
//      runnablesToStopLogging.addLast(l515ColorSubscription::destroy);

      // Add callback for Ouster depth maps
      var ousterDepthSubscription = ros2Helper.subscribe(ROS2Tools.OUSTER_DEPTH);
      ousterDepthSubscription.addCallback(this::logDepthOuster);
      runnablesToStopLogging.addLast(ousterDepthSubscription::destroy);

      realtimeROS2Node.spin();

      runnablesToStopLogging.addLast(realtimeROS2Node::destroy);

      //      "D435 Video", ros2Node, ROS2Tools.VIDEO, ROS2VideoFormat.JPEGYUVI420

      //      new IHMCROS2Callback<>(ros2Node, ROS2Tools.BLACKFLY_VIDEO.get(RobotSide.RIGHT), this::logBigVideoPacket);
      //      bigVideoPacketROS2Callback = new ROS2Callback<>(ros2Node, ROS2Tools.BLACKFLY_VIDEO.get(RobotSide.RIGHT), this::logBigVideoPacket);
      //      new ROS2Callback<>(ros2Node, ROS2Tools.BLACKFLY_VIDEO.get(RobotSide.LEFT), this::logBigVideoPacket);

      executorService.scheduleAtFixedRate(this::collectStatistics, 0, 10, TimeUnit.MILLISECONDS);
   }

   public void stopLogging()
   {
      while (!runnablesToStopLogging.isEmpty())
      {
         runnablesToStopLogging.pollFirst().run();
      }
   }

   public void collectStatistics()
   {
      //      LogTools.info("Collecting Statistics");
   }

   /*
    * Log methods are simple high-level calls to request logging for direct message/packet types.
    * They internally call Store methods for actually deploying threads to write compressed data to files.
    * */

   public void logRobotConfigurationData(RobotConfigurationData data)
   {
      LogTools.info("Robot Configuration Data Received: {}", data.getMonotonicTime());

      storeFloatArray(ROOT_POSITION_NAME, data.getRootPosition());
      storeFloatArray(ROOT_ORIENTATION_NAME, data.getRootOrientation());
      storeFloatArray(JOINT_ANGLES_NAME, data.getJointAngles().toArray());
      storeFloatArray(JOINT_VELOCITIES_NAME, data.getJointVelocities().toArray());
      storeFloatArray(JOINT_TORQUES_NAME, data.getJointTorques().toArray());
   }

   public void logLidarScanMessage(FusedSensorHeadPointCloudMessage message)
   {
      LogTools.info("OUSTER POINT CLOUD Received.");

      ousterCloudPacket.set(message);
      storePointCloud(OUSTER_CLOUD_NAME, ousterCloudPacket);
   }

   public void logDepthOuster(VideoPacket packet)
   {
      LogTools.info("Depth Map Received: {}", packet.data_);
      storeVideoPacket(OUSTER_DEPTH_NAME, packet);
   }

   public void logColorD435(VideoPacket videoPacket)
   {
      LogTools.info("Logging D435 Color: ", videoPacket.toString());
      storeVideoPacket(D435_COLOR_NAME, videoPacket);
   }

   public void logDepthD435(VideoPacket videoPacket)
   {
      LogTools.info("Logging D435 Depth: ", videoPacket.toString());
      storeVideoPacket(D435_DEPTH_NAME, videoPacket);
   }

   public void logDepthL515(VideoPacket videoPacket)
   {
      LogTools.info("Logging L515 Depth: ", videoPacket.toString());
      storeVideoPacket(L515_DEPTH_NAME, videoPacket);
   }

   public void logColorL515(VideoPacket videoPacket)
   {
      LogTools.info("Logging L515 Color: ", videoPacket.toString());
      storeVideoPacket(L515_COLOR_NAME, videoPacket);
   }

   public void logColorZED2(VideoPacket videoPacket)
   {
      LogTools.info("Logging L515 Color: ", videoPacket.toString());
      BytedecoOpenCVTools.displayVideoPacketColor(videoPacket);
   }

   public void convertBigVideoPacketToMat(BigVideoPacket packet, Mat mat)
   {
      IDLSequence.Byte imageEncodedTByteArrayList = packet.getData();
      imageEncodedTByteArrayList.toArray(messageDepthDataArray);
      messageEncodedBytePointer.put(messageDepthDataArray, 0, imageEncodedTByteArrayList.size());
      messageEncodedBytePointer.limit(imageEncodedTByteArrayList.size());

      inputJPEGMat.cols(imageEncodedTByteArrayList.size());
      inputJPEGMat.data(messageEncodedBytePointer);

      opencv_imgcodecs.imdecode(inputJPEGMat, opencv_imgcodecs.IMREAD_UNCHANGED, inputYUVI420Mat);

      //        updateImageDimensions(mat, inputYUVI420Mat.cols(), (int) (inputYUVI420Mat.rows() / 1.5f));
      opencv_imgproc.cvtColor(inputYUVI420Mat, mat, opencv_imgproc.COLOR_YUV2RGBA_I420);
   }

   public void logBigVideoPacket(BigVideoPacket packet)
   {
      LogTools.info("BIG Video Received.");
      //      Mat mat = new Mat(packet.getImageHeight(), packet.getImageHeight(), opencv_core.CV_8UC3);
      //      convertBigVideoPacketToMat(packet, mat);
      //      logImage(mat);
   }


   /*
    *  Store methods which actually deploy threads and call HDF5 specific functions for storing compressed data.
    */

   public void storeDepthMap(String namespace, BigVideoPacket message)
   {
      Group group = hdf5Manager.getGroup(namespace);
//      ThreadTools.startAThread(() ->
//                               {
                                  synchronized (this)
                                  {
                                     LogTools.info("{} Storing Buffer: {}", namespace, depthMapCount);
                                     depthMapCount = (int) hdf5Manager.getCount(namespace);

                                     IDLSequence.Byte imageEncodedTByteArrayList = message.getData();
                                     imageEncodedTByteArrayList.toArray(messageDepthDataArray);

                                     HDF5Tools.storeByteArray(group, depthMapCount, messageDepthDataArray, message.getData().size());
                                     LogTools.info("{} Done Storing Buffer: {}", namespace, depthMapCount);
                                  }
//                               }, "depth_map_logger_thread");
   }

   public void storeVideoPacket(String namespace, VideoPacket packet)
   {
      long begin_store = System.nanoTime();
      Group group = hdf5Manager.getGroup(namespace);
//      ThreadTools.startAThread(() ->
//                               {

                                  byte[] heapArray = buffers.get(namespace);
                                  int imageCount = counts.get(namespace);
                                  IDLSequence.Byte imageEncodedTByteArrayList = packet.getData();

                                  LogTools.info("{} Storing Buffer: {}", namespace, imageCount);
                                  counts.put(namespace, imageCount + 1);

//                                  LogTools.info("{} ByteArray: {}", namespace, Arrays.toString(heapArray));

                                  // Logging into HDF5
                                  imageEncodedTByteArrayList.toArray(heapArray, 0, packet.getData().size() + 4);
                                  HDF5Tools.storeByteArray(group, imageCount, heapArray, imageEncodedTByteArrayList.size() + 4);

                                  LogTools.info("{} Done Storing Buffer: {}", namespace, imageCount);
//                               }, "video_packet_logger_thread -> " + namespace);
      long end_store = System.nanoTime();


   }

   public void storePointCloud(String namespace, LidarScanMessage message)
   {
      Group group = hdf5Manager.getGroup(namespace);

      ThreadTools.startAThread(() ->
                               {
                                  synchronized (this)
                                  {
                                     LogTools.info("{} Storing Buffer: {}", namespace, pointCloudCount);
                                     pointCloudCount = (int) hdf5Manager.getCount(namespace);
                                     HDF5Tools.storeByteArray(group, pointCloudCount, message.getScan().toArray(), message.getScan().size());
                                     LogTools.info("{} Done Storing Buffer: {}", namespace, pointCloudCount);

                                     //                                         pointCloudCount++;
                                  }
                               }, "lidar_scan_logger_thread");
   }

   public void storePointCloud(String namespace, FusedSensorHeadPointCloudMessage message)
   {
      Group group = hdf5Manager.getGroup(namespace);

      ThreadTools.startAThread(() ->
                               {
                                  synchronized (this)
                                  {
                                     LogTools.info("{} Storing Buffer: {}", namespace, pointCloudCount);
                                     pointCloudCount = (int) hdf5Manager.getCount(namespace);
                                     HDF5Tools.storeByteArray(group, pointCloudCount, message.getScan().toArray(), message.getScan().size());
                                     LogTools.info("{} Done Storing Buffer: {}", namespace, pointCloudCount);
                                  }
                               }, "fused_point_cloud_logger_thread");
   }

   public void storeFloatArray(String namespace, float[] array)
   {
      Group group = hdf5Manager.getGroup(namespace);
      TFloatArrayList buffer = hdf5Manager.getBuffer(namespace);
      buffer.addAll(array);

      int bufferSize = hdf5Manager.getBufferIndex(namespace) / array.length;
      LogTools.info("Buffer Index: {} {}", bufferSize, HDF5Manager.MAX_BUFFER_SIZE - 1);
      if (bufferSize == (HDF5Manager.MAX_BUFFER_SIZE - 1))
      {
         LogTools.info("Thread Store Triggered");
         hdf5Manager.resetBuffer(namespace);

         ThreadTools.startAThread(() ->
                                  {
                                     long count = hdf5Manager.getCount(namespace);
                                     LogTools.info("Storing Buffer: {}", count);
                                     HDF5Tools.storeFloatArray2D(group, count, buffer, HDF5Manager.MAX_BUFFER_SIZE, array.length);
                                     LogTools.info("Done Storing Buffer: {}", count);
                                  }, "float_array_logger_thread -> " + namespace);
      }
   }

   public void storeFloatArray(String namespace, Point3D point)
   {
      float[] pointArray = new float[3];
      point.get(pointArray);
      storeFloatArray(namespace, pointArray);
   }

   public void storeFloatArray(String namespace, Quaternion orientation)
   {
      float[] pointArray = new float[4];
      orientation.get(pointArray);
      storeFloatArray(namespace, pointArray);
   }

   public static void main(String[] args)
   {
      SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");

      String defaultLogDirectory = System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator;
      String logDirectory = System.getProperty("perception.log.directory", defaultLogDirectory);
      String logFileName = dateFormat.format(new Date()) + "_" + "PerceptionLog.hdf5";

      PerceptionDataLogger logger = new PerceptionDataLogger();
      logger.startLogging(logDirectory + logFileName, "Robot");
   }
}


