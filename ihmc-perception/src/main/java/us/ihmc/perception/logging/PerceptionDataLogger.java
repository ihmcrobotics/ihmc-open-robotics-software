package us.ihmc.perception.logging;

import controller_msgs.msg.dds.RobotConfigurationData;
import gnu.trove.list.array.TFloatArrayList;
import gnu.trove.list.array.TLongArrayList;
import org.bytedeco.hdf5.Group;
import org.bytedeco.hdf5.global.hdf5;
import perception_msgs.msg.dds.*;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.idl.IDLSequence;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;
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
   private static final int BUFFER_SIZE = 2500000;

   private final HashMap<String, byte[]> byteBuffers = new HashMap<>();
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

   private HashMap<String, PerceptionLogChannel> channels = new HashMap<>();

   public PerceptionDataLogger()
   {
      channels.put(PerceptionLoggerConstants.D435_COLOR_NAME, new PerceptionLogChannel(PerceptionLoggerConstants.D435_COLOR_NAME, 0, 0));
      channels.put(PerceptionLoggerConstants.D435_DEPTH_NAME, new PerceptionLogChannel(PerceptionLoggerConstants.D435_DEPTH_NAME, 0, 0));
      channels.put(PerceptionLoggerConstants.L515_COLOR_NAME, new PerceptionLogChannel(PerceptionLoggerConstants.L515_COLOR_NAME, 0, 0));
      channels.put(PerceptionLoggerConstants.L515_DEPTH_NAME, new PerceptionLogChannel(PerceptionLoggerConstants.L515_DEPTH_NAME, 0, 0));
      channels.put(PerceptionLoggerConstants.OUSTER_CLOUD_NAME, new PerceptionLogChannel(PerceptionLoggerConstants.OUSTER_CLOUD_NAME, 0, 0));
      channels.put(PerceptionLoggerConstants.OUSTER_DEPTH_NAME, new PerceptionLogChannel(PerceptionLoggerConstants.OUSTER_DEPTH_NAME, 0, 0));
      channels.put(PerceptionLoggerConstants.ROBOT_CONFIGURATION_DATA_NAME,
                   new PerceptionLogChannel(PerceptionLoggerConstants.ROBOT_CONFIGURATION_DATA_NAME, 0, 0));

      communicationMode = CommunicationMode.INTERPROCESS;
   }

   public void openLogFile(String logFileName)
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
   }

   public void closeLogFile()
   {
      hdf5Manager.closeFile();
   }

   public void startLogging(String logFileName, String simpleRobotName)
   {
      openLogFile(logFileName);

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
      byteBuffers.put(PerceptionLoggerConstants.D435_COLOR_NAME, new byte[BUFFER_SIZE]);
      counts.put(PerceptionLoggerConstants.D435_COLOR_NAME, 0);

      // Add callback for D435 Depth images
      var d435DepthSubscription = ros2Helper.subscribe(ROS2Tools.D435_DEPTH);
      d435DepthSubscription.addCallback(this::logDepthD435);
      runnablesToStopLogging.addLast(d435DepthSubscription::destroy);
      byteBuffers.put(PerceptionLoggerConstants.D435_DEPTH_NAME, new byte[BUFFER_SIZE]);
      counts.put(PerceptionLoggerConstants.D435_DEPTH_NAME, 0);

      // Add callback for L515 Depth Maps
      var l515DepthSubscription = ros2Helper.subscribe(ROS2Tools.L515_DEPTH_IMAGE);
      l515DepthSubscription.addCallback(this::logDepthL515);
      runnablesToStopLogging.addLast(l515DepthSubscription::destroy);
      byteBuffers.put(PerceptionLoggerConstants.L515_DEPTH_NAME, new byte[BUFFER_SIZE]);
      counts.put(PerceptionLoggerConstants.L515_DEPTH_NAME, 0);

      // Add callback for L515 Color Images
      var l515ColorSubscription = ros2Helper.subscribe(ROS2Tools.L515_COLOR_IMAGE);
      l515ColorSubscription.addCallback(this::logColorL515);
      runnablesToStopLogging.addLast(l515ColorSubscription::destroy);
      byteBuffers.put(PerceptionLoggerConstants.L515_COLOR_NAME, new byte[BUFFER_SIZE]);
      counts.put(PerceptionLoggerConstants.L515_COLOR_NAME, 0);

      // Add callback for D435 Color images
      var zed2StereoSubscription = ros2Helper.subscribe(ROS2Tools.ZED2_STEREO_COLOR);
      zed2StereoSubscription.addCallback(this::logColorZED2);
      runnablesToStopLogging.addLast(zed2StereoSubscription::destroy);
      byteBuffers.put(PerceptionLoggerConstants.ZED2_COLOR_NAME, new byte[BUFFER_SIZE]);
      counts.put(PerceptionLoggerConstants.ZED2_COLOR_NAME, 0);

      // Add callback for Ouster depth maps
      var ousterDepthSubscription = ros2Helper.subscribe(ROS2Tools.OUSTER_DEPTH);
      ousterDepthSubscription.addCallback(this::logDepthOuster);
      runnablesToStopLogging.addLast(ousterDepthSubscription::destroy);

      realtimeROS2Node.spin();

      runnablesToStopLogging.addLast(realtimeROS2Node::destroy);

      executorService.scheduleAtFixedRate(this::collectStatistics, 0, 10, TimeUnit.MILLISECONDS);
   }

   public void stopLogging()
   {
      while (!runnablesToStopLogging.isEmpty())
      {
         runnablesToStopLogging.pollFirst().run();
      }

      for (PerceptionLogChannel channel : channels.values())
      {
         channel.resetIndex();
         channel.resetCount();
      }

      hdf5Manager.closeFile();
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

      if (channels.get(PerceptionLoggerConstants.ROBOT_CONFIGURATION_DATA_NAME).isEnabled())
      {
         storeLongArray(PerceptionLoggerConstants.ROBOT_CONFIGURATION_DATA_MONOTONIC_TIME, data.getMonotonicTime());
         storeFloatArray(PerceptionLoggerConstants.ROOT_POSITION_NAME, data.getRootPosition());
         storeFloatArray(PerceptionLoggerConstants.ROOT_ORIENTATION_NAME, data.getRootOrientation());
         storeFloatArray(PerceptionLoggerConstants.JOINT_ANGLES_NAME, data.getJointAngles().toArray());
         storeFloatArray(PerceptionLoggerConstants.JOINT_VELOCITIES_NAME, data.getJointVelocities().toArray());
         storeFloatArray(PerceptionLoggerConstants.JOINT_TORQUES_NAME, data.getJointTorques().toArray());
      }
   }

   public void logLidarScanMessage(FusedSensorHeadPointCloudMessage message)
   {
      LogTools.info("OUSTER POINT CLOUD Received.");

      ousterCloudPacket.set(message);
      if (channels.get(PerceptionLoggerConstants.OUSTER_CLOUD_NAME).isEnabled())
      {
         channels.get(PerceptionLoggerConstants.OUSTER_CLOUD_NAME).incrementCount();
         storePointCloud(PerceptionLoggerConstants.OUSTER_CLOUD_NAME, ousterCloudPacket);
      }
   }

   public void logDepthOuster(VideoPacket packet)
   {
      LogTools.info("Depth Map Received: {}", packet.data_);

      if (channels.get(PerceptionLoggerConstants.OUSTER_DEPTH_NAME).isEnabled())
      {
         channels.get(PerceptionLoggerConstants.OUSTER_DEPTH_NAME).incrementCount();
         storeCompressedImage(PerceptionLoggerConstants.OUSTER_DEPTH_NAME, packet);
      }
   }

   public void logColorD435(VideoPacket videoPacket)
   {
      LogTools.info("Logging D435 Color: ", videoPacket.toString());

      if (channels.get(PerceptionLoggerConstants.D435_COLOR_NAME).isEnabled())
      {
         channels.get(PerceptionLoggerConstants.D435_COLOR_NAME).incrementCount();
         storeCompressedImage(PerceptionLoggerConstants.D435_COLOR_NAME, videoPacket);
      }
   }

   public void logDepthD435(VideoPacket videoPacket)
   {
      LogTools.info("Logging D435 Depth: ", videoPacket.toString());

      if (channels.get(PerceptionLoggerConstants.D435_DEPTH_NAME).isEnabled())
      {
         channels.get(PerceptionLoggerConstants.D435_DEPTH_NAME).incrementCount();
         storeCompressedImage(PerceptionLoggerConstants.D435_DEPTH_NAME, videoPacket);
      }
   }

   public void logDepthL515(ImageMessage message)
   {
      LogTools.info("Logging L515 Depth: ", message.toString());

      if (channels.get(PerceptionLoggerConstants.L515_DEPTH_NAME).isEnabled())
      {
         channels.get(PerceptionLoggerConstants.L515_DEPTH_NAME).incrementCount();
         storeFloatArray(PerceptionLoggerConstants.L515_SENSOR_POSITION, message.getPosition());
         storeFloatArray(PerceptionLoggerConstants.L515_SENSOR_ORIENTATION, message.getOrientation());
         storeCompressedImage(PerceptionLoggerConstants.L515_DEPTH_NAME, message);
      }
   }

   public void logColorL515(ImageMessage videoPacket)
   {
      LogTools.info("Logging L515 Color: ", videoPacket.toString());

      if (channels.get(PerceptionLoggerConstants.L515_COLOR_NAME).isEnabled())
      {
         channels.get(PerceptionLoggerConstants.L515_COLOR_NAME).incrementCount();
         storeCompressedImage(PerceptionLoggerConstants.L515_COLOR_NAME, videoPacket);
      }
   }

   public void logColorZED2(VideoPacket videoPacket)
   {
      LogTools.info("Logging L515 Color: ", videoPacket.toString());

      if (channels.get(PerceptionLoggerConstants.ZED2_COLOR_NAME).isEnabled())
      {
         channels.get(PerceptionLoggerConstants.ZED2_COLOR_NAME).incrementCount();
         storeCompressedImage(PerceptionLoggerConstants.ZED2_COLOR_NAME, videoPacket);
      }

      //BytedecoOpenCVTools.displayVideoPacketColor(videoPacket);
   }

   /*
    *  Store methods which actually deploy threads and call HDF5 specific functions for storing compressed data.
    */
   public void storeCompressedImage(String namespace, VideoPacket packet)
   {
      executorService.submit(() ->
                             {
                                Group group = hdf5Manager.getGroup(namespace);

                                byte[] heapArray = byteBuffers.get(namespace);
                                int imageCount = counts.get(namespace);
                                IDLSequence.Byte imageEncodedTByteArrayList = packet.getData();

                                LogTools.info("{} Storing Buffer: {}", namespace, imageCount);
                                counts.put(namespace, imageCount + 1);

                                imageEncodedTByteArrayList.toArray(heapArray, 0, packet.getData().size() + 4);
                                HDF5Tools.storeByteArray(group, imageCount, heapArray, imageEncodedTByteArrayList.size() + 4);

                                LogTools.info("{} Done Storing Buffer: {}", namespace, imageCount);
                             });
   }

   public void storeCompressedImage(String namespace, ImageMessage packet)
   {
      long begin_store = System.nanoTime();
      Group group = hdf5Manager.getGroup(namespace);
      executorService.submit(() ->
                             {

                                byte[] heapArray = byteBuffers.get(namespace);
                                int imageCount = counts.get(namespace);
                                IDLSequence.Byte imageEncodedTByteArrayList = packet.getData();

                                LogTools.info("{} Storing Buffer: {}", namespace, imageCount);
                                counts.put(namespace, imageCount + 1);

                                imageEncodedTByteArrayList.toArray(heapArray, 0, packet.getData().size() + 4);
                                HDF5Tools.storeByteArray(group, imageCount, heapArray, imageEncodedTByteArrayList.size() + 4);

                                LogTools.info("{} Done Storing Buffer: {}", namespace, imageCount);
                             });
      long end_store = System.nanoTime();
   }

   public void storePointCloud(String namespace, LidarScanMessage message)
   {
      Group group = hdf5Manager.getGroup(namespace);

      executorService.submit(() ->
                             {
                                synchronized (this)
                                {
                                   LogTools.info("{} Storing Buffer: {}", namespace, pointCloudCount);
                                   pointCloudCount = (int) hdf5Manager.getCount(namespace);
                                   HDF5Tools.storeByteArray(group, pointCloudCount, message.getScan().toArray(), message.getScan().size());
                                   LogTools.info("{} Done Storing Buffer: {}", namespace, pointCloudCount);

                                   //                                         pointCloudCount++;
                                }
                             });
   }

   public void storePointCloud(String namespace, FusedSensorHeadPointCloudMessage message)
   {
      Group group = hdf5Manager.getGroup(namespace);

      executorService.submit(() ->
                             {
                                synchronized (this)
                                {
                                   LogTools.info("{} Storing Buffer: {}", namespace, pointCloudCount);
                                   pointCloudCount = (int) hdf5Manager.getCount(namespace);
                                   HDF5Tools.storeByteArray(group, pointCloudCount, message.getScan().toArray(), message.getScan().size());
                                   LogTools.info("{} Done Storing Buffer: {}", namespace, pointCloudCount);
                                }
                             });
   }

   public void storeFloatArray(String namespace, float[] array)
   {
      executorService.submit(() ->
                             {
                                Group group = hdf5Manager.getGroup(namespace);
                                TFloatArrayList buffer = hdf5Manager.getFloatBuffer(namespace);
                                buffer.addAll(array);

                                int bufferSize = hdf5Manager.getBufferIndex(namespace) / array.length;
                                if (bufferSize == (HDF5Manager.MAX_BUFFER_SIZE - 1))
                                {
                                   long count = hdf5Manager.getCount(namespace);
                                   HDF5Tools.storeFloatArray2D(group, count, buffer, HDF5Manager.MAX_BUFFER_SIZE, array.length);
                                   hdf5Manager.resetBuffer(namespace);
                                }
                             });
   }

   public void storeLongArray(String namespace, long[] array)
   {
      Group group = hdf5Manager.getGroup(namespace);
      TLongArrayList buffer = hdf5Manager.getLongBuffer(namespace);
      buffer.addAll(array);

      int bufferSize = hdf5Manager.getBufferIndex(namespace) / array.length;
      //LogTools.info("Buffer Index: {} {}", bufferSize, HDF5Manager.MAX_BUFFER_SIZE - 1);
      if (bufferSize == (HDF5Manager.MAX_BUFFER_SIZE - 1))
      {
         hdf5Manager.resetBuffer(namespace);

         executorService.submit(() ->
                                {
                                   synchronized (this)
                                   {
                                      long count = hdf5Manager.getCount(namespace);
                                      HDF5Tools.storeLongArray2D(group, count, buffer, HDF5Manager.MAX_BUFFER_SIZE, array.length);
                                   }
                                });
      }
   }

   public void storeLongArray(String namespace, long value)
   {
      float[] pointArray = new float[1];
      pointArray[0] = value;
      storeFloatArray(namespace, pointArray);
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

   public HashMap<String, PerceptionLogChannel> getChannels()
   {
      return channels;
   }

   public void setChannelEnabled(String name, boolean enabled)
   {
      channels.get(name).setEnabled(enabled);
   }

   public static void main(String[] args)
   {
      SimpleDateFormat dateFormat = new SimpleDateFormat("yyyyMMdd_HHmmss");

      String defaultLogDirectory =
            System.getProperty("user.home") + File.separator + ".ihmc" + File.separator + "logs" + File.separator + "perception" + File.separator;
      String logDirectory = System.getProperty("perception.log.directory", defaultLogDirectory);
      String logFileName = dateFormat.format(new Date()) + "_" + "PerceptionLog.hdf5";

      PerceptionDataLogger logger = new PerceptionDataLogger();

      //logger.setChannelEnabled(logger.PerceptionLoggerConstants.ROBOT_CONFIGURATION_DATA_NAME, true);
      logger.setChannelEnabled(PerceptionLoggerConstants.L515_DEPTH_NAME, true);
      //      logger.setChannelEnabled(PerceptionLoggerConstants.L515_COLOR_NAME, true);

      logger.startLogging(logDirectory + logFileName, "Nadia");
   }
}


