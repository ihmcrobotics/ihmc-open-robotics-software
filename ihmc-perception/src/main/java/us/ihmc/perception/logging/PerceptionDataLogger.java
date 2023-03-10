package us.ihmc.perception.logging;

import controller_msgs.msg.dds.RobotConfigurationData;
import org.bytedeco.hdf5.Group;
import org.bytedeco.hdf5.global.hdf5;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.FloatPointer;
import org.bytedeco.javacpp.LongPointer;
import perception_msgs.msg.dds.*;
import us.ihmc.commons.Conversions;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.property.ROS2StoredPropertySetGroup;
import us.ihmc.communication.ros2.ROS2Helper;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.idl.IDLSequence;
import us.ihmc.log.LogTools;
import us.ihmc.perception.parameters.PerceptionConfigurationParameters;
import us.ihmc.perception.tools.PerceptionMessageTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.common.SampleInfo;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.tools.IHMCCommonPaths;
import us.ihmc.tools.thread.ExecutorServiceTools;

import java.io.File;
import java.nio.file.Paths;
import java.text.SimpleDateFormat;
import java.util.*;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;
import java.util.concurrent.atomic.AtomicReference;

/**
 * Architecture diagram:
 * <p></p>
 * <img src="https://www.ihmc.us/wp-content/uploads/2023/02/perceptionLoggerArchitecture-1024x754.png" width="800">
 */
public class PerceptionDataLogger
{
   private static final int BUFFER_SIZE = 1000000;

   private ROS2StoredPropertySetGroup ros2PropertySetGroup;
   private PerceptionConfigurationParameters parameters;

   private final HashMap<String, byte[]> byteArrays = new HashMap<>();
   private final HashMap<String, BytePointer> bytePointers = new HashMap<>();
   private final HashMap<String, LongPointer> longPointers = new HashMap<>();
   private final HashMap<String, FloatPointer> floatPointers = new HashMap<>();
   private final HashMap<String, Integer> counts = new HashMap<>();

   private final CommunicationMode communicationMode;
   private final ArrayDeque<Runnable> runnablesToStopLogging = new ArrayDeque<>();
   private final ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(1,
                                                                                                        getClass(),
                                                                                                        ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);
   private ROS2Node ros2Node;
   private ROS2Helper ros2Helper;
   private HDF5Manager hdf5Manager;
   private final HDF5Tools hdf5Tools = new HDF5Tools();
   private RealtimeROS2Node realtimeROS2Node;
   private String filePath;

   private AtomicReference<Boolean> stopLoggingRequest = new AtomicReference<>(false);

   private HashMap<String, PerceptionLogChannel> channels = new HashMap<>();
   private HashMap<String, AtomicReference<ImageMessage>> imageMessageReferences = new HashMap<>();
   private HashMap<String, AtomicReference<Pose3D>> transformMessageReferences = new HashMap<>();

   public PerceptionDataLogger()
   {
//      ros2PropertySetGroup = new ROS2StoredPropertySetGroup(ros2Helper);
//      ros2PropertySetGroup.registerStoredPropertySet(PerceptionComms.PERCEPTION_CONFIGURATION_PARAMETERS, parameters);

      channels.put(PerceptionLoggerConstants.D435_COLOR_NAME, new PerceptionLogChannel(PerceptionLoggerConstants.D435_COLOR_NAME, 0, 0));
      imageMessageReferences.put(PerceptionLoggerConstants.D435_COLOR_NAME, new AtomicReference<>(null));

      channels.put(PerceptionLoggerConstants.D435_DEPTH_NAME, new PerceptionLogChannel(PerceptionLoggerConstants.D435_DEPTH_NAME, 0, 0));
      imageMessageReferences.put(PerceptionLoggerConstants.D435_DEPTH_NAME, new AtomicReference<>(null));

      channels.put(PerceptionLoggerConstants.L515_COLOR_NAME, new PerceptionLogChannel(PerceptionLoggerConstants.L515_COLOR_NAME, 0, 0));
      imageMessageReferences.put(PerceptionLoggerConstants.L515_COLOR_NAME, new AtomicReference<>(null));

      channels.put(PerceptionLoggerConstants.L515_DEPTH_NAME, new PerceptionLogChannel(PerceptionLoggerConstants.L515_DEPTH_NAME, 0, 0));
      imageMessageReferences.put(PerceptionLoggerConstants.L515_DEPTH_NAME, new AtomicReference<>(null));

      channels.put(PerceptionLoggerConstants.OUSTER_DEPTH_NAME, new PerceptionLogChannel(PerceptionLoggerConstants.OUSTER_DEPTH_NAME, 0, 0));
      imageMessageReferences.put(PerceptionLoggerConstants.OUSTER_DEPTH_NAME, new AtomicReference<>(null));

      channels.put(PerceptionLoggerConstants.BLACKFLY_COLOR_NAME, new PerceptionLogChannel(PerceptionLoggerConstants.BLACKFLY_COLOR_NAME, 0, 0));
      imageMessageReferences.put(PerceptionLoggerConstants.BLACKFLY_COLOR_NAME, new AtomicReference<>(null));

      channels.put(PerceptionLoggerConstants.ZED2_COLOR_NAME, new PerceptionLogChannel(PerceptionLoggerConstants.ZED2_COLOR_NAME, 0, 0));
      imageMessageReferences.put(PerceptionLoggerConstants.ZED2_COLOR_NAME, new AtomicReference<>(null));

      channels.put(PerceptionLoggerConstants.MOCAP_RIGID_BODY_POSITION, new PerceptionLogChannel(PerceptionLoggerConstants.MOCAP_RIGID_BODY_POSITION, 0, 0));
      transformMessageReferences.put(PerceptionLoggerConstants.MOCAP_RIGID_BODY_POSITION, new AtomicReference<>(null));

      channels.put(PerceptionLoggerConstants.ROBOT_CONFIGURATION_DATA_NAME,
                   new PerceptionLogChannel(PerceptionLoggerConstants.ROBOT_CONFIGURATION_DATA_NAME, 0, 0));

      communicationMode = CommunicationMode.INTERPROCESS;
   }

   public void addChannel(String channelName)
   {
      channels.put(channelName, new PerceptionLogChannel(channelName, 0, 0));
      counts.put(channelName, 0);
   }

   public void openLogFile(String logFileName)
   {
      this.filePath = logFileName;
      File logFile = new File(logFileName);
      if (!logFile.exists() && !logFile.isDirectory())
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
      if (hdf5Manager != null)
      {
         hdf5Manager.closeFile();
         LogTools.info("HDF5 File Saved: {}", filePath);
      }
   }

   public void startLogging(String logFileName, String simpleRobotName)
   {
      openLogFile(logFileName);

      // Use both regular and real-time ROS2 nodes to assign callbacks to different message types
      ros2Node = ROS2Tools.createROS2Node(communicationMode.getPubSubImplementation(), "perception_logger_node");
      ros2Helper = new ROS2Helper(ros2Node);

      realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "perception_logger_realtime_node");

      // Add callback for Robot Configuration Data
      if (channels.get(PerceptionLoggerConstants.ROBOT_CONFIGURATION_DATA_NAME).isEnabled())
      {
         var robotConfigurationDataSubscription = ros2Helper.subscribe(ROS2Tools.getRobotConfigurationDataTopic(simpleRobotName));
         robotConfigurationDataSubscription.addCallback(this::logRobotConfigurationData);
         runnablesToStopLogging.addLast(robotConfigurationDataSubscription::destroy);
      }

      // Add callback for D435 Color images
      if (channels.get(PerceptionLoggerConstants.D435_COLOR_NAME).isEnabled())
      {
         byteArrays.put(PerceptionLoggerConstants.D435_COLOR_NAME, new byte[BUFFER_SIZE]);
         bytePointers.put(PerceptionLoggerConstants.D435_COLOR_NAME, new BytePointer(BUFFER_SIZE));
         counts.put(PerceptionLoggerConstants.D435_COLOR_NAME, 0);
         var d435VideoSubscription = ros2Helper.subscribe(ROS2Tools.D435_COLOR_IMAGE);
         d435VideoSubscription.addCallback(this::logColorD435);
         runnablesToStopLogging.addLast(d435VideoSubscription::destroy);
      }

      // Add callback for D435 Depth images
      if (channels.get(PerceptionLoggerConstants.D435_DEPTH_NAME).isEnabled())
      {
         byteArrays.put(PerceptionLoggerConstants.D435_DEPTH_NAME, new byte[BUFFER_SIZE]);
         bytePointers.put(PerceptionLoggerConstants.D435_DEPTH_NAME, new BytePointer(BUFFER_SIZE));
         counts.put(PerceptionLoggerConstants.D435_DEPTH_NAME, 0);
         var d435DepthSubscription = ros2Helper.subscribe(ROS2Tools.D435_DEPTH_IMAGE);
         d435DepthSubscription.addCallback(this::logDepthD435);
         runnablesToStopLogging.addLast(d435DepthSubscription::destroy);
      }

      // Add callback for L515 Depth Maps
      if (channels.get(PerceptionLoggerConstants.L515_DEPTH_NAME).isEnabled())
      {
         byteArrays.put(PerceptionLoggerConstants.L515_DEPTH_NAME, new byte[BUFFER_SIZE]);
         bytePointers.put(PerceptionLoggerConstants.L515_DEPTH_NAME, new BytePointer(BUFFER_SIZE));
         counts.put(PerceptionLoggerConstants.L515_DEPTH_NAME, 0);
         var l515DepthSubscription = ros2Helper.subscribe(ROS2Tools.L515_DEPTH_IMAGE);
         l515DepthSubscription.addCallback(this::logDepthL515);
         runnablesToStopLogging.addLast(l515DepthSubscription::destroy);
      }

      // Add callback for L515 Color Images
      if (channels.get(PerceptionLoggerConstants.L515_COLOR_NAME).isEnabled())
      {
         byteArrays.put(PerceptionLoggerConstants.L515_COLOR_NAME, new byte[BUFFER_SIZE]);
         bytePointers.put(PerceptionLoggerConstants.L515_COLOR_NAME, new BytePointer(BUFFER_SIZE));
         counts.put(PerceptionLoggerConstants.L515_COLOR_NAME, 0);
         var l515ColorSubscription = ros2Helper.subscribe(ROS2Tools.L515_COLOR_IMAGE);
         l515ColorSubscription.addCallback(this::logColorL515);
         runnablesToStopLogging.addLast(l515ColorSubscription::destroy);
      }

      // Add callback for D435 Color images
      if (channels.get(PerceptionLoggerConstants.ZED2_COLOR_NAME).isEnabled())
      {
         byteArrays.put(PerceptionLoggerConstants.ZED2_COLOR_NAME, new byte[BUFFER_SIZE]);
         bytePointers.put(PerceptionLoggerConstants.ZED2_COLOR_NAME, new BytePointer(BUFFER_SIZE));
         counts.put(PerceptionLoggerConstants.ZED2_COLOR_NAME, 0);
         var zed2StereoSubscription = ros2Helper.subscribe(ROS2Tools.ZED2_STEREO_COLOR);
         zed2StereoSubscription.addCallback(this::logColorZED2);
         runnablesToStopLogging.addLast(zed2StereoSubscription::destroy);
      }

      // Add callback for Ouster depth maps
      if (channels.get(PerceptionLoggerConstants.OUSTER_DEPTH_NAME).isEnabled())
      {
         SampleInfo sampleInfo = new SampleInfo();
         byteArrays.put(PerceptionLoggerConstants.OUSTER_DEPTH_NAME, new byte[BUFFER_SIZE]);
         bytePointers.put(PerceptionLoggerConstants.OUSTER_DEPTH_NAME, new BytePointer(BUFFER_SIZE));
         counts.put(PerceptionLoggerConstants.OUSTER_DEPTH_NAME, 0);
         ROS2Tools.createCallbackSubscription(realtimeROS2Node, ROS2Tools.OUSTER_DEPTH_IMAGE, ROS2QosProfile.BEST_EFFORT(), (subscriber) ->
         {
            LogTools.info("Depth Map Received");

            ImageMessage imageMessage = new ImageMessage();
            subscriber.takeNextData(imageMessage, sampleInfo);
            imageMessageReferences.get(PerceptionLoggerConstants.OUSTER_DEPTH_NAME).set(imageMessage);
            logDepthOuster(imageMessageReferences.get(PerceptionLoggerConstants.OUSTER_DEPTH_NAME).getAndSet(null));
         });
      }

      // TODO: Make this work with the new ImageMessage message for Blackfly images
      // Add callback for Blackfly Color images
//      LogTools.info("Blackfly Color Enabled: " + channels.get(PerceptionLoggerConstants.BLACKFLY_COLOR_NAME).isEnabled());
//      if (channels.get(PerceptionLoggerConstants.BLACKFLY_COLOR_NAME).isEnabled())
//      {
//         SampleInfo sampleInfo = new SampleInfo();
//         byteArrays.put(PerceptionLoggerConstants.BLACKFLY_COLOR_NAME, new byte[BUFFER_SIZE]);
//         bytePointers.put(PerceptionLoggerConstants.BLACKFLY_COLOR_NAME, new BytePointer(BUFFER_SIZE));
//         counts.put(PerceptionLoggerConstants.BLACKFLY_COLOR_NAME, 0);
//         ROS2Tools.createCallbackSubscription(realtimeROS2Node, ROS2Tools.BLACKFLY_FISHEYE_COLOR_IMAGE.get(RobotSide.RIGHT), ROS2QosProfile.BEST_EFFORT(), (subscriber) ->
//         {
//            subscriber.takeNextData(imageMessage, sampleInfo);
//            imageMessageReferences.get(PerceptionLoggerConstants.BLACKFLY_COLOR_NAME).set(imageMessage);
//            logColorBlackfly(bigVideoPacketReferences.get(PerceptionLoggerConstants.BLACKFLY_COLOR_NAME).getAndSet(null));
//
//            LogTools.info("BlackFly Image Received: {}", imageMessage.getAcquisitionTimeSecondsSinceEpoch());
//         });
//      }

      // Add callback for MoCap data
      LogTools.info("MoCap Logging Enabled: " + channels.get(PerceptionLoggerConstants.MOCAP_RIGID_BODY_POSITION).isEnabled());
      if (channels.get(PerceptionLoggerConstants.MOCAP_RIGID_BODY_POSITION).isEnabled())
      {
         counts.put(PerceptionLoggerConstants.MOCAP_RIGID_BODY_POSITION, 0);
         ROS2Tools.createCallbackSubscription(realtimeROS2Node, ROS2Tools.MOCAP_RIGID_BODY, ROS2QosProfile.BEST_EFFORT(), (subscriber) ->
         {
            Pose3D transformMessage = new Pose3D();
            subscriber.takeNextData(transformMessage, new SampleInfo());

            transformMessageReferences.get(PerceptionLoggerConstants.MOCAP_RIGID_BODY_POSITION).set(transformMessage);
            logMocapRigidBody(transformMessageReferences.get(PerceptionLoggerConstants.MOCAP_RIGID_BODY_POSITION).getAndSet(null));

            LogTools.info("Mocap Rigid Body Received: {} {} {}", transformMessage.getX(), transformMessage.getY(), transformMessage.getZ());
         });
      }

      realtimeROS2Node.spin();

      runnablesToStopLogging.addLast(realtimeROS2Node::destroy);

      executorService.scheduleAtFixedRate(this::collectStatistics, 0, 10, TimeUnit.MILLISECONDS);
   }

   public void stopLogging()
   {
      stopLoggingRequest.set(true);

      for (PerceptionLogChannel channel : channels.values())
      {
         if (channel.isEnabled())
         {
            boolean termination = false;
            try
            {
               termination = executorService.awaitTermination(500, TimeUnit.MILLISECONDS);
            }
            catch (InterruptedException e)
            {
               throw new RuntimeException(e);
            }

            LogTools.warn("Perception Logger: Thread Terminated [{}]: {}", channel.getName(), termination);
         }

         channel.resetIndex();
         channel.resetCount();

         while (!runnablesToStopLogging.isEmpty())
         {
            runnablesToStopLogging.pollFirst().run();
         }
      }

      closeLogFile();
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
         channels.get(PerceptionLoggerConstants.ROBOT_CONFIGURATION_DATA_NAME).incrementCount();

         storeLongs(PerceptionLoggerConstants.ROBOT_CONFIGURATION_DATA_MONOTONIC_TIME, data.getMonotonicTime());
         storeFloats(PerceptionLoggerConstants.ROOT_POSITION_NAME, data.getRootPosition());
         storeFloats(PerceptionLoggerConstants.ROOT_ORIENTATION_NAME, data.getRootOrientation());
         storeFloats(PerceptionLoggerConstants.JOINT_ANGLES_NAME, data.getJointAngles());
         storeFloats(PerceptionLoggerConstants.JOINT_VELOCITIES_NAME, data.getJointVelocities());
         storeFloats(PerceptionLoggerConstants.JOINT_TORQUES_NAME, data.getJointTorques());
      }
   }

   public void logDepthOuster(ImageMessage message)
   {
      LogTools.info("Depth Map Received: {}", message.getAcquisitionTime());

      if (channels.get(PerceptionLoggerConstants.OUSTER_DEPTH_NAME).isEnabled())
      {
         channels.get(PerceptionLoggerConstants.OUSTER_DEPTH_NAME).incrementCount();
         long timestamp =
               Conversions.secondsToNanoseconds(message.getAcquisitionTime().getSecondsSinceEpoch()) + message.getAcquisitionTime().getAdditionalNanos();
         storeLongs(PerceptionLoggerConstants.OUSTER_SENSOR_TIME, timestamp);
         storeFloats(PerceptionLoggerConstants.OUSTER_SENSOR_POSITION, message.getPosition());
         storeFloats(PerceptionLoggerConstants.OUSTER_SENSOR_ORIENTATION, message.getOrientation());
         storeCompressedImage(PerceptionLoggerConstants.OUSTER_DEPTH_NAME, message);
      }
   }

   public void logDepthL515(ImageMessage message)
   {
      LogTools.info("Logging L515 Depth: {} {}", message.getAcquisitionTime().getSecondsSinceEpoch(), message.getAcquisitionTime().getAdditionalNanos());

      if (channels.get(PerceptionLoggerConstants.L515_DEPTH_NAME).isEnabled())
      {
         channels.get(PerceptionLoggerConstants.L515_DEPTH_NAME).incrementCount();
         long timestamp =
               Conversions.secondsToNanoseconds(message.getAcquisitionTime().getSecondsSinceEpoch()) + message.getAcquisitionTime().getAdditionalNanos();
         storeLongs(PerceptionLoggerConstants.L515_SENSOR_TIME, timestamp);
         storeFloats(PerceptionLoggerConstants.L515_SENSOR_POSITION, message.getPosition());
         storeFloats(PerceptionLoggerConstants.L515_SENSOR_ORIENTATION, message.getOrientation());
         storeCompressedImage(PerceptionLoggerConstants.L515_DEPTH_NAME, message);
      }
   }

   public void logColorL515(ImageMessage message)
   {
      LogTools.info("Logging L515 Color: ", message.toString());

      if (channels.get(PerceptionLoggerConstants.L515_COLOR_NAME).isEnabled())
      {
         channels.get(PerceptionLoggerConstants.L515_COLOR_NAME).incrementCount();
         long timestamp =
               Conversions.secondsToNanoseconds(message.getAcquisitionTime().getSecondsSinceEpoch()) + message.getAcquisitionTime().getAdditionalNanos();
         storeCompressedImage(PerceptionLoggerConstants.L515_COLOR_NAME, message);
      }
   }

   public void logDepthD435(ImageMessage message)
   {
      LogTools.info("Logging D435 Depth: ", message.toString());

      if (channels.get(PerceptionLoggerConstants.D435_DEPTH_NAME).isEnabled())
      {
         channels.get(PerceptionLoggerConstants.D435_DEPTH_NAME).incrementCount();
         long timestamp =
               Conversions.secondsToNanoseconds(message.getAcquisitionTime().getSecondsSinceEpoch()) + message.getAcquisitionTime().getAdditionalNanos();
         storeLongs(PerceptionLoggerConstants.D435_SENSOR_TIME, timestamp);
         storeFloats(PerceptionLoggerConstants.D435_SENSOR_POSITION, message.getPosition());
         storeFloats(PerceptionLoggerConstants.D435_SENSOR_ORIENTATION, message.getOrientation());
         storeCompressedImage(PerceptionLoggerConstants.D435_DEPTH_NAME, message);
      }
   }

   public void logColorD435(ImageMessage message)
   {
      LogTools.info("Logging D435 Color: ", message.toString());

      if (channels.get(PerceptionLoggerConstants.D435_COLOR_NAME).isEnabled())
      {
         channels.get(PerceptionLoggerConstants.D435_COLOR_NAME).incrementCount();
         long timestamp =
               Conversions.secondsToNanoseconds(message.getAcquisitionTime().getSecondsSinceEpoch()) + message.getAcquisitionTime().getAdditionalNanos();
         storeCompressedImage(PerceptionLoggerConstants.D435_COLOR_NAME, message);
      }
   }

   public void logColorZED2(ImageMessage message)
   {
      LogTools.info("Logging ZED2 Color: ", message.toString());

      if (channels.get(PerceptionLoggerConstants.ZED2_COLOR_NAME).isEnabled())
      {
         channels.get(PerceptionLoggerConstants.ZED2_COLOR_NAME).incrementCount();
         long timestamp =
               Conversions.secondsToNanoseconds(message.getAcquisitionTime().getSecondsSinceEpoch()) + message.getAcquisitionTime().getAdditionalNanos();
         storeLongs(PerceptionLoggerConstants.ZED2_SENSOR_TIME, timestamp);
         storeCompressedImage(PerceptionLoggerConstants.ZED2_COLOR_NAME, message);
      }
   }

   public void logMocapRigidBody(Pose3D pose)
   {
      LogTools.info("Logging Mocap Rigid Body");

      if (channels.get(PerceptionLoggerConstants.MOCAP_RIGID_BODY_POSITION).isEnabled())
      {
         channels.get(PerceptionLoggerConstants.MOCAP_RIGID_BODY_POSITION).incrementCount();

         RigidBodyTransform transform = new RigidBodyTransform();
         pose.set(transform);
         Quaternion orientation = new Quaternion(transform.getRotation());
         storeFloats(PerceptionLoggerConstants.MOCAP_RIGID_BODY_POSITION, new Point3D(transform.getTranslation()));
         storeFloats(PerceptionLoggerConstants.MOCAP_RIGID_BODY_ORIENTATION, orientation);
      }
   }

   public void storeCompressedImage(String namespace, ImageMessage packet)
   {
      LogTools.info("Storing Compressed Image: {}", namespace);

      byte[] heapArray = byteArrays.get(namespace);
      IDLSequence.Byte imageEncodedTByteArrayList = packet.getData();
      imageEncodedTByteArrayList.toArray(heapArray, 0, packet.getData().size());

      BytePointer bytePointer = bytePointers.get(namespace);
      bytePointer.put(heapArray, 0, packet.getData().size());
      bytePointer.limit(packet.getData().size());
      storeBytesFromPointer(namespace, bytePointer);

      if (stopLoggingRequest.get())
      {
         channels.get(namespace).setEnabled(false);
      }
   }

   public void storeBytesFromPointer(String namespace, BytePointer bytePointer)
   {
      executorService.submit(() ->
     {
        synchronized (hdf5Manager)
        {
           Group group = hdf5Manager.getGroup(namespace);

           int imageCount = counts.get(namespace);
           counts.put(namespace, imageCount + 1);

           hdf5Tools.storeBytes(group, imageCount, bytePointer);
        }
     });
   }

   public void storePointCloud(String namespace, LidarScanMessage message)
   {
      executorService.submit(() ->
      {
         synchronized (hdf5Manager)
         {
            Group group = hdf5Manager.getGroup(namespace);
            int index = hdf5Manager.getCount(namespace);
            hdf5Tools.storeByteArray(group, index, message.getScan().toArray(), message.getScan().size());

            if (stopLoggingRequest.get())
            {
               channels.get(namespace).setEnabled(false);
            }
         }
      });
   }

   public void storeFloatsFromPointer(String namespace, FloatPointer floatPointer, int columns)
   {
      executorService.submit(() ->
      {
         synchronized (hdf5Manager)
         {
            Group group = hdf5Manager.getGroup(namespace);

            int count = counts.get(namespace);
            counts.put(namespace, count + 1);

            hdf5Tools.storeFloatArray2D(group, count, floatPointer, HDF5Manager.MAX_BUFFER_SIZE, columns);
            floatPointer.position(0);
            floatPointer.limit(0);
         }
      });
   }

   public void storeLongsFromPointer(String namespace, LongPointer longPointer, int columns)
   {
      executorService.submit(() ->
      {
         synchronized (hdf5Manager)
         {
            Group group = hdf5Manager.getGroup(namespace);

            int count = counts.get(namespace);
            counts.put(namespace, count + 1);

            hdf5Tools.storeLongArray2D(group, count, longPointer, HDF5Manager.MAX_BUFFER_SIZE, columns);
            longPointer.position(0);
            longPointer.limit(0);
         }
      });
   }

   public void storeLongs(String namespace, long value)
   {
      LongPointer pointer = longPointers.get(namespace);
      pointer.limit(pointer.limit() + 1);
      pointer.put(pointer.limit(), value);

      if (pointer.limit() == PerceptionLoggerConstants.MAX_BUFFER_SIZE)
      {
         storeLongsFromPointer(namespace, pointer, 1);
      }
   }

   public void storeFloats(String namespace, float value)
   {
      FloatPointer pointer = floatPointers.get(namespace);
      pointer.limit(pointer.limit() + 1);
      pointer.put(pointer.limit(), value);

      if (pointer.limit() == PerceptionLoggerConstants.MAX_BUFFER_SIZE)
      {
         storeFloatsFromPointer(namespace, pointer, 1);
      }
   }

   public void storeFloats(String namespace, Point3D point)
   {
      FloatPointer pointer = floatPointers.get(namespace);
      int startIndex = (int) pointer.limit();

      pointer.limit(pointer.limit() + 3);
      PerceptionMessageTools.copyToFloatPointer(point, pointer, startIndex);

      storeFloatsFromPointer(namespace, pointer, 3);
   }

   public void storeFloats(String namespace, Quaternion orientation)
   {
      FloatPointer pointer = floatPointers.get(namespace);
      int startIndex = (int) pointer.limit();

      pointer.limit(pointer.limit() + 4);
      PerceptionMessageTools.copyToFloatPointer(orientation, pointer, startIndex);

      storeFloatsFromPointer(namespace, pointer, 4);
   }

   public void storeFloats(String namespace, IDLSequence.Float floats)
   {
      FloatPointer pointer = floatPointers.get(namespace);
      int startIndex = (int) pointer.limit();

      pointer.limit(pointer.limit() + floats.size());
      PerceptionMessageTools.copyToFloatPointer(floats, pointer, startIndex);

      storeFloatsFromPointer(namespace, pointer, floats.size());
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

      String defaultLogDirectory = IHMCCommonPaths.PERCEPTION_LOGS_DIRECTORY.toString();
      String logDirectory = System.getProperty("perception.log.directory", defaultLogDirectory);
      String logFileName = dateFormat.format(new Date()) + "_" + "PerceptionLog.hdf5";

      PerceptionDataLogger logger = new PerceptionDataLogger();

      logger.setChannelEnabled(PerceptionLoggerConstants.ROBOT_CONFIGURATION_DATA_NAME, true);
      //      logger.setChannelEnabled(PerceptionLoggerConstants.L515_DEPTH_NAME, true);
      //      logger.setChannelEnabled(PerceptionLoggerConstants.L515_COLOR_NAME, true);

      //      logger.setChannelEnabled(PerceptionLoggerConstants.BLACKFLY_COLOR_NAME, true);
      //      logger.setChannelEnabled(PerceptionLoggerConstants.OUSTER_DEPTH_NAME, true);

      //      logger.setChannelEnabled(PerceptionLoggerConstants.MOCAP_RIGID_BODY_POSITION, true);

      logger.startLogging(Paths.get(logDirectory, logFileName).toString(), "Nadia");

      // TEST ROS2 node for Ouster Depth

      //      RealtimeROS2Node realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, StringTools.titleToSnakeCase("logger_realtime_ros2_node"));
      //
      //      ROS2Tools.createCallbackSubscription(realtimeROS2Node, ROS2Tools.OUSTER_DEPTH_IMAGE, ROS2QosProfile.BEST_EFFORT(), (message) -> {
      //         LogTools.info("Message Received: {}", message);
      //      });
      //
      //      ScheduledExecutorService executor = Executors.newSingleThreadScheduledExecutor();
      //      executor.scheduleAtFixedRate(realtimeROS2Node::spin, 0, 10, TimeUnit.MILLISECONDS);
   }
}


