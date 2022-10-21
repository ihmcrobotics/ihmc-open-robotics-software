package us.ihmc.rdx.logging;

import controller_msgs.msg.dds.RobotConfigurationData;
import org.apache.commons.lang.ArrayUtils;
import org.bytedeco.hdf5.Group;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import perception_msgs.msg.dds.BigVideoPacket;
import perception_msgs.msg.dds.FusedSensorHeadPointCloudMessage;
import perception_msgs.msg.dds.LidarScanMessage;
import us.ihmc.communication.CommunicationMode;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.idl.IDLSequence;
import us.ihmc.log.LogTools;
import us.ihmc.perception.HDF5Manager;
import us.ihmc.perception.HDF5Tools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.common.SampleInfo;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.RealtimeROS2Node;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.BackgroundExecutorManager;
import us.ihmc.tools.thread.ExecutorServiceTools;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.concurrent.ScheduledExecutorService;
import java.util.concurrent.TimeUnit;

import static org.bytedeco.hdf5.global.hdf5.H5F_ACC_RDWR;
import static org.bytedeco.hdf5.global.hdf5.H5F_ACC_TRUNC;

public class PerceptionDataLogger
{
   static final String FILE_NAME = "/home/quantum/Workspace/Data/Sensor_Logs/experimental.hdf5";
   private final HDF5Manager hdf5Manager;

   private final ROS2Node ros2Node;
   private final RealtimeROS2Node realtimeROS2Node;

   private final CommunicationMode communicationMode;

   private final byte[] messageDataHeapArray = new byte[25000000];
   private final BytePointer messageEncodedBytePointer = new BytePointer(25000000);
   private final Mat inputJPEGMat = new Mat(1, 1, opencv_core.CV_8UC1);
   private final Mat inputYUVI420Mat = new Mat(1, 1, opencv_core.CV_8UC1);
   private final Mat depthMap = new Mat(1, 1, opencv_core.CV_16UC1);

   private int pointCloudCount = 0;
   private int depthMapCount = 0;

   private final FusedSensorHeadPointCloudMessage ousterCloudPacket = new FusedSensorHeadPointCloudMessage();
   private LidarScanMessage lidarScanMessage = new LidarScanMessage();

   private final BigVideoPacket videoPacket = new BigVideoPacket();
   private final BigVideoPacket depthVideoPacket = new BigVideoPacket();

   private final SampleInfo ousterSampleInfo = new SampleInfo();
   private final SampleInfo depthSampleInfo = new SampleInfo();

   private ScheduledExecutorService executorService = ExecutorServiceTools.newScheduledThreadPool(1,
                                                                                                  getClass(),
                                                                                                  ExecutorServiceTools.ExceptionHandling.CATCH_AND_REPORT);
   private final BackgroundExecutorManager executor = new BackgroundExecutorManager(1);

   private int depthMessageCounter = 0;
   private int compressedImageCounter = 0;

   public PerceptionDataLogger()
   {
      communicationMode = CommunicationMode.INTERPROCESS;

      File f = new File(FILE_NAME);
      if (!f.exists() && !f.isDirectory())
      {

         LogTools.info("Creating HDF5 File: " + FILE_NAME);
         hdf5Manager = new HDF5Manager(FILE_NAME, H5F_ACC_TRUNC);
         hdf5Manager.getFile().openFile(FILE_NAME, H5F_ACC_RDWR);
      }
      else
      {
         LogTools.info("Opening Existing HDF5 File: " + FILE_NAME);
         hdf5Manager = new HDF5Manager(FILE_NAME, H5F_ACC_RDWR);
      }

      // Use both regular and real-time ROS2 nodes to assign callbacks to different message types
      ros2Node = ROS2Tools.createROS2Node(communicationMode.getPubSubImplementation(), "perception_logger_node");
      realtimeROS2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "perception_logger_realtime_node");

      // Add callback for Ouster scans
      new IHMCROS2Callback<>(ros2Node, ROS2Tools.OUSTER_POINT_CLOUD.withType(FusedSensorHeadPointCloudMessage.class), this::logLidarScanMessage);

      // Add callback for Robot Configuration Data
      new IHMCROS2Callback<>(ros2Node, RobotConfigurationData.class, ROS2Tools.getRobotConfigurationDataTopic("Nadia"), this::logRobotConfigurationData);

      // Add callback for L515 Depth maps
      ROS2Tools.createCallbackSubscription(realtimeROS2Node, ROS2Tools.L515_DEPTH, ROS2QosProfile.BEST_EFFORT(), subscriber ->
      {
            depthVideoPacket.getData().resetQuick();
            subscriber.takeNextData(depthVideoPacket, depthSampleInfo);
            logDepthMap(depthVideoPacket);
      });

      realtimeROS2Node.spin();

      //      "D435 Video", ros2Node, ROS2Tools.VIDEO, ROS2VideoFormat.JPEGYUVI420

      //      new ROS2Callback<>(ros2Node, ROS2Tools.L515_VIDEO.withType(BigVideoPacket.class), this::logBigVideoPacket);
      //      new IHMCROS2Callback<>(ros2Node, ROS2Tools.BLACKFLY_VIDEO.get(RobotSide.RIGHT), this::logBigVideoPacket);
      //      bigVideoPacketROS2Callback = new ROS2Callback<>(ros2Node, ROS2Tools.BLACKFLY_VIDEO.get(RobotSide.RIGHT), this::logBigVideoPacket);
      //      new ROS2Callback<>(ros2Node, ROS2Tools.BLACKFLY_VIDEO.get(RobotSide.LEFT), this::logBigVideoPacket);

      executorService.scheduleAtFixedRate(this::collectStatistics, 0, 10, TimeUnit.MILLISECONDS);
   }

   public void collectStatistics()
   {
//      LogTools.info("Collecting Statistics");
   }

   public void logRobotConfigurationData(RobotConfigurationData data)
   {
      LogTools.info("Robot Configuration Data Received: {}", data.getMonotonicTime());

      storeFloatArray("/robot/root/position/", data.getRootPosition());
      storeFloatArray("/robot/root/orientation/", data.getRootOrientation());
      storeFloatArray("/robot/joint_angles/", data.getJointAngles().toArray());
      storeFloatArray("/robot/joint_velocities/", data.getJointVelocities().toArray());
      storeFloatArray("/robot/joint_torques/", data.getJointTorques().toArray());
   }

   public void logLidarScanMessage(FusedSensorHeadPointCloudMessage message)
   {
      LogTools.info("OUSTER POINT CLOUD Received.");

      ousterCloudPacket.set(message);
      storePointCloud("/os_cloud_node/points/", ousterCloudPacket);
   }

   public void logDepthMap(BigVideoPacket packet)
   {
      LogTools.info("Depth Map Received.");
      storeDepthMap("/chest_l515/depth/image_compressed/", packet);
   }

   public void convertBigVideoPacketToMat(BigVideoPacket packet, Mat mat)
   {
      IDLSequence.Byte imageEncodedTByteArrayList = packet.getData();
      imageEncodedTByteArrayList.toArray(messageDataHeapArray);
      messageEncodedBytePointer.put(messageDataHeapArray, 0, imageEncodedTByteArrayList.size());
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

   public void logImage(Mat mat)
   {
      BytePointer jpegImageBytePointer = new BytePointer();
      IntPointer compressionParameters = new IntPointer(opencv_imgcodecs.IMWRITE_JPEG_QUALITY, 75);

      opencv_imgcodecs.imencode(".jpg", mat, jpegImageBytePointer, compressionParameters);

      HDF5Tools.storeCompressedImage(hdf5Manager.getGroup("/ihmc/logger/camera/"), compressedImageCounter, jpegImageBytePointer);
   }

   public void storeDepthMap(String namespace, BigVideoPacket message)
   {
      Group group = hdf5Manager.getGroup(namespace);
      executor.executeInBackground(() ->
                                   {
                                      synchronized (this)
                                      {
                                         LogTools.info("{} Storing Buffer: {}", namespace, depthMapCount);
                                         depthMapCount = (int) hdf5Manager.getCount(namespace);

                                         IDLSequence.Byte imageEncodedTByteArrayList = message.getData();
                                         imageEncodedTByteArrayList.toArray(messageDataHeapArray);
//                                         messageEncodedBytePointer.put(messageDataHeapArray, 0, imageEncodedTByteArrayList.size());
//                                         messageEncodedBytePointer.limit(imageEncodedTByteArrayList.size());

                                         HDF5Tools.storeByteArray(group, depthMapCount, messageDataHeapArray, message.getData().size());
                                         LogTools.info("{} Done Storing Buffer: {}", namespace, depthMapCount);
                                      }
                                   });
   }

   public void storePointCloud(String namespace, LidarScanMessage message)
   {
      Group group = hdf5Manager.getGroup(namespace);

      executor.executeInBackground(() ->
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

      executor.executeInBackground(() ->
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
      Group group = hdf5Manager.getGroup(namespace);
      Float[] objectArray = ArrayUtils.toObject(array);
      ArrayList<Float> buffer = hdf5Manager.getBuffer(namespace);
      buffer.addAll(Arrays.asList(objectArray));

      int bufferSize = hdf5Manager.getBufferIndex(namespace) / array.length;
      LogTools.info("Buffer Index: {} {}", bufferSize, HDF5Manager.MAX_BUFFER_SIZE - 1);
      if (bufferSize == (HDF5Manager.MAX_BUFFER_SIZE - 1))
      {
         LogTools.info("Thread Store Triggered");
         ArrayList<Float> data = new ArrayList<>(buffer);
         hdf5Manager.resetBuffer(namespace);

         executor.executeInBackground(() ->
                                      {
                                         long count = hdf5Manager.getCount(namespace);
                                         LogTools.info("Storing Buffer: {}", count);
                                         HDF5Tools.storeFloatArray2D(group, count, data, HDF5Manager.MAX_BUFFER_SIZE, array.length);
                                         LogTools.info("Done Storing Buffer: {}", count);
                                      });
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

      PerceptionDataLogger logger = new PerceptionDataLogger();

      //        HDF5Manager h5;
      //        File f = new File(FILE_NAME);
      //        if (!f.exists() && !f.isDirectory()) {
      //
      //            LogTools.info("Creating HDF5 File: " + FILE_NAME);
      //            h5 = new HDF5Manager(FILE_NAME, H5F_ACC_TRUNC);
      //            h5.getFile().openFile(FILE_NAME, H5F_ACC_RDWR);
      //        } else {
      //            LogTools.info("Opening Existing HDF5 File: " + FILE_NAME);
      //            h5 = new HDF5Manager(FILE_NAME, H5F_ACC_RDWR);
      //        }

      //
      ////        h5.getFile().createGroup("g1/");
      ////        h5.getFile().createGroup("g1/g2/");
      //
      //        DMatrixRMaj a = new DMatrixRMaj(5,5);
      //
      //        // Can assign values the usual way
      //        for(int i = 0; i< 3;i++)
      //        {
      //            for (int j = 0; j < 3; j++) {
      //                a.set(i, j, i + j + 1);
      //            }
      //        }
      //
      //        // Direct manipulation of each value is the fastest way to assign/read values
      //        a.set(1,1, 12);
      //        a.set(2,3,64);
      //
      //        a.print();
      //
      //        HDF5Tools.storeMatrix(h5.getGroup("/test/ejml/matrix"), a.data);
   }
}


