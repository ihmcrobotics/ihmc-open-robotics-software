package us.ihmc.gdx.logging;

import controller_msgs.msg.dds.BigVideoPacket;
import controller_msgs.msg.dds.FusedSensorHeadPointCloudMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import org.bytedeco.hdf5.H5File;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import us.ihmc.communication.IHMCROS2Callback;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.idl.IDLSequence;
import us.ihmc.perception.HDF5Tools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.pubsub.common.SampleInfo;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.ros2.ROS2Topic;
import us.ihmc.ros2.RealtimeROS2Node;

import static org.bytedeco.hdf5.global.hdf5.H5F_ACC_TRUNC;

public class PerceptionDataLogger
{
   static final int PCD_POINT_SIZE = 3;
   static final String FILE_NAME = "/home/bmishra/Workspace/Data/Sensor_Logs/HDF5/OusterL515Log.h5";
   private H5File file;

   private final BigVideoPacket videoPacket = new BigVideoPacket();
   private final SampleInfo depthSampleInfo = new SampleInfo();

   private final FusedSensorHeadPointCloudMessage ousterCloudPacket = new FusedSensorHeadPointCloudMessage();
   private final SampleInfo ousterSampleInfo = new SampleInfo();

   private final byte[] messageDataHeapArray = new byte[25000000];
   private final BytePointer messageEncodedBytePointer = new BytePointer(25000000);
   private final Mat inputJPEGMat = new Mat(1, 1, opencv_core.CV_8UC1);
   private final Mat inputYUVI420Mat = new Mat(1, 1, opencv_core.CV_8UC1);
   private final Mat depthMap = new Mat(1, 1, opencv_core.CV_16UC1);

   private int depthMessageCounter = 0;
   private int compressedImageCounter = 0;

   private int pointsPerSegment = 786432;
   private int numberOfSegments = 1;

   public PerceptionDataLogger()
   {

      file = new H5File(FILE_NAME, H5F_ACC_TRUNC);
      RealtimeROS2Node ros2Node = ROS2Tools.createRealtimeROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "perception_data_logger");

      new IHMCROS2Callback<>(ros2Node, ROS2Tools.L515_DEPTH.withType(BigVideoPacket.class), ROS2QosProfile.BEST_EFFORT(), this::logDepthMap);
      new IHMCROS2Callback<>(ros2Node, ROS2Tools.L515_VIDEO.withType(BigVideoPacket.class), ROS2QosProfile.BEST_EFFORT(), this::logBigVideoPacket);

      new IHMCROS2Callback<>(ros2Node,
                             ROS2Tools.OUSTER_LIDAR.withType(FusedSensorHeadPointCloudMessage.class),
                             ROS2QosProfile.BEST_EFFORT(),
                             this::logFusedOusterPointCloud);

      ROS2Topic<BigVideoPacket> blackflyTopicRight = ROS2Tools.BLACKFLY_VIDEO.get(RobotSide.RIGHT);
      new IHMCROS2Callback<>(ros2Node, blackflyTopicRight, ROS2QosProfile.BEST_EFFORT(), this::logBigVideoPacket);

      ROS2Topic<BigVideoPacket> blackflyTopicLeft = ROS2Tools.BLACKFLY_VIDEO.get(RobotSide.LEFT);
      new IHMCROS2Callback<>(ros2Node, blackflyTopicLeft, ROS2QosProfile.BEST_EFFORT(), this::logBigVideoPacket);

      new IHMCROS2Callback<>(ros2Node, ROS2Tools.getRobotConfigurationDataTopic("Nadia"), this::logRobotConfigurationData);
   }

   public void logRobotConfigurationData(RobotConfigurationData data)
   {

   }

   public void logFusedOusterPointCloud(FusedSensorHeadPointCloudMessage message)
   {

   }

   public void logDepthMap(BigVideoPacket packet)
   {
      convertBigVideoPacketToMat(videoPacket, depthMap);
      HDF5Tools.storeDepthMap(file, "/chest_l515/depth/image_rect_raw/" + depthMessageCounter, depthMap);
      depthMessageCounter += 1;
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
      Mat mat = new Mat(packet.getImageHeight(), packet.getImageHeight(), opencv_core.CV_8UC3);
      convertBigVideoPacketToMat(packet, mat);
      logImage(mat);
   }

   public void logImage(Mat mat)
   {
      BytePointer jpegImageBytePointer = new BytePointer();
      IntPointer compressionParameters = new IntPointer(opencv_imgcodecs.IMWRITE_JPEG_QUALITY, 75);

      opencv_imgcodecs.imencode(".jpg", mat, jpegImageBytePointer, compressionParameters);

      HDF5Tools.storeCompressedImage(file, "/ihmc/logger/camera/" + compressedImageCounter, jpegImageBytePointer);
   }

   public static void main(String[] args)
   {
      PerceptionDataLogger logger = new PerceptionDataLogger();
   }
}


