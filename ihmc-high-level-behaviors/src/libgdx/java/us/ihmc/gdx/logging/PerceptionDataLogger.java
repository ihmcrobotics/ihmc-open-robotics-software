package us.ihmc.gdx.logging;

import controller_msgs.msg.dds.BigVideoPacket;
import controller_msgs.msg.dds.FusedSensorHeadPointCloudMessage;
import controller_msgs.msg.dds.RobotConfigurationData;
import org.apache.commons.lang.ArrayUtils;
import org.bytedeco.hdf5.Group;
import org.bytedeco.javacpp.BytePointer;
import org.bytedeco.javacpp.IntPointer;
import org.bytedeco.opencv.global.opencv_core;
import org.bytedeco.opencv.global.opencv_imgcodecs;
import org.bytedeco.opencv.global.opencv_imgproc;
import org.bytedeco.opencv.opencv_core.Mat;
import org.ejml.data.DMatrix3x3;
import org.ejml.data.DMatrixRMaj;
import org.ejml.simple.SimpleMatrix;
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
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.*;
import us.ihmc.scs2.sessionVisualizer.jfx.managers.BackgroundExecutorManager;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;

import static org.bytedeco.hdf5.global.hdf5.*;

public class PerceptionDataLogger {
    static final String FILE_NAME = "/home/quantum/Workspace/Data/Sensor_Logs/experimental.h5";
    private final HDF5Manager h5;

    private final BigVideoPacket videoPacket = new BigVideoPacket();
    private final SampleInfo depthSampleInfo = new SampleInfo();

    private final FusedSensorHeadPointCloudMessage ousterCloudPacket = new FusedSensorHeadPointCloudMessage();
    private final SampleInfo ousterSampleInfo = new SampleInfo();

    private final ROS2Node ros2Node;

    private final byte[] messageDataHeapArray = new byte[25000000];
    private final BytePointer messageEncodedBytePointer = new BytePointer(25000000);
    private final Mat inputJPEGMat = new Mat(1, 1, opencv_core.CV_8UC1);
    private final Mat inputYUVI420Mat = new Mat(1, 1, opencv_core.CV_8UC1);
    private final Mat depthMap = new Mat(1, 1, opencv_core.CV_16UC1);

    private ROS2Callback<RobotConfigurationData> robotConfigurationCallback;

    private final BackgroundExecutorManager executor = new BackgroundExecutorManager(4);

    private int depthMessageCounter = 0;
    private int compressedImageCounter = 0;

    public PerceptionDataLogger() {

        File f = new File(FILE_NAME);
        if(!f.exists() && !f.isDirectory()) {

            LogTools.info("Creating HDF5 File: " + FILE_NAME);
            h5 = new HDF5Manager(FILE_NAME, H5F_ACC_TRUNC);
            h5.getFile().openFile(FILE_NAME, H5F_ACC_RDWR);
        }
        else
        {
            LogTools.info("Opening Existing HDF5 File: " + FILE_NAME);
            h5 = new HDF5Manager(FILE_NAME, H5F_ACC_RDWR);
        }

        ros2Node = ROS2Tools.createROS2Node(CommunicationMode.INTERPROCESS.getPubSubImplementation(), "logger");

//        new IHMCROS2Callback<>(ros2Node, ROS2Tools.L515_DEPTH.withType(BigVideoPacket.class), ROS2QosProfile.BEST_EFFORT(), this::logDepthMap);
//        new IHMCROS2Callback<>(ros2Node, ROS2Tools.L515_VIDEO.withType(BigVideoPacket.class), ROS2QosProfile.BEST_EFFORT(), this::logBigVideoPacket);
//
//        new IHMCROS2Callback<>(ros2Node,
//                ROS2Tools.OUSTER_LIDAR.withType(FusedSensorHeadPointCloudMessage.class),
//                ROS2QosProfile.BEST_EFFORT(),
//                this::logFusedOusterPointCloud);
//
//        ROS2Topic<BigVideoPacket> blackflyTopicRight = ROS2Tools.BLACKFLY_VIDEO.get(RobotSide.RIGHT);
//        new IHMCROS2Callback<>(ros2Node, blackflyTopicRight, ROS2QosProfile.BEST_EFFORT(), this::logBigVideoPacket);
//
//        ROS2Topic<BigVideoPacket> blackflyTopicLeft = ROS2Tools.BLACKFLY_VIDEO.get(RobotSide.LEFT);
//        new IHMCROS2Callback<>(ros2Node, blackflyTopicLeft, ROS2QosProfile.BEST_EFFORT(), this::logBigVideoPacket);

        robotConfigurationCallback = new ROS2Callback<>(ros2Node, RobotConfigurationData.class, ROS2Tools.getRobotConfigurationDataTopic("Nadia"), this::logRobotConfigurationData);

        while(true)
        {

        }
    }

    public void logRobotConfigurationData(RobotConfigurationData data) {

        LogTools.info("Message Received: {}", data.getMonotonicTime());

        storeFloatArray("/robot/joint_angles/", data.getJointAngles().toArray());
//        storeFloatArray("/robot/joint_velocities/", data.getJointVelocities().toArray());
//        storeFloatArray("/robot/joint_torques/", data.getJointTorques().toArray());
//        storePoint3D("/robot/root/position", data.getRootPosition());
//        storeQuaternion("/robot/root/orientation", data.getRootOrientation());
    }

    public void logFusedOusterPointCloud(FusedSensorHeadPointCloudMessage message) {

    }

    public void logDepthMap(BigVideoPacket packet) {
        convertBigVideoPacketToMat(videoPacket, depthMap);
        HDF5Tools.storeDepthMap(h5.getGroup("/chest_l515/depth/image_rect_raw/"), depthMessageCounter, depthMap);
        depthMessageCounter += 1;
    }

    public void convertBigVideoPacketToMat(BigVideoPacket packet, Mat mat) {
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

    public void logBigVideoPacket(BigVideoPacket packet) {
        Mat mat = new Mat(packet.getImageHeight(), packet.getImageHeight(), opencv_core.CV_8UC3);
        convertBigVideoPacketToMat(packet, mat);
        logImage(mat);
    }

    public void logImage(Mat mat) {
        BytePointer jpegImageBytePointer = new BytePointer();
        IntPointer compressionParameters = new IntPointer(opencv_imgcodecs.IMWRITE_JPEG_QUALITY, 75);

        opencv_imgcodecs.imencode(".jpg", mat, jpegImageBytePointer, compressionParameters);

        HDF5Tools.storeCompressedImage(h5.getGroup("/ihmc/logger/camera/"), compressedImageCounter, jpegImageBytePointer);
    }

//    HDF5Tools.storeMatrix(h5.getGroup(namespace), h5.getBuffer(namespace, array.length).data);
    public void storeFloatArray(String namespace, float[] array) {
        Group group = h5.getGroup(namespace);
        Float[] objectArray = ArrayUtils.toObject(array);
        ArrayList<Float> buffer = h5.getBuffer(namespace);
        buffer.addAll(Arrays.asList(objectArray));

        int bufferSize = h5.getBufferIndex(namespace) / array.length;
        LogTools.info("Buffer Index: {} {}", bufferSize, HDF5Manager.MAX_BUFFER_SIZE - 1);
        if (bufferSize == (HDF5Manager.MAX_BUFFER_SIZE - 1))
        {
            LogTools.info("Thread Store Triggered");
            ArrayList<Float> data = new ArrayList<>(buffer);
            h5.resetBuffer(namespace);

            executor.executeInBackground(() -> {
                long count = h5.getCount(namespace);
                LogTools.info("Storing Buffer: {}", count);
                HDF5Tools.storeFloatArray2D(group, count, data, array.length);
                LogTools.info("Done Storing Buffer: {}", count);
            });
        }
    }

    public void storePoint3D(String namespace, Point3D point) {

    }

    public void storeQuaternion(String namespace, Quaternion orientation) {

    }

    public static void main(String[] args) {

        PerceptionDataLogger logger = new PerceptionDataLogger();

        HDF5Manager h5;
        File f = new File(FILE_NAME);
        if(!f.exists() && !f.isDirectory()) {

            LogTools.info("Creating HDF5 File: " + FILE_NAME);
            h5 = new HDF5Manager(FILE_NAME, H5F_ACC_TRUNC);
            h5.getFile().openFile(FILE_NAME, H5F_ACC_RDWR);
        }
        else
        {
            LogTools.info("Opening Existing HDF5 File: " + FILE_NAME);
            h5 = new HDF5Manager(FILE_NAME, H5F_ACC_RDWR);
        }



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


