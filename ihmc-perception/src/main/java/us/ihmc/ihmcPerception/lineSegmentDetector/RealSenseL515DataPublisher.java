package us.ihmc.ihmcPerception.lineSegmentDetector;

import boofcv.struct.calib.CameraPinholeBrown;
import controller_msgs.msg.dds.StereoVisionPointCloudMessage;
import controller_msgs.msg.dds.VideoPacket;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.highgui.HighGui;
import sensor_msgs.PointCloud2;
import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.codecs.yuv.JPEGEncoder;
import us.ihmc.codecs.yuv.YUVPictureConverter;
import us.ihmc.communication.IHMCROS2Publisher;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.ihmcPerception.camera.RosCameraInfoSubscriber;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.Ros2Node;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;
import us.ihmc.utilities.ros.subscriber.RosCompressedImageSubscriber;
import us.ihmc.utilities.ros.subscriber.RosImageSubscriber;

import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.awt.image.DataBufferUShort;
import java.io.IOException;
import java.net.URI;
import java.net.URISyntaxException;
import java.nio.ByteBuffer;

public class RealSenseL515DataPublisher {

    private static final YUVPictureConverter converter = new YUVPictureConverter();
    private static final JPEGEncoder encoder = new JPEGEncoder();


    public static void main(String[] args) throws URISyntaxException {
        System.loadLibrary(Core.NATIVE_LIBRARY_NAME);
        new RealSenseL515DataPublisher();
    }

    public RealSenseL515DataPublisher() throws URISyntaxException {
        RosMainNode rosMainNode = new RosMainNode(new URI("http://localhost:11311"), "RealSenseL515DataPublisher", true);
        Ros2Node ros2Node = ROS2Tools.createRos2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "video_viewer");

        new RealSenseL515CompressedImageSubscriber(rosMainNode, ros2Node);
        new RealSenseL515DepthImageSubscriber(rosMainNode, ros2Node);
        // new RealSenseL515PointCloudSubscriber(rosMainNode, ros2Node);
        rosMainNode.execute();
    }

    private static class RealSenseL515CompressedImageSubscriber extends RosCompressedImageSubscriber {

        private String l515CompressedImageTopic = "/camera/color/image_raw/compressed";
        private String l515CameraInfoTopic = "/camera/color/camera_info";
        private BufferedImage latestBufferedImage;

        private RosCameraInfoSubscriber colorImageInfoSubscriber;

        private final IHMCROS2Publisher<VideoPacket> colorPublisher;


        public RealSenseL515CompressedImageSubscriber(RosMainNode rosNode, Ros2Node ros2Node) {
            super();
            colorPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, VideoPacket.class, ROS2Tools.IHMC_ROOT);
            colorImageInfoSubscriber = new RosCameraInfoSubscriber(l515CameraInfoTopic);
            rosNode.attachSubscriber(l515CompressedImageTopic, this);
            rosNode.attachSubscriber(l515CameraInfoTopic, colorImageInfoSubscriber);
        }

        @Override
        protected void imageReceived(long timeStamp, BufferedImage image) {
            System.out.println(image);
            if (latestBufferedImage == null) {
                latestBufferedImage = new BufferedImage(image.getWidth(), image.getHeight(), BufferedImage.TYPE_3BYTE_BGR);
            }
            latestBufferedImage.getGraphics().drawImage(image, 0, 0, null);
            CameraPinholeBrown intrinsicParameters = colorImageInfoSubscriber.getIntrinisicParameters();
            YUVPicture picture = converter.fromBufferedImage(latestBufferedImage, YUVPicture.YUVSubsamplingType.YUV420);
            ByteBuffer buffer = null;
            try {
                buffer = encoder.encode(picture, 35);
            } catch (IOException e) {
                e.printStackTrace();
            }
            byte[] data = new byte[buffer.remaining()];
            buffer.get(data);

            Point3D position = new Point3D();
            Quaternion orientation = new Quaternion();
            VideoPacket message = HumanoidMessageTools.createVideoPacket(VideoSource.MULTISENSE_LEFT_EYE, timeStamp, data, position, orientation, intrinsicParameters);
            colorPublisher.publish(message);
            picture.delete();
        }
    }

    public static class RealSenseL515DepthImageSubscriber extends RosImageSubscriber {

        private final String l515CompressedDepthTopic = "/camera/depth/image_rect_raw";
        private String l515CameraInfoTopic = "/camera/color/camera_info";

        private BufferedImage latestBufferedDepth;

        private final IHMCROS2Publisher<VideoPacket> depthPublisher;
        private final RosCameraInfoSubscriber depthImageInfoSubscriber;

        public RealSenseL515DepthImageSubscriber(RosMainNode rosNode, Ros2Node ros2Node){
            super();
            depthPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, VideoPacket.class, ROS2Tools.IHMC_ROOT);
            depthImageInfoSubscriber = new RosCameraInfoSubscriber(l515CameraInfoTopic);
            rosNode.attachSubscriber(l515CompressedDepthTopic, this);
            rosNode.attachSubscriber(l515CameraInfoTopic, depthImageInfoSubscriber);
        }

        @Override
        protected void imageReceived(long timeStamp, BufferedImage image) {
            System.out.println(image);

            if (latestBufferedDepth == null) {
                latestBufferedDepth = new BufferedImage(image.getWidth(), image.getHeight(), BufferedImage.TYPE_USHORT_GRAY);
            }
            latestBufferedDepth.getGraphics().drawImage(image, 0, 0, null);

            Mat mat = new Mat(latestBufferedDepth.getHeight(), latestBufferedDepth.getWidth(), CvType.CV_16U);

            short[] bufferData = ((DataBufferUShort) latestBufferedDepth.getRaster().getDataBuffer()).getData();
            mat.put(0, 0, bufferData);

            mat.convertTo(mat, CvType.CV_8U);


            // HighGui.namedWindow("LineEstimator", HighGui.WINDOW_AUTOSIZE);
            // HighGui.imshow("LineEstimator", mat);
            // int code = HighGui.waitKey(32);

            // CameraPinholeBrown intrinsicParameters = depthImageInfoSubscriber.getIntrinisicParameters();
            // YUVPicture picture = converter.fromBufferedImage(latestBufferedDepth, YUVPicture.YUVSubsamplingType.YUV420);
            // ByteBuffer buffer = null;
            // try {
            //     buffer = encoder.encode(picture, 35);
            // } catch (IOException e) {
            //     e.printStackTrace();
            // }
            // byte[] data = new byte[buffer.remaining()];
            // buffer.get(data);
            //
            // Point3D position = new Point3D();
            // Quaternion orientation = new Quaternion();
            // VideoPacket message = HumanoidMessageTools.createVideoPacket(VideoSource.MULTISENSE_LEFT_EYE, timeStamp, data, position, orientation, intrinsicParameters);
            // depthPublisher.publish(message);
            // picture.delete();

        }
    }


    public static class RealSenseL515PointCloudSubscriber extends AbstractRosTopicSubscriber<PointCloud2> {

        private String l515PointCloudTopic = "/camera/depth/color/points";
        private final IHMCROS2Publisher<StereoVisionPointCloudMessage> stereoVisionPublisher;

        public RealSenseL515PointCloudSubscriber(RosMainNode rosMainNode, Ros2Node ros2Node) {
            super(PointCloud2._TYPE);
            stereoVisionPublisher = ROS2Tools.createPublisherTypeNamed(ros2Node, StereoVisionPointCloudMessage.class, ROS2Tools.IHMC_ROOT);
            rosMainNode.attachSubscriber(l515PointCloudTopic, this);
        }

        @Override
        public void onNewMessage(PointCloud2 pointCloud2) {
            System.out.println("RECEIVED Points:(" + pointCloud2.getWidth() + "," + pointCloud2.getHeight() + ") Total Points:" + pointCloud2.getWidth() * pointCloud2.getHeight());
        }
    }
}
