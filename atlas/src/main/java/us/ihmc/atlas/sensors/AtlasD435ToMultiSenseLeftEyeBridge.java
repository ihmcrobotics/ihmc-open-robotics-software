package us.ihmc.atlas.sensors;

import boofcv.struct.calib.CameraPinholeBrown;
import perception_msgs.msg.dds.VideoPacket;
import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.ROS2SyncedRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.behaviors.tools.CommunicationHelper;
import us.ihmc.codecs.generated.YUVPicture;
import us.ihmc.codecs.yuv.JPEGEncoder;
import us.ihmc.codecs.yuv.YUVPictureConverter;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.euclid.referenceFrame.interfaces.FramePose3DReadOnly;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.frames.HumanoidReferenceFrames;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.tools.UnitConversions;
import us.ihmc.tools.thread.MissingThreadTools;
import us.ihmc.tools.thread.ResettableExceptionHandlingExecutorService;
import us.ihmc.tools.thread.Throttler;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosTools;
import us.ihmc.utilities.ros.subscriber.AbstractRosTopicSubscriber;

import javax.imageio.ImageIO;
import java.awt.image.BufferedImage;
import java.io.ByteArrayInputStream;
import java.nio.ByteBuffer;

public class AtlasD435ToMultiSenseLeftEyeBridge
{
   private static final double OUTPUT_FREQUENCY = UnitConversions.hertzToSeconds(10.0);

   public AtlasD435ToMultiSenseLeftEyeBridge()
   {
      RosMainNode ros1Node = RosTools.createRosNode(NetworkParameters.getROSURI(), "d435_to_left_eye");
      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "d435_to_left_eye");
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.REAL_ROBOT);
      CommunicationHelper ros2Helper = new CommunicationHelper(robotModel, ros2Node);
      ROS2SyncedRobotModel syncedRobot = ros2Helper.newSyncedRobot();
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      Throttler throttler = new Throttler();
      YUVPictureConverter converter = new YUVPictureConverter();
      JPEGEncoder encoder = new JPEGEncoder();

      int imageWidth = 640;
      int imageHeight = 360;
      double fx = 500.0;
      double fy = 500.0;
      CameraPinholeBrown depthCameraIntrinsics = new CameraPinholeBrown(fx, fy, 0, imageWidth / 2.0, imageHeight / 2.0, imageWidth, imageHeight);

      ResettableExceptionHandlingExecutorService executor = MissingThreadTools.newSingleThreadExecutor("D435ToLeftEyeBridge", true);

      AbstractRosTopicSubscriber<sensor_msgs.CompressedImage> subscriber
            = new AbstractRosTopicSubscriber<sensor_msgs.CompressedImage>(sensor_msgs.CompressedImage._TYPE)
      {
         @Override
         public void onNewMessage(sensor_msgs.CompressedImage ros1Image)
         {
            if (throttler.run(OUTPUT_FREQUENCY))
            {
               executor.submit(() ->
               {
                  syncedRobot.update();
//                  if (syncedRobot.getDataReceptionTimerSnapshot().isRunning(3.0))
                  {
                     try
                     {
                        byte[] payload = ros1Image.getData().array();
                        int offset = ros1Image.getData().arrayOffset();
                        BufferedImage bufferedImage = ImageIO.read(new ByteArrayInputStream(payload, offset, payload.length - offset));

                        YUVPicture picture = converter.fromBufferedImage(bufferedImage, YUVPicture.YUVSubsamplingType.YUV420);
                        ByteBuffer buffer = encoder.encode(picture, 75);

                        byte[] data = new byte[buffer.remaining()];
                        buffer.get(data);

                        FramePose3DReadOnly ousterPose = syncedRobot.getFramePoseReadOnly(HumanoidReferenceFrames::getOusterLidarFrame);
                        ousterPose.get(transformToWorld);

                        VideoPacket message = new VideoPacket();
                        message.setTimestamp(System.nanoTime());
                        message.getData().add(data);
                        message.getPosition().set(ousterPose.getPosition());
                        message.getOrientation().set(ousterPose.getOrientation());
                        message.setVideoSource(VideoPacket.VIDEO_SOURCE_MULTISENSE_LEFT_EYE);
                        message.getIntrinsicParameters().set(HumanoidMessageTools.toIntrinsicParametersMessage(depthCameraIntrinsics));

                        ros2Helper.publish(PerceptionAPI.VIDEO, message);
                     }
                     catch (Exception e)
                     {
                        LogTools.error(e.getMessage());
                        e.printStackTrace();
                     }
                  }
               });
            }
         }
      };
      ros1Node.attachSubscriber(RosTools.D435_VIDEO_COMPRESSED, subscriber);
      ros1Node.execute();

      Runtime.getRuntime().addShutdownHook(new Thread(() ->
      {
         executor.destroy();
         ros1Node.shutdown();
      }, "IHMC-D435ROS1ToLeftEyeBridgeShutdown"));
   }

   public static void main(String[] args)
   {
      new AtlasD435ToMultiSenseLeftEyeBridge();
   }
}
