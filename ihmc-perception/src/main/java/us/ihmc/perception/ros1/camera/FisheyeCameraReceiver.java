package us.ihmc.perception.ros1.camera;

import java.awt.image.BufferedImage;
import java.util.function.LongUnaryOperator;

import boofcv.struct.calib.CameraPinholeBrown;
import perception_msgs.msg.dds.FisheyePacket;
import us.ihmc.commons.PrintTools;
import us.ihmc.ros2.ROS2PublisherBasics;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.net.ConnectionStateListener;
import us.ihmc.communication.producers.CompressedVideoHandler;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.sensorProcessing.parameters.AvatarRobotCameraParameters;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.RosCompressedImageSubscriber;

public class FisheyeCameraReceiver extends CameraDataReceiver
{
   private static final boolean DEBUG = false;

   public FisheyeCameraReceiver(FullHumanoidRobotModelFactory fullRobotModelFactory, final AvatarRobotCameraParameters cameraParameters,
                                RobotConfigurationDataBuffer robotConfigurationDataBuffer, ROS2NodeInterface ros2Node,
                                LongUnaryOperator robotMonotonicTimeCalculator, final RosMainNode rosMainNode)
   {
      super(fullRobotModelFactory, cameraParameters.getSensorNameInSdf(), robotConfigurationDataBuffer, new CompressedFisheyeHandler(ros2Node),
            robotMonotonicTimeCalculator);

      if (!cameraParameters.useIntrinsicParametersFromRos())
      {
         throw new RuntimeException("You really want to use intrinisic parameters from ROS");
      }

      final RosCameraInfoSubscriber imageInfoSubscriber = new RosCameraInfoSubscriber(cameraParameters.getRosCameraInfoTopicName());
      rosMainNode.attachSubscriber(cameraParameters.getRosCameraInfoTopicName(), imageInfoSubscriber);

      final RobotSide robotSide = cameraParameters.getRobotSide();
      RosCompressedImageSubscriber imageSubscriberSubscriber = new RosCompressedImageSubscriber()
      {
         @Override
         protected void imageReceived(long timeStamp, BufferedImage image)
         {
            CameraPinholeBrown intrinsicParameters = imageInfoSubscriber.getIntrinisicParameters();
            if (DEBUG)
            {
               PrintTools.debug(this, "Received new fisheye image on " + cameraParameters.getRosTopic() + " " + image);
            }
            updateImage(VideoSource.getFisheyeSourceFromRobotSide(robotSide), image, timeStamp, intrinsicParameters);

         }
      };
      rosMainNode.attachSubscriber(cameraParameters.getRosTopic(), imageSubscriberSubscriber);

   }

   private static class CompressedFisheyeHandler implements CompressedVideoHandler
   {
      private final ROS2PublisherBasics<FisheyePacket> publisher;

      public CompressedFisheyeHandler(ROS2NodeInterface ros2Node)
      {
         publisher = ros2Node.createPublisher(ROS2Tools.typeNamedTopic(FisheyePacket.class).withTopic(ROS2Tools.IHMC_ROOT));
      }

      @Override
      public void onFrame(VideoSource videoSource, byte[] data, long timeStamp, Point3DReadOnly position, QuaternionReadOnly orientation,
                          CameraPinholeBrown intrinsicParameters)
      {
         if (DEBUG)
         {
            PrintTools.debug(this, videoSource.name() + " fisheye data size size is " + data.length);
         }
         publisher.publish(HumanoidMessageTools.createFisheyePacket(videoSource, timeStamp, data, position, orientation, intrinsicParameters));
      }

      @Override
      public void addNetStateListener(ConnectionStateListener compressedVideoDataServer)
      {
         // FIXME
      }

      @Override
      public boolean isConnected()
      {
         return true; // FIXME
      }

   }

}
