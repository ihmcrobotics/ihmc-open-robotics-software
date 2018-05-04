package us.ihmc.ihmcPerception.camera;

import java.awt.image.BufferedImage;

import boofcv.struct.calib.IntrinsicParameters;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.net.ConnectionStateListener;
import us.ihmc.communication.packetCommunicator.PacketCommunicator;
import us.ihmc.communication.producers.CompressedVideoHandler;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.euclid.tuple4D.interfaces.QuaternionReadOnly;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.kryo.PPSTimestampOffsetProvider;
import us.ihmc.robotModels.FullHumanoidRobotModelFactory;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;
import us.ihmc.sensorProcessing.parameters.DRCRobotCameraParameters;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.subscriber.RosCompressedImageSubscriber;

public class FisheyeCameraReceiver extends CameraDataReceiver
{
   private static final boolean DEBUG = false;
   public FisheyeCameraReceiver(FullHumanoidRobotModelFactory fullRobotModelFactory, final DRCRobotCameraParameters cameraParameters,
         RobotConfigurationDataBuffer robotConfigurationDataBuffer, PacketCommunicator packetCommunicator,
         PPSTimestampOffsetProvider ppsTimestampOffsetProvider, final RosMainNode rosMainNode)
   {
      super(fullRobotModelFactory, cameraParameters.getSensorNameInSdf(), robotConfigurationDataBuffer, new CompressedFisheyeHandler(packetCommunicator),
            ppsTimestampOffsetProvider);

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
            IntrinsicParameters intrinsicParameters = imageInfoSubscriber.getIntrinisicParameters();
            if(DEBUG)
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
      private final PacketCommunicator packetCommunicator;

      public CompressedFisheyeHandler(PacketCommunicator packetCommunicator)
      {
         this.packetCommunicator = packetCommunicator;
      }

      @Override
      public void onFrame(VideoSource videoSource, byte[] data, long timeStamp, Point3DReadOnly position, QuaternionReadOnly orientation,
            IntrinsicParameters intrinsicParameters)
      {
         if(DEBUG)
         {
            PrintTools.debug(this, videoSource.name() + " fisheye data size size is " + data.length);
         }
         packetCommunicator.send(HumanoidMessageTools.createFisheyePacket(videoSource, timeStamp, data, position, orientation, intrinsicParameters));
      }

      @Override
      public void addNetStateListener(ConnectionStateListener compressedVideoDataServer)
      {
         packetCommunicator.attachStateListener(compressedVideoDataServer);
      }

      @Override
      public boolean isConnected()
      {
         return packetCommunicator.isConnected();
      }

   }

}
