package us.ihmc.perception.ros1.camera;

import java.util.function.LongUnaryOperator;

import us.ihmc.communication.net.ObjectCommunicator;
import us.ihmc.communication.net.ObjectConsumer;
import us.ihmc.communication.producers.VideoSource;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.LocalVideoPacket;
import us.ihmc.robotModels.FullRobotModelFactory;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.ros2.ROS2NodeInterface;
import us.ihmc.ros2.ROS2QosProfile;
import us.ihmc.sensorProcessing.communication.producers.RobotConfigurationDataBuffer;

/**
 * Generate simulated camera data and camera info packet from SCS, we use only left eye.
 */
public class SCSCameraDataReceiver extends CameraDataReceiver implements ObjectConsumer<LocalVideoPacket>
{
   private static final boolean DEBUG = false;
   private final RobotSide robotSide;

   public SCSCameraDataReceiver(RobotSide robotSide,
                                FullRobotModelFactory fullRobotModelFactory,
                                String sensorNameInSdf,
                                RobotConfigurationDataBuffer robotConfigurationDataBuffer,
                                ObjectCommunicator scsSensorsCommunicator,
                                ROS2NodeInterface ros2Node,
                                LongUnaryOperator robotMonotonicTimeCalculator)
   {
      this(robotSide,
           fullRobotModelFactory,
           sensorNameInSdf,
           robotConfigurationDataBuffer,
           scsSensorsCommunicator,
           ros2Node,
           ROS2QosProfile.DEFAULT(),
           robotMonotonicTimeCalculator);
   }

   public SCSCameraDataReceiver(RobotSide robotSide,
                                FullRobotModelFactory fullRobotModelFactory,
                                String sensorNameInSdf,
                                RobotConfigurationDataBuffer robotConfigurationDataBuffer,
                                ObjectCommunicator scsSensorsCommunicator,
                                ROS2NodeInterface ros2Node,
                                ROS2QosProfile qosProfile,
                                LongUnaryOperator robotMonotonicTimeCalculator)
   {
      super(fullRobotModelFactory, sensorNameInSdf, robotConfigurationDataBuffer, new VideoPacketHandler(ros2Node, qosProfile), robotMonotonicTimeCalculator);

      this.robotSide = robotSide;

      scsSensorsCommunicator.attachListener(LocalVideoPacket.class, this);

      //      CameraLogger logger = DRCConfigParameters.LOG_PRIMARY_CAMERA_IMAGES ? new CameraLogger("left") : null;
   }

   @Override
   public void consumeObject(LocalVideoPacket object)
   {
      if (DEBUG)
      {
         System.out.println(getClass().getName() + ": received local video packet!");
      }
      updateImage(VideoSource.getMultisenseSourceFromRobotSide(robotSide),
                  object.getImage(),
                  object.getTimeStamp(),
                  HumanoidMessageTools.toIntrinsicParameters(object.getIntrinsicParameters()));
   }
}
