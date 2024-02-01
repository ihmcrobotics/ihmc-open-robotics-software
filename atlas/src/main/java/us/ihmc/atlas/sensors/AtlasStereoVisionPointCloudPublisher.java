package us.ihmc.atlas.sensors;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.atlas.parameters.AtlasSensorInformation;
import us.ihmc.atlas.ros.AtlasPPSTimestampOffsetProvider;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.stereoPointCloudPublisher.StereoVisionPointCloudPublisher;
import us.ihmc.avatar.ros.DRCROSPPSTimestampOffsetProvider;
import us.ihmc.avatar.ros.RobotROSClockCalculatorFromPPSOffset;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.PerceptionAPI;
import us.ihmc.communication.ROS2Tools;
import us.ihmc.communication.configuration.NetworkParameters;
import us.ihmc.log.LogTools;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.ros2.ROS2Node;
import us.ihmc.sensorProcessing.parameters.AvatarRobotPointCloudParameters;
import us.ihmc.utilities.ros.RosMainNode;
import us.ihmc.utilities.ros.RosTools;

public class AtlasStereoVisionPointCloudPublisher
{
   private final StereoVisionPointCloudPublisher multisenseStereoVisionPointCloudPublisher;

   public AtlasStereoVisionPointCloudPublisher()
   {
      AtlasRobotModel robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.REAL_ROBOT);
      ROS2Node ros2Node = ROS2Tools.createROS2Node(DomainFactory.PubSubImplementation.FAST_RTPS, "stereo_bridge");
      RosMainNode ros1Node = RosTools.createRosNode(NetworkParameters.getROSURI(), "stereo_bridge");

      AtlasSensorInformation sensorInformation = (AtlasSensorInformation) robotModel.getSensorInformation();
      DRCROSPPSTimestampOffsetProvider timestampOffsetProvider = AtlasPPSTimestampOffsetProvider.getInstance(sensorInformation);
      RobotROSClockCalculatorFromPPSOffset rosClockCalculator = new RobotROSClockCalculatorFromPPSOffset(timestampOffsetProvider);

      multisenseStereoVisionPointCloudPublisher = new StereoVisionPointCloudPublisher(robotModel, ros2Node, PerceptionAPI.MULTISENSE_STEREO_POINT_CLOUD);
      multisenseStereoVisionPointCloudPublisher.setROSClockCalculator(rosClockCalculator);
      AvatarRobotPointCloudParameters multisenseStereoParameters
            = sensorInformation.getPointCloudParameters(AtlasSensorInformation.MULTISENSE_STEREO_ID);
      multisenseStereoVisionPointCloudPublisher.receiveStereoPointCloudFromROS1(multisenseStereoParameters.getRosTopic(), ros1Node);
      multisenseStereoVisionPointCloudPublisher.setRangeFilter(0.2, 2.5);
      multisenseStereoVisionPointCloudPublisher.setPublisherPeriodInMillisecond(1500L);
      multisenseStereoVisionPointCloudPublisher.setMaximumNumberOfPoints(200000);

      ros1Node.execute();
      multisenseStereoVisionPointCloudPublisher.start();

      Runtime.getRuntime().addShutdownHook(new Thread(() ->
     {
        LogTools.info("Shutting down network processor modules.");
        multisenseStereoVisionPointCloudPublisher.shutdown();
        ThreadTools.sleep(10);
     }, getClass().getSimpleName() + "Shutdown"));
   }

   public static void main(String[] args)
   {
      new AtlasStereoVisionPointCloudPublisher();
   }
}
