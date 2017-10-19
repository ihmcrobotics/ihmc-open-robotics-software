package us.ihmc.atlas.multisenseTestbench;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotModelFactory;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.DRCNetworkModuleParameters;
import us.ihmc.avatar.networkProcessor.DRCNetworkProcessor;
import us.ihmc.communication.configuration.NetworkParameters;

import java.net.URI;

/**
 * @author Doug Stephen <a href="mailto:dstephen@ihmc.us">(dstephen@ihmc.us)</a>
 */
public class MultisenseTestBenchWithZeroPoseModuleNetworkProcessor
{
   private static final String DEFAULT_ROS_NAMESPACE = "/ihmc_ros/atlas";

   public MultisenseTestBenchWithZeroPoseModuleNetworkProcessor(DRCRobotModel robotModel, String rosNamespace)
   {
      URI rosUri = NetworkParameters.getROSURI();

      DRCNetworkModuleParameters params = new DRCNetworkModuleParameters();

      params.setRosUri(rosUri);
      params.enableRosModule(true);
      params.enableSensorModule(true);
      params.enableUiModule(true);
      params.enableZeroPoseRobotConfigurationPublisherModule(true);

      new DRCNetworkProcessor(robotModel, params);
   }

   public MultisenseTestBenchWithZeroPoseModuleNetworkProcessor(DRCRobotModel robotModel)
   {
      this(robotModel, DEFAULT_ROS_NAMESPACE);
   }

   public static void main(String[] args)
   {
      AtlasRobotModel robotModel = AtlasRobotModelFactory
            .createDRCRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS.name(), RobotTarget.HEAD_ON_A_STICK, false);

      new MultisenseTestBenchWithZeroPoseModuleNetworkProcessor(robotModel);
   }
}
