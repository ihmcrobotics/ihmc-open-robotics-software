package us.ihmc.atlas.sensors;

import static us.ihmc.pubsub.DomainFactory.PubSubImplementation.FAST_RTPS;

import java.net.URI;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.modules.RosModule;
import us.ihmc.avatar.networkProcessor.supportingPlanarRegionPublisher.BipedalSupportPlanarRegionPublisher;
import us.ihmc.communication.configuration.NetworkParameters;

public class AtlasSensorSuiteLauncher
{
   public AtlasSensorSuiteLauncher()
   {
      AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.REAL_ROBOT);

      AtlasSensorSuiteManager atlasSensorSuiteManager = atlasRobotModel.getSensorSuiteManager();
      URI rosCoreURI = NetworkParameters.getROSURI();
      atlasSensorSuiteManager.initializePhysicalSensors(rosCoreURI);

      new RosModule(atlasRobotModel, rosCoreURI, null, FAST_RTPS);

      new BipedalSupportPlanarRegionPublisher(atlasRobotModel, FAST_RTPS).start();

      atlasSensorSuiteManager.connect();
   }

   public static void main(String[] args)
   {
      new AtlasSensorSuiteLauncher();
   }
}
