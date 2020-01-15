package us.ihmc.atlas.sensors;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.modules.RosModule;
import us.ihmc.avatar.networkProcessor.supportingPlanarRegionPublisher.BipedalSupportPlanarRegionPublisher;
import us.ihmc.communication.configuration.NetworkParameters;

import java.io.IOException;
import java.net.URI;

import static us.ihmc.pubsub.DomainFactory.PubSubImplementation.FAST_RTPS;

public class AtlasSensorSuiteLauncher
{
   public AtlasSensorSuiteLauncher()
   {
      AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.REAL_ROBOT);

      AtlasSensorSuiteManager atlasSensorSuiteManager = (AtlasSensorSuiteManager) atlasRobotModel.getSensorSuiteManager();
      URI rosCoreURI = NetworkParameters.getROSURI();
      atlasSensorSuiteManager.initializePhysicalSensors(rosCoreURI);

      new RosModule(atlasRobotModel, rosCoreURI, null, FAST_RTPS);

      new BipedalSupportPlanarRegionPublisher(atlasRobotModel, FAST_RTPS).start();

      try
      {
         atlasSensorSuiteManager.connect();
      }
      catch (IOException e)
      {
         e.printStackTrace();
      }
   }

   public static void main(String[] args)
   {
      new AtlasSensorSuiteLauncher();
   }
}
