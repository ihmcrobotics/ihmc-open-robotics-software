package us.ihmc.atlas.sensors;

import us.ihmc.atlas.AtlasRobotModel;
import us.ihmc.atlas.AtlasRobotVersion;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.networkProcessor.modules.RosModule;
import us.ihmc.avatar.networkProcessor.supportingPlanarRegionPublisher.BipedalSupportPlanarRegionPublisher;
import us.ihmc.communication.configuration.NetworkParameters;

import java.net.URI;

public class AtlasSensorSuiteLauncher
{
   public AtlasSensorSuiteLauncher()
   {
      AtlasRobotModel atlasRobotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_DUAL_ROBOTIQ, RobotTarget.REAL_ROBOT);

      AtlasSensorSuiteManager atlasSensorSuiteManager = atlasRobotModel.getSensorSuiteManager();
      URI rosCoreURI = NetworkParameters.getROSURI();
      atlasSensorSuiteManager.initializePhysicalSensors(rosCoreURI);

      new RosModule(atlasRobotModel, rosCoreURI, null);

      new BipedalSupportPlanarRegionPublisher(atlasRobotModel).start();

      atlasSensorSuiteManager.connect();
   }

   public static void main(String[] args)
   {
      new AtlasSensorSuiteLauncher();
   }
}
