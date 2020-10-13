package us.ihmc.robotEnvironmentAwareness;

import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.depthOutputTopic;
import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.lidarOutputTopic;
import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.outputTopic;
import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.stereoOutputTopic;

import us.ihmc.robotEnvironmentAwareness.io.FilePropertyHelper;
import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;
import us.ihmc.robotEnvironmentAwareness.updaters.REANetworkProvider;
import us.ihmc.robotEnvironmentAwareness.updaters.REAPlanarRegionPublicNetworkProvider;

public class RemoteLidarBasedREAModuleLauncher
{
   private static final String MODULE_CONFIGURATION_FILE_NAME = "./Configurations/defaultREAModuleConfiguration.txt";

   public static void main(String[] args) throws Exception
   {
      REANetworkProvider networkProvider = new REAPlanarRegionPublicNetworkProvider(outputTopic, lidarOutputTopic, stereoOutputTopic, depthOutputTopic);
      LIDARBasedREAModule remoteModule = LIDARBasedREAModule.createRemoteModule(new FilePropertyHelper(MODULE_CONFIGURATION_FILE_NAME), networkProvider);
      remoteModule.start();
   }
}
