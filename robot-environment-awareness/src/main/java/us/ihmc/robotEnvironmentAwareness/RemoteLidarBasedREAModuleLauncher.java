package us.ihmc.robotEnvironmentAwareness;

import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;
import us.ihmc.robotEnvironmentAwareness.updaters.REANetworkProvider;
import us.ihmc.robotEnvironmentAwareness.updaters.REAPlanarRegionPublicNetworkProvider;

import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.*;
import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.depthOutputTopic;

public class RemoteLidarBasedREAModuleLauncher
{
   private static final String MODULE_CONFIGURATION_FILE_NAME = "./Configurations/defaultREAModuleConfiguration.txt";

   public static void main(String[] args) throws Exception
   {
      REANetworkProvider networkProvider = new REAPlanarRegionPublicNetworkProvider(outputTopic,
                                                                                    lidarOutputTopic,
                                                                                    stereoOutputTopic,
                                                                                    depthOutputTopic);
      LIDARBasedREAModule remoteModule = LIDARBasedREAModule.createRemoteModule(MODULE_CONFIGURATION_FILE_NAME, networkProvider);
      remoteModule.start();
   }
}
