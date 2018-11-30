package us.ihmc.robotEnvironmentAwareness;

import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;

public class RemoteLidarBasedREAModuleLauncher
{
   private static final String MODULE_CONFIGURATION_FILE_NAME = "./Configurations/defaultREAModuleConfiguration.txt";

   public static void main(String[] args) throws Exception
   {
      LIDARBasedREAModule remoteModule = LIDARBasedREAModule.createRemoteModule(MODULE_CONFIGURATION_FILE_NAME);
      remoteModule.start();
   }
}
