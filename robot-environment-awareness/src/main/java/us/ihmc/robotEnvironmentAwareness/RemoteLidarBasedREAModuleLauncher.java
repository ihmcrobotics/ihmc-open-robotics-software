package us.ihmc.robotEnvironmentAwareness;

import java.io.IOException;

import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;

public class RemoteLidarBasedREAModuleLauncher
{
   private static final String MODULE_CONFIGURATION_FILE_NAME = "./Configurations/defaultREAModuleConfiguration.txt";

   public static void main(String[] args) throws IOException
   {
      LIDARBasedREAModule remoteModule = LIDARBasedREAModule.createRemoteModule(MODULE_CONFIGURATION_FILE_NAME);
      remoteModule.start();
   }
}
