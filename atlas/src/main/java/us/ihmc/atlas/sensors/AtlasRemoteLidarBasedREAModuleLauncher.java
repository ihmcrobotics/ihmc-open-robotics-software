package us.ihmc.atlas.sensors;

import com.esotericsoftware.minlog.Log;
import us.ihmc.commons.exception.DefaultExceptionHandler;
import us.ihmc.commons.exception.ExceptionTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.messager.kryo.KryoMessager;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;
import us.ihmc.robotEnvironmentAwareness.io.FilePropertyHelper;
import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;
import us.ihmc.robotEnvironmentAwareness.updaters.REANetworkProvider;
import us.ihmc.robotEnvironmentAwareness.updaters.REAPlanarRegionPublicNetworkProvider;

import java.io.File;
import java.nio.file.Paths;

import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.*;

public class AtlasRemoteLidarBasedREAModuleLauncher
{
   private static final String MODULE_CONFIGURATION_FILE_NAME = "atlasREAModuleConfiguration.txt";

   public static void main(String[] args) throws Exception
   {
      Log.TRACE();

      REANetworkProvider networkProvider = new REAPlanarRegionPublicNetworkProvider(outputTopic, lidarOutputTopic, stereoOutputTopic, depthOutputTopic);
      File configurationFile = Paths.get(System.getProperty("user.home"), ".ihmc", MODULE_CONFIGURATION_FILE_NAME).toFile();

//      KryoMessager messager = KryoMessager.createTCPServer(REAModuleAPI.API, NetworkPorts.REA_MODULE_UI_PORT, getPrivateNetClassList());
//      messager.setAllowSelfSubmit(true);
//      messager.startMessager();

//      KryoMessager messager = KryoMessager.createServer(REAModuleAPI.API, NetworkPorts.REA_MODULE_UI_PORT.getPort(), "KryoREAUI", 1);
//      messager.startMessagerBlocking();

      SharedMemoryMessager messager = new SharedMemoryMessager(REAModuleAPI.API);
      messager.startMessager();

      LIDARBasedREAModule remoteModule = new LIDARBasedREAModule(messager, new FilePropertyHelper(configurationFile), networkProvider);
      remoteModule.start();
   }
}
