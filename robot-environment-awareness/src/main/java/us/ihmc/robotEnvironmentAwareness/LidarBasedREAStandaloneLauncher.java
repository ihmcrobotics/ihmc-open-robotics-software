package us.ihmc.robotEnvironmentAwareness;

import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.depthOutputTopic;
import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.lidarOutputTopic;
import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.outputTopic;
import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.stereoOutputTopic;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.robotEnvironmentAwareness.io.FilePropertyHelper;
import us.ihmc.robotEnvironmentAwareness.ui.LIDARBasedEnvironmentAwarenessUI;
import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;
import us.ihmc.robotEnvironmentAwareness.updaters.REANetworkProvider;
import us.ihmc.robotEnvironmentAwareness.updaters.REAPlanarRegionPublicNetworkProvider;

public class LidarBasedREAStandaloneLauncher extends Application
{
   private static final String MODULE_CONFIGURATION_FILE_NAME = "./Configurations/defaultREAModuleConfiguration.txt";

   private LIDARBasedEnvironmentAwarenessUI ui;
   private LIDARBasedREAModule module;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      REANetworkProvider networkProvider = new REAPlanarRegionPublicNetworkProvider(outputTopic,
                                                                                    lidarOutputTopic,
                                                                                    stereoOutputTopic,
                                                                                    depthOutputTopic);
      ui = LIDARBasedEnvironmentAwarenessUI.creatIntraprocessUI(primaryStage);
      module = LIDARBasedREAModule.createIntraprocessModule(new FilePropertyHelper(MODULE_CONFIGURATION_FILE_NAME), networkProvider);

      ui.show();
      module.start();
   }

   @Override
   public void stop() throws Exception
   {
      ui.stop();
      module.stop();

      Platform.exit();
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
