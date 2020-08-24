package us.ihmc.atlas.sensors;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.robotEnvironmentAwareness.io.FilePropertyHelper;
import us.ihmc.robotEnvironmentAwareness.ui.LIDARBasedEnvironmentAwarenessUI;
import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;
import us.ihmc.robotEnvironmentAwareness.updaters.REANetworkProvider;
import us.ihmc.robotEnvironmentAwareness.updaters.REAPlanarRegionPublicNetworkProvider;

import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.*;

public class AtlasLidarBasedREAStandaloneLauncher extends Application
{
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
      module = LIDARBasedREAModule.createIntraprocessModule(new FilePropertyHelper(this.getClass().getResource("atlasREAModuleConfiguration.txt")), networkProvider);

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
