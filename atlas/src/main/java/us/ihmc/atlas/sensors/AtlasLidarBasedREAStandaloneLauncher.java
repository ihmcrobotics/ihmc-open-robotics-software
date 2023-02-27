package us.ihmc.atlas.sensors;

import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.depthOutputTopic;
import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.lidarOutputTopic;
import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.outputTopic;
import static us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties.stereoOutputTopic;

import java.nio.file.Path;
import java.nio.file.Paths;

import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.javafx.ApplicationNoModule;
import us.ihmc.robotEnvironmentAwareness.io.FilePropertyHelper;
import us.ihmc.robotEnvironmentAwareness.ui.LIDARBasedEnvironmentAwarenessUI;
import us.ihmc.robotEnvironmentAwareness.updaters.LIDARBasedREAModule;
import us.ihmc.robotEnvironmentAwareness.updaters.REANetworkProvider;
import us.ihmc.robotEnvironmentAwareness.updaters.REAPlanarRegionPublicNetworkProvider;
import us.ihmc.tools.io.WorkspacePathTools;

public class AtlasLidarBasedREAStandaloneLauncher extends ApplicationNoModule
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
      Path configurationFilePath = WorkspacePathTools.handleWorkingDirectoryFuzziness("ihmc-open-robotics-software");
      configurationFilePath = Paths.get(configurationFilePath.toString(), "/atlas/src/main/resources/atlasREAModuleConfiguration.txt");
      module = LIDARBasedREAModule.createIntraprocessModule(new FilePropertyHelper(configurationFilePath.toFile()), networkProvider);

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
