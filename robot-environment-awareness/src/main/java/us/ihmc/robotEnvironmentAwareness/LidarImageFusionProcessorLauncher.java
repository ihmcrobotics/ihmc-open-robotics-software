package us.ihmc.robotEnvironmentAwareness;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.pubsub.DomainFactory;
import us.ihmc.robotEnvironmentAwareness.communication.LidarImageFusionAPI;
import us.ihmc.robotEnvironmentAwareness.fusion.LidarImageFusionProcessorCommunicationModule;
import us.ihmc.robotEnvironmentAwareness.fusion.LidarImageFusionProcessorUI;

public class LidarImageFusionProcessorLauncher extends Application
{
   private SharedMemoryJavaFXMessager messager;

   private LidarImageFusionProcessorUI ui;
   private LidarImageFusionProcessorCommunicationModule module;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      messager = new SharedMemoryJavaFXMessager(LidarImageFusionAPI.API);
      messager.startMessager();

      ui = LidarImageFusionProcessorUI.creatIntraprocessUI(messager, primaryStage);
      module = LidarImageFusionProcessorCommunicationModule.createIntraprocessModule(messager, DomainFactory.PubSubImplementation.FAST_RTPS);

      ui.show();
      module.start();
   }

   @Override
   public void stop() throws Exception
   {
      messager.closeMessager();

      ui.stop();
      module.stop();

      Platform.exit();
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}