package us.ihmc.atlas.sensors;

import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.javafx.ApplicationNoModule;
import us.ihmc.messager.javafx.SharedMemoryJavaFXMessager;
import us.ihmc.robotEnvironmentAwareness.communication.PerceptionSuiteAPI;
import us.ihmc.robotEnvironmentAwareness.ui.PerceptionSuiteUI;

public class AtlasPerceptionSuiteLauncher extends ApplicationNoModule
{
   private SharedMemoryJavaFXMessager messager;
   private AtlasPerceptionSuite module;
   private PerceptionSuiteUI ui;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      messager = new SharedMemoryJavaFXMessager(PerceptionSuiteAPI.API);
      messager.startMessager();

      module = AtlasPerceptionSuite.createIntraprocess(messager);
      ui = PerceptionSuiteUI.createIntraprocessUI(primaryStage, messager);

      module.start();
      ui.show();

      primaryStage.setOnCloseRequest(event -> stop());
   }

   @Override
   public void stop()
   {
      module.stop();
      ui.stop();
      messager.closeMessager();

      Platform.exit();
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
