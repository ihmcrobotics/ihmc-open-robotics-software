package us.ihmc.robotEnvironmentAwareness.polygonizer;

import javafx.stage.Stage;
import us.ihmc.javafx.ApplicationNoModule;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.messager.javafx.SharedMemoryJavaFXMessager;

public class PolygonizerVisualizerLauncher extends ApplicationNoModule
{
   private PolygonizerVisualizerUI ui;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      JavaFXMessager messager = new SharedMemoryJavaFXMessager(PolygonizerVisualizerUI.getMessagerAPI());
      ui = new PolygonizerVisualizerUI(messager, primaryStage);
      ui.show();
   }

   @Override
   public void stop() throws Exception
   {
      super.stop();
      ui.stop();
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
