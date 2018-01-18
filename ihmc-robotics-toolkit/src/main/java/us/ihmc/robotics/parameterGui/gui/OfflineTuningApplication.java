package us.ihmc.robotics.parameterGui.gui;

import javafx.application.Application;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;

public class OfflineTuningApplication extends Application
{
   private static final String FXML_FILE = "gui.fxml";
   private static final String CSS_FILE = "gui.css";

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      FXMLLoader mainLoader = new FXMLLoader();
      mainLoader.setLocation(getClass().getResource(FXML_FILE));
      Scene mainScene = new Scene(mainLoader.<Pane>load());
      mainScene.getStylesheets().add(getClass().getResource(CSS_FILE).toString());

      primaryStage.setScene(mainScene);
      primaryStage.setHeight(800.0);
      primaryStage.setWidth(600.0);
      primaryStage.show();
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
