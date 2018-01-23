package us.ihmc.robotics.parameterGui.gui;

import javafx.application.Application;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;

public class ParameterTuningApplication extends Application
{
   private GuiController controller;

   private static final String FXML_FILE = "gui.fxml";
   private static final String CSS_FILE = "gui.css";

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      FXMLLoader mainLoader = new FXMLLoader();
      mainLoader.setLocation(ParameterTuningApplication.class.getResource(FXML_FILE));
      Scene mainScene = new Scene(mainLoader.<Pane> load());
      mainScene.getStylesheets().add(ParameterTuningApplication.class.getResource(CSS_FILE).toString());
      controller = mainLoader.getController();

      primaryStage.setTitle("Parameter Tuner");
      primaryStage.setScene(mainScene);
      primaryStage.setHeight(800.0);
      primaryStage.setWidth(1400.0);
      primaryStage.show();
   }

   protected void addNetworkManager(ParameterGuiNetworkManager networkManager)
   {
      controller.addNetworkManager(networkManager);
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
