package us.ihmc.robotics.parameterGui.gui;

import java.io.File;
import java.io.FileInputStream;
import java.util.List;

import javafx.application.Application;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.layout.Pane;
import javafx.stage.FileChooser;
import javafx.stage.Stage;
import us.ihmc.robotics.parameterGui.ParameterTuningTools;
import us.ihmc.yoVariables.parameters.xml.Registry;

public class OfflineTuningApplication extends Application
{
   private static final String FXML_FILE = "gui.fxml";
   private static final String CSS_FILE = "gui.css";

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      FileChooser fileChooser = new FileChooser();
      fileChooser.setTitle("Select Parameter File");
      File file = fileChooser.showOpenDialog(primaryStage);
      List<Registry> registries = ParameterTuningTools.getParameters(new FileInputStream(file));

      FXMLLoader mainLoader = new FXMLLoader();
      mainLoader.setLocation(getClass().getResource(FXML_FILE));
      Scene mainScene = new Scene(mainLoader.<Pane>load());

      GuiController controller = mainLoader.getController();
      controller.setRegistries(registries);

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
