package us.ihmc.robotEnvironmentAwareness.ui;

import javafx.fxml.FXMLLoader;
import javafx.scene.layout.BorderPane;
import javafx.stage.Stage;

public class AtlasPerceptionSuiteUI
{
   private final BorderPane mainPane;

   private final Stage primaryStage;

   private AtlasPerceptionSuiteUI(Stage primaryStage) throws Exception
   {
      this.primaryStage = primaryStage;
      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));
      mainPane = loader.load();
   }

   public static AtlasPerceptionSuiteUI creatIntraprocessUI(Stage primaryStage) throws java.lang.Exception
   {
      return new AtlasPerceptionSuiteUI(primaryStage);
   }
}
