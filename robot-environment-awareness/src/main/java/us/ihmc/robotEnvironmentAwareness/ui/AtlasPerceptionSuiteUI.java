package us.ihmc.robotEnvironmentAwareness.ui;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.control.ToggleButton;
import javafx.scene.input.KeyCode;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.communication.KryoMessager;
import us.ihmc.robotEnvironmentAwareness.communication.PerceptionSuiteAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;

import java.io.IOException;

public class AtlasPerceptionSuiteUI
{
   private final Pane mainPane;
   private final Stage primaryStage;

   private final REAUIMessager messager;

   @FXML
   private ToggleButton runSlamModule;
   @FXML
   private ToggleButton runSlamUI;
   @FXML
   private ToggleButton runLidarREAModule;
   @FXML
   private ToggleButton runLidarREAUI;
   @FXML
   private ToggleButton runMapSegmentationModule;
   @FXML
   private ToggleButton runMapSegmentationUI;

   private AtlasPerceptionSuiteUI(REAUIMessager messager, Stage primaryStage) throws Exception
   {
      this.messager = messager;
      this.primaryStage = primaryStage;
      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));
      mainPane = loader.load();

      messager.bindBidirectionalGlobal(PerceptionSuiteAPI.RunRealSenseSLAM, runSlamModule.selectedProperty());
      messager.bindBidirectionalGlobal(PerceptionSuiteAPI.RunRealSenseSLAMUI, runSlamUI.selectedProperty());
      messager.bindBidirectionalGlobal(PerceptionSuiteAPI.RunLidarREA, runLidarREAModule.selectedProperty());
      messager.bindBidirectionalGlobal(PerceptionSuiteAPI.RunLidarREAUI, runLidarREAUI.selectedProperty());
      messager.bindBidirectionalGlobal(PerceptionSuiteAPI.RunMapSegmentation, runMapSegmentationModule.selectedProperty());
      messager.bindBidirectionalGlobal(PerceptionSuiteAPI.RunMapSegmentationUI, runMapSegmentationUI.selectedProperty());

      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(true);
      Scene mainScene = new Scene(mainPane, 600, 400);

      primaryStage.setScene(mainScene);
      primaryStage.setOnCloseRequest(event -> stop());
   }

   public void show() throws IOException
   {
      primaryStage.show();
   }

   public void stop()
   {
      try
      {
         messager.closeMessager();
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
   }


   public static AtlasPerceptionSuiteUI createIntraprocessUI(Stage primaryStage) throws java.lang.Exception
   {
      Messager moduleMessager = KryoMessager.createIntraprocess(PerceptionSuiteAPI.API,
                                                                NetworkPorts.SLAM_MODULE_UI_PORT,
                                                                REACommunicationProperties.getPrivateNetClassList());
      REAUIMessager uiMessager = new REAUIMessager(moduleMessager);
      return new AtlasPerceptionSuiteUI(uiMessager, primaryStage);
   }
}
