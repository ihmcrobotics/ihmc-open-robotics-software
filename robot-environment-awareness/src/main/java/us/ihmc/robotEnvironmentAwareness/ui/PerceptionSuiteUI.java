package us.ihmc.robotEnvironmentAwareness.ui;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.control.Button;
import javafx.scene.control.TextArea;
import javafx.scene.control.ToggleButton;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;
import us.ihmc.communication.util.NetworkPorts;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.communication.KryoMessager;
import us.ihmc.robotEnvironmentAwareness.communication.PerceptionSuiteAPI;
import us.ihmc.robotEnvironmentAwareness.communication.REACommunicationProperties;
import us.ihmc.robotEnvironmentAwareness.communication.REAUIMessager;

import java.io.IOException;

public class PerceptionSuiteUI
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
   @FXML
   private ToggleButton runRealSenseREAModule;
   @FXML
   private ToggleButton runRealSenseREAUI;
   @FXML
   private ToggleButton runLiveMapModule;
   @FXML
   private ToggleButton runLiveMapUI;
   @FXML
   private TextArea errorField;

   private PerceptionSuiteUI(REAUIMessager messager, Stage primaryStage) throws Exception
   {
      this.messager = messager;

      this.primaryStage = primaryStage;
      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));
      mainPane = loader.load();

      messager.bindBidirectionalGlobal(PerceptionSuiteAPI.GUIRunRealSenseSLAM, runSlamModule.selectedProperty());
      messager.bindBidirectionalGlobal(PerceptionSuiteAPI.GUIRunRealSenseSLAMUI, runSlamUI.selectedProperty());
      messager.bindBidirectionalGlobal(PerceptionSuiteAPI.GUIRunLidarREA, runLidarREAModule.selectedProperty());
      messager.bindBidirectionalGlobal(PerceptionSuiteAPI.GUIRunLidarREAUI, runLidarREAUI.selectedProperty());
      messager.bindBidirectionalGlobal(PerceptionSuiteAPI.GUIRunMapSegmentation, runMapSegmentationModule.selectedProperty());
      messager.bindBidirectionalGlobal(PerceptionSuiteAPI.GUIRunMapSegmentationUI, runMapSegmentationUI.selectedProperty());
      messager.bindBidirectionalGlobal(PerceptionSuiteAPI.GUIRunRealSenseREA, runRealSenseREAModule.selectedProperty());
      messager.bindBidirectionalGlobal(PerceptionSuiteAPI.GUIRunRealSenseREAUI, runRealSenseREAUI.selectedProperty());
      messager.bindBidirectionalGlobal(PerceptionSuiteAPI.GUIRunLiveMap, runLiveMapModule.selectedProperty());
      messager.bindBidirectionalGlobal(PerceptionSuiteAPI.GUIRunLiveMapUI, runLiveMapUI.selectedProperty());

      messager.bindBidirectionalGlobal(PerceptionSuiteAPI.RunRealSenseSLAM, runSlamModule.selectedProperty());
      messager.bindBidirectionalGlobal(PerceptionSuiteAPI.RunRealSenseSLAMUI, runSlamUI.selectedProperty());
      messager.bindBidirectionalGlobal(PerceptionSuiteAPI.RunLidarREA, runLidarREAModule.selectedProperty());
      messager.bindBidirectionalGlobal(PerceptionSuiteAPI.RunLidarREAUI, runLidarREAUI.selectedProperty());
      messager.bindBidirectionalGlobal(PerceptionSuiteAPI.RunMapSegmentation, runMapSegmentationModule.selectedProperty());
      messager.bindBidirectionalGlobal(PerceptionSuiteAPI.RunMapSegmentationUI, runMapSegmentationUI.selectedProperty());
      messager.bindBidirectionalGlobal(PerceptionSuiteAPI.RunRealSenseREA, runRealSenseREAModule.selectedProperty());
      messager.bindBidirectionalGlobal(PerceptionSuiteAPI.RunRealSenseREAUI, runRealSenseREAUI.selectedProperty());
      messager.bindBidirectionalGlobal(PerceptionSuiteAPI.RunLiveMap, runLiveMapModule.selectedProperty());
      messager.bindBidirectionalGlobal(PerceptionSuiteAPI.RunLiveMapUI, runLiveMapUI.selectedProperty());

      messager.registerTopicListener(PerceptionSuiteAPI.ErrorMessage, errorField::setText);

      primaryStage.setTitle(getClass().getSimpleName());
      Scene mainScene = new Scene(mainPane, 594, 200);

      primaryStage.setScene(mainScene);
      primaryStage.setOnCloseRequest(event -> stop());
   }

   public void clearError()
   {
      errorField.clear();
   }

   public void show() throws IOException
   {
      primaryStage.show();
   }

   public void stop()
   {
   }


   public static PerceptionSuiteUI createIntraprocessUI(Stage primaryStage, Messager messager) throws java.lang.Exception
   {
      REAUIMessager uiMessager = new REAUIMessager(messager);
      uiMessager.startMessager();
      return new PerceptionSuiteUI(uiMessager, primaryStage);
   }
}
