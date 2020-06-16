package us.ihmc.robotEnvironmentAwareness.ui.controller;

import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.scene.control.TextField;
import javafx.scene.control.ToggleButton;
import javafx.stage.DirectoryChooser;
import javafx.stage.Window;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;

import java.io.File;

public class StereoDataExporterAnchorPaneController extends REABasicUIController
{
   private final File defaultStereoDataFile = new File("Data/");

   @FXML
   private TextField stereoDataFolderTextField;
   @FXML
   private ToggleButton exportRecodingStereoButton;

   private final DirectoryChooser stereoDirectoryChooser = new DirectoryChooser();
   private Window ownerWindow;

   public StereoDataExporterAnchorPaneController()
   {
   }

   public void setMainWindow(Window ownerWindow)
   {
      this.ownerWindow = ownerWindow;
   }

   private Topic<Boolean> uiStereoDataExportRequestTopic = REAModuleAPI.UIStereoDataExportRequest;
   private Topic<String> uiStereoDataExporterDirectoryTopic = REAModuleAPI.UIStereoDataExporterDirectory;

   public void setUIStereoDataExportRequestTopic(Topic<Boolean> uiStereoDataExportRequestTopic)
   {
      this.uiStereoDataExportRequestTopic = uiStereoDataExportRequestTopic;
   }

   public void setUiStereoDataExporterDirectoryTopic(Topic<String> uiStereoDataExporterDirectoryTopic)
   {
      this.uiStereoDataExporterDirectoryTopic = uiStereoDataExporterDirectoryTopic;
   }

   @Override
   public void bindControls()
   {
      stereoDataFolderTextField.setText(defaultStereoDataFile.getAbsolutePath());
      uiMessager.bindBidirectionalGlobal(uiStereoDataExportRequestTopic, exportRecodingStereoButton.selectedProperty());
   }

   @FXML
   private void browseStereoOutputFolder()
   {
      stereoDirectoryChooser.setInitialDirectory(defaultStereoDataFile);
      String newPath = stereoDirectoryChooser.showDialog(ownerWindow).getAbsolutePath();
      uiMessager.submitMessageInternal(uiStereoDataExporterDirectoryTopic, newPath);
      Platform.runLater(() -> stereoDataFolderTextField.setText(newPath));
   }


}
