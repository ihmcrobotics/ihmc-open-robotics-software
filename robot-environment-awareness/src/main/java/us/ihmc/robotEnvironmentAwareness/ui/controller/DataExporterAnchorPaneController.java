package us.ihmc.robotEnvironmentAwareness.ui.controller;

import java.io.File;

import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.scene.control.TextField;
import javafx.scene.control.ToggleButton;
import javafx.stage.DirectoryChooser;
import javafx.stage.Window;
import us.ihmc.messager.MessagerAPIFactory.Topic;
import us.ihmc.robotEnvironmentAwareness.communication.REAModuleAPI;

public class DataExporterAnchorPaneController extends REABasicUIController
{
   private final File defaultSegmentationDataFile = new File("Data/Segmentation/");
   private final File defaultPlanarRegionDataFile = new File("Data/PlanarRegion/");
   private final File defaultStereoDataFile = new File("Data/");

   @FXML
   private TextField currentSegmentationDataFolderTextField;
   @FXML
   private TextField currentPlanarRegionDataFolderTextField;
   @FXML
   private TextField stereoDataFolderTextField;
   @FXML
   private ToggleButton exportRecodingStereoButton;

   private final DirectoryChooser segmentationDirectoryChooser = new DirectoryChooser();
   private final DirectoryChooser planarRegionDirectoryChooser = new DirectoryChooser();
   private final DirectoryChooser stereoDirectoryChooser = new DirectoryChooser();
   private Window ownerWindow;

   public DataExporterAnchorPaneController()
   {
   }

   public void setMainWindow(Window ownerWindow)
   {
      this.ownerWindow = ownerWindow;
   }

   private Topic<Boolean> uiStereoDataExportRequestTopic = REAModuleAPI.UIStereoDataExportRequest;
   private Topic<Boolean> uiSegmentationDataExportRequestTopic = REAModuleAPI.UISegmentationDataExportRequest;
   private Topic<Boolean> uiPlanarRegionDataExportRequestTopic = REAModuleAPI.UIPlanarRegionDataExportRequest;
   private Topic<String> uiStereoDataExporterDirectoryTopic = REAModuleAPI.UIStereoDataExporterDirectory;
   private Topic<String> uiSegmentationDataExporterDirectoryTopic = REAModuleAPI.UISegmentationDataExporterDirectory;
   private Topic<String> uiPlanarRegionDataExporterDirectoryTopic = REAModuleAPI.UIPlanarRegionDataExporterDirectory;

   public void setUIStereoDataExportRequestTopic(Topic<Boolean> uiStereoDataExportRequestTopic)
   {
      this.uiStereoDataExportRequestTopic = uiStereoDataExportRequestTopic;
   }

   public void setUiSegmentationDataExportRequestTopic(Topic<Boolean> uiSegmentationDataExportRequestTopic)
   {
      this.uiSegmentationDataExportRequestTopic = uiSegmentationDataExportRequestTopic;
   }

   public void setUiPlanarRegionDataExportRequestTopic(Topic<Boolean> uiPlanarRegionDataExportRequestTopic)
   {
      this.uiPlanarRegionDataExportRequestTopic = uiPlanarRegionDataExportRequestTopic;
   }

   public void setUiStereoDataExporterDirectoryTopic(Topic<String> uiStereoDataExporterDirectoryTopic)
   {
      this.uiStereoDataExporterDirectoryTopic = uiStereoDataExporterDirectoryTopic;
   }

   public void setUiSegmentationDataExporterDirectoryTopic(Topic<String> uiSegmentationDataExporterDirectoryTopic)
   {
      this.uiSegmentationDataExporterDirectoryTopic = uiSegmentationDataExporterDirectoryTopic;
   }

   public void setUiPlanarRegionDataExporterDirectoryTopic(Topic<String> uiPlanarRegionDataExporterDirectoryTopic)
   {
      this.uiPlanarRegionDataExporterDirectoryTopic = uiPlanarRegionDataExporterDirectoryTopic;
   }

   @Override
   public void bindControls()
   {
      currentSegmentationDataFolderTextField.setText(defaultSegmentationDataFile.getAbsolutePath());
      currentPlanarRegionDataFolderTextField.setText(defaultPlanarRegionDataFile.getAbsolutePath());
      stereoDataFolderTextField.setText(defaultStereoDataFile.getAbsolutePath());
      uiMessager.bindBidirectionalGlobal(uiStereoDataExportRequestTopic, exportRecodingStereoButton.selectedProperty());
   }

   @FXML
   private void exportSegmentation()
   {
      uiMessager.submitMessageInternal(uiSegmentationDataExportRequestTopic, true);
   }

   @FXML
   private void exportPlanarRegion()
   {
      uiMessager.submitMessageInternal(uiPlanarRegionDataExportRequestTopic, true);
   }

   @FXML
   private void browseSegmentationOutputFolder()
   {
      segmentationDirectoryChooser.setInitialDirectory(defaultSegmentationDataFile);
      String newPath = segmentationDirectoryChooser.showDialog(ownerWindow).getAbsolutePath();
      uiMessager.submitMessageInternal(uiSegmentationDataExporterDirectoryTopic, newPath);
      Platform.runLater(() -> currentSegmentationDataFolderTextField.setText(newPath));
   }

   @FXML
   private void browsePlanarRegionOutputFolder()
   {
      planarRegionDirectoryChooser.setInitialDirectory(defaultPlanarRegionDataFile);
      String newPath = planarRegionDirectoryChooser.showDialog(ownerWindow).getAbsolutePath();
      uiMessager.submitMessageInternal(uiPlanarRegionDataExporterDirectoryTopic, newPath);
      Platform.runLater(() -> currentPlanarRegionDataFolderTextField.setText(newPath));
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
