package us.ihmc.robotEnvironmentAwareness.ui.controller;

import java.io.File;

import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.scene.control.TextField;
import javafx.scene.control.ToggleButton;
import javafx.stage.DirectoryChooser;
import javafx.stage.Window;
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

   @Override
   public void bindControls()
   {
      currentSegmentationDataFolderTextField.setText(defaultSegmentationDataFile.getAbsolutePath());
      currentPlanarRegionDataFolderTextField.setText(defaultPlanarRegionDataFile.getAbsolutePath());
      stereoDataFolderTextField.setText(defaultStereoDataFile.getAbsolutePath());
      uiMessager.bindBidirectionalGlobal(REAModuleAPI.UIStereoDataExportRequest, exportRecodingStereoButton.selectedProperty());
   }

   @FXML
   private void browseSegmentationOutputFolder()
   {
      segmentationDirectoryChooser.setInitialDirectory(defaultSegmentationDataFile);
      String newPath = segmentationDirectoryChooser.showDialog(ownerWindow).getAbsolutePath();
      uiMessager.submitMessageInternal(REAModuleAPI.UISegmentationDataExporterDirectory, newPath);
      Platform.runLater(() -> currentSegmentationDataFolderTextField.setText(newPath));
   }

   @FXML
   private void browsePlanarRegionOutputFolder()
   {
      planarRegionDirectoryChooser.setInitialDirectory(defaultPlanarRegionDataFile);
      String newPath = planarRegionDirectoryChooser.showDialog(ownerWindow).getAbsolutePath();
      uiMessager.submitMessageInternal(REAModuleAPI.UIPlanarRegionDataExporterDirectory, newPath);
      Platform.runLater(() -> currentPlanarRegionDataFolderTextField.setText(newPath));
   }

   @FXML
   private void browseStereoOutputFolder()
   {
      stereoDirectoryChooser.setInitialDirectory(defaultStereoDataFile);
      String newPath = stereoDirectoryChooser.showDialog(ownerWindow).getAbsolutePath();
      uiMessager.submitMessageInternal(REAModuleAPI.UIStereoDataExporterDirectory, newPath);
      Platform.runLater(() -> stereoDataFolderTextField.setText(newPath));
   }

   @FXML
   private void exportSegmentation()
   {
      uiMessager.submitMessageInternal(REAModuleAPI.UISegmentationDataExportRequest, true);
   }

   @FXML
   private void exportPlanarRegion()
   {
      uiMessager.submitMessageInternal(REAModuleAPI.UIPlanarRegionDataExportRequest, true);
   }
}
