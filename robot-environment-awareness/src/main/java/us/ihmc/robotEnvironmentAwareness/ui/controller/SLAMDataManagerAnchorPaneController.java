package us.ihmc.robotEnvironmentAwareness.ui.controller;

import java.io.File;
import java.nio.file.Path;
import java.nio.file.Paths;
import java.util.concurrent.Executor;

import controller_msgs.msg.dds.PlanarRegionsListMessage;
import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.scene.control.TextField;
import javafx.scene.control.ToggleButton;
import javafx.stage.DirectoryChooser;
import javafx.stage.Window;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.robotEnvironmentAwareness.communication.SLAMModuleAPI;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools;
import us.ihmc.robotEnvironmentAwareness.tools.ExecutorServiceTools.ExceptionHandling;
import us.ihmc.robotics.PlanarRegionFileTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class SLAMDataManagerAnchorPaneController extends REABasicUIController
{
   @FXML
   private ToggleButton exportRawData;

   @FXML
   private TextField currentRawDataOutputFolderTextField;
   @FXML
   private TextField currentSLAMDataOutputFolderTextField;
   @FXML
   private TextField currentRawDataInputFolderTextField;
   @FXML
   private TextField currentPlanarRegionsInputFolderTextField;

   private final DirectoryChooser exportRawDataDirectoryChooser = new DirectoryChooser();
   private final DirectoryChooser exportSLAMDataDirectoryChooser = new DirectoryChooser();
   private final DirectoryChooser importRawDataDirectoryChooser = new DirectoryChooser();
   private final DirectoryChooser importPlanarRegionsDirectoryChooser = new DirectoryChooser();

   private final File defaultExportRawDataFile = new File("Data/SLAM/RawData/");
   private final File defaultExportSLAMDataFile = new File("Data/SLAM/SLAMData/");
   private final File defaultImportRawDataFile = new File("Data/SLAM/RawData/");
   private final File defaultImportPlanarRegionsFile = new File("Data/SLAM/PlanarRegion/");

   private Window ownerWindow;

   public SLAMDataManagerAnchorPaneController()
   {

   }

   public void setMainWindow(Window ownerWindow)
   {
      this.ownerWindow = ownerWindow;
   }

   @Override
   public void bindControls()
   {
      currentRawDataOutputFolderTextField.setText(defaultExportRawDataFile.getAbsolutePath());
      currentSLAMDataOutputFolderTextField.setText(defaultExportSLAMDataFile.getAbsolutePath());
      currentRawDataInputFolderTextField.setText(defaultImportRawDataFile.getAbsolutePath());
      currentPlanarRegionsInputFolderTextField.setText(defaultImportPlanarRegionsFile.getAbsolutePath());
      uiMessager.bindBidirectionalGlobal(SLAMModuleAPI.UIRawDataExportRequest, exportRawData.selectedProperty());
   }

   private final Executor importExecutor = ExecutorServiceTools.newSingleThreadScheduledExecutor(getClass(), ExceptionHandling.CANCEL_AND_REPORT);

   @FXML
   private void exportSLAMData()
   {
      //TODO: implement
   }

   @FXML
   private void importRawData()
   {
      //TODO: implement
   }

   @FXML
   private void importPlanarRegions()
   {
      String planarRegionsFilePath = currentPlanarRegionsInputFolderTextField.getText();
      File planarRegionsFile = new File(planarRegionsFilePath);
      PlanarRegionsList planarRegionDataToImport = PlanarRegionFileTools.importPlanarRegionData(planarRegionsFile);
      PlanarRegionsListMessage planarRegionsListMessage = PlanarRegionMessageConverter.convertToPlanarRegionsListMessage(planarRegionDataToImport);
      uiMessager.submitMessageInternal(SLAMModuleAPI.ImportedPlanarRegionsState, planarRegionsListMessage);
      uiMessager.submitMessageInternal(SLAMModuleAPI.ShowImportedPlanarRegions, true);
   }

   @FXML
   private void browseRawDataOutputFolder()
   {
      exportRawDataDirectoryChooser.setInitialDirectory(defaultExportRawDataFile);
      String newPath = exportRawDataDirectoryChooser.showDialog(ownerWindow).getAbsolutePath();
      uiMessager.submitMessageInternal(SLAMModuleAPI.UIRawDataExportDirectory, newPath);
      Platform.runLater(() -> currentRawDataOutputFolderTextField.setText(newPath));
   }

   @FXML
   private void browseSLAMDataOutputFolder()
   {
      exportSLAMDataDirectoryChooser.setInitialDirectory(defaultExportSLAMDataFile);
      String newPath = exportSLAMDataDirectoryChooser.showDialog(ownerWindow).getAbsolutePath();
      uiMessager.submitMessageInternal(SLAMModuleAPI.UISLAMDataExportDirectory, newPath);
      Platform.runLater(() -> currentSLAMDataOutputFolderTextField.setText(newPath));
   }

   @FXML
   private void browseRawDataInputFolder()
   {
      importRawDataDirectoryChooser.setInitialDirectory(defaultImportRawDataFile);
      String newPath = importRawDataDirectoryChooser.showDialog(ownerWindow).getAbsolutePath();
      Platform.runLater(() -> currentRawDataInputFolderTextField.setText(newPath));
   }

   @FXML
   private void browsePlanarRegionsInputFolder()
   {
      importPlanarRegionsDirectoryChooser.setInitialDirectory(defaultImportPlanarRegionsFile);
      String newPath = importPlanarRegionsDirectoryChooser.showDialog(ownerWindow).getAbsolutePath();
      Platform.runLater(() -> currentPlanarRegionsInputFolderTextField.setText(newPath));
   }
}
