package us.ihmc.footstepPlanning.ui.controllers;

import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.scene.control.TextField;
import javafx.stage.DirectoryChooser;
import javafx.stage.Window;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.pathPlanning.DataSetIOTools;

import java.io.File;

public class FootstepPlannerDataExporterAnchorPaneController
{
   private final DirectoryChooser directoryChooser = new DirectoryChooser();
   private final File defaultDataFolder;
   private Window ownerWindow;
   private JavaFXMessager messager;

   @FXML
   private TextField currentPlanarRegionDataFolderTextField;

   public FootstepPlannerDataExporterAnchorPaneController()
   {
      this.defaultDataFolder = new File(DataSetIOTools.RESOURCES_DIRECTORY + File.separator + DataSetIOTools.DATA_SET_DIRECTORY_PATH);
   }

   public void setMainWindow(Window ownerWindow)
   {
      this.ownerWindow = ownerWindow;
   }

   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;
      currentPlanarRegionDataFolderTextField.setText(defaultDataFolder.getAbsolutePath());
   }

   @FXML
   private void browsePlanarRegionOutputFolder()
   {
      directoryChooser.setInitialDirectory(defaultDataFolder);
      File result = directoryChooser.showDialog(ownerWindow);
      if (result == null)
         return;
      String newPath = result.getAbsolutePath();
      messager.submitMessage(FootstepPlannerMessagerAPI.ExportUnitTestPath, newPath);
      Platform.runLater(() -> currentPlanarRegionDataFolderTextField.setText(newPath));
   }

   @FXML
   private void exportPlanarRegion()
   {
      messager.submitMessage(FootstepPlannerMessagerAPI.ExportUnitTestPath, currentPlanarRegionDataFolderTextField.getText());
      messager.submitMessage(FootstepPlannerMessagerAPI.ExportUnitTestDataFile, true);
   }
}
