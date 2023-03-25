package us.ihmc.pathPlanning.visibilityGraphs.ui.controllers;

import java.io.File;

import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.scene.control.TextField;
import javafx.stage.DirectoryChooser;
import javafx.stage.Window;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.pathPlanning.DataSetIOTools;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics;

public class VisibilityGraphsDataExporterAnchorPaneController
{
   private final DirectoryChooser directoryChooser = new DirectoryChooser();
   private final File defaultDataFolder;
   private Window ownerWindow;
   private JavaFXMessager messager;

   @FXML
   private TextField currentPlanarRegionDataFolderTextField;

   public VisibilityGraphsDataExporterAnchorPaneController()
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
      messager.submitMessage(UIVisibilityGraphsTopics.exportUnitTestPath, newPath);
      Platform.runLater(() -> currentPlanarRegionDataFolderTextField.setText(newPath));
   }

   @FXML
   private void exportDataSet()
   {
      messager.submitMessage(UIVisibilityGraphsTopics.exportUnitTestPath, currentPlanarRegionDataFolderTextField.getText());
      messager.submitMessage(UIVisibilityGraphsTopics.exportUnitTestDataFile, true);
   }
}
