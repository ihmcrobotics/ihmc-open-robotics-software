package us.ihmc.pathPlanning.visibilityGraphs.ui.controllers;

import java.io.File;

import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.scene.control.TextField;
import javafx.stage.DirectoryChooser;
import javafx.stage.Window;
import us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityGraphsIOTools;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.SimpleUIMessager;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics;

public class VisibilityGraphsDataExporterAnchorPaneController
{
   private final DirectoryChooser planarRegionDirectoryChooser = new DirectoryChooser();
   private final File defaultPlanarRegionDataFile;
   private Window ownerWindow;
   private SimpleUIMessager messager;

   @FXML
   private TextField currentPlanarRegionDataFolderTextField;

   public VisibilityGraphsDataExporterAnchorPaneController()
   {
      File file = new File("..\\test\\resources\\" + VisibilityGraphsIOTools.DATA_FOLDER_NAME);
      if (!file.exists())
         file = new File(".");
      defaultPlanarRegionDataFile = file;
   }

   public void setMainWindow(Window ownerWindow)
   {
      this.ownerWindow = ownerWindow;
   }

   public void attachMessager(SimpleUIMessager messager)
   {
      this.messager = messager;
      currentPlanarRegionDataFolderTextField.setText(defaultPlanarRegionDataFile.getAbsolutePath());
   }

   @FXML
   private void browsePlanarRegionOutputFolder()
   {
      planarRegionDirectoryChooser.setInitialDirectory(defaultPlanarRegionDataFile);
      File result = planarRegionDirectoryChooser.showDialog(ownerWindow);
      if (result == null)
         return;
      String newPath = result.getAbsolutePath();
      messager.submitMessage(UIVisibilityGraphsTopics.exportUnitTestPath, newPath);
      Platform.runLater(() -> currentPlanarRegionDataFolderTextField.setText(newPath));
   }

   @FXML
   private void exportPlanarRegion()
   {
      messager.submitMessage(UIVisibilityGraphsTopics.exportUnitTestPath, currentPlanarRegionDataFolderTextField.getText());
      messager.submitMessage(UIVisibilityGraphsTopics.exportUnitTestDataFile, true);
   }
}
