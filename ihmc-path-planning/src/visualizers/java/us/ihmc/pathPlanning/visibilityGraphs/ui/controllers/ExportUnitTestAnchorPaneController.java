package us.ihmc.pathPlanning.visibilityGraphs.ui.controllers;

import java.io.File;
import java.nio.file.Path;

import javafx.fxml.FXML;
import javafx.stage.DirectoryChooser;
import javafx.stage.Window;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.SimpleUIMessager;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics;

public class ExportUnitTestAnchorPaneController
{
   private final DirectoryChooser planarRegionDirectoryChooser = new DirectoryChooser();
   private final File defaultPlanarRegionDataFile = new File("Data/PlanarRegion/");
   private Window ownerWindow;
   private SimpleUIMessager messager;

   Path folderPath;

   public void setMainWindow(Window ownerWindow)
   {
      this.ownerWindow = ownerWindow;
   }
   
   public void attachMessager(SimpleUIMessager messager)
   {
      this.messager = messager;
   }
   
   public void bindControls()
   {
   }

   @FXML
   private void browsePlanarRegionOutputFolder()
   {
      planarRegionDirectoryChooser.setInitialDirectory(defaultPlanarRegionDataFile);
      String newPath = planarRegionDirectoryChooser.showDialog(ownerWindow).getAbsolutePath();
      
      messager.submitMessage(UIVisibilityGraphsTopics.exportUnitTestPath, newPath);
   }

   @FXML
   private void exportPlanarRegion()
   {
      messager.submitMessage(UIVisibilityGraphsTopics.exportUnitTestDataFile, true);
   }
}
