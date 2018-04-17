package us.ihmc.pathPlanning.visibilityGraphs.ui.controllers;

import java.io.File;
import java.net.URISyntaxException;
import java.net.URL;

import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.scene.control.TextField;
import javafx.stage.DirectoryChooser;
import javafx.stage.Window;
import us.ihmc.commons.PrintTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityGraphsIOTools;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.SimpleUIMessager;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics;

public class VisibilityGraphsDataExporterAnchorPaneController
{
   private final DirectoryChooser directoryChooser = new DirectoryChooser();
   private final File defaultDataFolder;
   private Window ownerWindow;
   private SimpleUIMessager messager;

   @FXML
   private TextField currentPlanarRegionDataFolderTextField;

   public VisibilityGraphsDataExporterAnchorPaneController()
   {
      File file = new File(".");

      try
      {
         URL testDataFolderURL = Thread.currentThread().getContextClassLoader().getResource(VisibilityGraphsIOTools.TEST_DATA_URL);
         file = new File(testDataFolderURL.toURI());
      }
      catch(URISyntaxException e)
      {
         PrintTools.error("Could not load test data folder with URL: " + VisibilityGraphsIOTools.TEST_DATA_URL);
         e.printStackTrace();
      }

      defaultDataFolder = file;
   }

   public void setMainWindow(Window ownerWindow)
   {
      this.ownerWindow = ownerWindow;
   }

   public void attachMessager(SimpleUIMessager messager)
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
   private void exportPlanarRegion()
   {
      messager.submitMessage(UIVisibilityGraphsTopics.exportUnitTestPath, currentPlanarRegionDataFolderTextField.getText());
      messager.submitMessage(UIVisibilityGraphsTopics.exportUnitTestDataFile, true);
   }
}
