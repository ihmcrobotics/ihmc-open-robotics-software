package us.ihmc.quadrupedFootstepPlanning.ui.controllers;

import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.scene.control.TextField;
import javafx.stage.DirectoryChooser;
import javafx.stage.Window;
import us.ihmc.commons.PrintTools;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.tools.FootstepPlannerIOTools;

import java.io.File;
import java.net.URISyntaxException;
import java.net.URL;

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
      File file = new File(".");

      try
      {
         URL testDataFolderURL = Thread.currentThread().getContextClassLoader().getResource(FootstepPlannerIOTools.IN_DEVELOPMENT_TEST_DATA_URL);
         file = new File(testDataFolderURL.toURI());
      }
      catch(URISyntaxException e)
      {
         PrintTools.error("Could not load test data folder with URL: " + FootstepPlannerIOTools.IN_DEVELOPMENT_TEST_DATA_URL);
         e.printStackTrace();
      }

      defaultDataFolder = file;
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
      messager.submitMessage(FootstepPlannerMessagerAPI.exportUnitTestPath, newPath);
      Platform.runLater(() -> currentPlanarRegionDataFolderTextField.setText(newPath));
   }

   @FXML
   private void exportPlanarRegion()
   {
      messager.submitMessage(FootstepPlannerMessagerAPI.exportUnitTestPath, currentPlanarRegionDataFolderTextField.getText());
      messager.submitMessage(FootstepPlannerMessagerAPI.exportUnitTestDataFile, true);
   }
}
