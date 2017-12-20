package us.ihmc.pathPlanning.visibilityGraphs.ui.controllers;

import java.io.File;
import java.io.FilenameFilter;

import javafx.fxml.FXML;
import javafx.scene.control.MenuItem;
import javafx.stage.DirectoryChooser;
import javafx.stage.Window;
import us.ihmc.commons.PrintTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.VisibilityGraphsIOTools;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;
import us.ihmc.robotics.PlanarRegionFileTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class SimpleUIMenuController
{
   private static final boolean VERBOSE = true;

   private final DirectoryChooser directoryChooser = new DirectoryChooser();

   @FXML
   private MenuItem reloadMenuItem;

   private REAMessager messager;
   private Window ownerWindow;

   private File loadedFile = null;

   public SimpleUIMenuController()
   {
      File defaultDataFolder = new File("..\\visualizers\\resources\\Data");
      if (!defaultDataFolder.exists())
         defaultDataFolder = new File(".");
      directoryChooser.setInitialDirectory(defaultDataFolder);
   }

   public void attachMessager(REAMessager messager)
   {
      this.messager = messager;
   }

   public void setMainWindow(Window ownerWindow)
   {
      this.ownerWindow = ownerWindow;
      reloadMenuItem.setDisable(true);
   }

   @FXML
   public void loadPlanarRegion()
   {
      File result = directoryChooser.showDialog(ownerWindow);
      if (result == null)
         return;

      File[] vizGraphsParameterFile = result.listFiles((FilenameFilter) (dir, name) -> name.equals(VisibilityGraphsIOTools.INPUTS_PARAMETERS_FILENAME));
      if (vizGraphsParameterFile != null && vizGraphsParameterFile.length == 1)
         loadedFile = result.listFiles(File::isDirectory)[0];
      else
         loadedFile = result;
      PlanarRegionsList loadedPlanarRegions = PlanarRegionFileTools.importPlanRegionData(loadedFile);
      directoryChooser.setInitialDirectory(result.getParentFile());

      if (loadedPlanarRegions != null)
      {
         if (VERBOSE)
            PrintTools.info(this, "Loaded planar regions, broadcasting data.");
         messager.submitMessage(UIVisibilityGraphsTopics.GlobalReset, true);
         messager.submitMessage(UIVisibilityGraphsTopics.PlanarRegionData, loadedPlanarRegions);
         reloadMenuItem.setDisable(false);
      }
      else
      {
         if (VERBOSE)
            PrintTools.info(this, "Failed to load planar regions.");
         reloadMenuItem.setDisable(true);
         loadedFile = null;
      }
   }

   @FXML
   public void reloadPlanarRegion()
   {
      if (loadedFile == null)
         return;

      PlanarRegionsList loadedPlanarRegions = PlanarRegionFileTools.importPlanRegionData(loadedFile);

      if (loadedPlanarRegions != null)
      {
         if (VERBOSE)
            PrintTools.info(this, "Loaded planar regions, broadcasting data.");
         messager.submitMessage(UIVisibilityGraphsTopics.GlobalReset, true);
         messager.submitMessage(UIVisibilityGraphsTopics.PlanarRegionData, loadedPlanarRegions);
      }
   }
}
