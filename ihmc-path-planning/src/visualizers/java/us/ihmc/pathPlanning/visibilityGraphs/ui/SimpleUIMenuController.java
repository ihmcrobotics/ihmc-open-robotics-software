package us.ihmc.pathPlanning.visibilityGraphs.ui;

import javafx.fxml.FXML;
import javafx.scene.control.MenuItem;
import javafx.stage.Window;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionDataImporter;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class SimpleUIMenuController
{
   private static final boolean VERBOSE = true;

   @FXML
   private MenuItem reloadMenuItem;

   private REAMessager messager;
   private Window ownerWindow;

   private PlanarRegionsList loadedPlanarRegions = null;

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
      loadedPlanarRegions = PlanarRegionDataImporter.importUsingFileChooser(ownerWindow);

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
      }
   }

   @FXML
   public void reloadPlanarRegion()
   {
      if (loadedPlanarRegions != null)
      {
         if (VERBOSE)
            PrintTools.info(this, "Loaded planar regions, broadcasting data.");
         messager.submitMessage(UIVisibilityGraphsTopics.GlobalReset, true);
         messager.submitMessage(UIVisibilityGraphsTopics.PlanarRegionData, loadedPlanarRegions);
      }
   }
}
