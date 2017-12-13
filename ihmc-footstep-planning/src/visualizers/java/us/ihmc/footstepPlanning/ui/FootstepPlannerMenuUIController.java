package us.ihmc.footstepPlanning.ui;

import javafx.fxml.FXML;
import javafx.scene.control.MenuItem;
import javafx.stage.Window;
import us.ihmc.commons.PrintTools;
import us.ihmc.robotEnvironmentAwareness.communication.REAMessager;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionDataImporter;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class FootstepPlannerMenuUIController
{
   private static final boolean VERBOSE = true;

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
   }

   @FXML
   public void loadPlanarRegions()
   {
      loadedPlanarRegions = PlanarRegionDataImporter.importUsingFileChooser(ownerWindow);

      if (loadedPlanarRegions != null)
      {
         if (VERBOSE)
            PrintTools.info(this, "Loaded planar regions, broadcasting data.");
         messager.submitMessage(FootstepPlannerUserInterfaceAPI.GlobalResetTopic, true);
         messager.submitMessage(FootstepPlannerUserInterfaceAPI.PlanarRegionDataTopic, loadedPlanarRegions);
      }
      else
      {
         if (VERBOSE)
            PrintTools.info(this, "Failed to load planar regions.");
      }
   }
}
