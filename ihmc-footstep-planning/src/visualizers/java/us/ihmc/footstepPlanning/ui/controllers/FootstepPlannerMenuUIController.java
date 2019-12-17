package us.ihmc.footstepPlanning.ui.controllers;

import javafx.fxml.FXML;
import javafx.stage.Window;
import us.ihmc.commons.PrintTools;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.messager.Messager;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionDataImporter;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class FootstepPlannerMenuUIController
{
   private static final boolean VERBOSE = true;

   private Messager messager;
   private Window ownerWindow;

   private PlanarRegionsList loadedPlanarRegions = null;

   public void attachMessager(Messager messager)
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
         messager.submitMessage(FootstepPlannerMessagerAPI.GlobalReset, true);
         messager.submitMessage(FootstepPlannerMessagerAPI.PlanarRegionData, loadedPlanarRegions);
      }
      else
      {
         if (VERBOSE)
            PrintTools.info(this, "Failed to load planar regions.");
      }
   }
}
