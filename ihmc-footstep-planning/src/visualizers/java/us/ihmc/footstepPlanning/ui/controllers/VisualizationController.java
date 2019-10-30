package us.ihmc.footstepPlanning.ui.controllers;

import javafx.fxml.FXML;
import javafx.scene.control.CheckBox;
import us.ihmc.commons.PrintTools;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.*;

public class VisualizationController
{
   private static final boolean verbose = false;

   @FXML
   private CheckBox showBodyPathToggleButton;
   @FXML
   private CheckBox showInnerRegionMapsToggleButton;
   @FXML
   private CheckBox showInterRegionMapToggleButton;
   @FXML
   private CheckBox showStartMapToggleButton;
   @FXML
   private CheckBox showGoalMapToggleButton;
   @FXML
   private CheckBox showPlanarRegionsToggleButton;
   @FXML
   private CheckBox showClusterRawPointsToggleButton;
   @FXML
   private CheckBox showClusterNavigableExtrusionsToggleButton;
   @FXML
   private CheckBox showClusterNonNavigableExtrusionsToggleButton;
   @FXML
   private CheckBox showClusterPreferredNavigableExtrusionsToggleButton;
   @FXML
   private CheckBox showClusterPreferredNonNavigableExtrusionsToggleButton;
   @FXML
   private CheckBox showOccupancyMap;
   @FXML
   private CheckBox showSolution;
   @FXML
   private CheckBox showIntermediateSolution;
   @FXML
   public void requestStatistics()
   {
      if (verbose)
         PrintTools.info(this, "Clicked request statistics...");

      messager.submitMessage(RequestPlannerStatistics, true);
   }

   private JavaFXMessager messager;

   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;
   }

   public void bindControls()
   {
      messager.bindBidirectional(ShowBodyPath, showBodyPathToggleButton.selectedProperty(), true);

      messager.bindBidirectional(ShowPlanarRegions, showPlanarRegionsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowClusterRawPoints, showClusterRawPointsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowClusterNavigableExtrusions, showClusterNavigableExtrusionsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowClusterNonNavigableExtrusions, showClusterNonNavigableExtrusionsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowClusterPreferredNavigableExtrusions, showClusterPreferredNavigableExtrusionsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowClusterPreferredNonNavigableExtrusions, showClusterPreferredNonNavigableExtrusionsToggleButton.selectedProperty(), true);

      messager.bindBidirectional(ShowNavigableRegionVisibilityMaps, showInnerRegionMapsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowInterRegionVisibilityMap, showInterRegionMapToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowStartVisibilityMap, showStartMapToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowGoalVisibilityMap, showGoalMapToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowOccupancyMap, showOccupancyMap.selectedProperty(), true);
      messager.bindBidirectional(ShowFootstepPlan, showSolution.selectedProperty(), true);
      messager.bindBidirectional(ShowNodeData, showIntermediateSolution.selectedProperty(), true);
   }
}
