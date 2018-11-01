package us.ihmc.footstepPlanning.ui.controllers;

import javafx.fxml.FXML;
import javafx.scene.control.ToggleButton;
import us.ihmc.commons.PrintTools;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.*;

public class VisualizationController
{
   private static final boolean verbose = false;

   @FXML
   private ToggleButton showBodyPathToggleButton;
   @FXML
   private ToggleButton showInnerRegionMapsToggleButton;
   @FXML
   private ToggleButton showInterRegionMapToggleButton;
   @FXML
   private ToggleButton showStartMapToggleButton;
   @FXML
   private ToggleButton showGoalMapToggleButton;
   @FXML
   private ToggleButton showPlanarRegionsToggleButton;
   @FXML
   private ToggleButton showClusterRawPointsToggleButton;
   @FXML
   private ToggleButton showClusterNavigableExtrusionsToggleButton;
   @FXML
   private ToggleButton showClusterNonNavigableExtrusionsToggleButton;
   @FXML
   private ToggleButton showOccupancyMap;
   @FXML
   private ToggleButton showSolution;
   @FXML
   private ToggleButton showIntermediateSolution;
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

      messager.bindBidirectional(ShowPlanarRegionsTopic, showPlanarRegionsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowClusterRawPoints, showClusterRawPointsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowClusterNavigableExtrusions, showClusterNavigableExtrusionsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowClusterNonNavigableExtrusions, showClusterNonNavigableExtrusionsToggleButton.selectedProperty(), true);

      messager.bindBidirectional(ShowNavigableRegionVisibilityMaps, showInnerRegionMapsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowInterRegionVisibilityMap, showInterRegionMapToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowStartVisibilityMap, showStartMapToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowGoalVisibilityMap, showGoalMapToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowOccupancyMap, showOccupancyMap.selectedProperty(), true);
      messager.bindBidirectional(ShowFootstepPlanTopic, showSolution.selectedProperty(), true);
      messager.bindBidirectional(ShowNodeDataTopic, showIntermediateSolution.selectedProperty(), true);
   }
}
