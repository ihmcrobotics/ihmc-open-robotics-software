package us.ihmc.quadrupedFootstepPlanning.ui.controllers;

import javafx.fxml.FXML;
import javafx.scene.control.CheckBox;
import us.ihmc.commons.PrintTools;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.FootstepPlan;
import us.ihmc.quadrupedFootstepPlanning.footstepPlanning.communication.FootstepPlannerMessagerAPI;

import java.util.concurrent.atomic.AtomicReference;

public class VisualizationController
{
   private static final boolean verbose = false;

   private AtomicReference<FootstepPlan> footstepPlanReference;

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
   private CheckBox showSolution;
   @FXML
   public void requestStatistics()
   {
      if (verbose)
         PrintTools.info(this, "Clicked request statistics...");

      messager.submitMessage(FootstepPlannerMessagerAPI.RequestPlannerStatistics, true);
   }

   private JavaFXMessager messager;

   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;

      footstepPlanReference = messager.createInput(FootstepPlannerMessagerAPI.FootstepPlanTopic);
   }

   public void bindControls()
   {
      messager.bindBidirectional(FootstepPlannerMessagerAPI.ShowBodyPathTopic, showBodyPathToggleButton.selectedProperty(), true);

      messager.bindBidirectional(FootstepPlannerMessagerAPI.ShowPlanarRegionsTopic, showPlanarRegionsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(FootstepPlannerMessagerAPI.ShowClusterRawPoints, showClusterRawPointsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(FootstepPlannerMessagerAPI.ShowClusterNavigableExtrusions, showClusterNavigableExtrusionsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(FootstepPlannerMessagerAPI.ShowClusterNonNavigableExtrusions, showClusterNonNavigableExtrusionsToggleButton.selectedProperty(), true);

      messager.bindBidirectional(FootstepPlannerMessagerAPI.ShowNavigableRegionVisibilityMaps, showInnerRegionMapsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(FootstepPlannerMessagerAPI.ShowInterRegionVisibilityMap, showInterRegionMapToggleButton.selectedProperty(), true);
      messager.bindBidirectional(FootstepPlannerMessagerAPI.ShowStartVisibilityMap, showStartMapToggleButton.selectedProperty(), true);
      messager.bindBidirectional(FootstepPlannerMessagerAPI.ShowGoalVisibilityMap, showGoalMapToggleButton.selectedProperty(), true);
      messager.bindBidirectional(FootstepPlannerMessagerAPI.ShowFootstepPlanTopic, showSolution.selectedProperty(), true);
   }
}
