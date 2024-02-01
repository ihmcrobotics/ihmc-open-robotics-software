package us.ihmc.quadrupedFootstepPlanning.ui.controllers;

import java.util.concurrent.atomic.AtomicReference;

import javafx.fxml.FXML;
import javafx.scene.control.CheckBox;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawStepPlan;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.communication.PawStepPlannerMessagerAPI;

public class VisualizationController
{
   private static final boolean verbose = false;

   private AtomicReference<PawStepPlan> footstepPlanReference;

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

   private JavaFXMessager messager;

   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;

      footstepPlanReference = messager.createInput(PawStepPlannerMessagerAPI.FootstepPlanTopic);
   }

   public void bindControls()
   {
      messager.bindBidirectional(PawStepPlannerMessagerAPI.ShowBodyPathTopic, showBodyPathToggleButton.selectedProperty(), true);

      messager.bindBidirectional(PawStepPlannerMessagerAPI.ShowPlanarRegionsTopic, showPlanarRegionsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(PawStepPlannerMessagerAPI.ShowClusterRawPoints, showClusterRawPointsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(PawStepPlannerMessagerAPI.ShowClusterNavigableExtrusions, showClusterNavigableExtrusionsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(PawStepPlannerMessagerAPI.ShowClusterNonNavigableExtrusions, showClusterNonNavigableExtrusionsToggleButton.selectedProperty(), true);

      messager.bindBidirectional(PawStepPlannerMessagerAPI.ShowNavigableRegionVisibilityMaps, showInnerRegionMapsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(PawStepPlannerMessagerAPI.ShowInterRegionVisibilityMap, showInterRegionMapToggleButton.selectedProperty(), true);
      messager.bindBidirectional(PawStepPlannerMessagerAPI.ShowStartVisibilityMap, showStartMapToggleButton.selectedProperty(), true);
      messager.bindBidirectional(PawStepPlannerMessagerAPI.ShowGoalVisibilityMap, showGoalMapToggleButton.selectedProperty(), true);
      messager.bindBidirectional(PawStepPlannerMessagerAPI.ShowFootstepPlanTopic, showSolution.selectedProperty(), true);
   }
}
