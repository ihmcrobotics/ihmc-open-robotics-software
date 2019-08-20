package us.ihmc.quadrupedFootstepPlanning.ui.controllers;

import javafx.fxml.FXML;
import javafx.scene.control.CheckBox;
import us.ihmc.commons.PrintTools;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.PawPlan;
import us.ihmc.quadrupedFootstepPlanning.pawPlanning.communication.PawPlannerMessagerAPI;

import java.util.concurrent.atomic.AtomicReference;

public class VisualizationController
{
   private static final boolean verbose = false;

   private AtomicReference<PawPlan> footstepPlanReference;

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

      messager.submitMessage(PawPlannerMessagerAPI.RequestPlannerStatistics, true);
   }

   private JavaFXMessager messager;

   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;

      footstepPlanReference = messager.createInput(PawPlannerMessagerAPI.FootstepPlanTopic);
   }

   public void bindControls()
   {
      messager.bindBidirectional(PawPlannerMessagerAPI.ShowBodyPathTopic, showBodyPathToggleButton.selectedProperty(), true);

      messager.bindBidirectional(PawPlannerMessagerAPI.ShowPlanarRegionsTopic, showPlanarRegionsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(PawPlannerMessagerAPI.ShowClusterRawPoints, showClusterRawPointsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(PawPlannerMessagerAPI.ShowClusterNavigableExtrusions, showClusterNavigableExtrusionsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(PawPlannerMessagerAPI.ShowClusterNonNavigableExtrusions, showClusterNonNavigableExtrusionsToggleButton.selectedProperty(), true);

      messager.bindBidirectional(PawPlannerMessagerAPI.ShowNavigableRegionVisibilityMaps, showInnerRegionMapsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(PawPlannerMessagerAPI.ShowInterRegionVisibilityMap, showInterRegionMapToggleButton.selectedProperty(), true);
      messager.bindBidirectional(PawPlannerMessagerAPI.ShowStartVisibilityMap, showStartMapToggleButton.selectedProperty(), true);
      messager.bindBidirectional(PawPlannerMessagerAPI.ShowGoalVisibilityMap, showGoalMapToggleButton.selectedProperty(), true);
      messager.bindBidirectional(PawPlannerMessagerAPI.ShowFootstepPlanTopic, showSolution.selectedProperty(), true);
   }
}
