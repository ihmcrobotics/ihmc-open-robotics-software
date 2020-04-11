package us.ihmc.footstepPlanning.ui.controllers;

import javafx.fxml.FXML;
import javafx.scene.control.CheckBox;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.*;

public class VisualizationController
{
   @FXML
   private CheckBox showBodyPathToggleButton;
   @FXML
   private CheckBox showRobotToggleButton;
   @FXML
   private CheckBox showCoordinateSystem;
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
   private CheckBox showStart;
   @FXML
   private CheckBox showGoal;
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
   private CheckBox showLogGraphics;
   @FXML
   private CheckBox showPostProcessingInfo;
   @FXML
   private CheckBox renderAdjustedWaypoints;

   private JavaFXMessager messager;

   public void attachMessager(JavaFXMessager messager)
   {
      this.messager = messager;
   }

   public void bindControls()
   {
      // General
      messager.bindBidirectional(ShowRobot, showRobotToggleButton.selectedProperty(), false);
      messager.bindBidirectional(ShowPlanarRegions, showPlanarRegionsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowStart, showStart.selectedProperty(), true);
      messager.bindBidirectional(ShowGoal, showGoal.selectedProperty(), true);
      messager.bindBidirectional(ShowCoordinateSystem, showCoordinateSystem.selectedProperty(), true);

      // Body path planner
      messager.bindBidirectional(ShowBodyPath, showBodyPathToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowClusterPreferredNavigableExtrusions, showClusterPreferredNavigableExtrusionsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowClusterPreferredNonNavigableExtrusions, showClusterPreferredNonNavigableExtrusionsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowClusterNavigableExtrusions, showClusterNavigableExtrusionsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowClusterNonNavigableExtrusions, showClusterNonNavigableExtrusionsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowClusterRawPoints, showClusterRawPointsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowStartVisibilityMap, showStartMapToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowGoalVisibilityMap, showGoalMapToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowInterRegionVisibilityMap, showInterRegionMapToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowNavigableRegionVisibilityMaps, showInnerRegionMapsToggleButton.selectedProperty(), true);

      // Footstep planner
      messager.bindBidirectional(ShowFootstepPlan, showSolution.selectedProperty(), true);
      messager.bindBidirectional(ShowOccupancyMap, showOccupancyMap.selectedProperty(), true);
      messager.bindBidirectional(ShowLogGraphics, showLogGraphics.selectedProperty(), true);
      messager.bindBidirectional(ShowPostProcessingInfo, showPostProcessingInfo.selectedProperty(), true);
      messager.bindBidirectional(RenderShiftedWaypoints, renderAdjustedWaypoints.selectedProperty(), true);
   }

   @FXML
   public void reset()
   {
      messager.submitMessage(GlobalReset, true);
   }
}
