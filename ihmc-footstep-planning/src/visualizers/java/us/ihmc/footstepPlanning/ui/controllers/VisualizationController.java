package us.ihmc.footstepPlanning.ui.controllers;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.GlobalReset;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.RenderShiftedWaypoints;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.ShowBodyPath;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.ShowBodyPathLogGraphics;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.ShowClusterNavigableExtrusions;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.ShowClusterNonNavigableExtrusions;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.ShowClusterRawPoints;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.ShowCoordinateSystem;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.ShowFootstepPlan;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.ShowGoal;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.ShowGoalVisibilityMap;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.ShowHeightMap;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.ShowInterRegionVisibilityMap;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.ShowLogGraphics;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.ShowNavigableRegionVisibilityMaps;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.ShowOcTree;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.ShowPlanarRegions;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.ShowPostProcessingInfo;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.ShowRobot;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.ShowStart;
import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.ShowStartVisibilityMap;

import javafx.fxml.FXML;
import javafx.scene.control.CheckBox;
import us.ihmc.messager.javafx.JavaFXMessager;

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
   private CheckBox showOcTreeToggleButton;
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
   private CheckBox showSolution;
   @FXML
   private CheckBox showLogGraphics;
   @FXML
   private CheckBox showBodyPathLogGraphics;
   @FXML
   private CheckBox showHeightMap;
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
      messager.bindBidirectional(ShowOcTree, showOcTreeToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowStart, showStart.selectedProperty(), true);
      messager.bindBidirectional(ShowGoal, showGoal.selectedProperty(), true);
      messager.bindBidirectional(ShowCoordinateSystem, showCoordinateSystem.selectedProperty(), true);
      messager.bindBidirectional(ShowHeightMap, showHeightMap.selectedProperty(), true);

      // Body path planner
      messager.bindBidirectional(ShowBodyPath, showBodyPathToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowClusterNavigableExtrusions, showClusterNavigableExtrusionsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowClusterNonNavigableExtrusions, showClusterNonNavigableExtrusionsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowClusterRawPoints, showClusterRawPointsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowStartVisibilityMap, showStartMapToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowGoalVisibilityMap, showGoalMapToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowInterRegionVisibilityMap, showInterRegionMapToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowNavigableRegionVisibilityMaps, showInnerRegionMapsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowBodyPathLogGraphics, showBodyPathLogGraphics.selectedProperty(), true);

      // Footstep planner
      messager.bindBidirectional(ShowFootstepPlan, showSolution.selectedProperty(), true);
      messager.bindBidirectional(ShowLogGraphics, showLogGraphics.selectedProperty(), true);
      messager.bindBidirectional(ShowPostProcessingInfo, showPostProcessingInfo.selectedProperty(), true);
      messager.bindBidirectional(RenderShiftedWaypoints, renderAdjustedWaypoints.selectedProperty(), true);
   }

   @FXML
   public void reset()
   {
      messager.submitMessage(GlobalReset, true);
      messager.submitMessage(ShowFootstepPlan, true);
   }
}
