package us.ihmc.pathPlanning.visibilityGraphs.ui.controllers;

import javafx.fxml.FXML;
import javafx.scene.control.ToggleButton;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.SimpleUIMessager;

import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.*;

public class VisibilityGraphsAnchorPaneController
{
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
   private ToggleButton showWalkerToggleButton;

   private SimpleUIMessager messager;

   public void attachMessager(SimpleUIMessager messager)
   {
      this.messager = messager;
   }

   public void bindControls()
   {
      messager.bindBidirectional(ShowBodyPath, showBodyPathToggleButton.selectedProperty(), true);

      messager.bindBidirectional(ShowNavigableRegionVisibilityMaps, showInnerRegionMapsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowInterRegionVisibilityMap, showInterRegionMapToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowStartVisibilityMap, showStartMapToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowGoalVisibilityMap, showGoalMapToggleButton.selectedProperty(), true);

      messager.bindBidirectional(ShowPlanarRegions, showPlanarRegionsToggleButton.selectedProperty(), true);

      messager.bindBidirectional(ShowClusterRawPoints, showClusterRawPointsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowClusterNavigableExtrusions, showClusterNavigableExtrusionsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowClusterNonNavigableExtrusions, showClusterNonNavigableExtrusionsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(EnableWalkerAnimation, showWalkerToggleButton.selectedProperty(), true);
   }

   @FXML
   public void computePath()
   {
      messager.submitMessage(VisibilityGraphsComputePath, true);
   }

   @FXML
   public void computePathWithOcclusions()
   {
      messager.submitMessage(VisibilityGraphsComputePathWithOcclusions, true);
   }
}
