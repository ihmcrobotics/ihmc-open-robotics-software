package us.ihmc.pathPlanning.visibilityGraphs.ui;

import static us.ihmc.pathPlanning.visibilityGraphs.ui.UIVisibilityGraphsTopics.*;

import javafx.fxml.FXML;
import javafx.scene.control.ToggleButton;

public class VisibilityGraphsAnchorPaneController
{
   @FXML
   private ToggleButton showBodyPathToggleButton;
   @FXML
   private ToggleButton showLocalGraphsToggleButton;
   @FXML
   private ToggleButton showInterConnectionsToggleButton;
   @FXML
   private ToggleButton showPlanarRegionsToggleButton;
   @FXML
   private ToggleButton showClusterRawPointsToggleButton;
   @FXML
   private ToggleButton showClusterNavigableExtrusionsToggleButton;
   @FXML
   private ToggleButton showClusterNonNavigableExtrusionsToggleButton;
   @FXML
   private ToggleButton closeClusterNavigableExtrusionsToggleButton;
   @FXML
   private ToggleButton closeClusterNonNavigableExtrusionsToggleButton;

   private SimpleUIMessager messager;

   public void attachMessager(SimpleUIMessager messager)
   {
      this.messager = messager;
   }

   public void bindControls()
   {
      messager.bindBidirectional(ShowBodyPath, showBodyPathToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowLocalGraphs, showLocalGraphsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowInterConnections, showInterConnectionsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowPlanarRegions, showPlanarRegionsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowClusterRawPoints, showClusterRawPointsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowClusterNavigableExtrusions, showClusterNavigableExtrusionsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowClusterNonNavigableExtrusions, showClusterNonNavigableExtrusionsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(CloseClusterNavigableExtrusions, closeClusterNavigableExtrusionsToggleButton.selectedProperty(), true);
      messager.bindBidirectional(CloseClusterNonNavigableExtrusions, closeClusterNonNavigableExtrusionsToggleButton.selectedProperty(), true);
   }

   @FXML
   public void computePath()
   {
      messager.submitMessage(VisibilityGraphsComputePath, true);
   }
}
