package us.ihmc.robotEnvironmentAwareness.polygonizer;

import javafx.fxml.FXML;
import javafx.scene.control.ToggleButton;
import us.ihmc.messager.javafx.JavaFXMessager;

public class PolygonizerDisplayOptionTabController
{
   @FXML
   private ToggleButton showDelaunayTriangleButton;
   @FXML
   private ToggleButton showBorderVerticesButton;
   @FXML
   private ToggleButton showBorderEdgesButton;
   @FXML
   private ToggleButton showBorderTrianglesButton;
   @FXML
   private ToggleButton showConstraintEdgesButton;
   @FXML
   private ToggleButton showOrderedBorderEdgesButton;
   @FXML
   private ToggleButton showPriorityQueueButton;
   @FXML
   private ToggleButton showRawConcaveHullButton;
   @FXML
   private ToggleButton showConcavePocketsButton;
   @FXML
   private ToggleButton showProcessedConcaveHullButton;

   public void initialize(JavaFXMessager messager)
   {
      messager.bindBidirectional(MultipleConcaveHullViewer.ViewDelaunayTriangulation, showDelaunayTriangleButton.selectedProperty(), false);
      messager.bindBidirectional(MultipleConcaveHullViewer.ViewBorderVertices, showBorderVerticesButton.selectedProperty(), false);
      messager.bindBidirectional(MultipleConcaveHullViewer.ViewBorderEdges, showBorderEdgesButton.selectedProperty(), false);
      messager.bindBidirectional(MultipleConcaveHullViewer.ViewBorderTriangles, showBorderTrianglesButton.selectedProperty(), false);
      messager.bindBidirectional(MultipleConcaveHullViewer.ViewConstraintEdges, showConstraintEdgesButton.selectedProperty(), false);
      messager.bindBidirectional(MultipleConcaveHullViewer.ViewOrderedBorderEdges, showOrderedBorderEdgesButton.selectedProperty(), false);
      messager.bindBidirectional(MultipleConcaveHullViewer.ViewPriorityQueue, showPriorityQueueButton.selectedProperty(), false);
      messager.bindBidirectional(MultipleConcaveHullViewer.ViewRawConcaveHull, showRawConcaveHullButton.selectedProperty(), false);
      messager.bindBidirectional(MultipleConcaveHullViewer.ViewConcavePockets, showConcavePocketsButton.selectedProperty(), false);
      messager.bindBidirectional(MultipleConcaveHullViewer.ViewProcessedConcaveHull, showProcessedConcaveHullButton.selectedProperty(), false);
   }
}
