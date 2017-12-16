package us.ihmc.pathPlanning.visibilityGraphs.visualizer;

import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.CurrentDatasetPath;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.GoalPosition;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.NextDatasetRequest;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.PreviousDatasetRequest;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.ShowClusterNavigableExtrusions;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.ShowClusterNonNavigableExtrusions;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.ShowClusterRawPoints;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.ShowInterConnections;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.ShowLocalGraphs;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.StartPosition;

import java.io.IOException;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import javafx.application.Platform;
import javafx.beans.property.BooleanProperty;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.control.TextField;
import javafx.scene.control.ToggleButton;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.SimpleUIMessager;
import us.ihmc.pathPlanning.visibilityGraphs.ui.properties.Point3DProperty;
import us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.BodyPathMeshViewer;
import us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.ClusterMeshViewer;
import us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.NavigableRegionInnerVizMapMeshViewer;
import us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.NavigableRegionsInterConnectionViewer;
import us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.VisibilityGraphStartGoalViewer;
import us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.VizGraphsPlanarRegionViewer;

public class VisibilityGraphsTestVisualizer
{
   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));

   private final Stage primaryStage;
   private final SimpleUIMessager messager;
   private final VizGraphsPlanarRegionViewer planarRegionViewer;
   private final VisibilityGraphStartGoalViewer startGoalViewer;
   private final BodyPathMeshViewer bodyPathMeshViewer;
   private final ClusterMeshViewer clusterMeshViewer;
   private final NavigableRegionInnerVizMapMeshViewer navigableRegionInnerVizMapMeshViewer;
   private final NavigableRegionsInterConnectionViewer navigableRegionsInterConnectionViewer;

   @FXML
   private BorderPane mainPane;
   @FXML
   private TextField startTextField, goalTextField;
   @FXML
   private TextField currentDatasetTextField;
   @FXML
   private ToggleButton previousDatasetButton, nextDatasetButton;

   @FXML
   private ToggleButton showClusterRawPointsButton;
   @FXML
   private ToggleButton showClusterNavigableExtrusionsButton;
   @FXML
   private ToggleButton showClusterNonNavigableExtrusionsButton;
   @FXML
   private ToggleButton showRegionInnerConnectionsButton;
   @FXML
   private ToggleButton showRegionInterConnectionsButton;

   public VisibilityGraphsTestVisualizer(Stage primaryStage, SimpleUIMessager messager) throws IOException
   {
      this.primaryStage = primaryStage;
      this.messager = messager;
      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));
      Pane root = loader.load();

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      Pane subScene = view3dFactory.getSubSceneWrappedInsidePane();
      mainPane.setCenter(subScene);

      bindUIControls();

      planarRegionViewer = new VizGraphsPlanarRegionViewer(messager);
      startGoalViewer = new VisibilityGraphStartGoalViewer(messager);
      bodyPathMeshViewer = new BodyPathMeshViewer(messager, executorService);
      clusterMeshViewer = new ClusterMeshViewer(messager, executorService);
      navigableRegionInnerVizMapMeshViewer = new NavigableRegionInnerVizMapMeshViewer(messager, executorService);
      navigableRegionsInterConnectionViewer = new NavigableRegionsInterConnectionViewer(messager, executorService);

      view3dFactory.addNodeToView(planarRegionViewer.getRoot());
      view3dFactory.addNodeToView(startGoalViewer.getRoot());
      view3dFactory.addNodeToView(bodyPathMeshViewer.getRoot());
      view3dFactory.addNodeToView(clusterMeshViewer.getRoot());
      view3dFactory.addNodeToView(navigableRegionInnerVizMapMeshViewer.getRoot());
      view3dFactory.addNodeToView(navigableRegionsInterConnectionViewer.getRoot());

      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(true);
      Scene mainScene = new Scene(root, 600, 400);

      primaryStage.setScene(mainScene);
      primaryStage.setOnCloseRequest(event -> stop());
   }

   private void bindUIControls()
   {
      messager.bindBidirectional(PreviousDatasetRequest, previousDatasetButton.selectedProperty(), false);
      messager.bindBidirectional(NextDatasetRequest, nextDatasetButton.selectedProperty(), false);

      previousDatasetButton.disableProperty().bind(previousDatasetButton.selectedProperty());
      previousDatasetButton.disableProperty().bind(nextDatasetButton.selectedProperty());
      nextDatasetButton.disableProperty().bind(previousDatasetButton.selectedProperty());
      nextDatasetButton.disableProperty().bind(nextDatasetButton.selectedProperty());

      messager.registerTopicListener(CurrentDatasetPath, path -> currentDatasetTextField.setText(path == null ? "null" : path));

      Point3DProperty startProperty = new Point3DProperty(this, "startProperty", new Point3D(Double.NaN, Double.NaN, Double.NaN));
      Point3DProperty goalProperty = new Point3DProperty(this, "goalProperty", new Point3D(Double.NaN, Double.NaN, Double.NaN));

      startProperty.addListener((InvalidationListener) -> Platform.runLater(() -> startTextField.setText(startProperty.get().toString())));
      goalProperty.addListener((InvalidationListener) -> Platform.runLater(() -> goalTextField.setText(goalProperty.get().toString())));
      
      messager.bindPropertyToTopic(StartPosition, startProperty);
      messager.bindPropertyToTopic(GoalPosition, goalProperty);

      messager.bindBidirectional(ShowClusterRawPoints, showClusterRawPointsButton.selectedProperty(), false);
      messager.bindBidirectional(ShowClusterNavigableExtrusions, showClusterNavigableExtrusionsButton.selectedProperty(), false);
      messager.bindBidirectional(ShowClusterNonNavigableExtrusions, showClusterNonNavigableExtrusionsButton.selectedProperty(), false);
      messager.bindBidirectional(ShowLocalGraphs, showRegionInnerConnectionsButton.selectedProperty(), false);
      messager.bindBidirectional(ShowInterConnections, showRegionInterConnectionsButton.selectedProperty(), false);
   }

   public BooleanProperty getNextDatasetRequestedProperty()
   {
      return nextDatasetButton.selectedProperty();
   }

   public void show() throws IOException
   {
      primaryStage.show();
      planarRegionViewer.start();
      startGoalViewer.start();
      bodyPathMeshViewer.start();
      clusterMeshViewer.start();
      navigableRegionInnerVizMapMeshViewer.start();
      navigableRegionsInterConnectionViewer.start();
   }

   public void stop()
   {
      planarRegionViewer.stop();
      startGoalViewer.stop();
      bodyPathMeshViewer.stop();
      clusterMeshViewer.stop();
      navigableRegionInnerVizMapMeshViewer.stop();
      navigableRegionsInterConnectionViewer.stop();
      messager.closeMessager();
      Platform.exit();
   }
}
