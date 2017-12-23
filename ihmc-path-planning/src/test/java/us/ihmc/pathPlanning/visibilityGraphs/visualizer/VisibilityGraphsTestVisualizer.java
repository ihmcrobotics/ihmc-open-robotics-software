package us.ihmc.pathPlanning.visibilityGraphs.visualizer;

import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.CurrentDatasetPath;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.GoalEditModeEnabled;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.GoalPosition;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.GoalVisibilityMap;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.InterRegionVisibilityMap;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.NextDatasetRequest;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.PlanarRegionData;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.PreviousDatasetRequest;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.ReloadDatasetRequest;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.ShowBodyPath;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.ShowClusterNavigableExtrusions;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.ShowClusterNonNavigableExtrusions;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.ShowClusterRawPoints;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.ShowGoalVisibilityMap;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.ShowInterRegionVisibilityMap;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.ShowNavigableRegionVisibilityMaps;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.ShowPlanarRegions;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.ShowStartVisibilityMap;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.StartEditModeEnabled;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.StartPosition;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.StartVisibilityMap;

import java.io.IOException;
import java.util.List;
import java.util.concurrent.ExecutorService;
import java.util.concurrent.Executors;

import com.sun.javafx.scene.control.skin.LabeledText;

import javafx.application.Platform;
import javafx.beans.property.BooleanProperty;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.control.ListView;
import javafx.scene.control.TextField;
import javafx.scene.control.ToggleButton;
import javafx.scene.input.MouseButton;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.stage.Stage;
import us.ihmc.commons.PrintTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.SimpleUIMessager;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics;
import us.ihmc.pathPlanning.visibilityGraphs.ui.properties.Point3DProperty;
import us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.BodyPathMeshViewer;
import us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.ClusterMeshViewer;
import us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.NavigableRegionViewer;
import us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.PlanarRegionViewer;
import us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.StartGoalPositionViewer;
import us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.VisibilityMapHolderViewer;

public class VisibilityGraphsTestVisualizer
{
   private final ExecutorService executorService = Executors.newSingleThreadExecutor(ThreadTools.getNamedThreadFactory(getClass().getSimpleName()));

   private final Stage primaryStage;
   private final SimpleUIMessager messager;
   private final PlanarRegionViewer planarRegionViewer;
   private final PlanarRegionViewer planarRegionShadowViewer;
   private final StartGoalPositionViewer startGoalViewer;
   private final BodyPathMeshViewer bodyPathMeshViewer;
   private final ClusterMeshViewer clusterMeshViewer;
   private final NavigableRegionViewer navigableRegionMeshViewer;
   private final VisibilityMapHolderViewer interRegionConnectionsViewer;
   private final VisibilityMapHolderViewer startMapViewer;
   private final VisibilityMapHolderViewer goalMapViewer;
   private final WalkerCollisionsViewer walkerCollisionsViewer;

   @FXML
   private BorderPane mainPane;
   @FXML
   private TextField startTextField, goalTextField;
   @FXML
   private ToggleButton previousDatasetButton, reloadDatasetButton, nextDatasetButton, stopWalkingToggleButton;

   @FXML
   private ToggleButton showBodyPathToggleButton;
   @FXML
   private ToggleButton showPlanarRegionsToggleButton;
   @FXML
   private ToggleButton showClusterRawPointsToggleButton, showClusterNavigableExtrusionsToggleButton, showClusterNonNavigableExtrusionsToggleButton;
   @FXML
   private ToggleButton showInnerRegionMapsToggleButton, showInterRegionMapToggleButton;
   @FXML
   private ToggleButton showStartMapToggleButton, showGoalMapToggleButton;
   @FXML
   private ListView<String> datasetsListView;

   public VisibilityGraphsTestVisualizer(Stage primaryStage, SimpleUIMessager messager) throws IOException
   {
      this.primaryStage = primaryStage;
      this.messager = messager;
      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));
      Pane root = loader.load();

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(0.05, 150.0, true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      Pane subScene = view3dFactory.getSubSceneWrappedInsidePane();
      mainPane.setCenter(subScene);

      bindUIControls();

      planarRegionViewer = new PlanarRegionViewer(messager, PlanarRegionData, ShowPlanarRegions);
      planarRegionShadowViewer = new PlanarRegionViewer(messager, UIVisibilityGraphsTopics.ShadowPlanarRegionData, ShowPlanarRegions);
      planarRegionShadowViewer.setOpacity(0.);
      startGoalViewer = new StartGoalPositionViewer(messager, StartEditModeEnabled, GoalEditModeEnabled, StartPosition, GoalPosition);
      bodyPathMeshViewer = new BodyPathMeshViewer(messager, executorService);
      clusterMeshViewer = new ClusterMeshViewer(messager, executorService);
      navigableRegionMeshViewer = new NavigableRegionViewer(messager, executorService);
      walkerCollisionsViewer = new WalkerCollisionsViewer(messager);

      interRegionConnectionsViewer = new VisibilityMapHolderViewer(messager, executorService);
      interRegionConnectionsViewer.setCustomColor(Color.CRIMSON);
      interRegionConnectionsViewer.setTopics(ShowInterRegionVisibilityMap, InterRegionVisibilityMap);

      startMapViewer = new VisibilityMapHolderViewer(messager);
      startMapViewer.setCustomColor(Color.YELLOW);
      startMapViewer.setTopics(ShowStartVisibilityMap, StartVisibilityMap);

      goalMapViewer = new VisibilityMapHolderViewer(messager);
      goalMapViewer.setCustomColor(Color.CORNFLOWERBLUE);
      goalMapViewer.setTopics(ShowGoalVisibilityMap, GoalVisibilityMap);

      view3dFactory.addNodeToView(planarRegionViewer.getRoot());
      view3dFactory.addNodeToView(planarRegionShadowViewer.getRoot());
      view3dFactory.addNodeToView(startGoalViewer.getRoot());
      view3dFactory.addNodeToView(bodyPathMeshViewer.getRoot());
      view3dFactory.addNodeToView(clusterMeshViewer.getRoot());
      view3dFactory.addNodeToView(navigableRegionMeshViewer.getRoot());
      view3dFactory.addNodeToView(interRegionConnectionsViewer.getRoot());
      view3dFactory.addNodeToView(walkerCollisionsViewer.getRoot());
      view3dFactory.addNodeToView(startMapViewer.getRoot());
      view3dFactory.addNodeToView(goalMapViewer.getRoot());

      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(true);
      Scene mainScene = new Scene(root, 600, 400);

      primaryStage.setScene(mainScene);
      primaryStage.setOnCloseRequest(event -> stop());
   }

   private void bindUIControls()
   {
      messager.bindBidirectional(PreviousDatasetRequest, previousDatasetButton.selectedProperty(), false);
      messager.bindBidirectional(ReloadDatasetRequest, reloadDatasetButton.selectedProperty(), false);
      messager.bindBidirectional(NextDatasetRequest, nextDatasetButton.selectedProperty(), false);

      previousDatasetButton.disableProperty().bind(previousDatasetButton.selectedProperty());
      previousDatasetButton.disableProperty().bind(reloadDatasetButton.selectedProperty());
      previousDatasetButton.disableProperty().bind(nextDatasetButton.selectedProperty());

      reloadDatasetButton.disableProperty().bind(previousDatasetButton.selectedProperty());
      reloadDatasetButton.disableProperty().bind(reloadDatasetButton.selectedProperty());
      reloadDatasetButton.disableProperty().bind(nextDatasetButton.selectedProperty());

      nextDatasetButton.disableProperty().bind(previousDatasetButton.selectedProperty());
      nextDatasetButton.disableProperty().bind(reloadDatasetButton.selectedProperty());
      nextDatasetButton.disableProperty().bind(nextDatasetButton.selectedProperty());

      datasetsListView.disableProperty().bind(previousDatasetButton.selectedProperty());
      datasetsListView.disableProperty().bind(reloadDatasetButton.selectedProperty());
      datasetsListView.disableProperty().bind(nextDatasetButton.selectedProperty());

      messager.bindBidirectional(UIVisibilityGraphsTopics.StopWalker, stopWalkingToggleButton.selectedProperty(), false);

      Point3DProperty startProperty = new Point3DProperty(this, "startProperty", new Point3D(Double.NaN, Double.NaN, Double.NaN));
      Point3DProperty goalProperty = new Point3DProperty(this, "goalProperty", new Point3D(Double.NaN, Double.NaN, Double.NaN));

      startProperty.addListener((InvalidationListener) -> Platform.runLater(() -> startTextField.setText(startProperty.get().toString())));
      goalProperty.addListener((InvalidationListener) -> Platform.runLater(() -> goalTextField.setText(goalProperty.get().toString())));

      messager.bindPropertyToTopic(StartPosition, startProperty);
      messager.bindPropertyToTopic(GoalPosition, goalProperty);

      messager.bindBidirectional(ShowBodyPath, showBodyPathToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowPlanarRegions, showPlanarRegionsToggleButton.selectedProperty(), true);

      messager.bindBidirectional(ShowClusterRawPoints, showClusterRawPointsToggleButton.selectedProperty(), false);
      messager.bindBidirectional(ShowClusterNavigableExtrusions, showClusterNavigableExtrusionsToggleButton.selectedProperty(), false);
      messager.bindBidirectional(ShowClusterNonNavigableExtrusions, showClusterNonNavigableExtrusionsToggleButton.selectedProperty(), false);
      messager.bindBidirectional(ShowNavigableRegionVisibilityMaps, showInnerRegionMapsToggleButton.selectedProperty(), false);
      messager.bindBidirectional(ShowInterRegionVisibilityMap, showInterRegionMapToggleButton.selectedProperty(), false);
      messager.bindBidirectional(ShowStartVisibilityMap, showStartMapToggleButton.selectedProperty(), true);
      messager.bindBidirectional(ShowGoalVisibilityMap, showGoalMapToggleButton.selectedProperty(), true);

      messager.registerJavaFXSyncedTopicListener(UIVisibilityGraphsTopics.AllDatasetPaths, this::showDatasets);
      messager.registerJavaFXSyncedTopicListener(CurrentDatasetPath, path -> datasetsListView.getSelectionModel().select(path));

      datasetsListView.setOnMouseClicked(this::handleDatasetSelection);
   }

   private void handleDatasetSelection(MouseEvent event)
   {
      if (event.getButton() == MouseButton.PRIMARY && event.getClickCount() == 2 && event.getTarget() instanceof LabeledText)
      {
         PrintTools.info("Submitting new dataset request");
         messager.submitMessage(CurrentDatasetPath, datasetsListView.getSelectionModel().getSelectedItem());
      }
   }

   private void showDatasets(List<String> allDatasets)
   {
      datasetsListView.getItems().clear();
      datasetsListView.getItems().addAll(allDatasets);
   }

   public BooleanProperty getNextDatasetRequestedProperty()
   {
      return nextDatasetButton.selectedProperty();
   }

   public void show() throws IOException
   {
      primaryStage.show();
      planarRegionViewer.start();
      planarRegionShadowViewer.start();
      startGoalViewer.start();
      bodyPathMeshViewer.start();
      clusterMeshViewer.start();
      navigableRegionMeshViewer.start();
      interRegionConnectionsViewer.start();
      walkerCollisionsViewer.start();
      startMapViewer.start();
      goalMapViewer.start();
   }

   public void stop()
   {
      planarRegionViewer.stop();
      planarRegionShadowViewer.stop();
      startGoalViewer.stop();
      bodyPathMeshViewer.stop();
      clusterMeshViewer.stop();
      navigableRegionMeshViewer.stop();
      interRegionConnectionsViewer.stop();
      walkerCollisionsViewer.stop();
      messager.closeMessager();
      startMapViewer.stop();
      goalMapViewer.stop();
      Platform.exit();
   }
}
