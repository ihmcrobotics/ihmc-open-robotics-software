package us.ihmc.pathPlanning.visibilityGraphs.ui;

import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.API;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.GoalEditModeEnabled;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.GoalPosition;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.PlanarRegionData;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.RandomizePlanarRegionIDRequest;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.ShowGoalPosition;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.ShowPlanarRegions;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.ShowStartPosition;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.StartEditModeEnabled;
import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.StartPosition;

import java.io.IOException;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.control.SplitPane;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.pathPlanning.visibilityGraphs.ui.controllers.DatasetNavigationAccordionController;
import us.ihmc.pathPlanning.visibilityGraphs.ui.controllers.SimpleUIMenuController;
import us.ihmc.pathPlanning.visibilityGraphs.ui.controllers.StartGoalAnchorPaneController;
import us.ihmc.pathPlanning.visibilityGraphs.ui.controllers.VisibilityGraphsAnchorPaneController;
import us.ihmc.pathPlanning.visibilityGraphs.ui.controllers.VisibilityGraphsDataExporterAnchorPaneController;
import us.ihmc.pathPlanning.visibilityGraphs.ui.controllers.VisibilityGraphsParametersAnchorPaneController;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.SimpleUIMessager;
import us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.PlanarRegionViewer;
import us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.StartGoalPositionViewer;
import us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.VisibilityGraphsRenderer;

public class SimpleVisibilityGraphsUI
{
   private final SimpleUIMessager messager = new SimpleUIMessager(API);
   private final Stage primaryStage;
   private final BorderPane mainPane;

   private final PlanarRegionViewer planarRegionViewer;
   private final StartGoalPositionEditor startGoalEditor;
   private final StartGoalPositionViewer startGoalViewer;
   private final VisibilityGraphsRenderer visibilityGraphsRenderer;
   private final VisibilityGraphsDataExporter dataExporter;
   private final PlanarRegionIDRandomizer planarRegionIDRandomizer;

   @FXML
   private SplitPane centerSplitPane;
   @FXML
   private SimpleUIMenuController simpleUIMenuController;
   @FXML
   private StartGoalAnchorPaneController startGoalAnchorPaneController;
   @FXML
   private VisibilityGraphsAnchorPaneController visibilityGraphsAnchorPaneController;
   @FXML
   private VisibilityGraphsDataExporterAnchorPaneController visibilityGraphsDataExporterAnchorPaneController;
   @FXML
   private VisibilityGraphsParametersAnchorPaneController visibilityGraphsParametersAnchorPaneController;
   @FXML
   private DatasetNavigationAccordionController datasetNavigationAccordionController;

   public SimpleVisibilityGraphsUI(Stage primaryStage) throws IOException
   {
      this.primaryStage = primaryStage;

      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));

      mainPane = loader.load();

      messager.startMessager();

      simpleUIMenuController.attachMessager(messager);
      simpleUIMenuController.setMainWindow(primaryStage);
      startGoalAnchorPaneController.attachMessager(messager);
      startGoalAnchorPaneController.bindControls();
      visibilityGraphsAnchorPaneController.attachMessager(messager);
      visibilityGraphsAnchorPaneController.bindControls();

      visibilityGraphsDataExporterAnchorPaneController.attachMessager(messager);
      visibilityGraphsParametersAnchorPaneController.attachMessager(messager);
      visibilityGraphsParametersAnchorPaneController.bindControls();

      datasetNavigationAccordionController.attachMessager(messager);
      datasetNavigationAccordionController.setMainWindow(primaryStage);
      datasetNavigationAccordionController.bindControls();
      datasetNavigationAccordionController.load();

      dataExporter = new VisibilityGraphsDataExporter(messager);
      planarRegionIDRandomizer = new PlanarRegionIDRandomizer(messager);
      planarRegionIDRandomizer.setTopics(RandomizePlanarRegionIDRequest, PlanarRegionData);
      planarRegionIDRandomizer.start();

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      Pane subScene = view3dFactory.getSubSceneWrappedInsidePane();
      centerSplitPane.getItems().add(subScene);

      planarRegionViewer = new PlanarRegionViewer(messager, PlanarRegionData, ShowPlanarRegions);
      view3dFactory.addNodeToView(planarRegionViewer.getRoot());
      planarRegionViewer.start();
      startGoalEditor = new StartGoalPositionEditor(messager, subScene, StartEditModeEnabled, GoalEditModeEnabled, StartPosition, GoalPosition);
      startGoalEditor.start();

      startGoalViewer = new StartGoalPositionViewer(messager);
      startGoalViewer.setEditStartGoalTopics(StartEditModeEnabled, GoalEditModeEnabled);
      startGoalViewer.setPositionStartGoalTopics(StartPosition, GoalPosition);
      startGoalViewer.setShowStartGoalTopics(ShowStartPosition, ShowGoalPosition);
      view3dFactory.addNodeToView(startGoalViewer.getRoot());
      startGoalViewer.start();
      visibilityGraphsRenderer = new VisibilityGraphsRenderer(messager);
      view3dFactory.addNodeToView(visibilityGraphsRenderer.getRoot());
      visibilityGraphsRenderer.start();

      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(true);
      Scene mainScene = new Scene(mainPane, 600, 400);

      primaryStage.setScene(mainScene);
      primaryStage.setOnCloseRequest(event -> stop());
   }

   public void show(boolean showPlanarRegionFileChooser) throws IOException
   {
      primaryStage.show();

      if (showPlanarRegionFileChooser)
         simpleUIMenuController.loadPlanarRegion();
   }

   public void stop()
   {
      messager.closeMessager();
      planarRegionViewer.stop();
      startGoalEditor.stop();
      startGoalViewer.stop();
      visibilityGraphsRenderer.stop();
      dataExporter.stop();
      planarRegionIDRandomizer.stop();
   }
}
