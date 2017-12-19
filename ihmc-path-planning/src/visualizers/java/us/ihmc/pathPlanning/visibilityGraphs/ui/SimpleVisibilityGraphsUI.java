package us.ihmc.pathPlanning.visibilityGraphs.ui;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.pathPlanning.visibilityGraphs.ui.controllers.*;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.SimpleUIMessager;
import us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.PlanarRegionViewer;
import us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.StartGoalPositionViewer;
import us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.VisibilityGraphsRenderer;

import java.io.IOException;

import static us.ihmc.pathPlanning.visibilityGraphs.ui.messager.UIVisibilityGraphsTopics.*;

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

      dataExporter = new VisibilityGraphsDataExporter(messager);

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      Pane subScene = view3dFactory.getSubSceneWrappedInsidePane();
      mainPane.setCenter(subScene);

      planarRegionViewer = new PlanarRegionViewer(messager, PlanarRegionData, ShowPlanarRegions);
      view3dFactory.addNodeToView(planarRegionViewer.getRoot());
      planarRegionViewer.start();
      startGoalEditor = new StartGoalPositionEditor(messager, subScene, StartEditModeEnabled, GoalEditModeEnabled, StartPosition, GoalPosition);
      startGoalEditor.start();

      startGoalViewer = new StartGoalPositionViewer(messager, StartEditModeEnabled, GoalEditModeEnabled, StartPosition, GoalPosition);
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
   }
}
