package us.ihmc.footstepPlanning.ui;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;
import us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI;
import us.ihmc.footstepPlanning.graphSearch.parameters.DefaultFootstepPlanningParameters;
import us.ihmc.footstepPlanning.graphSearch.parameters.FootstepPlannerParameters;
import us.ihmc.footstepPlanning.tools.FootstepPlannerDataExporter;
import us.ihmc.footstepPlanning.ui.components.*;
import us.ihmc.footstepPlanning.ui.controllers.*;
import us.ihmc.footstepPlanning.ui.viewers.*;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.pathPlanning.visibilityGraphs.ui.StartGoalPositionEditor;
import us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.PlanarRegionViewer;

import static us.ihmc.footstepPlanning.communication.FootstepPlannerMessagerAPI.*;

/**
 * This class is the visualization element of the footstep planner. It also contains a graphical interface for
 * setting planner parameters to be used by the footstep planner itself.
 */
public class FootstepPlannerUI
{
   private static final boolean VERBOSE = true;

   private final JavaFXMessager messager;
   private final Stage primaryStage;
   private final BorderPane mainPane;

   private final PlanarRegionViewer planarRegionViewer;
   private final StartGoalPositionEditor startGoalEditor;
   private final FootPositionEditor footPositionEditor;
   private final StartGoalPositionViewer startGoalPositionViewer;
   private final StartGoalOrientationViewer startGoalOrientationViewer;
   private final FootstepPathMeshViewer pathViewer;
   private final StartGoalOrientationEditor orientationEditor;
   private final NodeCheckerRenderer nodeCheckerRenderer;
   private final FootstepPlannerDataExporter dataExporter;
   private final BodyPathMeshViewer bodyPathMeshViewer;
   private final VisibilityGraphsRenderer visibilityGraphsRenderer;
   private final OccupancyMapRenderer graphRenderer;

   @FXML
   private FootstepPlannerMenuUIController footstepPlannerMenuUIController;
   @FXML
   private StatusTabController statusTabController;
   @FXML
   private StartGoalTabController startGoalTabController;
   @FXML
   private FootstepNodeCheckingUIController footstepNodeCheckingUIController;
   @FXML
   private FootstepPlannerParametersUIController footstepPlannerParametersUIController;
   @FXML
   private BodyCollisionCheckingUIController bodyCollisionCheckingUIController;
   @FXML
   private FootstepPlannerCostsUIController footstepPlannerCostsUIController;
   @FXML
   private FootstepPlannerDataExporterAnchorPaneController dataExporterAnchorPaneController;

   @FXML
   private VisualizationController visibilityGraphsUIController;

   public FootstepPlannerUI(Stage primaryStage, FootstepPlannerParameters plannerParameters) throws Exception
   {
      this(primaryStage, new SharedMemoryJavaFXMessager(FootstepPlannerMessagerAPI.API), plannerParameters);
      messager.startMessager();
   }

   public FootstepPlannerUI(Stage primaryStage, JavaFXMessager messager) throws Exception
   {
      this(primaryStage, messager, new DefaultFootstepPlanningParameters());
   }

   public FootstepPlannerUI(Stage primaryStage, JavaFXMessager messager, FootstepPlannerParameters plannerParameters) throws Exception
   {
      this.primaryStage = primaryStage;
      this.messager = messager;

      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));

      mainPane = loader.load();

      footstepPlannerCostsUIController.setPlannerParameters(plannerParameters);
      footstepPlannerParametersUIController.setPlannerParameters(plannerParameters);

      footstepPlannerMenuUIController.attachMessager(messager);
      statusTabController.attachMessager(messager);
      startGoalTabController.attachMessager(messager);
      footstepPlannerParametersUIController.attachMessager(messager);
      bodyCollisionCheckingUIController.attachMessager(messager);
      footstepPlannerCostsUIController.attachMessager(messager);
      footstepNodeCheckingUIController.attachMessager(messager);
      visibilityGraphsUIController.attachMessager(messager);
      dataExporterAnchorPaneController.attachMessager(messager);

      footstepPlannerMenuUIController.setMainWindow(primaryStage);

      statusTabController.bindControls();
      startGoalTabController.bindControls();
      footstepPlannerParametersUIController.bindControls();
      bodyCollisionCheckingUIController.bindControls();
      footstepPlannerCostsUIController.bindControls();
      footstepNodeCheckingUIController.bindControls();
      visibilityGraphsUIController.bindControls();

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      Pane subScene = view3dFactory.getSubSceneWrappedInsidePane();

      this.planarRegionViewer = new PlanarRegionViewer(messager, PlanarRegionDataTopic, ShowPlanarRegionsTopic);
      this.startGoalPositionViewer = new StartGoalPositionViewer(messager, StartPositionEditModeEnabledTopic, GoalPositionEditModeEnabledTopic,
                                                                 StartPositionTopic, LowLevelGoalPositionTopic, GoalPositionTopic);
      this.startGoalOrientationViewer = new StartGoalOrientationViewer(messager);
      this.startGoalEditor = new StartGoalPositionEditor(messager, subScene, StartPositionEditModeEnabledTopic, GoalPositionEditModeEnabledTopic,
                                                         StartPositionTopic, GoalPositionTopic);
      this.footPositionEditor = new FootPositionEditor(messager, subScene);
      this.orientationEditor = new StartGoalOrientationEditor(messager, view3dFactory.getSubScene());
      this.pathViewer = new FootstepPathMeshViewer(messager);
      this.nodeCheckerRenderer = new NodeCheckerRenderer(messager);
      this.dataExporter = new FootstepPlannerDataExporter(messager);
      this.bodyPathMeshViewer = new BodyPathMeshViewer(messager);
      this.visibilityGraphsRenderer = new VisibilityGraphsRenderer(messager);
      this.graphRenderer = new OccupancyMapRenderer(messager);

      view3dFactory.addNodeToView(planarRegionViewer.getRoot());
      view3dFactory.addNodeToView(startGoalPositionViewer.getRoot());
      view3dFactory.addNodeToView(startGoalOrientationViewer.getRoot());
      view3dFactory.addNodeToView(pathViewer.getRoot());
      view3dFactory.addNodeToView(nodeCheckerRenderer.getRoot());
      view3dFactory.addNodeToView(bodyPathMeshViewer.getRoot());
      view3dFactory.addNodeToView(visibilityGraphsRenderer.getRoot());
      view3dFactory.addNodeToView(graphRenderer.getRoot());

      planarRegionViewer.start();
      startGoalPositionViewer.start();
      startGoalOrientationViewer.start();
      startGoalEditor.start();
      orientationEditor.start();
      pathViewer.start();
      nodeCheckerRenderer.start();
      footPositionEditor.start();
      bodyPathMeshViewer.start();
      visibilityGraphsRenderer.start();
      graphRenderer.start();

      mainPane.setCenter(subScene);
      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(true);
      Scene mainScene = new Scene(mainPane, 600, 400);

      primaryStage.setScene(mainScene);
      primaryStage.setOnCloseRequest(event -> stop());
   }

   public JavaFXMessager getMessager()
   {
      return messager;
   }

   public void show()
   {
      primaryStage.show();
   }

   public void stop()
   {
      planarRegionViewer.stop();
      startGoalPositionViewer.stop();
      startGoalOrientationViewer.stop();
      startGoalEditor.stop();
      orientationEditor.stop();
      pathViewer.stop();
      nodeCheckerRenderer.stop();
      dataExporter.stop();
      bodyPathMeshViewer.stop();
      visibilityGraphsRenderer.stop();
      graphRenderer.stop();
   }

   public static FootstepPlannerUI createMessagerUI(Stage primaryStage, JavaFXMessager messager) throws Exception
   {
      return new FootstepPlannerUI(primaryStage, messager);
   }

   public static FootstepPlannerUI createMessagerUI(Stage primaryStage, JavaFXMessager messager, FootstepPlannerParameters plannerParameters) throws Exception
   {
      return new FootstepPlannerUI(primaryStage, messager, plannerParameters);
   }
}
