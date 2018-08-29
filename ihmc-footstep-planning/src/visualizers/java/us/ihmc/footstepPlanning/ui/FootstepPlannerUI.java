package us.ihmc.footstepPlanning.ui;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.pathPlanning.visibilityGraphs.ui.StartGoalPositionEditor;
import us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.PlanarRegionViewer;
import us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.StartGoalPositionViewer;

import static us.ihmc.footstepPlanning.ui.FootstepPlannerUserInterfaceAPI.*;

public class FootstepPlannerUI
{
   private static final boolean VERBOSE = true;

   private final JavaFXMessager messager = new SharedMemoryJavaFXMessager(FootstepPlannerUserInterfaceAPI.API);
   private final Stage primaryStage;
   private final BorderPane mainPane;

   private final PlanarRegionViewer planarRegionViewer;
   private final StartGoalPositionEditor startGoalEditor;
   private final FootPositionEditor footPositionEditor;
   private final StartGoalPositionViewer startGoalPositionViewer;
   private final StartGoalOrientationViewer startGoalOrientationViewer;
   private final FootstepPathRenderer pathRenderer;
   private final StartGoalOrientationEditor orientationEditor;
   private final NodeCheckerRenderer nodeCheckerRenderer;

   @FXML
   private FootstepPlannerMenuUIController footstepPlannerMenuUIController;

   @FXML
   private StartGoalTabController startGoalTabController;

   @FXML
   private FootstepNodeCheckingUIController footstepNodeCheckingUIController;

   @FXML
   private FootstepPlannerParametersUIController footstepPlannerParametersUIController;

   public FootstepPlannerUI(Stage primaryStage) throws Exception
   {
      this.primaryStage = primaryStage;

      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));

      mainPane = loader.load();
      messager.startMessager();

      footstepPlannerMenuUIController.attachMessager(messager);
      startGoalTabController.attachMessager(messager);
      footstepPlannerParametersUIController.attachMessager(messager);
      footstepNodeCheckingUIController.attachMessager(messager);

      footstepPlannerMenuUIController.setMainWindow(primaryStage);

      startGoalTabController.bindControls();
      footstepPlannerParametersUIController.bindControls();
      footstepNodeCheckingUIController.bindControls();

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      Pane subScene = view3dFactory.getSubSceneWrappedInsidePane();

      this.planarRegionViewer = new PlanarRegionViewer(messager, PlanarRegionDataTopic, ShowPlanarRegionsTopic);
      this.startGoalPositionViewer = new StartGoalPositionViewer(messager, StartPositionEditModeEnabledTopic, GoalPositionEditModeEnabledTopic, StartPositionTopic, GoalPositionTopic);
      this.startGoalOrientationViewer = new StartGoalOrientationViewer(messager);
      this.startGoalEditor = new StartGoalPositionEditor(messager, subScene, StartPositionEditModeEnabledTopic, GoalPositionEditModeEnabledTopic, StartPositionTopic,
                                                    GoalPositionTopic);
      this.footPositionEditor = new FootPositionEditor(messager, subScene);
      this.orientationEditor = new StartGoalOrientationEditor(messager, view3dFactory.getSubScene());
      this.pathRenderer = new FootstepPathRenderer(messager);
      this.nodeCheckerRenderer = new NodeCheckerRenderer(messager);

      view3dFactory.addNodeToView(planarRegionViewer.getRoot());
      view3dFactory.addNodeToView(startGoalPositionViewer.getRoot());
      view3dFactory.addNodeToView(startGoalOrientationViewer.getRoot());
      view3dFactory.addNodeToView(pathRenderer.getRoot());
      view3dFactory.addNodeToView(nodeCheckerRenderer.getRoot());

      planarRegionViewer.start();
      startGoalPositionViewer.start();
      startGoalOrientationViewer.start();
      startGoalEditor.start();
      orientationEditor.start();
      pathRenderer.start();
      nodeCheckerRenderer.start();
      footPositionEditor.start();

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
      pathRenderer.stop();
   }
}
