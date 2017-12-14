package us.ihmc.footstepPlanning.ui;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.pathPlanning.visibilityGraphs.ui.PlanarRegionViewer;
import us.ihmc.pathPlanning.visibilityGraphs.ui.StartGoalPositionEditor;
import us.ihmc.pathPlanning.visibilityGraphs.ui.StartGoalPositionViewer;
import us.ihmc.pathPlanning.visibilityGraphs.ui.messager.SimpleUIMessager;

import java.io.IOException;

import static us.ihmc.footstepPlanning.ui.FootstepPlannerUserInterfaceAPI.*;

public class FootstepPlannerUI
{
   private final SimpleUIMessager messager = new SimpleUIMessager(FootstepPlannerUserInterfaceAPI.API);
   private final Stage primaryStage;
   private final BorderPane mainPane;

   private final PlanarRegionViewer planarRegionViewer;
   private final StartGoalPositionEditor startGoalEditor;
   private final StartGoalPositionViewer startGoalViewer;

   @FXML
   private FootstepPlannerMenuUIController footstepPlannerMenuUIController;

   @FXML
   private StartGoalTabController startGoalTabController;

   public FootstepPlannerUI(Stage primaryStage) throws IOException
   {
      this.primaryStage = primaryStage;

      FXMLLoader loader = new FXMLLoader();
      loader.setController(this);
      loader.setLocation(getClass().getResource(getClass().getSimpleName() + ".fxml"));

      mainPane = loader.load();
      messager.startMessager();

      footstepPlannerMenuUIController.attachMessager(messager);
      startGoalTabController.attachMessager(messager);

      footstepPlannerMenuUIController.setMainWindow(primaryStage);

      startGoalTabController.bindControls();

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      Pane subScene = view3dFactory.getSubSceneWrappedInsidePane();
      mainPane.setCenter(subScene);

      planarRegionViewer = new PlanarRegionViewer(messager, PlanarRegionDataTopic, ShowPlanarRegionsTopic);
      view3dFactory.addNodeToView(planarRegionViewer.getRoot());
      planarRegionViewer.start();

      startGoalViewer = new StartGoalPositionViewer(messager, StartEditModeEnabledTopic, GoalEditModeEnabledTopic, StartPositionTopic, GoalPositionTopic);
      view3dFactory.addNodeToView(startGoalViewer.getRoot());
      startGoalViewer.start();

      startGoalEditor = new StartGoalPositionEditor(messager, subScene, StartEditModeEnabledTopic, GoalEditModeEnabledTopic, StartPositionTopic,
                                                    GoalPositionTopic);
      startGoalEditor.start();

      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(true);
      Scene mainScene = new Scene(mainPane, 600, 400);

      primaryStage.setScene(mainScene);
      primaryStage.setOnCloseRequest(event -> stop());
   }

   public void show() throws IOException
   {
      primaryStage.show();
   }

   public void stop()
   {
      planarRegionViewer.stop();
   }
}
