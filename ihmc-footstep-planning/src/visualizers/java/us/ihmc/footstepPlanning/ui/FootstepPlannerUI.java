package us.ihmc.footstepPlanning.ui;

import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.pathPlanning.visibilityGraphs.ui.SimpleUIMessager;
import us.ihmc.pathPlanning.visibilityGraphs.ui.VizGraphsPlanarRegionViewer;

import java.io.IOException;
import java.net.URL;

public class FootstepPlannerUI
{
   private final SimpleUIMessager messager = new SimpleUIMessager(FootstepPlannerUserInterfaceAPI.API);
   private final Stage primaryStage;
   private final BorderPane mainPane;

   private final PlanarRegionViewer planarRegionViewer;

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

      planarRegionViewer = new PlanarRegionViewer(messager);
      view3dFactory.addNodeToView(planarRegionViewer.getRoot());
      planarRegionViewer.start();

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
