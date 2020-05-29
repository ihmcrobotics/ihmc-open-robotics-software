package us.ihmc.avatar.stepConstraintModule;

import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.control.CheckBox;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.PlanarRegionViewer;

import java.io.IOException;

public class StepConstraintCalculatorViewer
{
   private final Stage primaryStage;
   private final JavaFXMessager messager;
   private final PlanarRegionViewer planarRegionViewer;
   private final StepConstraintRegionViewer stepConstraintRegionViewer;

   @FXML
   private BorderPane mainPane;
   @FXML
   private CheckBox showPlanarRegions, showConstraintRegions;


   public StepConstraintCalculatorViewer(Stage primaryStage, JavaFXMessager messager) throws Exception
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

      this.planarRegionViewer = new PlanarRegionViewer(messager,
                                                       StepConstraintCalculatorViewerAPI.PlanarRegionData,
                                                       StepConstraintCalculatorViewerAPI.ShowPlanarRegions);
      this.stepConstraintRegionViewer = new StepConstraintRegionViewer(messager,
                                                                       StepConstraintCalculatorViewerAPI.StepConstraintRegionData,
                                                                       StepConstraintCalculatorViewerAPI.ShowStepConstraintRegions);

      view3dFactory.addNodeToView(planarRegionViewer.getRoot());
      view3dFactory.addNodeToView(stepConstraintRegionViewer.getRoot());

      messager.bindBidirectional(StepConstraintCalculatorViewerAPI.ShowPlanarRegions, showPlanarRegions.selectedProperty(), false);
      messager.bindBidirectional(StepConstraintCalculatorViewerAPI.ShowStepConstraintRegions, showConstraintRegions.selectedProperty(), false);

      messager.submitMessage(StepConstraintCalculatorViewerAPI.ShowPlanarRegions, false);
      messager.submitMessage(StepConstraintCalculatorViewerAPI.ShowStepConstraintRegions, true);

      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(true);
      Scene mainScene = new Scene(root, 600, 400);

      primaryStage.setScene(mainScene);
      primaryStage.setOnCloseRequest(event -> stop());
   }

   public void show() throws IOException
   {
      primaryStage.show();
      planarRegionViewer.start();
      stepConstraintRegionViewer.start();
   }

   public void stop()
   {
      planarRegionViewer.stop();
      stepConstraintRegionViewer.stop();
      try
      {
         messager.closeMessager();
      }
      catch (Exception e)
      {
         e.printStackTrace();
      }
      Platform.exit();
   }
}