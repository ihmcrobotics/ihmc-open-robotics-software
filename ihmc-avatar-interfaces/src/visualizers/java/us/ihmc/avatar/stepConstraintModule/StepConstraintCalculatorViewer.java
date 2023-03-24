package us.ihmc.avatar.stepConstraintModule;

import java.io.IOException;

import javafx.application.Platform;
import javafx.fxml.FXML;
import javafx.fxml.FXMLLoader;
import javafx.scene.Scene;
import javafx.scene.control.CheckBox;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.Pane;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.pathPlanning.visibilityGraphs.ui.viewers.PlanarRegionViewer;

public class StepConstraintCalculatorViewer
{
   private final Stage primaryStage;
   private final JavaFXMessager messager;
   private final PlanarRegionViewer planarRegionViewer;
   private final PlanarRegionViewer tooSmallRegionViewer;
   private final PlanarRegionViewer tooSteepRegionViewer;
   private final PlanarRegionViewer maskedRegionsViewer;
   private final StepConstraintRegionViewer stepConstraintRegionViewer;
   private final ObstacleExtrusionViewer obstacleExtrusionViewer;
   private final ObstacleExtrusionViewer maskedRegionsExtrusionsViewer;

   @FXML
   private BorderPane mainPane;
   @FXML
   private CheckBox showPlanarRegions, showConstraintRegions, showObstacleExtrusions, showObstacleRawPoints, showTooSmallRegions, showTooSteepRegions;
   @FXML
   private CheckBox showMaskedRegions;


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
      this.tooSmallRegionViewer = new PlanarRegionViewer(messager,
                                                         StepConstraintCalculatorViewerAPI.TooSmallRegionData,
                                                         StepConstraintCalculatorViewerAPI.ShowTooSmallRegions);
      this.tooSteepRegionViewer = new PlanarRegionViewer(messager,
                                                         StepConstraintCalculatorViewerAPI.TooSteepRegionData,
                                                         StepConstraintCalculatorViewerAPI.ShowTooSteepRegions);
      this.maskedRegionsViewer = new PlanarRegionViewer(messager,
                                                        StepConstraintCalculatorViewerAPI.MaskedRegionsData,
                                                        StepConstraintCalculatorViewerAPI.ShowMaskedRegions);
      this.stepConstraintRegionViewer = new StepConstraintRegionViewer(messager,
                                                                       StepConstraintCalculatorViewerAPI.StepConstraintRegionData,
                                                                       StepConstraintCalculatorViewerAPI.ShowStepConstraintRegions);
      this.obstacleExtrusionViewer = new ObstacleExtrusionViewer(messager);
      obstacleExtrusionViewer.setTopics(null, StepConstraintCalculatorViewerAPI.ShowExtrusionPoints, StepConstraintCalculatorViewerAPI.ShowObstacleExtrusions,
                                        StepConstraintCalculatorViewerAPI.ObstacleExtrusionsData);
      this.maskedRegionsExtrusionsViewer = new ObstacleExtrusionViewer(messager);
      maskedRegionsExtrusionsViewer.setTopics(null, StepConstraintCalculatorViewerAPI.ShowExtrusionPoints, StepConstraintCalculatorViewerAPI.ShowMaskedRegions,
                                              StepConstraintCalculatorViewerAPI.MaskedRegionsObstacleExtrusionsData);


            view3dFactory.addNodeToView(planarRegionViewer.getRoot());
      view3dFactory.addNodeToView(stepConstraintRegionViewer.getRoot());
      view3dFactory.addNodeToView(obstacleExtrusionViewer.getRoot());
      view3dFactory.addNodeToView(tooSmallRegionViewer.getRoot());
      view3dFactory.addNodeToView(tooSteepRegionViewer.getRoot());
      view3dFactory.addNodeToView(maskedRegionsViewer.getRoot());
      view3dFactory.addNodeToView(maskedRegionsExtrusionsViewer.getRoot());

      messager.bindBidirectional(StepConstraintCalculatorViewerAPI.ShowPlanarRegions, showPlanarRegions.selectedProperty(), false);
      messager.bindBidirectional(StepConstraintCalculatorViewerAPI.ShowStepConstraintRegions, showConstraintRegions.selectedProperty(), false);
      messager.bindBidirectional(StepConstraintCalculatorViewerAPI.ShowObstacleExtrusions, showObstacleExtrusions.selectedProperty(), false);
      messager.bindBidirectional(StepConstraintCalculatorViewerAPI.ShowExtrusionPoints, showObstacleRawPoints.selectedProperty(), false);
      messager.bindBidirectional(StepConstraintCalculatorViewerAPI.ShowTooSmallRegions, showTooSmallRegions.selectedProperty(), false);
      messager.bindBidirectional(StepConstraintCalculatorViewerAPI.ShowTooSteepRegions, showTooSteepRegions.selectedProperty(), false);
      messager.bindBidirectional(StepConstraintCalculatorViewerAPI.ShowMaskedRegions, showMaskedRegions.selectedProperty(), false);

      messager.submitMessage(StepConstraintCalculatorViewerAPI.ShowPlanarRegions, false);
      messager.submitMessage(StepConstraintCalculatorViewerAPI.ShowStepConstraintRegions, true);
      messager.submitMessage(StepConstraintCalculatorViewerAPI.ShowObstacleExtrusions, true);
      messager.submitMessage(StepConstraintCalculatorViewerAPI.ShowExtrusionPoints, true);
      messager.submitMessage(StepConstraintCalculatorViewerAPI.ShowTooSmallRegions, false);
      messager.submitMessage(StepConstraintCalculatorViewerAPI.ShowTooSteepRegions, false);
      messager.submitMessage(StepConstraintCalculatorViewerAPI.ShowMaskedRegions, false);

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
      obstacleExtrusionViewer.start();
      maskedRegionsExtrusionsViewer.start();
      tooSteepRegionViewer.start();
      tooSmallRegionViewer.start();
      maskedRegionsViewer.start();
   }

   public void stop()
   {
      planarRegionViewer.stop();
      stepConstraintRegionViewer.stop();
      obstacleExtrusionViewer.stop();
      maskedRegionsExtrusionsViewer.stop();
      tooSteepRegionViewer.stop();
      tooSmallRegionViewer.stop();
      maskedRegionsViewer.stop();
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