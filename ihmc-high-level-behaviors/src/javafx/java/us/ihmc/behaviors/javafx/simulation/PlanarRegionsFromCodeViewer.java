package us.ihmc.behaviors.javafx.simulation;

import java.util.function.Supplier;

import javafx.stage.Stage;
import us.ihmc.avatar.environments.BehaviorPlanarRegionEnvironments;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javafx.ApplicationNoModule;
import us.ihmc.pathPlanning.PlannerTestEnvironments;
import us.ihmc.pathPlanning.visibilityGraphs.ui.graphics.PlanarRegionsGraphic;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class PlanarRegionsFromCodeViewer extends ApplicationNoModule
{
   public static final Supplier<PlanarRegionsList> PLANNER_ENVIRONMENT = PlannerTestEnvironments::getMazeCorridor;
   public static final Supplier<PlanarRegionsList> BEHAVIOR_ENVIRONMENT = BehaviorPlanarRegionEnvironments::generateTriplePalletCinderBlockAngledStepsUpAndDown;

   public static final Supplier<PlanarRegionsList> PLANAR_REGIONS_TO_VIEW = BEHAVIOR_ENVIRONMENT;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      View3DFactory view3dFactory = new View3DFactory(1200, 800);
      FocusBasedCameraMouseEventHandler camera = view3dFactory.addCameraController(0.05, 2000.0, true);
      double isoZoomOut = 10.0;
      camera.changeCameraPosition(-isoZoomOut, -isoZoomOut, isoZoomOut);
      view3dFactory.addWorldCoordinateSystem(0.5);
      view3dFactory.addDefaultLighting();

      PlanarRegionsGraphic regionsGraphic = new PlanarRegionsGraphic();
      regionsGraphic.generateMeshes(PLANAR_REGIONS_TO_VIEW.get());
      regionsGraphic.update();

      view3dFactory.addNodeToView(regionsGraphic);

      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(false);
      primaryStage.setScene(view3dFactory.getScene());

      primaryStage.show();
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
