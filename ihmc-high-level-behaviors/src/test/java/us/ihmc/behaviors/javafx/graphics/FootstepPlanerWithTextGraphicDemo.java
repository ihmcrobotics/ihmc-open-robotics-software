package us.ihmc.behaviors.javafx.graphics;

import java.util.ArrayList;

import javafx.stage.Stage;
import us.ihmc.behaviors.tools.footstepPlanner.MinimalFootstep;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.footstepPlanning.tools.PlannerTools;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javafx.ApplicationNoModule;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;

public class FootstepPlanerWithTextGraphicDemo extends ApplicationNoModule
{
   private FootstepPlanGraphic footstepPlanGraphic;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      View3DFactory view3dFactory = new View3DFactory(1200, 800);
      FocusBasedCameraMouseEventHandler camera = view3dFactory.addCameraController(0.05, 2000.0, true);
      double isoZoomOut = 10.0;
      camera.changeCameraPosition(-isoZoomOut, -isoZoomOut, isoZoomOut);
      view3dFactory.addWorldCoordinateSystem(0.5);
      view3dFactory.addDefaultLighting();

      footstepPlanGraphic = new FootstepPlanGraphic();
      footstepPlanGraphic.setTransparency(0.2);

      ArrayList<MinimalFootstep> footsteps = new ArrayList<>();
      SideDependentList<ConvexPolygon2D> defaultFootPolygons = PlannerTools.createDefaultFootPolygons();

      footsteps.add(new MinimalFootstep(RobotSide.LEFT, new Pose3D(0.0, 0.2, 0.0, 0.0, 0.0, 0.0), defaultFootPolygons.get(RobotSide.LEFT), "Left"));
      footsteps.add(new MinimalFootstep(RobotSide.RIGHT, new Pose3D(0.0, -0.2, 0.0, 0.0, 0.0, 0.0), defaultFootPolygons.get(RobotSide.RIGHT), "Right"));
      footstepPlanGraphic.generateMeshes(footsteps);

      view3dFactory.addNodeToView(footstepPlanGraphic);

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
