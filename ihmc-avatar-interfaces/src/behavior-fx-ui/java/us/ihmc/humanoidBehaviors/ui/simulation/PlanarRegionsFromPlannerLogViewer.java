package us.ihmc.humanoidBehaviors.ui.simulation;

import javafx.application.Application;
import javafx.stage.Stage;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogLoader;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.pathPlanning.visibilityGraphs.ui.graphics.PlanarRegionsGraphic;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.io.File;

public class PlanarRegionsFromPlannerLogViewer extends Application
{
   private static final String filepath = System.getProperty("filepath");

   @Override
   public void start(Stage primaryStage)
   {
      View3DFactory view3dFactory = new View3DFactory(1200, 800);
      FocusBasedCameraMouseEventHandler camera = view3dFactory.addCameraController(0.05, 2000.0, true);
      double isoZoomOut = 10.0;
      camera.changeCameraPosition(-isoZoomOut, -isoZoomOut, isoZoomOut);
      view3dFactory.addWorldCoordinateSystem(0.3);
      view3dFactory.addDefaultLighting();

      FootstepPlannerLogLoader loader = new FootstepPlannerLogLoader();
      loader.load(new File(filepath));
      PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(loader.getLog()
                                                                                                          .getRequestPacket()
                                                                                                          .getPlanarRegionsListMessage());

      PlanarRegionsGraphic regionsGraphic = new PlanarRegionsGraphic();
      regionsGraphic.generateMeshes(planarRegionsList);
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
