package us.ihmc.humanoidBehaviors.ui.simulation;

import javafx.application.Platform;
import javafx.stage.Stage;
import us.ihmc.communication.packets.PlanarRegionMessageConverter;
import us.ihmc.footstepPlanning.log.FootstepPlannerLogLoader;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javafx.applicationCreator.JavaFXApplicationCreator;
import us.ihmc.log.LogTools;
import us.ihmc.pathPlanning.visibilityGraphs.ui.graphics.PlanarRegionsGraphic;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionTools;
import us.ihmc.robotics.geometry.PlanarRegionsList;

import java.io.File;
import java.util.List;

public class PlanarRegionsFromPlannerLogViewer
{
   private static final String filepath = System.getProperty("filepath");

   public PlanarRegionsFromPlannerLogViewer()
   {
      Stage primaryStage = new Stage();
      View3DFactory view3dFactory = new View3DFactory(1200, 800);
      FocusBasedCameraMouseEventHandler camera = view3dFactory.addCameraController(0.05, 2000.0, true);
      double isoZoomOut = 10.0;
      camera.changeCameraPosition(-isoZoomOut, -isoZoomOut, isoZoomOut);
      view3dFactory.addWorldCoordinateSystem(0.3);
      view3dFactory.addDefaultLighting();

      FootstepPlannerLogLoader loader = new FootstepPlannerLogLoader();
      if (loader.load(new File(filepath)) != FootstepPlannerLogLoader.LoadResult.LOADED)
      {
         return;
      }

      PlanarRegionsList planarRegionsList = PlanarRegionMessageConverter.convertToPlanarRegionsList(loader.getLog()
                                                                                                          .getRequestPacket()
                                                                                                          .getPlanarRegionsListMessage());

      PlanarRegionsList modifiedList = new PlanarRegionsList();
      List<PlanarRegion> planarRegionsAsList = planarRegionsList.getPlanarRegionsAsList();
      for (int i = 0; i < planarRegionsAsList.size(); i++)
      {
         PlanarRegion planarRegion = planarRegionsAsList.get(i);
         LogTools.info("Index: {}, Id: {}, Area: {}", i, planarRegion.getRegionId(), PlanarRegionTools.computePlanarRegionArea(planarRegion));
         modifiedList.addPlanarRegion(planarRegion);
      }

      PlanarRegionsGraphic regionsGraphic = new PlanarRegionsGraphic();
      regionsGraphic.generateMeshes(modifiedList);
      regionsGraphic.update();

      view3dFactory.addNodeToView(regionsGraphic);

      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(false);
      primaryStage.setScene(view3dFactory.getScene());

      primaryStage.show();
   }

   public static void main(String[] args)
   {
      JavaFXApplicationCreator.createAJavaFXApplication();
      Platform.runLater(PlanarRegionsFromPlannerLogViewer::new);
   }
}
