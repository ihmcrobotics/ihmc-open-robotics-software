package us.ihmc.humanoidBehaviors.tools;

import javafx.application.Platform;
import javafx.stage.Stage;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javafx.applicationCreator.JavaFXApplicationCreator;
import us.ihmc.pathPlanning.PlannerTestEnvironments;
import us.ihmc.pathPlanning.visibilityGraphs.ui.graphics.PlanarRegionsGraphic;

public class FakeREAVirtualCameraTest //extends Application // TODO: Maybe extract JFX app
{
   private static boolean VISUALIZE = Boolean.parseBoolean(System.getProperty("visualize")); // To visualize, pass -Dvisualize=true

//   @Override
//   public void start(Stage primaryStage)
//   {
//      View3DFactory view3dFactory = new View3DFactory(1200, 800);
//      view3dFactory.addCameraController(0.05, 2000.0,true);
//      view3dFactory.addWorldCoordinateSystem(0.3);
//      view3dFactory.addDefaultLighting();
//
//      PlanarRegionsGraphic regionsGraphic = new PlanarRegionsGraphic();
//      regionsGraphic.generateMeshes(PlannerTestEnvironments.getTrickCorridor());
//      regionsGraphic.update();
//
//      view3dFactory.addNodeToView(regionsGraphic);
//
//      primaryStage.setTitle(getClass().getSimpleName());
//      primaryStage.setMaximized(false);
//      primaryStage.setScene(view3dFactory.getScene());
//
//      primaryStage.show();
//   }
//
//   @Override
//   public void stop()
//   {
//   }

   public void startOnAThread()
   {
//      ThreadTools.startAThread(Application::launch, getClass().getSimpleName() + "JavaFXApp");
   }

   @Test
   public void testFakeREACutsComplexField()
   {
      JavaFXApplicationCreator.createAJavaFXApplication();

      Platform.runLater(() ->
      {
         View3DFactory view3dFactory = new View3DFactory(1200, 800);
         view3dFactory.addCameraController(0.05, 2000.0, true);
         view3dFactory.addWorldCoordinateSystem(0.3);
         view3dFactory.addDefaultLighting();

         PlanarRegionsGraphic regionsGraphic = new PlanarRegionsGraphic();
         regionsGraphic.generateMeshes(PlannerTestEnvironments.getTrickCorridor());
         regionsGraphic.update();

         view3dFactory.addNodeToView(regionsGraphic);

         Stage primaryStage = new Stage();
         primaryStage.setTitle(getClass().getSimpleName());
         primaryStage.setMaximized(false);
         primaryStage.setScene(view3dFactory.getScene());

         primaryStage.show();
      });

//      startOnAThread();

      ThreadTools.sleepForever();
      if (VISUALIZE) ThreadTools.sleepForever();
   }
}
