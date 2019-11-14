package us.ihmc.humanoidBehaviors.tools;

import javafx.application.Platform;
import javafx.stage.Stage;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidBehaviors.ui.simulation.BehaviorPlanarRegionEnvironments;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javafx.applicationCreator.JavaFXApplicationCreator;
import us.ihmc.pathPlanning.PlannerTestEnvironments;
import us.ihmc.pathPlanning.visibilityGraphs.ui.graphics.PlanarRegionsGraphic;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.referenceFrames.PoseReferenceFrame;
import us.ihmc.tools.gui.AWTTools;

import java.awt.*;

// TODO: Measure speed
public class FakeREAVirtualCameraTest
{
   private static boolean VISUALIZE = Boolean.parseBoolean(System.getProperty("visualize")); // To visualize, pass -Dvisualize=true
   private static int windowCount = 0; // for tiling

   @BeforeAll
   static public void beforeAll()
   {
      if (VISUALIZE) JavaFXApplicationCreator.createAJavaFXApplication();
   }

   @AfterEach
   public void afterEach()
   {
      if (VISUALIZE) ThreadTools.sleepForever();
   }

   @Test
   public void testFakeREACutsComplexField()
   {
      compareMapToSimulatedView(PlannerTestEnvironments.getTrickCorridor(), new Pose3D(), 0, 1);
   }

   @Test
   public void testFakeREA1()
   {
      compareMapToSimulatedView(PlannerTestEnvironments.getTrickCorridorWCutFloor(), new Pose3D(), 2, 3);
   }

   @Test
   public void testFakeREA2()
   {
      compareMapToSimulatedView(BehaviorPlanarRegionEnvironments.realDataFromAtlasSLAMDataset20190710(), new Pose3D(), 4, 5);
      Pose3D pose3D = new Pose3D();
      pose3D.appendYawRotation(Math.toRadians(-135.0));
      compareMapToSimulatedView(BehaviorPlanarRegionEnvironments.realDataFromAtlasSLAMDataset20190710(), pose3D, 6, 7);
      pose3D = new Pose3D();
      pose3D.appendYawRotation(Math.toRadians(180.0));
      compareMapToSimulatedView(BehaviorPlanarRegionEnvironments.realDataFromAtlasSLAMDataset20190710(), pose3D, 6, 7);
   }

   private void compareMapToSimulatedView(PlanarRegionsList map, Pose3DReadOnly pose3D, int mapWindowNumber, int resultWindowNumber)
   {
      PlanarRegionsList trickCorridor = map;
      PoseReferenceFrame cameraFrame = new PoseReferenceFrame("camera", ReferenceFrame.getWorldFrame());
      cameraFrame.setPoseAndUpdate(pose3D);
      FakeREAVirtualCamera fakeREAVirtualCamera = new FakeREAVirtualCamera(90.0, 90.0, cameraFrame);
      PlanarRegionsList virtualCamera = fakeREAVirtualCamera.filterMapToVisible(trickCorridor);
      createAndShowPlanarRegionWindow(trickCorridor, mapWindowNumber);
      createAndShowPlanarRegionWindow(virtualCamera, resultWindowNumber);
   }

   private void createAndShowPlanarRegionWindow(PlanarRegionsList planarRegionsList, int windowCount)
   {
      if (!VISUALIZE) return;

      Platform.runLater(() ->
      {
         int windowWidth = 400;
         int windowHeight = 300;
         View3DFactory view3dFactory = new View3DFactory(windowWidth, windowHeight);
         FocusBasedCameraMouseEventHandler camera = view3dFactory.addCameraController(0.05, 2000.0, true);
         double isoZoomOut = 15.0;
         camera.changeCameraPosition(-isoZoomOut, -isoZoomOut, isoZoomOut);
         view3dFactory.addWorldCoordinateSystem(0.3);
         view3dFactory.addDefaultLighting();

         PlanarRegionsGraphic regionsGraphic = new PlanarRegionsGraphic();
         regionsGraphic.generateMeshes(planarRegionsList);
         regionsGraphic.update();

         view3dFactory.addNodeToView(regionsGraphic);

         Stage primaryStage = new Stage();
         primaryStage.setTitle(getClass().getSimpleName());
         primaryStage.setMaximized(false);
         primaryStage.setScene(view3dFactory.getScene());

         Dimension dimensionForSmallestScreen = AWTTools.getDimensionOfSmallestScreen();
         int numberOfColumns = (int) Math.floor(dimensionForSmallestScreen.getWidth() / windowWidth);
         primaryStage.setX((windowCount % numberOfColumns) * windowWidth);
         primaryStage.setY((windowCount / numberOfColumns) * windowHeight);
         primaryStage.show();
         primaryStage.toFront();
      });
   }
}
