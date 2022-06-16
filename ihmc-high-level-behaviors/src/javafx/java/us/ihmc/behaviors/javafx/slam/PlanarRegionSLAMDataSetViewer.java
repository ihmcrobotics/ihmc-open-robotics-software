package us.ihmc.behaviors.javafx.slam;

import java.nio.file.Path;
import java.nio.file.Paths;

import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javafx.ApplicationNoModule;
import us.ihmc.pathPlanning.visibilityGraphs.ui.graphics.PlanarRegionsGraphic;
import us.ihmc.robotics.PlanarRegionFileTools;

public class PlanarRegionSLAMDataSetViewer extends ApplicationNoModule
{
   @Override
   public void start(Stage primaryStage) throws Exception
   {
      View3DFactory view3dFactory = new View3DFactory(1200, 800);
      view3dFactory.addCameraController(0.05, 2000.0, true);
      view3dFactory.addWorldCoordinateSystem(0.3);
      view3dFactory.addDefaultLighting();

      importPlanarRegionsList(view3dFactory, "20190710_174025_PlanarRegion");
      importPlanarRegionsList(view3dFactory, "20190710_174208_PlanarRegion");
      importPlanarRegionsList(view3dFactory, "20190710_174422_PlanarRegion");

      primaryStage.setTitle(getClass().getSimpleName());
      primaryStage.setMaximized(false);
      primaryStage.setScene(view3dFactory.getScene());

      primaryStage.show();
   }

   private void importPlanarRegionsList(View3DFactory view3dFactory, String dataSetName)
   {
      PlanarRegionsGraphic regionsGraphic = new PlanarRegionsGraphic();
      Path SLAMDataSet1 = Paths.get(
            "ihmc-open-robotics-software/robot-environment-awareness/Data/PlanarRegion/190710_SLAM_PlanarRegionFittingExamples/" + dataSetName);
      regionsGraphic.generateMeshes(PlanarRegionFileTools.importPlanarRegionData(SLAMDataSet1.toFile()));
      regionsGraphic.update();
      view3dFactory.addNodeToView(regionsGraphic);
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
