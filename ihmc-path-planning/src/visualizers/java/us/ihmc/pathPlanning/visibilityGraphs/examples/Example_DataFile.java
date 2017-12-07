package us.ihmc.pathPlanning.visibilityGraphs.examples;

import java.util.ArrayList;
import java.util.List;

import javafx.application.Application;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import us.ihmc.commons.PrintTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette;
import us.ihmc.pathPlanning.visibilityGraphs.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.NavigableRegionsManager;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PointCloudTools;
import us.ihmc.robotics.geometry.PlanarRegion;

public class Example_DataFile extends Application
{
   private static final String dataFile = "Data/PlanarRegions_FailingTest.txt";

   ArrayList<PlanarRegion> regions;
   Point3D start = new Point3D();
   Point3D goal = new Point3D();

   public Example_DataFile()
   {
      regions = PointCloudTools.loadPlanarRegionsFromFile(dataFile, start, goal);

      if (start.containsNaN())
      {
         PrintTools.warn("Start position was not contained in data file.");
         start.setToZero();
      }
      if (goal.containsNaN())
      {
         PrintTools.warn("Goal position was not contained in data file.");
         goal.setToZero();
      }
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      View3DFactory view3dFactory = new View3DFactory(640, 480);
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);

      TextureColorPalette colorPalette = new TextureColorAdaptivePalette();
      JavaFXMultiColorMeshBuilder javaFXMultiColorMeshBuilder = new JavaFXMultiColorMeshBuilder(colorPalette);

      long startTime = System.currentTimeMillis();

      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(new DefaultVisibilityGraphParameters(), regions, javaFXMultiColorMeshBuilder);
      List<Point3D> path = navigableRegionsManager.calculateBodyPath(start, goal);

      System.out.println("Total Vis. Graphs Time: " + (System.currentTimeMillis() - startTime));

      javaFXMultiColorMeshBuilder.addSphere(0.03f, start, Color.GREEN);
      javaFXMultiColorMeshBuilder.addSphere(0.03f, goal, Color.RED);

      for (PlanarRegion region : navigableRegionsManager.getListOfAccesibleRegions())
      {
         visualizeRegion(region, Color.GREEN, javaFXMultiColorMeshBuilder);
      }

      for (PlanarRegion region : navigableRegionsManager.getListOfObstacleRegions())
      {
         visualizeRegion(region, Color.RED, javaFXMultiColorMeshBuilder);
      }

      if (path.size() < 2)
      {
         PrintTools.warn("No path returned.");
      }

      for (int i = 1; i < path.size(); i++)
      {
         Point3D from = path.get(i - 1);
         Point3D to = path.get(i);

         javaFXMultiColorMeshBuilder.addLine(new Point3D(from.getX(), from.getY(), from.getZ()), new Point3D(to.getX(), to.getY(), to.getZ()), 0.025,
                                             Color.RED);
      }

      for (int i = 0; i < path.size(); i++)
      {
         Point3D to = path.get(i);

         javaFXMultiColorMeshBuilder.addSphere(0.03f, to, Color.YELLOW);

      }

      MeshView meshView = new MeshView(javaFXMultiColorMeshBuilder.generateMesh());
      meshView.setMaterial(javaFXMultiColorMeshBuilder.generateMaterial());
      view3dFactory.addNodeToView(meshView);

      primaryStage.setScene(view3dFactory.getScene());
      primaryStage.show();
   }

   public void visualizeRegion(PlanarRegion region, Color color, JavaFXMultiColorMeshBuilder javaFXMultiColorMeshBuilder)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      region.getTransformToWorld(transform);

      for (int i = 0; i < region.getNumberOfConvexPolygons(); i++)
      {
         javaFXMultiColorMeshBuilder.addPolygon(transform, region.getConvexPolygon(i), color);
      }
   }

   public static void main(String[] args)
   {
      launch();
   }
}
