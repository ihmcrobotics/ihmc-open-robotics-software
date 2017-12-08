package us.ihmc.pathPlanning.visibilityGraphs.examples;

import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Random;

import javafx.application.Application;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PointCloudTools;
import us.ihmc.robotics.geometry.PlanarRegion;

/**
 * User: Matt Date: 1/14/13
 */
public class Example_LoadPlanarRegions extends Application
{
   ArrayList<Cluster> clusters = new ArrayList<>();
   ArrayList<PlanarRegion> regions = new ArrayList<>();
   JavaFXMultiColorMeshBuilder javaFXMultiColorMeshBuilder;

   public Example_LoadPlanarRegions()
   {
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      View3DFactory view3dFactory = new View3DFactory(640, 480);
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);

      TextureColorPalette colorPalette = new TextureColorAdaptivePalette();
      javaFXMultiColorMeshBuilder = new JavaFXMultiColorMeshBuilder(colorPalette);

      regions = PointCloudTools.loadPlanarRegionsFromFile("Data/PlanarRegions_RD_Cinder_Maze.txt");

//      Random rand = new Random();
//      for (PlanarRegion region : regions)
//      {
//         int id = rand.nextInt();
//         region.setRegionId(id);
//
//         RigidBodyTransform transform = new RigidBodyTransform();
//         region.getTransformToWorld(transform);
//
//         Color color = getRegionColor(region.getRegionId());
//         for (int i = 0; i < region.getNumberOfConvexPolygons(); i++)
//         {
//            javaFXMultiColorMeshBuilder.addPolygon(transform, region.getConvexPolygon(i), color);
//         }
//      }

                  visualizeRegion(regions.get(0), Color.BEIGE);
                  visualizeRegion(regions.get(1), Color.BLACK);
                  visualizeRegion(regions.get(2), Color.BEIGE);
      //            visualizeRegion(regions.get(3), Color.BEIGE);
      //            visualizeRegion(regions.get(4), Color.BEIGE);
      //            visualizeRegion(regions.get(5), Color.BEIGE);
      //            visualizeRegion(regions.get(6), Color.BEIGE);
      //          visualizeRegion(regions.get(7), Color.BEIGE);
      //    visualizeRegion(regions.get(11), Color.BEIGE);
      //    visualizeRegion(regions.get(12), Color.BEIGE);
      //    visualizeRegion(regions.get(13), Color.BEIGE);
      //    visualizeRegion(regions.get(14), Color.BEIGE);
      //    visualizeRegion(regions.get(15), Color.BEIGE);
      //    visualizeRegion(regions.get(16), Color.BEIGE);
      //    visualizeRegion(regions.get(17), Color.BEIGE);
      //    visualizeRegion(regions.get(18), Color.BEIGE);
      //    visualizeRegion(regions.get(19), Color.BEIGE);
      //      visualizeRegion(regions.get(31), Color.BEIGE);
      //      visualizeRegion(regions.get(32), Color.BEIGE);
      //      visualizeRegion(regions.get(33), Color.BEIGE);
      //      visualizeRegion(regions.get(34), Color.BEIGE);
      //      visualizeRegion(regions.get(35), Color.BEIGE);
      //      visualizeRegion(regions.get(36), Color.BEIGE);
      //      visualizeRegion(regions.get(37), Color.BEIGE);
      //      visualizeRegion(regions.get(38), Color.BEIGE);

      MeshView meshView = new MeshView(javaFXMultiColorMeshBuilder.generateMesh());
      meshView.setMaterial(javaFXMultiColorMeshBuilder.generateMaterial());
      view3dFactory.addNodeToView(meshView);

      primaryStage.setScene(view3dFactory.getScene());
      primaryStage.show();
   }

   public void visualizeRegion(PlanarRegion region, Color color)
   {
      RigidBodyTransform transform = new RigidBodyTransform();
      region.getTransformToWorld(transform);

      for (int i = 0; i < region.getNumberOfConvexPolygons(); i++)
      {
         javaFXMultiColorMeshBuilder.addPolygon(transform, region.getConvexPolygon(i), color);
      }
   }

   private Color getRegionColor(int regionId)
   {
      java.awt.Color awtColor = new java.awt.Color(regionId);
      return Color.rgb(awtColor.getRed(), awtColor.getGreen(), awtColor.getBlue());
   }

   public static void main(String[] args)
   {
      launch();
   }
}