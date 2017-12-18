package us.ihmc.pathPlanning.visibilityGraphs.examples;

import java.util.ArrayList;

import javafx.application.Application;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.robotics.geometry.PlanarRegion;

/**
 * User: Matt Date: 1/14/13
 */
public class Test_ShowPoints extends Application
{
   ArrayList<Cluster> clusters = new ArrayList<>();
   ArrayList<PlanarRegion> regions = new ArrayList<>();

   double extrusionDistance = 0.60;

   public Test_ShowPoints()
   {
   }

   private Color getRegionColor(int regionId)
   {
      java.awt.Color awtColor = new java.awt.Color(regionId);
      return Color.rgb(awtColor.getRed(), awtColor.getGreen(), awtColor.getBlue());
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      View3DFactory view3dFactory = new View3DFactory(640, 480);
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);

      TextureColorPalette colorPalette = new TextureColorAdaptivePalette();
      JavaFXMultiColorMeshBuilder javaFXMultiColorMeshBuilder = new JavaFXMultiColorMeshBuilder(colorPalette);

      ArrayList<Point3D> points = new ArrayList<>();
      points.add(new Point3D(1.670, -1.273, 0.021));
      points.add(new Point3D(2.092, 0.325, 0.021));
      points.add(new Point3D(2.149, 0.342, 0.021));
      points.add(new Point3D(2.485, 0.429, 0.021));
      points.add(new Point3D(2.618, 0.460, 0.021));
      points.add(new Point3D(2.718, 0.481, 0.0211));
      points.add(new Point3D(2.844, 0.446, 0.021));
      points.add(new Point3D(2.876, 0.402, 0.021));
      points.add(new Point3D(2.628, -1.167, 0.021));
      points.add(new Point3D(2.569, -1.184, 0.021));
      points.add(new Point3D(2.413, -1.207, 0.021));
      points.add(new Point3D(2.252, -1.229, 0.021));
      points.add(new Point3D(1.771, -1.291, 0.021));

      for (Point3D point : points)
      {
         javaFXMultiColorMeshBuilder.addSphere(0.1f, point, Color.AQUAMARINE);
      }

      for (int i = 1; i < points.size(); i++)
      {
         javaFXMultiColorMeshBuilder.addLine(points.get(i - 1), points.get(i), 0.005, Color.BLACK);
      }
      //
      //      for (int i = 1; i < cluster4.getListOfNonNavigableExtrusions().size(); i++)
      //      {
      //         drawLine(getZUpNode(),
      //                  new Point3D(cluster4.getListOfNavigableExtrusions().get(i - 1).getX(), cluster4.getListOfNavigableExtrusions().get(i - 1).getY(), 0),
      //                  new Point3D(cluster4.getListOfNavigableExtrusions().get(i).getX(), cluster4.getListOfNavigableExtrusions().get(i).getY(), 0), ColorRGBA.Red,
      //                  6);
      //      }
      //
      //      VisibilityGraph visibilityGraph = new VisibilityGraph(clusterMgr);
      //      visibilityGraph.createStaticVisibilityMap(new Point2D(), new Point2D(10, 0));
      //
      //      for (DefaultWeightedEdge edge : visibilityGraph.getVisibilityMap().edgeSet())
      //      {
      //         Point2D edgeSource = visibilityGraph.getVisibilityMap().getEdgeSource(edge);
      //         Point2D edgeTarget = visibilityGraph.getVisibilityMap().getEdgeTarget(edge);
      //
      //         drawLine(getZUpNode(), new Point3D(edgeSource.getX(), edgeSource.getY(), 0), new Point3D(edgeTarget.getX(), edgeTarget.getY(), 0), ColorRGBA.Cyan, 3);
      //      }

      //      ArrayList<DefaultWeightedEdge> edges = (ArrayList<DefaultWeightedEdge>) DijkstraShortestPath.findPathBetween(visibilityGraph.getVisibilityMap(),
      //                                                                                                                   new Point2D(), new Point2D(10, 0));

      //      ArrayList<Point2D> path = visibilityGraph.solve(new Point2D(), new Point2D(10, 0));
      //      for (int i = 1; i < path.size(); i++)
      //      {
      //         drawLine(getZUpNode(), new Point3D(path.get(i - 1).getX(), path.get(i - 1).getY(), 0), new Point3D(path.get(i).getX(), path.get(i - 1).getY(), 0),
      //                  ColorRGBA.Blue, 5);
      //      }

      //      for (DefaultWeightedEdge edge : edges)
      //      {
      //         Point2D from = visibilityGraph.getVisibilityMap().getEdgeSource(edge);
      //         Point2D to = visibilityGraph.getVisibilityMap().getEdgeTarget(edge);
      //         drawLine(getZUpNode(), new Point3D(from.getX(), from.getY(), 0), new Point3D(to.getX(), to.getY(), 0), ColorRGBA.Red, 6);
      //      }
      //
      //      System.out.println(edges.size());

      MeshView meshView = new MeshView(javaFXMultiColorMeshBuilder.generateMesh());
      meshView.setMaterial(javaFXMultiColorMeshBuilder.generateMaterial());
      view3dFactory.addNodeToView(meshView);

      primaryStage.setScene(view3dFactory.getScene());
      primaryStage.show();
   }

   public static void main(String[] args)
   {
      launch();
   }

}