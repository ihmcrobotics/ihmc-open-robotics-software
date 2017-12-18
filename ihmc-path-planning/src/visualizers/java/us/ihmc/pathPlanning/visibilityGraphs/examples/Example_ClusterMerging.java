package us.ihmc.pathPlanning.visibilityGraphs.examples;

import java.util.ArrayList;
import java.util.List;

import javafx.application.Application;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster.ExtrusionSide;
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster.Type;
import us.ihmc.pathPlanning.visibilityGraphs.tools.ClusterTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PointCloudTools;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PointCloudTools.WindingOrder;
import us.ihmc.robotics.geometry.PlanarRegion;

/**
 * User: Matt Date: 1/14/13
 */
public class Example_ClusterMerging extends Application
{
   ArrayList<Cluster> clusters = new ArrayList<>();
   ArrayList<PlanarRegion> regions = new ArrayList<>();

   double extrusionDistance = 0.20;


   ArrayList<Integer> indicesToRemOther = new ArrayList<>();
   ArrayList<Integer> indicesToAddFromHome = new ArrayList<>();

   JavaFXMultiColorMeshBuilder javaFXMultiColorMeshBuilder;

   public Example_ClusterMerging()
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
      javaFXMultiColorMeshBuilder = new JavaFXMultiColorMeshBuilder(colorPalette);

      createClosedSquare_OutsideExtrusion();
      createLine_Extrusion();

      ClusterTools.performExtrusions(new Point2D(), extrusionDistance, clusters);

      //START

      Cluster homeCluster = clusters.get(0);
      Cluster clusterToCheck = clusters.get(1);

      ArrayList<Point2D> navigablePoints = generateListOfPointsInsideHomeCluster(homeCluster.getNavigableExtrusionsInLocal(),
                                                                                 clusterToCheck.getNavigableExtrusionsInLocal());
      ArrayList<Point2D> nonNavigablePoints = generateListOfPointsInsideHomeCluster(homeCluster.getNonNavigableExtrusionsInLocal(),
                                                                                    clusterToCheck.getNonNavigableExtrusionsInLocal());

      //      for (int i = 1; i < navigablePoints.size(); i++)
      //      {
      //         Point3D pt1 = new Point3D(navigablePoints.get(i - 1).getX(), navigablePoints.get(i - 1).getY(), 0);
      //         Point3D pt2 = new Point3D(navigablePoints.get(i).getX(), navigablePoints.get(i).getY(), 0);
      //
      //         javaFXMultiColorMeshBuilder.addLine(pt1, pt2, 0.006, Color.ORANGE);
      //      }

      for (int i = 1; i < nonNavigablePoints.size(); i++)
      {
         Point3D pt1 = new Point3D(nonNavigablePoints.get(i - 1).getX(), nonNavigablePoints.get(i - 1).getY(), 0);
         Point3D pt2 = new Point3D(nonNavigablePoints.get(i).getX(), nonNavigablePoints.get(i).getY(), 0);

         javaFXMultiColorMeshBuilder.addLine(pt1, pt2, 0.006, Color.ORANGE);
      }

      for (int i = 0; i < nonNavigablePoints.size(); i++)
      {
         Point3D pt1 = new Point3D(nonNavigablePoints.get(i).getX(), nonNavigablePoints.get(i).getY(), 0);

         javaFXMultiColorMeshBuilder.addSphere(0.03f, new Point3D(pt1.getX(), pt1.getY(), 0), Color.RED);
      }

      for (Cluster cluster : clusters)
      {
         for (Point3D point : cluster.getRawPointsInWorld())
         {
            javaFXMultiColorMeshBuilder.addSphere(0.03f, point, Color.AQUAMARINE);
         }

         for (int i = 1; i < cluster.getRawPointsInLocal().size(); i++)
         {
            javaFXMultiColorMeshBuilder.addLine(cluster.getRawPointsInWorld().get(i - 1), cluster.getRawPointsInWorld().get(i), 0.005, Color.AQUAMARINE);
         }
         //      
         //         for (Point3D point : cluster.getListOfSafeNormals())
         //         {
         //            javaFXMultiColorMeshBuilder.addSphere(0.03f, point, Color.WHITE);
         //         }
         //
         //      for (Point2D point : cluster4.getListOfNonNavigableExtrusions())
         //      {
         //         javaFXMultiColorMeshBuilder.addSphere(0.03f, new Point3D(point.getX(), point.getY(), 0), Color.YELLOW);
         //      }
         //      
            javaFXMultiColorMeshBuilder.addMultiLine(cluster.getNonNavigableExtrusionsInWorld(), 0.005, Color.YELLOW, false);
      }

      MeshView meshView = new MeshView(javaFXMultiColorMeshBuilder.generateMesh());
      meshView.setMaterial(javaFXMultiColorMeshBuilder.generateMaterial());
      view3dFactory.addNodeToView(meshView);

      primaryStage.setScene(view3dFactory.getScene());
      primaryStage.show();
   }

   public ArrayList<Point2D> generateListOfPointsInsideHomeCluster(List<Point2D> homePoints, List<Point2D> pointsToCheck)
   {
      int startj = -1;
      int endj = -1;
      int starti = -1;
      int endi = -1;

      ArrayList<Point2D> pointsInsideNewCluster = new ArrayList<>();

      for (int i = 1; i < homePoints.size(); i++)
      {
         Point2D from1 = homePoints.get(i - 1);
         Point2D to1 = homePoints.get(i);

         for (int j = 1; j < pointsToCheck.size(); j++)
         {
            Point2D from2 = pointsToCheck.get(j - 1);
            Point2D to2 = pointsToCheck.get(j);

            Point2D intersection = EuclidGeometryTools.intersectionBetweenTwoLineSegment2Ds(from1, to1, from2, to2);

            if (intersection != null)
            {
               pointsInsideNewCluster.add(intersection);
               //               javaFXMultiColorMeshBuilder.addSphere(0.03f, new Point3D(intersection.getX(), intersection.getY(), 0), Color.RED);
               if (startj == -1)
               {
                  startj = j;
                  starti = i;
                  continue;
               }
               if (startj != -1 && endj == -1)
               {
                  endi = i;
                  endj = j;
                  continue;
               }
            }
         }
      }

      System.out.println("Final: " + startj + "   " + endj);

      ConvexPolygon2D homePol = new ConvexPolygon2D(homePoints);
      homePol.update();

      for (Point2D point : pointsToCheck)
      {
         if (homePol.isPointInside(point))
         {
            pointsInsideNewCluster.add(point);
         }
      }

      ConvexPolygon2D regionPol = new ConvexPolygon2D(pointsToCheck);
      regionPol.update();

      for (Point2D point : homePoints)
      {
         if (regionPol.isPointInside(point))
         {
            pointsInsideNewCluster.add(point);
         }
      }

      ArrayList<Point3D> points3d = new ArrayList<>();

      for (Point2D point : pointsInsideNewCluster)
      {
         points3d.add(new Point3D(point.getX(), point.getY(), 0));
      }

      Point3D centroid = PointCloudTools.getCentroid(points3d);

      ArrayList<Point3D> orderedPoints = PointCloudTools.orderPoints(points3d, WindingOrder.CW, centroid);

      pointsInsideNewCluster.clear();
      for (Point3D point : orderedPoints)
      {
         pointsInsideNewCluster.add(new Point2D(point.getX(), point.getY()));
      }

      return pointsInsideNewCluster;
   }

   private void createClosedSquare_OutsideExtrusion()
   {
      Cluster cluster4 = new Cluster();
      cluster4.setType(Type.POLYGON);

      cluster4.addRawPointInWorld(new Point3D(-0.975, 0.475 + 2, 0.000));
      cluster4.addRawPointInWorld(new Point3D(0.975, 0.475 + 2, 0.000));
      cluster4.addRawPointInWorld(new Point3D(0.975, -0.475 + 2, 0.000));
      cluster4.addRawPointInWorld(new Point3D(-0.975, -0.475 + 2, 0.000));

      cluster4.setClusterClosure(true);
      cluster4.setExtrusionSide(ExtrusionSide.OUTSIDE);
      clusters.add(cluster4);
   }

   private void createLine_Extrusion()
   {
      Cluster cluster4 = new Cluster();
      cluster4.setType(Type.LINE);

      cluster4.addRawPointInWorld(new Point3D(-0.975 + 2, 0.475 + 1.5, 0.000));
      cluster4.addRawPointInWorld(new Point3D(0.975 + 2, 0.475 + 1.5, 0.000));

      clusters.add(cluster4);
   }

   public static void main(String[] args)
   {
      launch();
   }

}