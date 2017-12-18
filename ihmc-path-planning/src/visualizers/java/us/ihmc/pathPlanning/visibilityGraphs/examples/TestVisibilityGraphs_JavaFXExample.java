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
import us.ihmc.pathPlanning.visibilityGraphs.clusterManagement.Cluster.Type;
import us.ihmc.pathPlanning.visibilityGraphs.tools.ClusterTools;
import us.ihmc.robotics.geometry.PlanarRegion;

/**
 * User: Matt Date: 1/14/13
 */
public class TestVisibilityGraphs_JavaFXExample extends Application
{
   ArrayList<Cluster> clusters = new ArrayList<>();
   ArrayList<PlanarRegion> regions = new ArrayList<>();

   double extrusionDistance = 0.60;

   public TestVisibilityGraphs_JavaFXExample()
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

      loadPointCloudFromFile("PlanarRegions_201709260105.txt");

      for (PlanarRegion region : regions)
      {
         RigidBodyTransform transform = new RigidBodyTransform();
         region.getTransformToWorld(transform);

         Color color = getRegionColor(region.getRegionId());
         for (int i = 0; i < region.getNumberOfConvexPolygons(); i++)
         {
            javaFXMultiColorMeshBuilder.addPolygon(transform, region.getConvexPolygon(i), color);
         }
      }


      //      Cluster cluster = new Cluster();
      //      clusterMgr.addCluster(cluster);
      //      cluster.setType(Type.LINE);
      //      cluster.addRawPoint(new Point3D(5, -1, 0));
      //      cluster.addRawPoint(new Point3D(5,1,0));
      //
      //      Cluster cluster1 = new Cluster();
      //      clusterMgr.addCluster(cluster1);
      //      cluster1.setType(Type.LINE);
      //      cluster1.addRawPoint(new Point3D(5, -1, 0));
      //      cluster1.addRawPoint(new Point3D(6, 1, 0));
      //      
      //      Cluster cluster2 = new Cluster();
      //      clusterMgr.addCluster(cluster2);
      //      cluster2.setType(Type.LINE);
      //      cluster2.addRawPoint(new Point3D(9, -1, 0));
      //      cluster2.addRawPoint(new Point3D(8, 1, 0));
      //      
      //      Cluster cluster3 = new Cluster();
      //      clusterMgr.addCluster(cluster3);
      //      cluster3.setType(Type.LINE);
      //      cluster3.addRawPoint(new Point3D(9, 3, 0));
      //      cluster3.addRawPoint(new Point3D(8, 3, 0));

      Cluster cluster4 = new Cluster();
      clusters.add(cluster4);
      cluster4.setType(Type.POLYGON);
      cluster4.setClusterClosure(true);

      cluster4.addRawPointInWorld(new Point3D(5, -1, 0));
      cluster4.addRawPointInWorld(new Point3D(5, 1, 0));
      cluster4.addRawPointInWorld(new Point3D(6, 1, 0));
      cluster4.addRawPointInWorld(new Point3D(6, -1, 0));

      ClusterTools.performExtrusions(new Point2D(), extrusionDistance, clusters);

      for (Point3D point : cluster4.getRawPointsInWorld())
      {
         javaFXMultiColorMeshBuilder.addSphere(0.1f, point, Color.AQUAMARINE);
      }

      for (int i = 1; i < cluster4.getRawPointsInWorld().size(); i++)
      {
         javaFXMultiColorMeshBuilder.addLine(cluster4.getRawPointsInWorld().get(i - 1), cluster4.getRawPointsInWorld().get(i), 0.005, Color.BLACK);
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

   public ArrayList<Point3D> loadPointCloudFromFile(String fileName)
   {
      ArrayList<Cluster> clusters = new ArrayList<>();
      BufferedReader br = null;
      FileReader fr = null;

      try
      {

         //br = new BufferedReader(new FileReader(FILENAME));
         fr = new FileReader(fileName);
         br = new BufferedReader(fr);

         String sCurrentLine;

         double averageX = 0.0;
         double averageY = 0.0;
         double averageZ = 0.0;

         int index = 0;

         Cluster cluster = new Cluster();
         int nPacketsRead = 0;

         ArrayList<Point3D> pointsTemp = new ArrayList<>();

         while ((sCurrentLine = br.readLine()) != null)
         {
            //            System.out.println(sCurrentLine);

            if (sCurrentLine.contains("PR_"))
            {
               if (!pointsTemp.isEmpty())
               {
                  cluster.addRawPointsInWorld(pointsTemp, true);
                  pointsTemp.clear();
               }

               cluster = new Cluster();
               clusters.add(cluster);
               nPacketsRead++;
               //               System.out.println("New cluster created");
            }

            else if (sCurrentLine.contains("RBT,"))
            {
               //               System.out.println("Transformation read");
               sCurrentLine = sCurrentLine.substring(sCurrentLine.indexOf(",") + 1, sCurrentLine.length());

               sCurrentLine = sCurrentLine.replace("(", "");
               sCurrentLine = sCurrentLine.replace(")", "");

               String[] points = sCurrentLine.split(",");

               float x = (float) Double.parseDouble(points[0]);
               float y = (float) Double.parseDouble(points[1]);
               float z = (float) Double.parseDouble(points[2]);
               Vector3D translation = new Vector3D(x, y, z);

               float qx = (float) Double.parseDouble(points[3]);
               float qy = (float) Double.parseDouble(points[4]);
               float qz = (float) Double.parseDouble(points[5]);
               float qs = (float) Double.parseDouble(points[6]);
               Quaternion quat = new Quaternion(qx, qy, qz, qs);

               RigidBodyTransform rigidBodyTransform = new RigidBodyTransform(quat, translation);
               cluster.setTransformToWorld(rigidBodyTransform);
            }
            else
            {
               //               System.out.println("adding point");

               String[] points = sCurrentLine.split(",");

               float x = (float) Double.parseDouble(points[0]);
               float y = (float) Double.parseDouble(points[1]);
               float z = (float) Double.parseDouble(points[2]);

               pointsTemp.add(new Point3D(x, y, z));

               averageX = averageX + x;
               averageY = averageY + y;
               averageZ = averageZ + z;

               index++;
            }
         }

         Random random = new Random();
         for (Cluster cluster1 : clusters)
         {
            ArrayList<Point2D> vertices = new ArrayList<>();

            for (Point3D pt : cluster1.getRawPointsInWorld())
            {
               vertices.add(new Point2D(pt.getX(), pt.getY()));
            }

            ConvexPolygon2D convexPolygon = new ConvexPolygon2D(vertices);

            PlanarRegion planarRegion = new PlanarRegion(cluster1.getTransformToWorld(), convexPolygon);
            planarRegion.setRegionId(random.nextInt());

            regions.add(planarRegion);
         }

         System.out.println("Loaded " + regions.size() + " regions");
      }
      catch (IOException e)
      {

         e.printStackTrace();

      } finally
      {

         try
         {

            if (br != null)
               br.close();

            if (fr != null)
               fr.close();

         }
         catch (IOException ex)
         {

            ex.printStackTrace();

         }

      }
      return null;

   }

   public static void main(String[] args)
   {
      launch();
   }

}