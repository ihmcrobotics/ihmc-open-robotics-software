package us.ihmc.pathPlanning.visibilityGraphs.examples;
//package test;
//
//import java.io.BufferedReader;
//import java.io.FileReader;
//import java.io.IOException;
//import java.util.ArrayList;
//import java.util.Random;
//
//import clusterManagement.Cluster;
//import clusterManagement.ClusterMgr;
//import clusterManagement.Cluster.Type;
//import javafx.application.Application;
//import javafx.scene.paint.Color;
//import javafx.scene.shape.MeshView;
//import javafx.stage.Stage;
//import tools.PointCloudTools;
//import us.ihmc.euclid.geometry.ConvexPolygon2D;
//import us.ihmc.euclid.transform.RigidBodyTransform;
//import us.ihmc.euclid.tuple2D.Point2D;
//import us.ihmc.euclid.tuple3D.Point3D;
//import us.ihmc.euclid.tuple3D.Vector3D;
//import us.ihmc.euclid.tuple4D.Quaternion;
//import us.ihmc.javaFXToolkit.scenes.View3DFactory;
//import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
//import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
//import us.ihmc.javaFXToolkit.shapes.TextureColorPalette;
//import us.ihmc.robotics.geometry.PlanarRegion;
//
///**
// * User: Matt Date: 1/14/13
// */
//public class TestVisibilityGraphs_AddExtraPointsInsideCluster extends Application
//{
//   JavaFXMultiColorMeshBuilder javaFXMultiColorMeshBuilder;
//
//   public TestVisibilityGraphs_AddExtraPointsInsideCluster()
//   {
//   }
//
//   @Override
//   public void start(Stage primaryStage) throws Exception
//   {
//      View3DFactory view3dFactory = new View3DFactory(640, 480);
//      view3dFactory.addCameraController(true);
//      view3dFactory.addWorldCoordinateSystem(0.3);
//
//      TextureColorPalette colorPalette = new TextureColorAdaptivePalette();
//      javaFXMultiColorMeshBuilder = new JavaFXMultiColorMeshBuilder(colorPalette);
//
//      ArrayList<Point3D> points = new ArrayList<>();
//      //      points.add(new Point3D(1, 1, 0));
//      //      points.add(new Point3D(2, 1, 0));
//      //      points.add(new Point3D(2, 2, 0));
//      //      points.add(new Point3D(1, 2, 0));
//
//      points.add(new Point3D(0.468, 6.468, 0.000));
//      points.add(new Point3D( 0.468, -6.468, 0.000));
//      points.add(new Point3D(-0.468, -6.468, 0.000));
//      points.add(new Point3D(-0.468, 6.468, 0.000));
//      points.add(new Point3D(0.468, 6.468, 0.000));
//
//      /*
//       * ( 0.468, 6.468 ) ( 0.468, -6.468 ) (-0.468, -6.468 ) (-0.468, 6.468 ) (
//       * 0.468, 6.468 )
//       */
//
//      /*
//       * (-0.475, 6.475, 0.000 ) ( 0.475, 6.475, 0.000 ) ( 0.475, -6.475, 0.000
//       * ) (-0.475, -6.475, 0.000 ) (-0.475, 6.475, 0.000 ) ( 0.475, 6.475,
//       * 0.000 )
//       */
//
//      /*
//       * ( 0.000, 1.100 ) ( 0.283, 0.983 ) ( 0.400, 0.450 ) ( 0.283, -0.083 ) (
//       * 0.000, -0.200 ) (-0.283, -0.083 ) (-0.400, 0.450 ) (-0.283, 0.983 ) (
//       * 0.000, 1.100 )
//       */
//
//      Cluster cluster = new Cluster(points, true);
//
//      PointCloudTools.doBrakeDownOn3DPoints(cluster.getRawPointsInCluster(), 0.25);
//
//      javaFXMultiColorMeshBuilder.addSphere(0.03f, new Point3D(cluster.getRawPointsInCluster().get(0).getX(), cluster.getRawPointsInCluster().get(0).getY(), 0),
//                                            Color.GREEN);
//      for (int i = 1; i < cluster.getRawPointsInCluster().size(); i++)
//      {
//         Point3D pt1 = new Point3D(cluster.getRawPointsInCluster().get(i).getX(), cluster.getRawPointsInCluster().get(i).getY(), 0);
//         Point3D pt2 = new Point3D(cluster.getRawPointsInCluster().get(i - 1).getX(), cluster.getRawPointsInCluster().get(i - 1).getY(), 0);
//
//         javaFXMultiColorMeshBuilder.addLine(pt1, pt2, 0.0082, Color.YELLOW);
//         javaFXMultiColorMeshBuilder.addSphere(0.03f, pt1, Color.GREEN);
//      }
//
//      MeshView meshView = new MeshView(javaFXMultiColorMeshBuilder.generateMesh());
//      meshView.setMaterial(javaFXMultiColorMeshBuilder.generateMaterial());
//      view3dFactory.addNodeToView(meshView);
//
//      primaryStage.setScene(view3dFactory.getScene());
//      primaryStage.show();
//   }
//
//   public static void main(String[] args)
//   {
//      launch();
//   }
//
//}