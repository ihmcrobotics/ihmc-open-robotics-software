package us.ihmc.octoMap;

import javafx.application.Application;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import javafx.scene.shape.Sphere;
import javafx.stage.Stage;
import us.ihmc.commons.Conversions;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.rotationConversion.RotationMatrixConversion;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Point3D32;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.jOctoMap.iterators.OcTreeIterable;
import us.ihmc.jOctoMap.iterators.OcTreeIteratorFactory;
import us.ihmc.jOctoMap.node.NormalOcTreeNode;
import us.ihmc.jOctoMap.ocTree.NormalOcTree;
import us.ihmc.jOctoMap.pointCloud.PointCloud;
import us.ihmc.jOctoMap.pointCloud.ScanCollection;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette1D;
import us.ihmc.robotEnvironmentAwareness.geometry.IntersectionPlaneBoxCalculator;
import us.ihmc.robotics.lists.GenericTypeBuilder;
import us.ihmc.robotics.lists.RecyclingArrayList;

public class NormalOcTreeVisualizer extends Application
{
   public final NormalOcTree ocTree = new NormalOcTree(0.025);
   private static final boolean SHOW_FREE_CELLS = false;
   private static final Color FREE_COLOR = new Color(Color.YELLOW.getRed(), Color.YELLOW.getGreen(), Color.YELLOW.getBlue(), 0.0);
   private static final boolean SHOW_POINT_CLOUD = true;
   private static final boolean SHOW_HIT_LOCATIONS = true;

   public NormalOcTreeVisualizer()
   {

      Point3D lidarPosition = new Point3D(0.0, 0.0, 2.0);
      //      callUpdateNode();
      //      callInsertPointCloud();
      createPlane(lidarPosition, 12.0, 0.0, -0.05);
//      createBowl(0.5, new Point3D());
      System.out.println("Number of leafs: " + ocTree.getNumberOfLeafNodes());
      System.out.println("Initialized octree");
      System.out.println("Computing normals");
      long startTime = System.nanoTime();
      ocTree.update(new ScanCollection(pointcloud, lidarPosition));
      ocTree.update(new ScanCollection(pointcloud, lidarPosition));
      ocTree.update(new ScanCollection(pointcloud, lidarPosition));
      ocTree.update(new ScanCollection(pointcloud, lidarPosition));
      ocTree.update(new ScanCollection(pointcloud, lidarPosition));
      ocTree.update(new ScanCollection(pointcloud, lidarPosition));
      long endTime = System.nanoTime();
      System.out.println("Done computing normals: time it took = " + Conversions.nanosecondsToSeconds(endTime - startTime));
   }
   
   PointCloud pointcloud = new PointCloud();

   private void callInsertPointCloud()
   {
      Point3D origin = new Point3D(0.01, 0.01, 0.02);
      Point3D pointOnSurface = new Point3D(4.01, 0.01, 0.01);

      for (int i = 0; i < 100; i++)
      {
         for (int j = 0; j < 100; j++)
         {
            Point3D rotated = new Point3D(pointOnSurface);
            RotationMatrix rotation = new RotationMatrix();
            RotationMatrixConversion.convertYawPitchRollToMatrix(Math.toRadians(i * 0.5), Math.toRadians(j * 0.5), 0.0, rotation);
            rotation.transform(rotated);
            pointcloud.add(rotated);
         }
      }

      ocTree.update(new ScanCollection(pointcloud, origin));
   }

   public void createPlane(Point3D lidarPosition, double pitch, double roll, double z)
   {
      pointcloud.clear();

      double planeSize = 1.00;

      for (double x = -0.5 * planeSize; x < 0.5 * planeSize; x += 0.7 * ocTree.getResolution())
      {
         for (double y = -0.5 * planeSize; y < 0.5 * planeSize; y += 0.7 * ocTree.getResolution())
         {
            Point3D point = new Point3D(x, y, 0.0);
            RotationMatrix rotation = new RotationMatrix();
            RotationMatrixConversion.convertYawPitchRollToMatrix(0.0, Math.toRadians(pitch), Math.toRadians(roll), rotation);
            rotation.transform(point);
            point.setZ(point.getZ() + z);

            pointcloud.add(point);
         }
      }
      ocTree.update(new ScanCollection(pointcloud, lidarPosition));
   }

   public void createBowl(double radius, Point3D center)
   {
      Point3D origin = new Point3D(0.0, 0.0, center.getZ() + 0.0);

      double res = 0.05;
      for (double yaw = 0.0; yaw < 2.0 * Math.PI; yaw += res)
      {
         for (double pitch = 0.0; pitch < 0.5 * Math.PI; pitch += res)
         {
            double x = Math.cos(pitch) * Math.cos(yaw) * radius + center.getX();
            double y = Math.cos(pitch) * Math.sin(yaw) * radius + center.getY();
            double z = - Math.sin(pitch) * radius + center.getZ();
            pointcloud.add(x, y, z);
         }
      }

      ocTree.update(new ScanCollection(pointcloud, origin));
   }

   private final RecyclingArrayList<Point3D> plane = new RecyclingArrayList<>(GenericTypeBuilder.createBuilderWithEmptyConstructor(Point3D.class));
   private final IntersectionPlaneBoxCalculator intersectionPlaneBoxCalculator = new IntersectionPlaneBoxCalculator();

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      primaryStage.setTitle("OcTree Visualizer");

      View3DFactory view3dFactory = new View3DFactory(600, 400);
      view3dFactory.addCameraController();
      view3dFactory.addWorldCoordinateSystem(0.3);

      TextureColorPalette1D palette = new TextureColorPalette1D();
      palette.setHueBased(0.9, 0.8);
      JavaFXMultiColorMeshBuilder occupiedMeshBuilder = new JavaFXMultiColorMeshBuilder(palette);
      JavaFXMeshBuilder freeMeshBuilder = new JavaFXMeshBuilder();

      OcTreeIterable<NormalOcTreeNode> leafIterable = OcTreeIteratorFactory.createLeafIterable(ocTree.getRoot(), 14);
      for (NormalOcTreeNode node : leafIterable)
      {
         double boxSize = node.getSize();
         Point3D nodeCenter = new Point3D();
         node.getCoordinate(nodeCenter);

         if (ocTree.isNodeOccupied(node))
         {
            Vector3D planeNormal = new Vector3D();
            node.getNormal(planeNormal);
            boolean isNormalSet = node.isNormalSet();
            Color normalBasedColor = getNormalBasedColor(planeNormal, isNormalSet);
            if (isNormalSet)
            {
               Point3D pointOnPlane = new Point3D();
               if (node.isHitLocationSet())
                  node.getHitLocation(pointOnPlane);
               else
                  pointOnPlane.set(nodeCenter);
               intersectionPlaneBoxCalculator.setCube(boxSize, nodeCenter);
               intersectionPlaneBoxCalculator.setPlane(pointOnPlane, planeNormal);
               intersectionPlaneBoxCalculator.computeIntersections(plane);
               occupiedMeshBuilder.addPolyon(plane, normalBasedColor);
               if (SHOW_HIT_LOCATIONS)
                  occupiedMeshBuilder.addCube(0.01, pointOnPlane, DEFAULT_COLOR);
               
               

               for (int j = 0; j < plane.size(); j++)
               {
                  Point3D intersection = plane.get(j);

                  Vector3D sub = new Vector3D();
                  sub.sub(intersection, pointOnPlane);

                  Vector3D v0 = new Vector3D();
                  Vector3D v1 = new Vector3D();
                  Vector3D v3 = new Vector3D();
                  Point3D nextIntersection = plane.get((j + 1) % plane.size());
                  Point3D previousIntersection = plane.get(j == 0 ? plane.size() - 1 : j - 1);
                  v0.sub(intersection, nextIntersection);
                  v1.sub(intersection, previousIntersection);
                  v3.cross(v0, v1);
                  if (v3.dot(planeNormal) < 0.0 || Math.abs(sub.dot(planeNormal)) > 0.01)
                  {
                     System.err.println("node size: " + boxSize);
                     System.err.println("      Point3D cubeCenter = new Point3D" + nodeCenter + ";");
                     System.err.println("      Point3D pointOnPlane = new Point3D" + pointOnPlane + ";");
                     System.err.println("      Vector3d planeNormal = new Vector3d" + planeNormal + ";");
                     System.out.println();
                  }
               }
               
               
               
               
            }
            else
               occupiedMeshBuilder.addCube((float) boxSize, new Point3D32(nodeCenter), normalBasedColor);
         }
         else if (SHOW_FREE_CELLS)
         {
            freeMeshBuilder.addCube((float) boxSize, new Point3D32(nodeCenter));
         }
      }

      MeshView occupiedMeshView = new MeshView();
      occupiedMeshView.setMesh(occupiedMeshBuilder.generateMesh());
      occupiedMeshView.setMaterial(occupiedMeshBuilder.generateMaterial());
      occupiedMeshView.setMouseTransparent(true);
      view3dFactory.addNodeToView(occupiedMeshView);

      if (SHOW_FREE_CELLS)
      {
         MeshView freeMeshView = new MeshView();
         freeMeshView.setMesh(freeMeshBuilder.generateMesh());
         PhongMaterial material = new PhongMaterial();
         material.setDiffuseColor(FREE_COLOR);
         freeMeshView.setMaterial(material);
         view3dFactory.addNodeToView(freeMeshView);
      }

      if (SHOW_POINT_CLOUD)
      {
         for (int i = 0; i < pointcloud.getNumberOfPoints(); i++)
         {
            Sphere sphere = new Sphere(0.0025);
            sphere.setTranslateX(pointcloud.getPoint(i).getX());
            sphere.setTranslateY(pointcloud.getPoint(i).getY());
            sphere.setTranslateZ(pointcloud.getPoint(i).getZ());
            view3dFactory.addNodeToView(sphere);
         }
      }

      primaryStage.setScene(view3dFactory.getScene());
      primaryStage.show();
   }

   private static final Color DEFAULT_COLOR = Color.DARKCYAN;

   public Color getNormalBasedColor(Vector3D normal, boolean isNormalSet)
   {
      Color color = DEFAULT_COLOR;

      if (isNormalSet)
      {
         Vector3D zUp = new Vector3D(0.0, 0.0, 1.0);
         normal.normalize();
         double angle = Math.abs(zUp.dot(normal));
         double hue = 120.0 * angle;
         color = Color.hsb(hue, 1.0, 1.0);
      }
      return color;
   }

   public static void main(String[] args)
   {

      //      new OcTreeVisualizer();

      Application.launch(args);
   }
}
