package us.ihmc.pathPlanning.visibilityGraphs.examples;

import java.util.ArrayList;
import java.util.List;

import org.jgrapht.graph.DefaultWeightedEdge;
import org.jgrapht.graph.SimpleWeightedGraph;

import javafx.application.Application;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette;
import us.ihmc.pathPlanning.visibilityGraphs.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.NavigableRegion;
import us.ihmc.pathPlanning.visibilityGraphs.NavigableRegionsManager;
import us.ihmc.pathPlanning.visibilityGraphs.tools.PointCloudTools;
import us.ihmc.robotics.geometry.PlanarRegion;

/**
 * User: Matt Date: 1/14/13
 */
public class Example_IsStartGoalInsideRegion extends Application
{
   ArrayList<PlanarRegion> regions = new ArrayList<>();
   ArrayList<SimpleWeightedGraph<Point3D, DefaultWeightedEdge>> visMaps = new ArrayList<>();
   ArrayList<NavigableRegion> listOfNavigableRegions = new ArrayList<>();

   SimpleWeightedGraph<Point3D, DefaultWeightedEdge> globalVisMap = new SimpleWeightedGraph<>(DefaultWeightedEdge.class);
   JavaFXMultiColorMeshBuilder javaFXMultiColorMeshBuilder;

   Point3D startPos = new Point3D(12, 8, 0); //-0.7, 3.9, 0 with region 31
   Point3D goalPos = new Point3D(-2.5, 0.5, 0); // on region 34

   public Example_IsStartGoalInsideRegion()
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

      regions = PointCloudTools.loadPlanarRegionsFromFile("Data/PlanarRegions_NRI_Maze.txt");

      startPos = projectPointToPlane(startPos, regions.get(0));
      goalPos = projectPointToPlane(goalPos, regions.get(0));
      
      javaFXMultiColorMeshBuilder.addSphere(0.045f, startPos, Color.GREEN);
      javaFXMultiColorMeshBuilder.addSphere(0.045f, goalPos, Color.RED);

      long startTime = System.currentTimeMillis();

      NavigableRegionsManager navigableRegionsManager = new NavigableRegionsManager(new DefaultVisibilityGraphParameters(), regions, javaFXMultiColorMeshBuilder);
      List<Point3D> path = navigableRegionsManager.calculateBodyPath(startPos, goalPos);
      
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

   public void visualize()
   {

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

   private Vector3D calculateNormal1(PlanarRegion region)
   {
      if (!region.getConvexHull().isEmpty())
      {
         FramePoint3D fpt1 = new FramePoint3D();
         fpt1.set(new Point3D(region.getConvexHull().getVertex(0).getX(), region.getConvexHull().getVertex(0).getY(), 0));
         RigidBodyTransform trans = new RigidBodyTransform();
         region.getTransformToWorld(trans);
         fpt1.applyTransform(trans);

         FramePoint3D fpt2 = new FramePoint3D();
         fpt2.set(new Point3D(region.getConvexHull().getVertex(1).getX(), region.getConvexHull().getVertex(1).getY(), 0));
         fpt2.applyTransform(trans);

         FramePoint3D fpt3 = new FramePoint3D();
         fpt3.set(new Point3D(region.getConvexHull().getVertex(2).getX(), region.getConvexHull().getVertex(2).getY(), 0));
         fpt3.applyTransform(trans);

         Vector3D normal = EuclidGeometryTools.normal3DFromThreePoint3Ds(fpt1.getPoint(), fpt2.getPoint(), fpt3.getPoint());
         return normal;
      }
      return null;
   }

   public Point3D projectPointToPlane(Point3D pointToProject, PlanarRegion regionToProjectTo)
   {
      Vector3D normal = calculateNormal1(regionToProjectTo);
      Point2D point2D = (Point2D) regionToProjectTo.getConvexHull().getVertex(0);
      Point3D point3D = new Point3D(point2D.getX(), point2D.getY(), 0);

      FramePoint3D fpt1 = new FramePoint3D();
      fpt1.set(point3D);
      RigidBodyTransform trans = new RigidBodyTransform();
      regionToProjectTo.getTransformToWorld(trans);
      fpt1.applyTransform(trans);

      Point3D projectedPoint = new Point3D();
      if (!EuclidGeometryTools.orthogonalProjectionOnPlane3D(pointToProject, fpt1.getPoint(), normal, projectedPoint))
      {
         projectedPoint = null;
      }
      return projectedPoint;
   }

   public static void main(String[] args)
   {
      launch();
   }
}