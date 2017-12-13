package us.ihmc.pathPlanning.visibilityGraphs.examples;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.stream.Collectors;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.collections.ObservableList;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DReadOnly;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.javaFXToolkit.shapes.TextureColorPalette;
import us.ihmc.pathPlanning.visibilityGraphs.DefaultVisibilityGraphParameters;
import us.ihmc.pathPlanning.visibilityGraphs.NavigableRegionsManager;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OcTreeMeshBuilder;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionDataImporter;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.robotics.lists.ListWrappingIndexTools;

/**
 * User: Matt Date: 1/14/13
 */
public class Example_Concavity extends Application
{
   private static final File defaultFile = new File("../../Data/20171026_131304_PlanarRegion_Ramp_2Story");

   public Example_Concavity()
   {
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      View3DFactory view3dFactory = new View3DFactory(640, 480);
      view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);

      TextureColorPalette colorPalette = new TextureColorAdaptivePalette();
      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(colorPalette);

      PlanarRegionsList planarRegionData;
      
      if (defaultFile != null)
         planarRegionData = PlanarRegionDataImporter.importPlanRegionData(defaultFile);
      else
         planarRegionData = PlanarRegionDataImporter.importUsingFileChooser(primaryStage);
      if (planarRegionData == null)
         Platform.exit();

      List<PlanarRegion> regions = planarRegionData.getPlanarRegionsAsList();
      ArrayList<PlanarRegion> filteredRegions = new ArrayList<>();

      for (PlanarRegion region : regions)
      {
         if (region.getConcaveHullSize() > 2)
         {
            filteredRegions.add(region);
         }
      }

      System.out.println(regions.size() + "   " + filteredRegions.size());

      NavigableRegionsManager manager = new NavigableRegionsManager(new DefaultVisibilityGraphParameters(), filteredRegions);

      Point3D start = new Point3D(-0.45, -0.25, 0.05);
      Point3D goal = new Point3D(1.25, 1.45, 0.05);

      meshBuilder.addSphere(0.01, start, Color.GREEN);
      meshBuilder.addSphere(0.01, goal, Color.RED);

      List<Point3DReadOnly> bodyPath = manager.calculateBodyPath(start, goal);

      meshBuilder.addMultiLine(bodyPath, 0.01, Color.YELLOW, false);

      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(meshBuilder.generateMaterial());
      meshView.setMouseTransparent(true);
      view3dFactory.addNodeToView(meshView);
      filteredRegions.stream().map(this::createRegionGraphics).forEach(view3dFactory::addNodeToView);

      primaryStage.setScene(view3dFactory.getScene());
      primaryStage.show();
   }

   private Node createRegionGraphics(PlanarRegion data)
   {
      Group regionGroup = new Group();
      ObservableList<Node> children = regionGroup.getChildren();

      children.add(createConcaveHullGraphics(data));
      children.add(createConvexPolygonGraphics(data));

      return regionGroup;
   }

   private Node createConcaveHullGraphics(PlanarRegion data)
   {
      int regionId = data.getRegionId();
      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(16));
      RigidBodyTransform transform = new RigidBodyTransform();
      data.getTransformToWorld(transform);

      Point2D[] concaveHullVerticesLocal = data.getConcaveHull();
      Color regionColor = OcTreeMeshBuilder.getRegionColor(regionId);

      List<Point3D> concaveHullVertices = Arrays.stream(concaveHullVerticesLocal).map(Point3D::new).map(p -> {
         transform.transform(p);
         return p;
      }).collect(Collectors.toList());

      for (int vertexIndex = 0; vertexIndex < concaveHullVertices.size(); vertexIndex++)
      {
         Point3D vertex = concaveHullVertices.get(vertexIndex);
         Point3D nextVertex = ListWrappingIndexTools.getNext(vertexIndex, concaveHullVertices);
         Color lineColor = Color.hsb(regionColor.getHue(), regionColor.getSaturation(), regionColor.getBrightness());
         meshBuilder.addLine(vertex, nextVertex, 0.0015, lineColor);
      }
      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(meshBuilder.generateMaterial());
      meshView.setMouseTransparent(true);
      return meshView;
   }

   private Node createConvexPolygonGraphics(PlanarRegion data)
   {
      List<ConvexPolygon2D> convexPolygons = new ArrayList<>();
      for (int i = 0; i < data.getNumberOfConvexPolygons(); i++)
         convexPolygons.add(data.getConvexPolygon(i));

      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(64));

      int regionId = data.getRegionId();
      RigidBodyTransform rigidBodyTransform = new RigidBodyTransform();
      data.getTransformToWorld(rigidBodyTransform);
      Color regionColor = OcTreeMeshBuilder.getRegionColor(regionId);

      for (int i = 0; i < convexPolygons.size(); i++)
      {
         ConvexPolygon2D convexPolygon = convexPolygons.get(i);
         Color color = Color.hsb(regionColor.getHue(), 0.9, 0.5 + 0.5 * ((double) i / (double) convexPolygons.size()));
         meshBuilder.addPolygon(rigidBodyTransform, convexPolygon, color);
      }

      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(meshBuilder.generateMaterial());
      return meshView;
   }

   public static void main(String[] args)
   {
      launch();
   }
}