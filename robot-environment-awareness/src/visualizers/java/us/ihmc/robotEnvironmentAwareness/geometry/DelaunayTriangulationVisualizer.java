package us.ihmc.robotEnvironmentAwareness.geometry;

import java.util.Collections;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;

import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple4D.Quaternion;

import com.vividsolutions.jts.geom.MultiPoint;
import com.vividsolutions.jts.triangulate.ConformingDelaunayTriangulationBuilder;
import com.vividsolutions.jts.triangulate.quadedge.QuadEdge;
import com.vividsolutions.jts.triangulate.quadedge.QuadEdgeSubdivision;
import com.vividsolutions.jts.triangulate.quadedge.QuadEdgeTriangle;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.collections.ObservableList;
import javafx.event.EventHandler;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.Scene;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.input.MouseEvent;
import javafx.scene.paint.Color;
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerTools;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OcTreeMeshBuilder;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionSegmentationRawDataImporter;

public class DelaunayTriangulationVisualizer extends Application
{
   private static final boolean VISUALIZE_EDGES = false;
   private static final boolean VISUALIZE_PRIMARY_EDGES = true;
   private static final boolean VISUALIZE_ORDERED_BORDER_EDGES = true;

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      primaryStage.setTitle(getClass().getSimpleName());

//    PlanarRegionSegmentationDataImporter dataImporter = new PlanarRegionSegmentationDataImporter(new File("../../Data/20161210_184102_PlanarRegionSegmentation_Sim_CB"));
      PlanarRegionSegmentationRawDataImporter dataImporter = PlanarRegionSegmentationRawDataImporter.createImporterWithFileChooser(primaryStage);
      if (dataImporter == null)
         Platform.exit();
      dataImporter.loadPlanarRegionSegmentationData();
      List<PlanarRegionSegmentationRawData> regionsRawData = dataImporter.getPlanarRegionSegmentationRawData();

      View3DFactory view3dFactory = new View3DFactory(600, 400);
      view3dFactory.addCameraController();
      view3dFactory.addWorldCoordinateSystem(0.3);

      Map<Node, Integer> nodeToRegionId = new HashMap<>();

      Point3D average = PolygonizerVisualizer.computeAverage(regionsRawData, Collections.emptySet());
      average.negate();

      for (PlanarRegionSegmentationRawData rawData : regionsRawData)
      {
         Node regionGraphics = createRegionGraphics(rawData);
         regionGraphics.setManaged(false);
         PolygonizerVisualizer.translateNode(regionGraphics, average);
         nodeToRegionId.put(regionGraphics, rawData.getRegionId());
         view3dFactory.addNodeToView(regionGraphics);
      }

      Scene scene = view3dFactory.getScene();
      scene.addEventHandler(MouseEvent.MOUSE_PRESSED, new EventHandler<MouseEvent>()
      {
         @Override
         public void handle(MouseEvent event)
         {
            if (!event.isAltDown())
               return;
            Node intersectedNode = event.getPickResult().getIntersectedNode();
            if (intersectedNode == null)
               return;

            Integer regionId = nodeToRegionId.get(intersectedNode);

            while (regionId == null && intersectedNode.getParent() != null)
            {
               intersectedNode = intersectedNode.getParent();
               regionId = nodeToRegionId.get(intersectedNode);
            }

            if (regionId != null)
            {
               System.out.println("Region picked: " + regionId);
               for (Entry<Node, Integer> nodeAndId : nodeToRegionId.entrySet())
               {
                  if (nodeAndId.getValue() != regionId)
                     nodeAndId.getKey().setVisible(false);
               }
            }
         }
      });

      primaryStage.addEventHandler(KeyEvent.KEY_PRESSED, event -> {
         if (event.getCode() == KeyCode.F5)
            nodeToRegionId.keySet().stream().forEach(node -> node.setVisible(true));
      });

      primaryStage.setScene(scene);
      primaryStage.show();
   }

   private static Node createRegionGraphics(PlanarRegionSegmentationRawData rawData)
   {
      Group regionGroup = new Group();
      ObservableList<Node> children = regionGroup.getChildren();

      QuadEdgeSubdivision quadEdgeSubdivision = createQuadEdgeSubdivision(rawData);
      if (VISUALIZE_EDGES)
         children.add(createEdgesGraphics(quadEdgeSubdivision, rawData));
      if (VISUALIZE_PRIMARY_EDGES)
         children.add(createPrimaryEdgesGraphics(quadEdgeSubdivision, rawData));
      if (VISUALIZE_ORDERED_BORDER_EDGES)
         children.add(createOrderedBorderEdgesGraphics(quadEdgeSubdivision, rawData));
      return regionGroup;
   }

   private static QuadEdgeSubdivision createQuadEdgeSubdivision(PlanarRegionSegmentationRawData rawData)
   {
      List<Point2D> point2ds = rawData.getPointCloudInPlane();
      MultiPoint multiPoint = SimpleConcaveHullFactory.createMultiPoint(point2ds);

      ConformingDelaunayTriangulationBuilder conformingDelaunayTriangulationBuilder = new ConformingDelaunayTriangulationBuilder();
      conformingDelaunayTriangulationBuilder.setSites(multiPoint);
      return conformingDelaunayTriangulationBuilder.getSubdivision();
   }

   @SuppressWarnings("unchecked")
   private static Node createEdgesGraphics(QuadEdgeSubdivision quadEdgeSubdivision, PlanarRegionSegmentationRawData rawData)
   {
      List<QuadEdge> edges = (List<QuadEdge>) quadEdgeSubdivision.getEdges();

      int regionId = rawData.getRegionId();
      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(16));
      Point3D planeOrigin = rawData.getOrigin();
      Quaternion planeOrientation = rawData.getOrientation();
      Color regionColor = OcTreeMeshBuilder.getRegionColor(regionId);

      for (QuadEdge edge : edges)
      {
         Point3D dest = PolygonizerTools.toPointInWorld(edge.dest().getX(), edge.dest().getY(), planeOrigin, planeOrientation);
         Point3D orig = PolygonizerTools.toPointInWorld(edge.orig().getX(), edge.orig().getY(), planeOrigin, planeOrientation);
         meshBuilder.addLine(dest, orig, 0.0015, regionColor);
      }
      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(meshBuilder.generateMaterial());
      meshView.setMouseTransparent(true);
      return meshView;
   }

   @SuppressWarnings("unchecked")
   private static Node createPrimaryEdgesGraphics(QuadEdgeSubdivision quadEdgeSubdivision, PlanarRegionSegmentationRawData rawData)
   {
      List<QuadEdge> primaryEdges = (List<QuadEdge>) quadEdgeSubdivision.getPrimaryEdges(false);

      int regionId = rawData.getRegionId();
      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(16));
      Point3D planeOrigin = rawData.getOrigin();
      Quaternion planeOrientation = rawData.getOrientation();
      Color regionColor = OcTreeMeshBuilder.getRegionColor(regionId);

      for (QuadEdge edge : primaryEdges)
      {
         Point3D dest = PolygonizerTools.toPointInWorld(edge.dest().getX(), edge.dest().getY(), planeOrigin, planeOrientation);
         Point3D orig = PolygonizerTools.toPointInWorld(edge.orig().getX(), edge.orig().getY(), planeOrigin, planeOrientation);
         meshBuilder.addLine(dest, orig, 0.0015, regionColor);
      }
      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(meshBuilder.generateMaterial());
      meshView.setMouseTransparent(true);
      return meshView;
   }

   @SuppressWarnings("unchecked")
   private static Node createOrderedBorderEdgesGraphics(QuadEdgeSubdivision quadEdgeSubdivision, PlanarRegionSegmentationRawData rawData)
   {
      List<QuadEdgeTriangle> delaunayTriangles = QuadEdgeTriangle.createOn(quadEdgeSubdivision);
      List<QuadEdge> orderedBorderEdges = SimpleConcaveHullFactory.computeIntermediateVariables(delaunayTriangles, null).getOrderedBorderEdges();

      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(16));
      Point3D planeOrigin = rawData.getOrigin();
      Quaternion planeOrientation = rawData.getOrientation();

      double startHue = 0.0;
      double endHue = 240.0;

      double lineStartBirghtness = 0.2;
      double lineEndBirghtness = 1.0;

      for (int edgeIndex = 0; edgeIndex < orderedBorderEdges.size(); edgeIndex++)
      {
         QuadEdge edge = orderedBorderEdges.get(edgeIndex);
         Point3D dest = PolygonizerTools.toPointInWorld(edge.dest().getX(), edge.dest().getY(), planeOrigin, planeOrientation);
         Point3D orig = PolygonizerTools.toPointInWorld(edge.orig().getX(), edge.orig().getY(), planeOrigin, planeOrientation);
         double alpha = edgeIndex / (double) orderedBorderEdges.size();
         double lineHue = (1.0 - alpha) * startHue + alpha * endHue;
         Color startColor = Color.hsb(lineHue, 0.9, lineStartBirghtness);
         Color endColor = Color.hsb(lineHue, 0.9, lineEndBirghtness);
         meshBuilder.addLine(orig, dest, 0.002, startColor, endColor);
      }
      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(meshBuilder.generateMaterial());
      return meshView;
   }

   public static void main(String[] args)
   {
      launch(args);
   }
}
