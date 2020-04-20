package us.ihmc.robotEnvironmentAwareness.geometry;

import java.io.File;
import java.io.IOException;
import java.text.DecimalFormat;
import java.text.NumberFormat;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Map.Entry;
import java.util.PriorityQueue;
import java.util.Random;
import java.util.Set;
import java.util.function.Supplier;

import org.apache.commons.lang3.tuple.Pair;

import com.vividsolutions.jts.triangulate.quadedge.QuadEdge;
import com.vividsolutions.jts.triangulate.quadedge.QuadEdgeTriangle;
import com.vividsolutions.jts.triangulate.quadedge.Vertex;

import javafx.application.Application;
import javafx.application.Platform;
import javafx.beans.Observable;
import javafx.beans.property.BooleanProperty;
import javafx.beans.property.DoubleProperty;
import javafx.beans.property.IntegerProperty;
import javafx.beans.property.ObjectProperty;
import javafx.beans.property.SimpleBooleanProperty;
import javafx.beans.property.SimpleIntegerProperty;
import javafx.beans.property.SimpleObjectProperty;
import javafx.collections.ObservableList;
import javafx.event.EventHandler;
import javafx.scene.Group;
import javafx.scene.Node;
import javafx.scene.Scene;
import javafx.scene.control.Label;
import javafx.scene.control.TextField;
import javafx.scene.input.KeyCode;
import javafx.scene.input.KeyEvent;
import javafx.scene.input.MouseEvent;
import javafx.scene.layout.BorderPane;
import javafx.scene.layout.FlowPane;
import javafx.scene.layout.Pane;
import javafx.scene.paint.Color;
import javafx.scene.paint.PhongMaterial;
import javafx.scene.shape.MeshView;
import javafx.stage.Stage;
import us.ihmc.commons.lists.ListWrappingIndexTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.javaFXToolkit.cameraControllers.FocusBasedCameraMouseEventHandler;
import us.ihmc.javaFXToolkit.scenes.View3DFactory;
import us.ihmc.javaFXToolkit.shapes.JavaFXMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.JavaFXMultiColorMeshBuilder;
import us.ihmc.javaFXToolkit.shapes.TextureColorAdaptivePalette;
import us.ihmc.robotEnvironmentAwareness.geometry.SimpleConcaveHullFactory.ConcaveHullFactoryResult;
import us.ihmc.robotEnvironmentAwareness.geometry.SimpleConcaveHullFactory.ConcaveHullVariables;
import us.ihmc.robotEnvironmentAwareness.planarRegion.IntersectionEstimationParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionIntersectionCalculator;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerParameters;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PolygonizerTools;
import us.ihmc.robotEnvironmentAwareness.ui.graphicsBuilders.OcTreeMeshBuilder;
import us.ihmc.robotEnvironmentAwareness.ui.io.PlanarRegionSegmentationRawDataImporter;
import us.ihmc.robotics.linearAlgebra.PrincipalComponentAnalysis3D;

public class PolygonizerVisualizer extends Application
{
   private static final boolean VISUALIZE_POINT_CLOUD = true;
   private static final boolean VISUALIZE_DELAUNAY_TRIANGULATION = true;
   private static final boolean VISUALIZE_CONCAVE_HULL = false;
   private static final boolean VISUALIZE_BORDER_EDGES = false;
   private static final boolean VISUALIZE_BORDER_TRIANGLES = false;
   private static final boolean VISUALIZE_PRIORITY_QUEUE = false;
   private static final boolean VISUALIZE_CONVEX_DECOMPOSITION = false;
   private static final boolean VISUALIZE_BORDER_VERTICES = false;
   private static final boolean VISUALIZE_CONCAVE_POCKETS = false;
   private static final boolean VISUALIZE_ORDERED_BORDER_EDGES = true;

   private static final boolean FILTER_CONCAVE_HULLS = false;

   private static final int[] onlyRegionWithId = {};

   private File defaultFile = null; //new File("../../Data/Segmentation/20161210_184102_PlanarRegionSegmentation_Sim_CB");

   private final Random random = new Random(54645L);
   private final Supplier<Color> randomColorSupplier = () -> Color.hsb(360.0 * random.nextDouble(), 0.8 * random.nextDouble() + 0.1, 0.9);
   private final ConcaveHullFactoryParameters parameters = new ConcaveHullFactoryParameters();
   private final PolygonizerParameters polygonizerParameters = new PolygonizerParameters();
   private final IntersectionEstimationParameters intersectionParameters = new IntersectionEstimationParameters();

   private final Map<Integer, PlanarRegionSegmentationRawData> idToRawData = new HashMap<>();
   private final IntegerProperty currentRegionIdProperty = new SimpleIntegerProperty(this, "currentRegionId", -1);

   private final BooleanProperty showIntersections = new SimpleBooleanProperty(this, "showIntersections", true);
   private final BooleanProperty showConstraintEdges = new SimpleBooleanProperty(this, "showConstraintEdges", false);

   public PolygonizerVisualizer() throws IOException
   {
      parameters.setEdgeLengthThreshold(0.05);
      //      parameters.setAllowSplittingConcaveHull(false);
      //      parameters.setRemoveAllTrianglesWithTwoBorderEdges(false);
      //      parameters.setMaxNumberOfIterations(0);
   }

   @Override
   public void start(Stage primaryStage) throws Exception
   {
      primaryStage.setTitle(getClass().getSimpleName());

      PlanarRegionSegmentationRawDataImporter dataImporter;
      if (defaultFile != null)
         dataImporter = new PlanarRegionSegmentationRawDataImporter(defaultFile);
      else
         dataImporter = PlanarRegionSegmentationRawDataImporter.createImporterWithFileChooser(primaryStage);
      if (dataImporter == null)
         Platform.exit();
      dataImporter.loadPlanarRegionSegmentationData();
      List<PlanarRegionSegmentationRawData> regionsRawData = dataImporter.getPlanarRegionSegmentationRawData();
      regionsRawData.forEach(rawData -> idToRawData.put(rawData.getRegionId(), rawData));

      View3DFactory view3dFactory = View3DFactory.createSubscene();
      FocusBasedCameraMouseEventHandler cameraController = view3dFactory.addCameraController(true);
      view3dFactory.addWorldCoordinateSystem(0.3);

      Set<Integer> regionIdFilterSet = new HashSet<>();
      Arrays.stream(onlyRegionWithId).forEach(regionIdFilterSet::add);

      if (regionIdFilterSet.size() == 1 || regionsRawData.size() == 1)
      {
         PlanarRegionSegmentationRawData rawData;
         if (regionsRawData.size() == 1)
            rawData = regionsRawData.get(0);
         else
            rawData = regionsRawData.stream().filter(region -> regionIdFilterSet.contains(region.getRegionId())).findFirst().get();

         currentRegionIdProperty.set(rawData.getRegionId());
         RigidBodyTransform transform = rawData.getTransformFromLocalToWorld();
         transform.invert();

         Node regionGraphics = createRegionGraphics(rawData);
         REAGraphics3DTools.transformNode(regionGraphics, transform);
         view3dFactory.addNodeToView(regionGraphics);
      }
      else
      {
         Map<Node, Integer> nodeToRegionId = new HashMap<>();

         Point3D average = computeAverage(regionsRawData, regionIdFilterSet);
         average.negate();

         PlanarRegionIntersectionCalculator.computeIntersections(regionsRawData, intersectionParameters);

         for (PlanarRegionSegmentationRawData rawData : regionsRawData)
         {
            if (regionIdFilterSet.isEmpty() || regionIdFilterSet.contains(rawData.getRegionId()))
            {
               Node regionGraphics = createRegionGraphics(rawData);
               regionGraphics.setManaged(false);
               REAGraphics3DTools.translateNode(regionGraphics, average);
               nodeToRegionId.put(regionGraphics, rawData.getRegionId());
               view3dFactory.addNodeToView(regionGraphics);
            }
         }

         view3dFactory.addEventHandler(MouseEvent.MOUSE_PRESSED, new EventHandler<MouseEvent>()
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
                  currentRegionIdProperty.set(regionId);
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
         primaryStage.addEventHandler(KeyEvent.KEY_PRESSED, event -> {
            if (event.getCode() == KeyCode.I)
               showIntersections.set(!showIntersections.get());
         });
         primaryStage.addEventHandler(KeyEvent.KEY_PRESSED, event -> {
            if (event.getCode() == KeyCode.C)
               showConstraintEdges.set(!showConstraintEdges.get());
         });
      }

      BorderPane mainPane = new BorderPane();
      view3dFactory.bindSubSceneSizeToPaneSize(mainPane);
      mainPane.setCenter(view3dFactory.getSubScene());
      mainPane.setTop(setupStatisticViz(cameraController));

      primaryStage.setScene(new Scene(mainPane, 800, 400, true));
      primaryStage.show();
   }

   private Pane setupStatisticViz(FocusBasedCameraMouseEventHandler cameraController)
   {
      FlowPane dataPane = new FlowPane();

      DoubleProperty xProperty = cameraController.getTranslate().xProperty();
      DoubleProperty yProperty = cameraController.getTranslate().yProperty();
      DoubleProperty zProperty = cameraController.getTranslate().zProperty();

      Label xFocusLabel = new Label("x focus: ");
      Label yFocusLabel = new Label("y focus: ");
      Label zFocusLabel = new Label("z focus: ");

      TextField xFocus = new TextField("0");
      xFocus.setPrefWidth(50.0);
      TextField yFocus = new TextField("0");
      yFocus.setPrefWidth(50.0);
      TextField zFocus = new TextField("0");
      zFocus.setPrefWidth(50.0);

      xFocus.setEditable(false);
      yFocus.setEditable(false);
      zFocus.setEditable(false);

      NumberFormat formatter1 = new DecimalFormat("0.000;-0.000");

      xProperty.addListener((Observable o) -> xFocus.setText(formatter1.format(xProperty.getValue())));
      yProperty.addListener((Observable o) -> yFocus.setText(formatter1.format(yProperty.getValue())));
      zProperty.addListener((Observable o) -> zFocus.setText(formatter1.format(zProperty.getValue())));

      dataPane.getChildren().addAll(xFocusLabel, xFocus, yFocusLabel, yFocus, zFocusLabel, zFocus);

      ObjectProperty<Vector3D> standardDeviationProperty = new SimpleObjectProperty<>(this, "standardDeviation", new Vector3D());
      currentRegionIdProperty.addListener((Observable o) -> standardDeviationProperty.set(computePrincipalStandardDeviationValues(idToRawData.get(currentRegionIdProperty.get()))));

      Label regionIdLabel = new Label("current region ID: ");
      TextField regionId = new TextField("-1");
      regionId.setPrefWidth(100.0);
      regionId.setEditable(false);
      Label stdXLabel = new Label("standard dev. x: ");
      TextField stdx = new TextField("-1");
      stdx.setPrefWidth(75.0);
      stdx.setEditable(false);
      Label stdYLabel = new Label("y: ");
      TextField stdy = new TextField("-1");
      stdy.setPrefWidth(75.0);
      stdy.setEditable(false);
      Label stdZLabel = new Label("z: ");
      TextField stdz = new TextField("-1");
      stdz.setPrefWidth(75.0);
      stdz.setEditable(false);

      Label stdVolumeLabel = new Label("volume: ");
      TextField stdVolume = new TextField("-1");
      stdVolume.setPrefWidth(75.0);
      stdVolume.setEditable(false);
      Label stdDensityLabel = new Label("density: ");
      TextField stdDensity = new TextField("-1");
      stdDensity.setPrefWidth(75.0);
      stdDensity.setEditable(false);

      NumberFormat formatter2 = new DecimalFormat("0.###E0");

      currentRegionIdProperty.addListener((Observable o) -> regionId.setText(Integer.toString(currentRegionIdProperty.get())));
      standardDeviationProperty.addListener((Observable o) -> stdx.setText(formatter2.format(standardDeviationProperty.get().getX())));
      standardDeviationProperty.addListener((Observable o) -> stdy.setText(formatter2.format(standardDeviationProperty.get().getY())));
      standardDeviationProperty.addListener((Observable o) -> stdz.setText(formatter2.format(standardDeviationProperty.get().getZ())));
      standardDeviationProperty.addListener((Observable o) -> stdVolume.setText(formatter2.format(ellipsoidVolume(standardDeviationProperty.get()))));
      standardDeviationProperty.addListener((Observable o) -> stdDensity.setText(formatter2.format(idToRawData.get(currentRegionIdProperty.get()).size()
            / ellipsoidVolume(standardDeviationProperty.get()))));

      dataPane.getChildren().addAll(regionIdLabel, regionId, stdXLabel, stdx, stdYLabel, stdy, stdZLabel, stdz, stdVolumeLabel, stdVolume, stdDensityLabel,
                                    stdDensity);

      return dataPane;
   }

   public double ellipsoidVolume(Vector3D radii)
   {
      return ellipsoidVolume(radii.getX(), radii.getY(), radii.getZ());
   }

   public double ellipsoidVolume(double xRadius, double yRadius, double zRadius)
   {
      return 4.0 / 3.0 * Math.PI * xRadius * yRadius * zRadius * 100.0 * 100.0 * 100.0;
   }

   public static Vector3D computePrincipalStandardDeviationValues(PlanarRegionSegmentationRawData rawData)
   {
      PrincipalComponentAnalysis3D pca = new PrincipalComponentAnalysis3D();
      rawData.stream().forEach(point -> pca.addPoint(point.getX(), point.getY(), point.getZ()));
      pca.compute();
      Vector3D principalStandardDeviation = new Vector3D();
      pca.getStandardDeviation(principalStandardDeviation);
      return principalStandardDeviation;
   }

   public static Point3D computeAverage(List<PlanarRegionSegmentationRawData> regionsRawData, Set<Integer> regionIdFilterSet)
   {
      PointMean average = new PointMean();

      for (PlanarRegionSegmentationRawData rawData : regionsRawData)
      {
         if (regionIdFilterSet.isEmpty() || regionIdFilterSet.contains(rawData.getRegionId()))
            rawData.stream().forEach(average::update);
      }

      return average;
   }

   private Node createRegionGraphics(PlanarRegionSegmentationRawData rawData)
   {
      Group regionGroup = new Group();
      ObservableList<Node> children = regionGroup.getChildren();

      List<Point2D> pointsInPlane = rawData.getPointCloudInPlane();
      List<LineSegment2D> intersections = rawData.getIntersectionsInPlane();
      ConcaveHullFactoryResult concaveHullFactoryResult = SimpleConcaveHullFactory.createConcaveHull(pointsInPlane, intersections, parameters);

      if (VISUALIZE_CONCAVE_HULL)
         children.add(createConcaveHullGraphics(rawData, concaveHullFactoryResult));
      if (VISUALIZE_POINT_CLOUD)
         children.add(createRegionPointCloudGraphics(rawData));
      if (VISUALIZE_BORDER_TRIANGLES)
         children.add(createBorderTrianglesGraphics(rawData, concaveHullFactoryResult));
      if (VISUALIZE_DELAUNAY_TRIANGULATION)
         children.add(createDelaunayTriangulationGraphics(rawData, concaveHullFactoryResult));
      if (VISUALIZE_BORDER_EDGES)
         children.add(createBorderEdgesGraphics(rawData, concaveHullFactoryResult));
      if (VISUALIZE_CONVEX_DECOMPOSITION)
         children.add(createConvexDecompositionGraphics(rawData, concaveHullFactoryResult));
      if (VISUALIZE_PRIORITY_QUEUE)
         children.add(createPriorityQueueGraphics(rawData, concaveHullFactoryResult));
      if (VISUALIZE_BORDER_VERTICES)
         children.add(createBorderVerticesGraphics(rawData, concaveHullFactoryResult));
      if (VISUALIZE_CONCAVE_POCKETS)
         children.add(createConcavePocketsGraphics(rawData, concaveHullFactoryResult));
      if (VISUALIZE_ORDERED_BORDER_EDGES)
         children.add(createOrderedBorderEdgesGraphics(rawData, concaveHullFactoryResult));
      children.add(createIntersectionsGraphics(rawData));
      children.add(createConstraintEdgesGraphics(rawData, concaveHullFactoryResult));

      return regionGroup;
   }

   private Node createConcavePocketsGraphics(PlanarRegionSegmentationRawData rawData, ConcaveHullFactoryResult concaveHullFactoryResult)
   {
      JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();
      ConcaveHullCollection concaveHullCollection = concaveHullFactoryResult.getConcaveHullCollection();
      RigidBodyTransform transform = rawData.getTransformFromLocalToWorld();

      for (ConcaveHull concaveHull : concaveHullCollection)
      {
         Set<ConcaveHullPocket> pockets = concaveHull.findConcaveHullPockets(polygonizerParameters.getDepthThreshold());

         for (ConcaveHullPocket pocket : pockets)
         {
            List<Point2D> pocketVertices = ListWrappingIndexTools.subListInclusive(pocket.getStartBridgeIndex(), pocket.getEndBridgeIndex(),
                                                                                   concaveHull.getConcaveHullVertices());
            Point2D average = new Point2D();
            average.interpolate(pocket.getStartBridgeVertex(), pocket.getEndBridgeVertex(), 0.5);
            pocketVertices.add(0, average);
            ConcaveHullTools.ensureClockwiseOrdering(pocketVertices);
            meshBuilder.addPolygon(transform, pocketVertices);
         }
      }

      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(new PhongMaterial(OcTreeMeshBuilder.getRegionColor(rawData.getRegionId())));
      return meshView;
   }

   private Node createBorderEdgesGraphics(PlanarRegionSegmentationRawData rawData, ConcaveHullFactoryResult concaveHullFactoryResult)
   {
      MeshView meshView = ConcaveHullFactoryGraphicsTools.borderEdgesToMultiLine(rawData.getTransformFromLocalToWorld(), concaveHullFactoryResult,
                                                                                 OcTreeMeshBuilder.getRegionColor(rawData.getRegionId()), 0.0015);
      meshView.setMouseTransparent(true);
      return meshView;
   }

   private Node createRegionPointCloudGraphics(PlanarRegionSegmentationRawData rawData)
   {
      JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();

      rawData.stream().forEach(point -> meshBuilder.addTetrahedron(0.0025, point));
      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(new PhongMaterial(OcTreeMeshBuilder.getRegionColor(rawData.getRegionId())));
      meshView.setMouseTransparent(true);
      return meshView;
   }

   private Node createBorderVerticesGraphics(PlanarRegionSegmentationRawData rawData, ConcaveHullFactoryResult concaveHullFactoryResult)
   {
      JavaFXMeshBuilder meshBuilder = new JavaFXMeshBuilder();
      Point3D planeOrigin = rawData.getOrigin();
      Quaternion planeOrientation = rawData.getOrientation();

      for (ConcaveHullVariables intermediateVariables : concaveHullFactoryResult.getIntermediateVariables())
      {
         for (Vertex vertex2d : intermediateVariables.getBorderVertices())
         {
            Point3D vertex3d = PolygonizerTools.toPointInWorld(vertex2d.getX(), vertex2d.getY(), planeOrigin, planeOrientation);
            meshBuilder.addSphere(0.003, vertex3d);
         }
      }
      MeshView meshView = new MeshView(meshBuilder.generateMesh());
      meshView.setMaterial(new PhongMaterial(OcTreeMeshBuilder.getRegionColor(rawData.getRegionId())));
      meshView.setMouseTransparent(true);
      return meshView;
   }

   private Node createConcaveHullGraphics(PlanarRegionSegmentationRawData rawData, ConcaveHullFactoryResult concaveHullFactoryResult)
   {
      ConcaveHullCollection concaveHullCollection = concaveHullFactoryResult.getConcaveHullCollection();

      for (ConcaveHull concaveHull : concaveHullCollection)
      {

         if (FILTER_CONCAVE_HULLS)
         {
            double shallowAngleThreshold = polygonizerParameters.getShallowAngleThreshold();
            double peakAngleThreshold = polygonizerParameters.getPeakAngleThreshold();
            double lengthThreshold = polygonizerParameters.getLengthThreshold();

            for (int i = 0; i < 5; i++)
            {
               ConcaveHullPruningFilteringTools.filterOutPeaksAndShallowAngles(shallowAngleThreshold, peakAngleThreshold, concaveHull);
               ConcaveHullPruningFilteringTools.filterOutShortEdges(lengthThreshold, concaveHull);
            }
         }
      }

      MeshView meshView = REAGraphics3DTools.multiLine(rawData.getTransformFromLocalToWorld(), concaveHullCollection,
                                                       OcTreeMeshBuilder.getRegionColor(rawData.getRegionId()), 0.0015);
      meshView.setMouseTransparent(true);
      return meshView;
   }

   private Node createDelaunayTriangulationGraphics(PlanarRegionSegmentationRawData rawData, ConcaveHullFactoryResult concaveHullFactoryResult)
   {
      List<Triangle3D> delaunayTriangles = JTSTools.extractAllTrianglesInWorld(concaveHullFactoryResult, rawData.getTransformFromLocalToWorld());
      return REAGraphics3DTools.triangles(delaunayTriangles, t -> randomColorSupplier.get());
   }

   private Node createBorderTrianglesGraphics(PlanarRegionSegmentationRawData rawData, ConcaveHullFactoryResult concaveHullFactoryResult)
   {
      List<Triangle3D> borderTriangles = JTSTools.extractBorderTrianglesInWorld(concaveHullFactoryResult, rawData.getTransformFromLocalToWorld());
      return REAGraphics3DTools.triangles(borderTriangles, t -> randomColorSupplier.get());
   }

   private Node createPriorityQueueGraphics(PlanarRegionSegmentationRawData rawData, ConcaveHullFactoryResult concaveHullFactoryResult)
   {
      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(512));

      Color regionColor = OcTreeMeshBuilder.getRegionColor(rawData.getRegionId());

      for (ConcaveHullVariables intermediateVariables : concaveHullFactoryResult.getIntermediateVariables())
      {
         PriorityQueue<Pair<QuadEdge, QuadEdgeTriangle>> queue = intermediateVariables.getSortedByLengthQueue();

         for (Pair<QuadEdge, QuadEdgeTriangle> edgeAndTriangle : queue)
         {
            LineSegment3D edge = JTSTools.quadEdgeToLineSegment3D(edgeAndTriangle.getLeft());
            edge.applyTransform(rawData.getTransformFromLocalToWorld());
            boolean isEdgeTooLong = edge.length() > parameters.getEdgeLengthThreshold();
            Color lineColor = Color.hsb(regionColor.getHue(), regionColor.getSaturation(), isEdgeTooLong ? 0.25 : regionColor.getBrightness());
            meshBuilder.addLine(edge.getFirstEndpoint(), edge.getSecondEndpoint(), 0.0015, lineColor);

            Triangle3D triangle = JTSTools.quadEdgeTriangleToTriangle(edgeAndTriangle.getRight());
            triangle.applyTransform(rawData.getTransformFromLocalToWorld());

            meshBuilder.addPolygon(Arrays.asList(triangle.getA(), triangle.getB(), triangle.getC()), randomColorSupplier.get());
         }
      }

      MeshView trianglesMeshView = new MeshView(meshBuilder.generateMesh());
      trianglesMeshView.setMaterial(meshBuilder.generateMaterial());
      return trianglesMeshView;
   }

   private Node createConvexDecompositionGraphics(PlanarRegionSegmentationRawData rawData, ConcaveHullFactoryResult concaveHullFactoryResult)
   {
      ConcaveHullCollection concaveHullCollection = concaveHullFactoryResult.getConcaveHullCollection();
      double depthThreshold = polygonizerParameters.getDepthThreshold();
      List<ConvexPolygon2D> convexPolygons = new ArrayList<>();
      ConcaveHullDecomposition.recursiveApproximateDecomposition(concaveHullCollection, depthThreshold, convexPolygons);

      JavaFXMultiColorMeshBuilder meshBuilder = new JavaFXMultiColorMeshBuilder(new TextureColorAdaptivePalette(64));

      int regionId = rawData.getRegionId();
      RigidBodyTransform rigidBodyTransform = rawData.getTransformFromLocalToWorld();
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

   private static Node createOrderedBorderEdgesGraphics(PlanarRegionSegmentationRawData rawData, ConcaveHullFactoryResult concaveHullFactoryResult)
   {
      RigidBodyTransform transformToWorld = rawData.getTransformFromLocalToWorld();
      MeshView meshView = ConcaveHullFactoryGraphicsTools.orderedBorderEdgesToRainbowMultiLine(transformToWorld, concaveHullFactoryResult, 0.002);
      meshView.setMouseTransparent(true);
      return meshView;
   }

   private Node createIntersectionsGraphics(PlanarRegionSegmentationRawData rawData)
   {
      MeshView meshView = REAGraphics3DTools.multiLine(rawData.getIntersectionsInWorld(), Color.RED, 0.00125);
      meshView.visibleProperty().bind(showIntersections);
      return meshView;
   }

   private Node createConstraintEdgesGraphics(PlanarRegionSegmentationRawData rawData, ConcaveHullFactoryResult concaveHullFactoryResult)
   {
      List<LineSegment3D> constraintEdges = JTSTools.extractConstraintEdges(concaveHullFactoryResult, rawData.getTransformFromLocalToWorld());
      MeshView meshView = REAGraphics3DTools.multiLine(constraintEdges, Color.BLACK, 0.002);
      meshView.visibleProperty().bind(showConstraintEdges);
      return meshView;
   }

   public static void main(String[] args)
   {
      Application.launch();
   }
}
