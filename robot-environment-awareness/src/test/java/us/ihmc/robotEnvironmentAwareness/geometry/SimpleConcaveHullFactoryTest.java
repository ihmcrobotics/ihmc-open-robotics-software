package us.ihmc.robotEnvironmentAwareness.geometry;

import static org.junit.jupiter.api.Assertions.assertTimeout;
import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertNull;
import static us.ihmc.robotics.Assert.assertTrue;

import java.time.Duration;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.Set;
import java.util.concurrent.atomic.AtomicReference;
import java.util.stream.Collectors;
import java.util.stream.Stream;

import org.apache.commons.lang3.mutable.MutableBoolean;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import com.sun.javafx.application.PlatformImpl;

import javafx.stage.Stage;
import javafx.stage.WindowEvent;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.Line3D;
import us.ihmc.euclid.geometry.LineSegment3D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.interfaces.Point3DBasics;
import us.ihmc.messager.Messager;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.messager.javafx.SharedMemoryJavaFXMessager;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.polygonizer.Polygonizer;
import us.ihmc.robotEnvironmentAwareness.polygonizer.Polygonizer.Output;
import us.ihmc.robotEnvironmentAwareness.polygonizer.PolygonizerManager;
import us.ihmc.robotEnvironmentAwareness.polygonizer.PolygonizerVisualizerUI;

public class SimpleConcaveHullFactoryTest
{
   private static boolean VISUALIZE = false;

   private Messager messager;
   private MutableBoolean uiIsGoingDown = new MutableBoolean(false);

   @BeforeEach
   public void setup() throws Exception
   {
      uiIsGoingDown.setFalse();

      if (VISUALIZE)
      {
         SharedMemoryJavaFXMessager jfxMessager = new SharedMemoryJavaFXMessager(PolygonizerVisualizerUI.getMessagerAPI());
         messager = jfxMessager;
         createVisualizer(jfxMessager);
      }
      else
      {
         messager = new SharedMemoryMessager(PolygonizerVisualizerUI.getMessagerAPI());
         messager.startMessager();
         new PolygonizerManager(messager);
      }
   }

   private void createVisualizer(JavaFXMessager messager)
   {
      AtomicReference<PolygonizerVisualizerUI> ui = new AtomicReference<>(null);

      PlatformImpl.startup(() ->
      {
         try
         {
            Stage primaryStage = new Stage();
            primaryStage.addEventHandler(WindowEvent.WINDOW_CLOSE_REQUEST, event -> uiIsGoingDown.setTrue());

            ui.set(new PolygonizerVisualizerUI(messager, primaryStage));
            ui.get().show();
         }
         catch (Exception e)
         {
            e.printStackTrace();
         }
      });

      while (ui.get() == null)
         ThreadTools.sleep(200);
   }

   @AfterEach
   public void tearDown()
   {
      if (VISUALIZE)
      {
         while (!uiIsGoingDown.booleanValue())
            ThreadTools.sleep(100);
      }
   }

   @Test
   public void testNullPointerExceptionBug20191213()
   {
      List<Point3D> pointcloud = new ArrayList<>();
      pointcloud.add(new Point3D(5.714, 0.921, 0.0));
      pointcloud.add(new Point3D(5.553, 0.922, 0.0));
      pointcloud.add(new Point3D(5.395, 0.924, 0.0));
      pointcloud.add(new Point3D(5.239, 0.925, 0.0));
      pointcloud.add(new Point3D(5.086, 0.926, 0.0));
      pointcloud.add(new Point3D(4.935, 0.928, 0.0));
      pointcloud.add(new Point3D(4.786, 0.929, 0.0));
      pointcloud.add(new Point3D(4.639, 0.930, 0.0));
      pointcloud.add(new Point3D(4.493, 0.931, 0.0));
      pointcloud.add(new Point3D(4.350, 0.932, 0.0));

      PlanarRegionSegmentationRawData data = new PlanarRegionSegmentationRawData(1, Axis3D.Z, new Point3D(), pointcloud);
      ConcaveHullFactoryParameters parameters = new ConcaveHullFactoryParameters();
      messager.submitMessage(Polygonizer.PolygonizerParameters, parameters);

      AtomicReference<List<Output>> output = messager.createInput(Polygonizer.PolygonizerOutput, null);
      messager.submitMessage(PolygonizerManager.PlanarRegionSemgentationData, Collections.singletonList(data));

      assertTimeout(Duration.ofSeconds(10), () ->
      {
         while (output.get() == null)
            ThreadTools.sleep(100);
      });

      assertEquals(1, output.get().size());
      assertNull(output.get().get(0).getConcaveHullFactoryResult());
   }

   @Test
   public void testSimplePointcloudFormingASquare()
   {
      List<Point2D> expectedHull = new ArrayList<>();
      List<Point3D> pointcloud = new ArrayList<>();

      double xOffset = 0.4;
      double yOffset = 0.0;

      double size = 0.1;
      double density = 0.005;
      double sizeN = size / density;

      for (int i = 0; i < sizeN; i++)
      {
         for (int j = 0; j < sizeN; j++)
         {
            double x = i * density + xOffset;
            double y = j * density + yOffset;
            pointcloud.add(new Point3D(x, y, 0.0));

            if (i == 0 || i == sizeN - 1 || j == 0 || j == sizeN - 1)
               expectedHull.add(new Point2D(x, y));
         }
      }

      PlanarRegionSegmentationRawData data = new PlanarRegionSegmentationRawData(1, Axis3D.Z, new Point3D(), pointcloud);
      ConcaveHullFactoryParameters parameters = new ConcaveHullFactoryParameters();
      parameters.setRemoveAllTrianglesWithTwoBorderEdges(false);
      parameters.setTriangulationTolerance(1.0e-3);
      parameters.setEdgeLengthThreshold(1.1 * density);
      messager.submitMessage(Polygonizer.PolygonizerParameters, parameters);

      AtomicReference<List<Output>> output = messager.createInput(Polygonizer.PolygonizerOutput, null);
      messager.submitMessage(PolygonizerManager.PlanarRegionSemgentationData, Collections.singletonList(data));

      while (output.get() == null)
         ThreadTools.sleep(100);

      assertEquals(1, output.get().size());
      ConcaveHullCollection concaveHullCollection = output.get().get(0).getConcaveHullFactoryResult().getConcaveHullCollection();

      assertEquals(1, concaveHullCollection.getNumberOfConcaveHulls());
      ConcaveHull concaveHull = new ArrayList<>(concaveHullCollection.getConcaveHulls()).get(0);

      ConvexPolygon2D convexPolygon = new ConvexPolygon2D(Vertex3DSupplier.asVertex3DSupplier(pointcloud));

      for (int i = 0; i < concaveHull.getNumberOfVertices(); i++)
      {
         assertEquals(0.0, convexPolygon.signedDistance(concaveHull.getVertex(i)), 1.0e-7);
      }

      for (Point2D vertex : concaveHull)
      {
         assertTrue(expectedHull.contains(vertex));
      }
   }

   @Test
   public void testRandomCircleBasedConvexPointCloud()
   {
      Random random = new Random(5435);
      List<Point2D> expectedHull = EuclidGeometryRandomTools.nextCircleBasedConvexPolygon2D(random, 0.0, 0.08, 100);

      List<Point3D> pointcloud = new ArrayList<>();
      expectedHull.stream().map(Point3D::new).forEach(pointcloud::add);
      int numberOfPoints = 2000;
      for (int i = 0; i < numberOfPoints; i++)
      {
         pointcloud.add(new Point3D(nextPointInPointCloudHull(random, expectedHull)));
      }

      PlanarRegionSegmentationRawData data = new PlanarRegionSegmentationRawData(1, Axis3D.Z, new Point3D(), pointcloud);
      ConcaveHullFactoryParameters parameters = new ConcaveHullFactoryParameters();
      parameters.setTriangulationTolerance(1.0e-5);
      parameters.setEdgeLengthThreshold(0.15);
      messager.submitMessage(Polygonizer.PolygonizerParameters, parameters);

      AtomicReference<List<Output>> output = messager.createInput(Polygonizer.PolygonizerOutput, null);
      messager.submitMessage(PolygonizerManager.PlanarRegionSemgentationData, Collections.singletonList(data));

      while (output.get() == null)
         ThreadTools.sleep(100);

      ConcaveHullCollection concaveHullCollection = output.get().get(0).getConcaveHullFactoryResult().getConcaveHullCollection();

      assertEquals(1, concaveHullCollection.getNumberOfConcaveHulls());
      ConcaveHull concaveHull = new ArrayList<>(concaveHullCollection.getConcaveHulls()).get(0);

      for (Point2D vertex : concaveHull)
      {
         assertTrue(expectedHull.contains(vertex));
      }
   }

   @Test
   public void testPointCloudWithSurroundingLineConstraints()
   {
      List<Point3D> pointcloud = new ArrayList<>();
      pointcloud.add(new Point3D(0.5, -0.1, 0.0));
      pointcloud.add(new Point3D(0.5, 0.1, 0.0));
      pointcloud.add(new Point3D(1.5, 0.1, 0.0));
      pointcloud.add(new Point3D(1.5, -0.1, 0.0));

      List<LineSegment3D> lineConstraints = new ArrayList<>();
      lineConstraints.add(new LineSegment3D(0.0, -0.5, 0.0, 0.0, 0.5, 0.0));
      lineConstraints.add(new LineSegment3D(2.0, -0.5, 0.0, 2.0, 0.5, 0.0));
      lineConstraints.add(new LineSegment3D(0.0, 0.5, 0.0, 2.0, 0.5, 0.0));
      lineConstraints.add(new LineSegment3D(0.0, -0.5, 0.0, 2.0, -0.5, 0.0));

      PlanarRegionSegmentationRawData data = new PlanarRegionSegmentationRawData(1, Axis3D.Z, new Point3D(), pointcloud);
      data.addIntersections(lineConstraints);

      ConcaveHullFactoryParameters parameters = new ConcaveHullFactoryParameters();
      parameters.setTriangulationTolerance(1.0e-5);
      parameters.setEdgeLengthThreshold(0.15);
      messager.submitMessage(Polygonizer.PolygonizerParameters, parameters);

      AtomicReference<List<Output>> output = messager.createInput(Polygonizer.PolygonizerOutput, null);
      messager.submitMessage(PolygonizerManager.PlanarRegionSemgentationData, Collections.singletonList(data));

      while (output.get() == null)
         ThreadTools.sleep(100);

      ConcaveHullCollection concaveHullCollection = output.get().get(0).getConcaveHullFactoryResult().getConcaveHullCollection();
      assertEquals(1, concaveHullCollection.getNumberOfConcaveHulls());
   }

   @Test
   public void testSomeLineConstraints()
   {
      List<Point3D> pointcloud = new ArrayList<>();

      double xOffset = 0.4;
      double yOffset = 0.0;

      double size = 0.1;
      double density = 0.005;

      for (int i = 0; i < size / density; i++)
      {
         for (int j = 0; j < size / density; j++)
         {
            double x = i * density + xOffset;
            double y = j * density + yOffset;
            pointcloud.add(new Point3D(x, y, 0.0));
         }
      }

      Random random = new Random(34543);
      PlanarRegionSegmentationRawData data = new PlanarRegionSegmentationRawData(random.nextInt(), Axis3D.Z, new Point3D(), pointcloud);

      List<LineSegment3D> polygon1 = Arrays.asList(new LineSegment3D(0.25, -0.025, 0, 0.25, -0.20, 0),
                                                   new LineSegment3D(0.35, -0.025, 0, 0.35, -0.20, 0),
                                                   new LineSegment3D(0.25, -0.025, 0, 0.35, -0.025, 0),
                                                   new LineSegment3D(0.25, -0.20, 0, 0.35, -0.20, 0));
      polygon1.forEach(segment -> segment.translate(0.09, 0.00, 0.0));

      data.addIntersections(polygon1);

      AtomicReference<List<Output>> output = messager.createInput(Polygonizer.PolygonizerOutput, null);
      messager.submitMessage(PolygonizerManager.PlanarRegionSemgentationData, Collections.singletonList(data));

      while (output.get() == null)
         ThreadTools.sleep(100);

      ConcaveHullCollection concaveHullCollection = output.get().get(0).getConcaveHullFactoryResult().getConcaveHullCollection();
      assertEquals(1, concaveHullCollection.getNumberOfConcaveHulls());
   }

   @Test
   public void testOverlappingLineConstraints()
   {
      Random random = new Random(34543);
      List<Point3D> pointcloud = new ArrayList<>();

      double xOffset = 0.4;
      double yOffset = 0.0;

      double size = 0.1;
      double density = 0.005;

      for (int i = 0; i < size / density; i++)
      {
         for (int j = 0; j < size / density; j++)
         {
            double x = i * density + xOffset + 0.0001 * random.nextDouble();
            double y = j * density + yOffset + 0.0001 * random.nextDouble();
            pointcloud.add(new Point3D(x, y, 0.0));
         }
      }

      PlanarRegionSegmentationRawData data = new PlanarRegionSegmentationRawData(random.nextInt(), Axis3D.Z, new Point3D(), pointcloud);

      List<LineSegment3D> polygon1 = Arrays.asList(new LineSegment3D(0.25, -0.025, 0, 0.25, -0.20, 0),
                                                   new LineSegment3D(0.35, -0.025, 0, 0.35, -0.20, 0),
                                                   new LineSegment3D(0.25, -0.025, 0, 0.35, -0.025, 0),
                                                   new LineSegment3D(0.25, -0.20, 0, 0.35, -0.20, 0));

      List<LineSegment3D> polygon2 = polygon1.stream().map(LineSegment3D::new).peek(segment -> segment.translate(0.09, 0.005, 0.0))
                                             .collect(Collectors.toList());

      data.addIntersections(polygon1);
      data.addIntersections(polygon2);

      AtomicReference<List<Output>> output = messager.createInput(Polygonizer.PolygonizerOutput, null);
      messager.submitMessage(PolygonizerManager.PlanarRegionSemgentationData, Collections.singletonList(data));

      while (output.get() == null)
         ThreadTools.sleep(100);

      ConcaveHullCollection concaveHullCollection = output.get().get(0).getConcaveHullFactoryResult().getConcaveHullCollection();
      assertEquals(1, concaveHullCollection.getNumberOfConcaveHulls());

      List<LineSegment3D> extractedConstraintEdges = JTSTools.extractConstraintEdges(output.get().get(0).getConcaveHullFactoryResult(),
                                                                                     new RigidBodyTransform());
      Set<Point3DBasics> constraintVertices = extractedConstraintEdges.stream().flatMap(edge -> Stream.of(edge.getFirstEndpoint(), edge.getSecondEndpoint()))
                                                                      .collect(Collectors.toSet());

      assertTrue(data.getIntersectionsInWorld().stream().flatMap(edge -> Stream.of(edge.getFirstEndpoint(), edge.getSecondEndpoint()))
                     .allMatch(constraintVertices::contains));

      for (LineSegment3D edge : data.getIntersectionsInWorld())
      {
         assertTrue(extractedConstraintEdges.stream().anyMatch(extractedEdge -> isExtractedEdgeAMatch(edge, extractedEdge)));
      }
   }

   private boolean isExtractedEdgeAMatch(LineSegment3D edge, LineSegment3D extractedEdge)
   {
      boolean foundMatchingEdge;
      boolean areCollinear = new Line3D(edge).isCollinear(new Line3D(extractedEdge), 1.0e-5);
      double distance = edge.distance(extractedEdge);
      foundMatchingEdge = areCollinear && MathTools.epsilonEquals(0.0, distance, 1.0e-12);
      return foundMatchingEdge;
   }

   public static Point2D nextPointInPointCloudHull(Random random, List<? extends Point2DReadOnly> points)
   {
      ConvexPolygon2D convexPolygon2D = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(points));
      double theta = EuclidCoreRandomTools.nextDouble(random, 0.0, 2.0 * Math.PI);

      Vector2D direction = new Vector2D(Math.cos(theta), Math.sin(theta));
      Point2DBasics[] intersectionWithRay = convexPolygon2D.intersectionWithRay(new Line2D(convexPolygon2D.getCentroid(), direction));
      Point2D next = new Point2D();
      double alpha = Math.sqrt(EuclidCoreRandomTools.nextDouble(random, 0.0, 1.0)); // Using the square root tends make the generated point cloud closer to being uniformly distributed.
      next.interpolate(convexPolygon2D.getCentroid(), intersectionWithRay[0], alpha);
      return next;
   }
}
