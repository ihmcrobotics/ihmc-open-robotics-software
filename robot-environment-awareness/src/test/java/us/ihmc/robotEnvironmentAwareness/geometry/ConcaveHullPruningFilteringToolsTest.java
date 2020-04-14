package us.ihmc.robotEnvironmentAwareness.geometry;

import static us.ihmc.robotics.Assert.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Random;
import java.util.concurrent.atomic.AtomicReference;

import org.apache.commons.lang3.mutable.MutableBoolean;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import javafx.stage.WindowEvent;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tools.EuclidCoreRandomTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DBasics;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.javaFXToolkit.messager.JavaFXMessager;
import us.ihmc.javaFXToolkit.messager.SharedMemoryJavaFXMessager;
import us.ihmc.javaFXToolkit.starter.ApplicationRunner;
import us.ihmc.messager.Messager;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.polygonizer.Polygonizer;
import us.ihmc.robotEnvironmentAwareness.polygonizer.Polygonizer.Output;
import us.ihmc.robotEnvironmentAwareness.polygonizer.PolygonizerManager;
import us.ihmc.robotEnvironmentAwareness.polygonizer.PolygonizerVisualizerUI;

public class ConcaveHullPruningFilteringToolsTest
{
   private static boolean VISUALIZE = true;

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

      ApplicationRunner.runApplication(primaryStage ->
      {
         try
         {
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
   public void testSimplePointcloudFormingASquare()
   {
      List<Point2D> expectedHull = new ArrayList<>();

      double xOffset = 0.4;
      double yOffset = 0.0;
      double size = 0.1;
      double density = 0.005;

      List<Point3D> pointcloud = newSquarePointcloud(expectedHull, xOffset, yOffset, size, density);

      PlanarRegionSegmentationRawData data = new PlanarRegionSegmentationRawData(1, Axis3D.Z, new Point3D(), pointcloud);
      ConcaveHullFactoryParameters parameters = new ConcaveHullFactoryParameters();
      parameters.setRemoveAllTrianglesWithTwoBorderEdges(false);
      parameters.setTriangulationTolerance(1.0e-3);
      parameters.setEdgeLengthThreshold(1.1 * density);
      messager.submitMessage(Polygonizer.PolygonizerParameters, parameters);
      messager.submitMessage(Polygonizer.PolygonizerPostProcessor, collection -> ConcaveHullPruningFilteringTools.filterOutNarrowRegions(0.05, collection));

      AtomicReference<List<Output>> output = messager.createInput(Polygonizer.PolygonizerOutput, null);
      messager.submitMessage(PolygonizerManager.PlanarRegionSemgentationData, Collections.singletonList(data));

      while (output.get() == null)
         ThreadTools.sleep(100);

      assertEquals(1, output.get().size());
      ConcaveHullCollection concaveHullCollection = output.get().get(0).getProcessedConcaveHullCollection();

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
      messager.submitMessage(Polygonizer.PolygonizerPostProcessor, collection -> ConcaveHullPruningFilteringTools.filterOutNarrowRegions(0.05, collection));

      AtomicReference<List<Output>> output = messager.createInput(Polygonizer.PolygonizerOutput, null);
      messager.submitMessage(PolygonizerManager.PlanarRegionSemgentationData, Collections.singletonList(data));

      while (output.get() == null)
         ThreadTools.sleep(100);

      ConcaveHullCollection concaveHullCollection = output.get().get(0).getProcessedConcaveHullCollection();

      assertEquals(1, concaveHullCollection.getNumberOfConcaveHulls());
      ConcaveHull concaveHull = new ArrayList<>(concaveHullCollection.getConcaveHulls()).get(0);

      for (Point2D vertex : concaveHull)
      {
         assertTrue(expectedHull.contains(vertex));
      }
   }

   @Test
   public void testTwoSquaresAlmostTouchingAtCorner()
   {
      double size = 0.3;
      double density = 0.005;
      double offsetX = -0.005;
      Point2D position = new Point2D(0.33, 0.33);

      Point2D positionA = new Point2D(position);
      positionA.add(0.15 + offsetX, 0.15);
      List<Point2D> expectedHullA = new ArrayList<>();
      List<Point3D> pointcloudA = newSquarePointcloud(expectedHullA, positionA, size, density);

      Point2D positionB = new Point2D(position);
      positionB.add(-0.15 - offsetX, -0.15);
      List<Point2D> expectedHullB = new ArrayList<>();
      List<Point3D> pointcloudB = newSquarePointcloud(expectedHullB, positionB, size, density);

      List<Point3D> pointcloud = new ArrayList<>();
      pointcloud.addAll(pointcloudA);
      pointcloud.addAll(pointcloudB);
      Collections.shuffle(pointcloud);

      PlanarRegionSegmentationRawData data = new PlanarRegionSegmentationRawData(1, Axis3D.Z, new Point3D(), pointcloud);
      ConcaveHullFactoryParameters parameters = new ConcaveHullFactoryParameters();
      parameters.setRemoveAllTrianglesWithTwoBorderEdges(false);
      parameters.setTriangulationTolerance(1.0e-3);
      parameters.setEdgeLengthThreshold(1.1 * density);
      messager.submitMessage(Polygonizer.PolygonizerParameters, parameters);
      messager.submitMessage(Polygonizer.PolygonizerPostProcessor, collection ->
      {
         ConcaveHullCollection filtered = new ConcaveHullCollection(collection);

         ConcaveHullPruningFilteringTools.filterOutPeaksAndShallowAngles(Math.toRadians(1.0), Math.toRadians(170.0), filtered);

         return ConcaveHullPruningFilteringTools.filterOutNarrowRegions(0.01, filtered);
      });

      AtomicReference<List<Output>> outputReference = messager.createInput(Polygonizer.PolygonizerOutput, null);
      messager.submitMessage(PolygonizerManager.PlanarRegionSemgentationData, Collections.singletonList(data));

      while (outputReference.get() == null)
         ThreadTools.sleep(100);

      assertEquals(1, outputReference.get().size());

      Output output = outputReference.get().get(0);
      ConcaveHullCollection concaveHullCollection = output.getProcessedConcaveHullCollection();

      assertEquals(1, output.getConcaveHullFactoryResult().getConcaveHullCollection().getNumberOfConcaveHulls());
      assertEquals(2, concaveHullCollection.getNumberOfConcaveHulls());

      for (ConcaveHull concaveHull : concaveHullCollection.getConcaveHulls())
      {
         assertEquals(5, concaveHull.getNumberOfVertices());

         Point2D average = EuclidGeometryTools.averagePoint2Ds(concaveHull.getConcaveHullVertices());

         List<Point3D> pointcloudToCompareAgainst;

         if (average.distance(positionA) < average.distance(positionB))
            pointcloudToCompareAgainst = pointcloudA;
         else
            pointcloudToCompareAgainst = pointcloudB;

         ConvexPolygon2D convexPolygon = new ConvexPolygon2D(Vertex3DSupplier.asVertex3DSupplier(pointcloudToCompareAgainst));

         for (int i = 0; i < concaveHull.getNumberOfVertices(); i++)
         {
            assertEquals(0.0, convexPolygon.signedDistance(concaveHull.getVertex(i)), 1.0e-7);
         }
      }

   }

   private List<Point3D> newSquarePointcloud(List<Point2D> expectedHull, Point2DReadOnly position, double size, double density)
   {
      return newSquarePointcloud(expectedHull, position.getX(), position.getY(), size, density);
   }

   private List<Point3D> newSquarePointcloud(List<Point2D> expectedHull, double xOffset, double yOffset, double size, double density)
   {
      List<Point3D> pointcloud = new ArrayList<>();
      int sizeN = (int) Math.round(size / density + 1.0);

      for (int i = 0; i < sizeN; i++)
      {
         for (int j = 0; j < sizeN; j++)
         {
            double x = i * density + xOffset - size / 2.0;
            double y = j * density + yOffset - size / 2.0;
            pointcloud.add(new Point3D(x, y, 0.0));

            if (expectedHull != null)
            {
               if (i == 0 || i == sizeN - 1 || j == 0 || j == sizeN - 1)
                  expectedHull.add(new Point2D(x, y));
            }
         }
      }
      return pointcloud;
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
