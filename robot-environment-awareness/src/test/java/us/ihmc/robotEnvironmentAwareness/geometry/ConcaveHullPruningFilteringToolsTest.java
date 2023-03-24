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
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.geometry.interfaces.Vertex3DSupplier;
import us.ihmc.euclid.geometry.tools.EuclidGeometryRandomTools;
import us.ihmc.euclid.geometry.tools.EuclidGeometryTools;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.javaFXToolkit.starter.ApplicationRunner;
import us.ihmc.messager.Messager;
import us.ihmc.messager.SharedMemoryMessager;
import us.ihmc.messager.javafx.JavaFXMessager;
import us.ihmc.messager.javafx.SharedMemoryJavaFXMessager;
import us.ihmc.robotEnvironmentAwareness.planarRegion.PlanarRegionSegmentationRawData;
import us.ihmc.robotEnvironmentAwareness.polygonizer.Polygonizer;
import us.ihmc.robotEnvironmentAwareness.polygonizer.Polygonizer.Output;
import us.ihmc.robotEnvironmentAwareness.polygonizer.PolygonizerManager;
import us.ihmc.robotEnvironmentAwareness.polygonizer.PolygonizerVisualizerUI;

public class ConcaveHullPruningFilteringToolsTest
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
   public void testNarrowPassageCutterWithSimplePointcloudFormingASquare()
   {
      /*
       * This test is helpful for verifying that the filter does not deteriorate the initial hull at
       * corners.
       */
      List<Point2D> expectedHull = new ArrayList<>();

      double xOffset = 0.33;
      double yOffset = 0.33;
      double size = 0.3;
      double density = 0.005;

      List<Point3D> pointcloud = newSquarePointcloud(expectedHull, xOffset, yOffset, size, density);

      PlanarRegionSegmentationRawData data = new PlanarRegionSegmentationRawData(1, Axis3D.Z, new Point3D(), pointcloud);
      ConcaveHullFactoryParameters parameters = new ConcaveHullFactoryParameters();
      parameters.setRemoveAllTrianglesWithTwoBorderEdges(false);
      parameters.setTriangulationTolerance(1.0e-3);
      parameters.setEdgeLengthThreshold(1.1 * density);
      messager.submitMessage(Polygonizer.PolygonizerParameters, parameters);
      messager.submitMessage(Polygonizer.PolygonizerPostProcessor,
                             collection -> ConcaveHullPruningFilteringTools.concaveHullNarrowPassageCutter(0.05, collection));

      AtomicReference<List<Output>> output = messager.createInput(Polygonizer.PolygonizerOutput, null);
      messager.submitMessage(PolygonizerManager.PlanarRegionSemgentationData, Collections.singletonList(data));

      while (output.get() == null)
         ThreadTools.sleep(100);

      assertEquals(1, output.get().size());
      ConcaveHullCollection concaveHullCollection = output.get().get(0).getProcessedConcaveHullCollection();

      assertEquals(1, concaveHullCollection.getNumberOfConcaveHulls());
      ConcaveHull concaveHull = new ArrayList<>(concaveHullCollection.getConcaveHulls()).get(0);

      ConvexPolygon2D convexPolygon = new ConvexPolygon2D(Vertex3DSupplier.asVertex3DSupplier(pointcloud));
      assertEquals(convexPolygon.getArea(), new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(concaveHull.getConcaveHullVertices())).getArea(), 1.0e-12);

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
   public void testNarrowPassageCutterWithRandomCircleBasedConvexPointCloud()
   {
      /*
       * Useful to verify that for circle-like region, the filter does not deteriorate it.
       */
      Random random = new Random(5435);
      List<Point2D> expectedHull = EuclidGeometryRandomTools.nextCircleBasedConvexPolygon2D(random, 0.0, 0.08, 100);

      List<Point3D> pointcloud = new ArrayList<>();
      expectedHull.stream().map(Point3D::new).forEach(pointcloud::add);
      int numberOfPoints = 10000;
      for (int i = 0; i < numberOfPoints; i++)
      {
         pointcloud.add(new Point3D(SimpleConcaveHullFactoryTest.nextPointInPointCloudHull(random, expectedHull)));
      }

      PlanarRegionSegmentationRawData data = new PlanarRegionSegmentationRawData(1, Axis3D.Z, new Point3D(), pointcloud);
      ConcaveHullFactoryParameters parameters = new ConcaveHullFactoryParameters();
      parameters.setTriangulationTolerance(1.0e-5);
      parameters.setEdgeLengthThreshold(0.15);
      messager.submitMessage(Polygonizer.PolygonizerParameters, parameters);
      messager.submitMessage(Polygonizer.PolygonizerPostProcessor,
                             collection -> ConcaveHullPruningFilteringTools.concaveHullNarrowPassageCutter(0.05, collection));

      AtomicReference<List<Output>> output = messager.createInput(Polygonizer.PolygonizerOutput, null);
      messager.submitMessage(PolygonizerManager.PlanarRegionSemgentationData, Collections.singletonList(data));

      while (output.get() == null)
         ThreadTools.sleep(100);

      ConcaveHullCollection processedConcaveHullCollection = output.get().get(0).getProcessedConcaveHullCollection();

      assertEquals(1, processedConcaveHullCollection.getNumberOfConcaveHulls());
      ConcaveHullCollection rawConcaveHullCollection = output.get().get(0).getConcaveHullFactoryResult().getConcaveHullCollection();
      assertEquals(rawConcaveHullCollection, processedConcaveHullCollection);

   }

   @Test
   public void testNarrowPassageCutterWithSingleNarrowSquare()
   {
      /*
       * Test that hull without concavities is not affected by the filter.
       */
      List<Point2D> expectedHull = new ArrayList<>();

      Point2D position = new Point2D();
      Vector2D size = new Vector2D(0.4, 0.08);
      double density = 0.01;

      List<Point3D> pointcloud = newSquarePointcloud(expectedHull, position, size, density);

      PlanarRegionSegmentationRawData data = new PlanarRegionSegmentationRawData(1, Axis3D.Z, new Point3D(), pointcloud);
      ConcaveHullFactoryParameters parameters = new ConcaveHullFactoryParameters();
      parameters.setRemoveAllTrianglesWithTwoBorderEdges(false);
      parameters.setTriangulationTolerance(1.0e-3);
      parameters.setEdgeLengthThreshold(1.1 * density);
      messager.submitMessage(Polygonizer.PolygonizerParameters, parameters);
      messager.submitMessage(Polygonizer.PolygonizerPostProcessor,
                             collection -> ConcaveHullPruningFilteringTools.concaveHullNarrowPassageCutter(0.05, collection));

      AtomicReference<List<Output>> output = messager.createInput(Polygonizer.PolygonizerOutput, null);
      messager.submitMessage(PolygonizerManager.PlanarRegionSemgentationData, Collections.singletonList(data));

      while (output.get() == null)
         ThreadTools.sleep(100);

      assertEquals(1, output.get().size());
      ConcaveHullCollection concaveHullCollection = output.get().get(0).getProcessedConcaveHullCollection();
      assertEquals(output.get().get(0).getConcaveHullFactoryResult().getConcaveHullCollection(), concaveHullCollection);
   }

   @Test
   public void testNarrowPassageCutterWithTwoSquaresAlmostTouchingAtCornerFromRawConcaveHull()
   {
      /*
       * Test for validating the main feature of the filter: split a concave hull into 2 when the hull
       * looks like it consists in 2 hulls connected with a narrow passage. The result should be the best
       * approximation possible of these 2 hulls.
       */
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
         return ConcaveHullPruningFilteringTools.concaveHullNarrowPassageCutter(0.01, collection);
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
         Point2D average = EuclidGeometryTools.averagePoint2Ds(concaveHull.getConcaveHullVertices());

         List<Point3D> pointcloudToCompareAgainst;

         if (average.distance(positionA) < average.distance(positionB))
            pointcloudToCompareAgainst = pointcloudA;
         else
            pointcloudToCompareAgainst = pointcloudB;

         ConvexPolygon2D convexPolygon = new ConvexPolygon2D(Vertex3DSupplier.asVertex3DSupplier(pointcloudToCompareAgainst));

         for (int i = 0; i < concaveHull.getNumberOfVertices(); i++)
         {
            Point2D vertex = concaveHull.getVertex(i);
            assertEquals(0.0, convexPolygon.signedDistance(vertex), 1.0e-7);
         }
      }
   }

   @Test
   public void testNarrowPassageCutterWithTwoSquaresAlmostTouchingAtCornerFromFilteredConcaveHull()
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

         return ConcaveHullPruningFilteringTools.concaveHullNarrowPassageCutter(0.01, filtered);
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

   private List<Point3D> newSquarePointcloud(List<Point2D> expectedHull, Point2DReadOnly position, Vector2DReadOnly size, double density)
   {
      return newSquarePointcloud(expectedHull, position.getX(), position.getY(), size.getX(), size.getY(), density);
   }

   private List<Point3D> newSquarePointcloud(List<Point2D> expectedHull, double xOffset, double yOffset, double size, double density)
   {
      return newSquarePointcloud(expectedHull, xOffset, yOffset, size, size, density);
   }

   private List<Point3D> newSquarePointcloud(List<Point2D> expectedHull, double xOffset, double yOffset, double xSize, double ySize, double density)
   {
      List<Point3D> pointcloud = new ArrayList<>();
      int xSizeN = (int) Math.round(xSize / density + 1.0);
      int ySizeN = (int) Math.round(ySize / density + 1.0);

      for (int xIndex = 0; xIndex < xSizeN; xIndex++)
      {
         for (int yIndex = 0; yIndex < ySizeN; yIndex++)
         {
            double x = xIndex * density + xOffset - xSize / 2.0;
            double y = yIndex * density + yOffset - ySize / 2.0;
            pointcloud.add(new Point3D(x, y, 0.0));

            if (expectedHull != null)
            {
               if (xIndex == 0 || xIndex == xSizeN - 1 || yIndex == 0 || yIndex == xSizeN - 1)
                  expectedHull.add(new Point2D(x, y));
            }
         }
      }
      return pointcloud;
   }
}
