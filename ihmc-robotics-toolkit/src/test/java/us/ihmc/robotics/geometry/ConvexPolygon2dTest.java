package us.ihmc.robotics.geometry;

import static org.junit.Assert.*;

import java.awt.Color;
import java.util.ArrayList;
import java.util.Random;

import org.junit.Test;

import us.ihmc.commons.MutationTestFacilitator;
import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.Line2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.EuclidFrameRandomTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple2D.Vector2D;
import us.ihmc.euclid.tuple2D.interfaces.Point2DReadOnly;
import us.ihmc.euclid.tuple3D.Vector3D;

public class ConvexPolygon2dTest
{
   private static final boolean VERBOSE = false;

   private Random random = new Random(1176L);
   private static final boolean PLOT_RESULTS = false;
   private static final boolean WAIT_FOR_BUTTON_PUSH = false;
   private static final double epsilon = 1e-10;

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testConstructors()
   {
      ConvexPolygon2D defaultConstructor = new ConvexPolygon2D();
      assertEquals("Number of vertices should be zero", 0.0, defaultConstructor.getNumberOfVertices(), epsilon);
      assertTrue(defaultConstructor.isUpToDate());

      int numberOfVertices = 4;
      ArrayList<Point2D> verticesList = new ArrayList<Point2D>();
      verticesList.add(new Point2D(0.0, 0.0));
      verticesList.add(new Point2D(0.0, 1.0));
      verticesList.add(new Point2D(1.0, 0.0));
      verticesList.add(new Point2D(1.0, 1.0));

      ConvexPolygon2D listInt = new ConvexPolygon2D(verticesList, numberOfVertices);
      assertEquals("Number of vertices should be 4", 4.0, listInt.getNumberOfVertices(), epsilon);

      ConvexPolygon2D list = new ConvexPolygon2D(verticesList);
      assertEquals("Number of vertices should be 4", 4.0, list.getNumberOfVertices(), epsilon);

      double[][] verticesArray = {{0.0, 0.0}, {0.0, 1.0}, {1.0, 0.0}, {1.0, 1.0}};

      ConvexPolygon2D doubleInt = new ConvexPolygon2D(verticesArray, numberOfVertices);
      assertEquals("Number of vertices should be four", 4.0, doubleInt.getNumberOfVertices(), epsilon);
      assertTrue(doubleInt.isUpToDate());

      ConvexPolygon2D doubles = new ConvexPolygon2D(verticesArray);
      assertEquals("Number of vertices should be four", 4.0, doubles.getNumberOfVertices(), epsilon);
      assertTrue(doubles.isUpToDate());

      ConvexPolygon2D polygon = new ConvexPolygon2D(doubles);
      assertEquals("Number of vertices should be four", 4.0, polygon.getNumberOfVertices(), epsilon);
      assertTrue(polygon.isUpToDate());

      ConvexPolygon2D polygonPolygon = new ConvexPolygon2D(doubleInt, doubles);
      assertEquals("Number of vertices should be four", 4.0, polygonPolygon.getNumberOfVertices(), epsilon);
      assertTrue(polygonPolygon.isUpToDate());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testTranslate()
   {
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(new Point2D(0.0, 0.0));
      polygon.update();

      Vector2D translation1 = new Vector2D(-0.1, 0.0);
      polygon.translate(translation1);
      assertTrue(polygon.getVertex(0).epsilonEquals(translation1, epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testClear()
   {
      ArrayList<Point2D> verticesList = new ArrayList<Point2D>();
      verticesList.add(new Point2D(0.0, 0.0));
      verticesList.add(new Point2D(0.0, 1.0));
      verticesList.add(new Point2D(1.0, 0.0));
      verticesList.add(new Point2D(1.0, 1.0));

      ConvexPolygon2D list = new ConvexPolygon2D(verticesList);
      assertEquals("Number of vertices should be 4", 4.0, list.getNumberOfVertices(), epsilon);
      assertTrue(list.isUpToDate());
      list.clearAndUpdate();
      assertEquals("Number of vertices should be 0", 0.0, list.getNumberOfVertices(), epsilon);
      assertTrue(list.isUpToDate());
      list.clear();
      assertFalse(list.isUpToDate());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testSetAndUpdates()
   {
      ConvexPolygon2D doubleInt = new ConvexPolygon2D();
      int numberOfVertices = 4;
      double[][] verticesArray = {{0.0, 0.0}, {0.0, 1.0}, {1.0, 0.0}, {1.0, 1.0}};
      doubleInt.setAndUpdate(verticesArray, numberOfVertices);
      assertEquals("Number of vertices should be 4", 4.0, doubleInt.getNumberOfVertices(), epsilon);
      assertTrue(doubleInt.isUpToDate());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetCentroid()
   {
      double[][] verticesArray = {{0.0, 0.0}, {0.0, 1.0}, {1.0, 0.0}, {1.0, 1.0}};
      ConvexPolygon2D doubles = new ConvexPolygon2D(verticesArray);
      Point2D centroid = new Point2D();

      doubles.getCentroid(centroid);
      assertEquals("Centroids should be equal", centroid, doubles.getCentroid());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetBoundingBox()
   {
      double[][] verticesArray = {{0.0, 0.0}, {0.0, 1.0}, {1.0, 0.0}, {1.0, 1.0}};
      ConvexPolygon2D doubles = new ConvexPolygon2D(verticesArray);
      BoundingBox2D box = doubles.getBoundingBox();

      assertEquals("Bounding boxes should be equal", box.getMinPoint().getX(), 0.0, epsilon);
      assertEquals("Bounding boxes should be equal", box.getMinPoint().getX(), 0.0, epsilon);
      assertEquals("Bounding boxes should be equal", box.getMaxPoint().getY(), 1.0, epsilon);
      assertEquals("Bounding boxes should be equal", box.getMaxPoint().getY(), 1.0, epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetNextVertexCCWGetPreviousVertexCCW()
   {
      double[][] verticesArray = {{0.0, 0.0}, {0.0, 1.0}, {1.0, 0.0}, {1.0, 1.0}};
      ConvexPolygon2D doubles = new ConvexPolygon2D(verticesArray);

      Point2DReadOnly oneNext = doubles.getNextVertexCCW(0); //(1.0, 0.0)
      Point2DReadOnly twoNext = doubles.getNextVertexCCW(1); //(1.0, 1.0)
      Point2DReadOnly threeNext = doubles.getNextVertexCCW(2); //(0.0, 1.0)
      Point2DReadOnly fourNext = doubles.getNextVertexCCW(3); //(0.0, 0.0)

      Point2DReadOnly onePrev = doubles.getPreviousVertexCCW(0); //(0.0, 1.0)
      Point2DReadOnly twoPrev = doubles.getPreviousVertexCCW(1); //(0.0, 0.0)
      Point2DReadOnly threePrev = doubles.getPreviousVertexCCW(2); //(1.0, 0.0)
      Point2DReadOnly fourPrev = doubles.getPreviousVertexCCW(3); //(1.0, 1.0)

      assertEquals("Points should be equal", oneNext, threePrev);
      assertEquals("Points should be equal", twoNext, fourPrev);
      assertEquals("Points should be equal", threeNext, onePrev);
      assertEquals("Points should be equal", fourNext, twoPrev);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testScale()
   {
      double scaleFactor = 2.0;
      double[][] verticesArray = {{-1.0, 1.0}, {1.0, 1.0}, {1.0, -1.0}, {-1.0, -1.0}};

      ConvexPolygon2D polygon = new ConvexPolygon2D(verticesArray);
      polygon.scale(scaleFactor);

      Point2DReadOnly oneNext = polygon.getNextVertexCCW(0); //(2.0, -2.0)
      Point2DReadOnly twoNext = polygon.getNextVertexCCW(1); //(2.0, 2.0)
      Point2DReadOnly threeNext = polygon.getNextVertexCCW(2); //(-2.0, 2.0)
      Point2DReadOnly fourNext = polygon.getNextVertexCCW(3); //(-2.0, -2.0)

      Point2D one = new Point2D(2.0, -2.0);
      Point2D two = new Point2D(2.0, 2.0);
      Point2D three = new Point2D(-2.0, 2.0);
      Point2D four = new Point2D(-2.0, -2.0);

      assertEquals("These should be equal", oneNext, one);
      assertEquals("These should be equal", twoNext, two);
      assertEquals("These should be equal", threeNext, three);
      assertEquals("These should be equal", fourNext, four);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIsPointInside()
   {
      Random random = new Random(4564656L);
      double[][] verticesArray = {{-10.0, 10.0}, {10.0, 10.0}, {10.0, -10.0}, {-10.0, -10.0}};
      ConvexPolygon2D doubles = new ConvexPolygon2D(verticesArray);

      for (int i = 0; i < 10; i++)
      {
         int x = random.nextInt(10);
         int y = random.nextInt(10);
         assertTrue(doubles.isPointInside(x, y));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIntersectionWith()
   {
      double[][] verticesArray1 = {{-10.0, 10.0}, {10.0, 10.0}, {10.0, -10.0}, {-10.0, -10.0}};
      double[][] verticesArray2 = {{-5.0, 5.0}, {5.0, 5.0}, {5.0, -5.0}, {-5.0, -5.0}};
      double[][] verticesArray3 = {{15.0, 20.0}, {20.0, 20.0}, {20.0, 15.0}, {15.0, 15.0}};
      double[][] verticesArray4 = {{-5.0, -10.0}, {-5.0, 10.0}, {15.0, -10.0}, {15.0, 10.0}};

      ConvexPolygon2D polygon1 = new ConvexPolygon2D(verticesArray1);
      ConvexPolygon2D polygon2 = new ConvexPolygon2D(verticesArray2);
      ConvexPolygon2D polygon3 = new ConvexPolygon2D(verticesArray3);
      ConvexPolygon2D polygon4 = new ConvexPolygon2D(verticesArray4);

      ConvexPolygon2D noIntersect = new ConvexPolygon2D();
      ConvexPolygon2D allIntersect = new ConvexPolygon2D();
      ConvexPolygon2D someIntersect = new ConvexPolygon2D();

      assertTrue(ConvexPolygonTools.computeIntersectionOfPolygons(polygon1, polygon2, allIntersect));
      assertFalse("Should be false", ConvexPolygonTools.computeIntersectionOfPolygons(polygon1, polygon3, noIntersect));
      assertTrue(ConvexPolygonTools.computeIntersectionOfPolygons(polygon1, polygon4, someIntersect));
      //
      //      System.out.println("No" + noIntersect);
      //      System.out.println("Some" + someIntersect);
      //      System.out.println("all" + allIntersect);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testTriangleConstructor()
   {
      double[][] vertices = new double[][] {{0.0, 0.0}, {2.0, 0.0}, {1.0, 0.1}};

      ReferenceFrame zUpFrame = ReferenceFrame.constructARootFrame("someFrame");

      FrameConvexPolygon2d polygon = ConvexPolygon2dTestHelpers.constructPolygon(zUpFrame, vertices);
      assertEquals(3, polygon.getNumberOfVertices());

      ConvexPolygon2dTestHelpers.verifyPolygonContains(polygon, new FramePoint2D(zUpFrame, 0.0, 0.0), 1e-10);
      ConvexPolygon2dTestHelpers.verifyPolygonContains(polygon, new FramePoint2D(zUpFrame, 2.0, 0.0), 1e-10);
      ConvexPolygon2dTestHelpers.verifyPolygonContains(polygon, new FramePoint2D(zUpFrame, 1.0, 0.1), 1e-10);

      ConvexPolygon2dTestHelpers.verifyPointsAreClockwise(polygon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIntersectionWhenFullyInside()
   {
      ArrayList<Point2D> listOfPoints = new ArrayList<Point2D>();
      listOfPoints.add(new Point2D(0.0, 0.0));
      listOfPoints.add(new Point2D(1.0, 0.0));
      listOfPoints.add(new Point2D(0.0, 1.0));
      listOfPoints.add(new Point2D(1.0, 1.0));

      ConvexPolygon2D convexPolygon2dA = new ConvexPolygon2D(listOfPoints);

      listOfPoints.clear();
      listOfPoints.add(new Point2D(-1.0, -1.0));
      listOfPoints.add(new Point2D(2.0, -1.0));
      listOfPoints.add(new Point2D(-1.0, 2.0));
      listOfPoints.add(new Point2D(2.0, 2.0));

      ConvexPolygon2D convexPolygon2dB = new ConvexPolygon2D(listOfPoints);

      ConvexPolygon2D intersection = new ConvexPolygon2D();
      ConvexPolygonTools.computeIntersectionOfPolygons(convexPolygon2dA, convexPolygon2dB, intersection);
      boolean epsilonEquals = intersection.epsilonEquals(convexPolygon2dA, 1e-7);
      assertTrue(epsilonEquals);

      ConvexPolygonTools.computeIntersectionOfPolygons(convexPolygon2dB, convexPolygon2dA, intersection);
      epsilonEquals = intersection.epsilonEquals(convexPolygon2dA, 1e-7);
      assertTrue(epsilonEquals);

      listOfPoints.clear();
      listOfPoints.add(new Point2D(0.1904001452623111, 0.07922536690619195));
      listOfPoints.add(new Point2D(0.1923482408479345, 0.5736513188711437));
      listOfPoints.add(new Point2D(0.24837080387208538, 0.5533707067242215));
      listOfPoints.add(new Point2D(0.2560381177005394, 0.550093244819894));
      listOfPoints.add(new Point2D(0.3021057612864858, 0.5276338625408057));
      listOfPoints.add(new Point2D(0.35302325196142154, 0.49669456810449586));
      listOfPoints.add(new Point2D(0.4006211967955147, 0.4608579046936889));
      listOfPoints.add(new Point2D(0.4444302495375464, 0.42047724478458476));
      listOfPoints.add(new Point2D(0.4840184248413931, 0.3759507675720234));
      listOfPoints.add(new Point2D(0.5189953579184864, 0.3277175326673503));
      listOfPoints.add(new Point2D(0.5490161537848919, 0.27625315068916595));
      listOfPoints.add(new Point2D(0.5737847881469639, 0.2220650934377122));
      listOfPoints.add(new Point2D(0.5930570263906623, 0.16568768989757945));
      listOfPoints.add(new Point2D(0.606642831891427, 0.10767685741135981));
      listOfPoints.add(new Point2D(0.1904001452623111, 0.07922536690619195));
      convexPolygon2dA = new ConvexPolygon2D(listOfPoints);

      listOfPoints.clear();
      listOfPoints.add(new Point2D(-0.26792484945022277, 0.5164452162023662));
      listOfPoints.add(new Point2D(-0.21938799685279367, 0.5422255592213991));
      listOfPoints.add(new Point2D(-0.1686958167513698, 0.5634565512568254));
      listOfPoints.add(new Point2D(-0.11627362387979798, 0.5799600612101443));
      listOfPoints.add(new Point2D(-0.06256124802966133, 0.591597622242303));
      listOfPoints.add(new Point2D(-0.008009343814616467, 0.5982715935305327));
      listOfPoints.add(new Point2D(0.04692439038709253, 0.5999259794889963));
      listOfPoints.add(new Point2D(0.10177905258832422, 0.5965468995798632));
      listOfPoints.add(new Point2D(0.1560944042274756, 0.5881627047730331));
      listOfPoints.add(new Point2D(0.20941473163667895, 0.5748437396773916));
      listOfPoints.add(new Point2D(0.26129266954548536, 0.5567017523393519));
      listOfPoints.add(new Point2D(0.3112929545402855, 0.5338889566605598));
      listOfPoints.add(new Point2D(0.3589960769873979, 0.5065967553012091));
      listOfPoints.add(new Point2D(0.40400180077966186, 0.4750541337839984));
      listOfPoints.add(new Point2D(0.4459325213753508, 0.43952573927242716));
      listOfPoints.add(new Point2D(0.48443643395497327, 0.4003096601427597));
      listOfPoints.add(new Point2D(0.5191904851146687, 0.3577349249793695));
      listOfPoints.add(new Point2D(0.5499030833310595, 0.31215874197725635));
      listOfPoints.add(new Point2D(0.5763165454563693, 0.26396350191354906));
      listOfPoints.add(new Point2D(0.5982092587173592, 0.2135535698334953));
      listOfPoints.add(new Point2D(0.6153975400785948, 0.16135189236915945));
      listOfPoints.add(new Point2D(0.6277371773697216, 0.10779644915591552));
      listOfPoints.add(new Point2D(0.6351246392464617, 0.05333657811986491));
      listOfPoints.add(new Point2D(0.6374979438335908, -0.00157079453245079));
      listOfPoints.add(new Point2D(0.634837178761848, -0.056464987991108585));
      listOfPoints.add(new Point2D(0.0, 0.06));
      convexPolygon2dB = new ConvexPolygon2D(listOfPoints);

      ConvexPolygonTools.computeIntersectionOfPolygons(convexPolygon2dB, convexPolygon2dA, intersection);
      epsilonEquals = intersection.epsilonEquals(convexPolygon2dA, 1e-14);
      assertTrue(epsilonEquals);

      ConvexPolygonTools.computeIntersectionOfPolygons(convexPolygon2dA, convexPolygon2dB, intersection);
      epsilonEquals = intersection.epsilonEquals(convexPolygon2dA, 1e-14);
      assertTrue(epsilonEquals);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIntersectionWhenFullyInsideWithRepeatedPoint()
   {
      ArrayList<Point2D> listOfPoints = new ArrayList<Point2D>();
      listOfPoints.add(new Point2D(0.19, 0.0));
      listOfPoints.add(new Point2D(0.192, 0.6));
      listOfPoints.add(new Point2D(0.25, 0.5));
      listOfPoints.add(new Point2D(0.19, 0.0));
      ConvexPolygon2D convexPolygon2dA = new ConvexPolygon2D(listOfPoints);

      listOfPoints.clear();
      listOfPoints.add(new Point2D(-1.0, -1.0));
      listOfPoints.add(new Point2D(2.0, -1.0));
      listOfPoints.add(new Point2D(-1.0, 2.0));
      listOfPoints.add(new Point2D(2.0, 2.0));
      ConvexPolygon2D convexPolygon2dB = new ConvexPolygon2D(listOfPoints);

      ConvexPolygon2D intersection = new ConvexPolygon2D();
      ConvexPolygonTools.computeIntersectionOfPolygons(convexPolygon2dA, convexPolygon2dB, intersection);
      boolean epsilonEquals = intersection.epsilonEquals(convexPolygon2dA, 1e-14);
      assertTrue(epsilonEquals);

      ConvexPolygonTools.computeIntersectionOfPolygons(convexPolygon2dB, convexPolygon2dA, intersection);
      epsilonEquals = intersection.epsilonEquals(convexPolygon2dA, 1e-14);
      assertTrue(epsilonEquals);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testConstructorWithRepeatedPoints()
   {
      ArrayList<Point2D> listOfPoints = new ArrayList<Point2D>();
      listOfPoints.add(new Point2D(0.0, 0.0));
      listOfPoints.add(new Point2D(1.0, 1.0));
      listOfPoints.add(new Point2D(1.0, 1.0));
      listOfPoints.add(new Point2D(1.0, 1.0));
      listOfPoints.add(new Point2D(1.0, 0.0));
      listOfPoints.add(new Point2D(0.0, 0.0));
      ConvexPolygon2D convexPolygon2d = new ConvexPolygon2D(listOfPoints);

      // Above point list contains 3 unique points
      assertEquals(3, convexPolygon2d.getNumberOfVertices());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetClosestPointToRay1()
   {
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(new Point2D(-1.0, 0.0));
      polygon.addVertex(new Point2D(0.0, 1.0));
      polygon.addVertex(new Point2D(2.0, 0.0));
      polygon.addVertex(new Point2D(1.0, -1.0));
      polygon.update();

      Line2D ray1 = new Line2D(new Point2D(5.0, -3.0), new Vector2D(0.0, 1.0));
      assertPointsEqual(new Point2D(2.0, 0.0), polygon.getClosestPointWithRay(ray1));

      Line2D ray2 = new Line2D(new Point2D(1.0, 1.0), new Vector2D(0.5, 0.5));
      assertPointsEqual(new Point2D(4.0/5.0, 3.0/5.0), polygon.getClosestPointWithRay(ray2));

      Line2D ray3 = new Line2D(new Point2D(1.0, 1.0), new Vector2D(-0.5, 0.1));
      assertPointsEqual(new Point2D(0.0, 1.0), polygon.getClosestPointWithRay(ray3));

      Line2D ray4 = new Line2D(new Point2D(-0.75, 0.75), new Vector2D(0.0, 0.1));
      assertPointsEqual(new Point2D(-0.5, 0.5), polygon.getClosestPointWithRay(ray4));

      Line2D ray5 = new Line2D(new Point2D(-0.75, 0.75), new Vector2D(0.3, 0.3));
      assertPointsEqual(new Point2D(-0.5, 0.5), polygon.getClosestPointWithRay(ray5));

      Line2D ray6 = new Line2D(new Point2D(-0.75, 0.75), new Vector2D(-0.3, -0.3));
      assertPointsEqual(new Point2D(-0.5, 0.5), polygon.getClosestPointWithRay(ray6));

      Line2D ray7 = new Line2D(new Point2D(-0.75, 0.75), new Vector2D(0.3, 0.31));
      assertPointsEqual(new Point2D(-0.5, 0.5), polygon.getClosestPointWithRay(ray7));

      Line2D ray8 = new Line2D(new Point2D(-0.75, 0.75), new Vector2D(0.3, 0.29));
      assertPointsEqual(new Point2D(0.0, 1.0), polygon.getClosestPointWithRay(ray8));

      Line2D ray9 = new Line2D(new Point2D(1.75, -0.75), new Vector2D(1.0, 1.0));
      assertPointsEqual(new Point2D(1.5, -0.5), polygon.getClosestPointWithRay(ray9));

      Line2D ray10 = new Line2D(new Point2D(1.75, -0.75), new Vector2D(-0.3, -0.3));
      assertPointsEqual(new Point2D(1.5, -0.5), polygon.getClosestPointWithRay(ray10));

      Line2D ray11 = new Line2D(new Point2D(1.0, -1.2), new Vector2D(-2.0, 1.0));
      assertPointsEqual(new Point2D(1.0, -1.0), polygon.getClosestPointWithRay(ray11));

      Line2D ray12 = new Line2D(new Point2D(1.0, -1.2), new Vector2D(2.0, -1.0));
      assertPointsEqual(new Point2D(1.0, -1.0), polygon.getClosestPointWithRay(ray12));

      Line2D ray13 = new Line2D(new Point2D(-0.1, -0.7), new Vector2D(-2.0, 1.0));
      assertPointsEqual(new Point2D(0.0, -0.5), polygon.getClosestPointWithRay(ray13));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetClosestPointToRay2()
   {
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      assertTrue(polygon.getClosestPointWithRay(new Line2D(0.0, 0.0, 1.0, 0.0)) == null);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetClosestPointToRay3()
   {
      Point2D vertex = new Point2D(1.0, -1.0);

      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(vertex);
      polygon.update();

      Line2D ray1 = new Line2D(new Point2D(5.0, -3.0), new Vector2D(0.0, 1.0));
      assertPointsEqual(vertex, polygon.getClosestPointWithRay(ray1));

      Line2D ray2 = new Line2D(new Point2D(0.0, 0.0), new Vector2D(1.0, 0.0));
      assertPointsEqual(vertex, polygon.getClosestPointWithRay(ray2));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetClosestPointToRay4()
   {
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(new Point2D(2.0, -5.0));
      polygon.addVertex(new Point2D(1.0, -6.0));
      polygon.update();

      Line2D ray1 = new Line2D(new Point2D(1.0, -5.0), new Vector2D(1.0, 0.1));
      assertPointsEqual(new Point2D(2.0, -5.0), polygon.getClosestPointWithRay(ray1));

      Line2D ray2 = new Line2D(new Point2D(1.25, -5.25), new Vector2D(0.75, 0.3));
      assertPointsEqual(new Point2D(2.0, -5.0), polygon.getClosestPointWithRay(ray2));

      Line2D ray3 = new Line2D(new Point2D(1.25, -5.25), new Vector2D(0.75, 0.8));
      assertPointsEqual(new Point2D(1.5, -5.5), polygon.getClosestPointWithRay(ray3));

      Line2D ray4 = new Line2D(new Point2D(1.25, -5.25), new Vector2D(1.0, 1.0));
      assertPointsEqual(new Point2D(1.5, -5.5), polygon.getClosestPointWithRay(ray4));

      Line2D ray5 = new Line2D(new Point2D(1.25, -5.25), new Vector2D(-1.0, -1.0));
      assertPointsEqual(new Point2D(1.5, -5.5), polygon.getClosestPointWithRay(ray5));

      Line2D ray6 = new Line2D(new Point2D(1.75, -5.75), new Vector2D(1.0, 1.0));
      assertPointsEqual(new Point2D(1.5, -5.5), polygon.getClosestPointWithRay(ray6));

      Line2D ray7 = new Line2D(new Point2D(1.75, -5.75), new Vector2D(-1.0, -1.0));
      assertPointsEqual(new Point2D(1.5, -5.5), polygon.getClosestPointWithRay(ray7));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testNANRay()
   {
      ArrayList<Point2D> listOfPoints = new ArrayList<Point2D>();
      listOfPoints.add(new Point2D(0.11429999999999998, 0.1397));
      listOfPoints.add(new Point2D(0.11429999999999998, 0.04444999999999999));
      listOfPoints.add(new Point2D(-0.047625, 0.04444999999999999));
      listOfPoints.add(new Point2D(-0.047625, 0.1397));

      ConvexPolygon2D convexPolygon2d = new ConvexPolygon2D(listOfPoints);

      Point2D pont2d = new Point2D(Double.NaN, Double.NaN);
      Vector2D vector2d = new Vector2D(Double.NaN, Double.NaN);
      Line2D line2d = new Line2D(pont2d, vector2d);

      convexPolygon2d.intersectionWithRay(line2d);
      System.out.println("done");
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testIntersectionWithLine1()
   {
      // add in order so vertices do not get changed when update is called.
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(new Point2D(0.0, 0.0));
      polygon.addVertex(new Point2D(-1.0, 0.0));
      polygon.addVertex(new Point2D(0.0, 1.0));
      polygon.addVertex(new Point2D(1.0, 1.0));
      polygon.update();

      Point2D result1 = new Point2D(0.6, 0.4);
      Point2D result2 = new Point2D(0.1, 0.9);

      Line2D line1 = new Line2D(new Point2D(0.0, 0.5), new Vector2D(0.1, 0.0));
      Point2D[] expected1 = new Point2D[] {new Point2D(-0.5, 0.5), new Point2D(0.5, 0.5)};
      assertPointsEqual(expected1, polygon.intersectionWith(line1), false);

      Line2D line2 = new Line2D(new Point2D(1.0, 0.0), new Vector2D(0.0, -8.0));
      Point2D[] expected2 = new Point2D[] {new Point2D(1.0, 1.0)};
      assertPointsEqual(expected2, polygon.intersectionWith(line2), false);
      assertTrue(polygon.intersectionWith(line2, result1, result2) == 1);
      assertPointsEqual(expected2[0], result1);

      Line2D line3 = new Line2D(new Point2D(0.0, 1.0), new Vector2D(0.5, 0.0));
      Point2D[] expected3 = new Point2D[] {new Point2D(0.0, 1.0), new Point2D(1.0, 1.0)};
      assertPointsEqual(expected3, polygon.intersectionWith(line3), false);
      assertTrue(polygon.intersectionWith(line3, result1, result2) == 2);
      assertPointsEqual(expected3[0], result1);
      assertPointsEqual(expected3[1], result2);

      Line2D line4 = new Line2D(new Point2D(0.5, 10.0), new Vector2D(0.0, 0.1));
      Point2D[] expected4 = new Point2D[] {new Point2D(0.5, 1.0), new Point2D(0.5, 0.5)};
      assertPointsEqual(expected4, polygon.intersectionWith(line4), false);

      Line2D line5 = new Line2D(new Point2D(-1.0, -0.5), new Vector2D(1.0, 1.0));
      Point2D[] expected5 = new Point2D[] {new Point2D(-0.5, 0.0), new Point2D(0.5, 1.0)};
      assertPointsEqual(expected5, polygon.intersectionWith(line5), false);

      Line2D line6 = new Line2D(new Point2D(0.0, -1.5), new Vector2D(1.0, 1.0));
      Point2D[] expected6 = null;
      result1.set(0.0, 0.0);
      result2.set(0.0, 0.0);
      assertPointsEqual(expected6, polygon.intersectionWith(line6), false);
      assertTrue(polygon.intersectionWith(line6, result1, result2) == 0);

      Line2D line7 = new Line2D(new Point2D(0.0, -1.5), new Vector2D(0.0, 2.0));
      Point2D[] expected7 = new Point2D[] {new Point2D(0.0, 0.0), new Point2D(0.0, 1.0)};
      assertPointsEqual(expected7, polygon.intersectionWith(line7), false);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testIntersectionWithLine2()
   {
      // line polygon
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(new Point2D(1.0, 0.0));
      polygon.addVertex(new Point2D(-1.0, 0.0));
      polygon.update();

      Line2D line1 = new Line2D(new Point2D(-1.0, 1.0), new Vector2D(0.0, -0.8));
      Point2D[] expected1 = new Point2D[] {new Point2D(-1.0, 0.0)};
      assertPointsEqual(expected1, polygon.intersectionWith(line1), false);

      Line2D line2 = new Line2D(new Point2D(-0.5, 1.0), new Vector2D(0.0, -0.8));
      Point2D[] expected2 = new Point2D[] {new Point2D(-0.5, 0.0)};
      assertPointsEqual(expected2, polygon.intersectionWith(line2), false);

      Line2D line3 = new Line2D(new Point2D(1.5, 1.0), new Vector2D(0.0, -0.8));
      Point2D[] expected3 = null;
      assertPointsEqual(expected3, polygon.intersectionWith(line3), false);

      Line2D line4 = new Line2D(new Point2D(-0.8, 0.0), new Vector2D(0.1, 0.0));
      Point2D[] expected4 = new Point2D[] {new Point2D(-1.0, 0.0), new Point2D(1.0, 0.0)};
      assertPointsEqual(expected4, polygon.intersectionWith(line4), false);

      Line2D line5 = new Line2D(new Point2D(1.0, 0.0), new Vector2D(0.0, -0.1));
      Point2D[] expected5 = new Point2D[] {new Point2D(1.0, 0.0)};
      assertPointsEqual(expected5, polygon.intersectionWith(line5), false);

      Line2D line6 = new Line2D(new Point2D(-1.0, 0.0), new Vector2D(0.0, -0.1));
      Point2D[] expected6 = new Point2D[] {new Point2D(-1.0, 0.0)};
      assertPointsEqual(expected6, polygon.intersectionWith(line6), false);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testIntersectionWithLine3()
   {
      // point polygon
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(new Point2D(1.0, 0.0));
      polygon.update();

      Line2D line1 = new Line2D(new Point2D(3.0, 1.0), new Vector2D(-2.0, -1.0));
      Point2D[] expected1 = new Point2D[] {new Point2D(1.0, 0.0)};
      assertPointsEqual(expected1, polygon.intersectionWith(line1), false);

      Line2D line2 = new Line2D(new Point2D(2.0, 1.0), new Vector2D(-1.3, -0.8));
      Point2D[] expected2 = null;
      assertPointsEqual(expected2, polygon.intersectionWith(line2), false);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testIntersectionWithLine4()
   {
      // empty polygon
      ConvexPolygon2D polygon = new ConvexPolygon2D();

      Line2D line1 = new Line2D(new Point2D(3.0, 1.0), new Vector2D(-1.6, -0.8));
      Point2D[] expected1 = null;
      assertPointsEqual(expected1, polygon.intersectionWith(line1), false);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testIntersectionWithRay1()
   {
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(new Point2D(-1.0, -1.0));
      polygon.addVertex(new Point2D(1.0, -1.0));
      polygon.addVertex(new Point2D(-1.0, 1.0));
      polygon.addVertex(new Point2D(1.0, 1.0));
      polygon.update();

      Point2D result1 = new Point2D();
      Point2D result2 = new Point2D();

      Line2D ray1 = new Line2D(new Point2D(0.0, 0.0), new Vector2D(0.2, 0.0));
      Point2D[] expected1 = new Point2D[] {new Point2D(1.0, 0.0)};
      assertPointsEqual(expected1, polygon.intersectionWithRay(ray1), false);
      assertTrue(polygon.intersectionWithRay(ray1, result1, result2) == 1);

      Line2D ray2 = new Line2D(new Point2D(-1.0, 0.0), new Vector2D(0.2, 0.0));
      Point2D[] expected2 = new Point2D[] {new Point2D(1.0, 0.0), new Point2D(-1.0, 0.0)};
      assertPointsEqual(expected2, polygon.intersectionWithRay(ray2), false);
      assertTrue(polygon.intersectionWithRay(ray2, result1, result2) == 2);

      Line2D ray3 = new Line2D(new Point2D(2.0, 0.0), new Vector2D(0.2, 0.0));
      Point2D[] expected3 = null;
      assertPointsEqual(expected3, polygon.intersectionWithRay(ray3), false);
      assertTrue(polygon.intersectionWithRay(ray3, result1, result2) == 0);

      Line2D ray4 = new Line2D(new Point2D(1.0, 1.0), new Vector2D(0.2, -0.1));
      Point2D[] expected4 = new Point2D[] {new Point2D(1.0, 1.0)};
      assertPointsEqual(expected4, polygon.intersectionWithRay(ray4), false);
      assertTrue(polygon.intersectionWithRay(ray4, result1, result2) == 1);

      Line2D ray5 = new Line2D(new Point2D(1.5, 1.0), new Vector2D(0.2, -0.1));
      Point2D[] expected5 = null;
      assertPointsEqual(expected5, polygon.intersectionWithRay(ray5), false);
      assertTrue(polygon.intersectionWithRay(ray5, result1, result2) == 0);

      Line2D ray6 = new Line2D(new Point2D(-1.0, -2.0), new Vector2D(0.3, 0.3));
      Point2D[] expected6 = new Point2D[] {new Point2D(0.0, -1.0), new Point2D(1.0, 0.0)};
      assertPointsEqual(expected6, polygon.intersectionWithRay(ray6), false);
      assertTrue(polygon.intersectionWithRay(ray6, result1, result2) == 2);

      Line2D ray7 = new Line2D(new Point2D(-1.0, -2.0), new Vector2D(0.0, 1.7));
      Point2D[] expected7 = new Point2D[] {new Point2D(-1.0, -1.0), new Point2D(-1.0, 1.0)};
      assertPointsEqual(expected7, polygon.intersectionWithRay(ray7), false);
      assertTrue(polygon.intersectionWithRay(ray7, result1, result2) == 2);

      Line2D ray8 = new Line2D(new Point2D(-0.5, 0.5), new Vector2D(-0.3, -0.3));
      Point2D[] expected8 = new Point2D[] {new Point2D(-1.0, 0.0)};
      assertPointsEqual(expected8, polygon.intersectionWithRay(ray8), false);
      assertTrue(polygon.intersectionWithRay(ray8, result1, result2) == 1);

      Line2D ray9 = new Line2D(new Point2D(-0.5, 0.5), new Vector2D(0.15, 0.3));
      Point2D[] expected9 = new Point2D[] {new Point2D(-0.25, 1.0)};
      assertPointsEqual(expected9, polygon.intersectionWithRay(ray9), false);
      assertTrue(polygon.intersectionWithRay(ray9, result1, result2) == 1);

      Line2D ray10 = new Line2D(new Point2D(0.5, 0.5), new Vector2D(-0.15, 0.3));
      Point2D[] expected10 = new Point2D[] {new Point2D(0.25, 1.0)};
      assertPointsEqual(expected10, polygon.intersectionWithRay(ray10), false);
      assertTrue(polygon.intersectionWithRay(ray10, result1, result2) == 1);

      Line2D ray11 = new Line2D(new Point2D(0.5, 0.5), new Vector2D(0.15, 0.3));
      Point2D[] expected11 = new Point2D[] {new Point2D(0.75, 1.0)};
      assertPointsEqual(expected11, polygon.intersectionWithRay(ray11), false);
      assertTrue(polygon.intersectionWithRay(ray11, result1, result2) == 1);

      Line2D ray12 = new Line2D(new Point2D(0.5, 0.5), new Vector2D(0.15, -0.3));
      Point2D[] expected12 = new Point2D[] {new Point2D(1.0, -0.5)};
      assertPointsEqual(expected12, polygon.intersectionWithRay(ray12), false);
      assertTrue(polygon.intersectionWithRay(ray12, result1, result2) == 1);

      Line2D ray13 = new Line2D(new Point2D(0.5, 0.5), new Vector2D(0.0, -0.3));
      Point2D[] expected13 = new Point2D[] {new Point2D(0.5, -1.0)};
      assertPointsEqual(expected13, polygon.intersectionWithRay(ray13), false);
      assertTrue(polygon.intersectionWithRay(ray13, result1, result2) == 1);

      Line2D ray14 = new Line2D(new Point2D(0.5, 0.5), new Vector2D(0.0, 0.3));
      Point2D[] expected14 = new Point2D[] {new Point2D(0.5, 1.0)};
      assertPointsEqual(expected14, polygon.intersectionWithRay(ray14), false);
      assertTrue(polygon.intersectionWithRay(ray14, result1, result2) == 1);

      Line2D ray15 = new Line2D(new Point2D(1.5, 1.5), new Vector2D(0.0, 0.3));
      Point2D[] expected15 = null;
      assertPointsEqual(expected15, polygon.intersectionWithRay(ray15), false);
      assertTrue(polygon.intersectionWithRay(ray15, result1, result2) == 0);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIsInside()
   {
      double[][] polygonPoints = new double[][] {{-0.05107802536335158, 0.04155594197133163}, {-0.05052044462374434, 0.1431544119584275},
            {0.12219695435431863, 0.14220652470109518}, {0.12219695435431865, -0.041946248489056696}, {0.12163937361471142, -0.1435447184761526},
            {-0.05107802536335154, -0.14259683121882027}};

      Point2D testPoint = new Point2D(-0.04907805548171582, 2.6934439541712686E-4);

      ConvexPolygon2D polygon = new ConvexPolygon2D(polygonPoints);

      boolean isInside = polygon.isPointInside(testPoint);
      System.out.println("isInside = " + isInside);

      assertTrue(isInside);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testIsPointInside1()
   {
      // single point polygon
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(new Point2D(1.0, 1.0));
      polygon.update();

      Point2D point1 = new Point2D(1.0, 1.0);
      assertTrue(polygon.isPointInside(point1, epsilon));

      Point2D point2 = new Point2D(0.8, 0.9);
      assertFalse(polygon.isPointInside(point2));

      Point2D point3 = new Point2D(0.8, 1.1);
      assertTrue(polygon.isPointInside(point3, 0.3));

      Point2D point4 = new Point2D(1.0, 0.9);
      assertFalse(polygon.isPointInside(point4));

      Point2D point5 = new Point2D(2.0, 1.0);
      assertFalse(polygon.isPointInside(point5));
      assertTrue(polygon.isPointInside(point5, 1.0));

      Point2D point6 = new Point2D(1.0, 2.0);
      assertFalse(polygon.isPointInside(point6));
      assertTrue(polygon.isPointInside(point6, 1.0));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testIsPointInside2()
   {
      // line polygon
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(new Point2D(0.0, 0.0));
      polygon.addVertex(new Point2D(1.0, 0.0));
      polygon.update();

      Point2D point1 = new Point2D(0.1, 0.0);
      assertTrue(polygon.isPointInside(point1, epsilon));

      Point2D point2 = new Point2D(0.1, 0.1);
      assertFalse(polygon.isPointInside(point2, epsilon));

      Point2D point3 = new Point2D(1.5, 0.0);
      assertFalse(polygon.isPointInside(point3, epsilon));

      Point2D point4 = new Point2D(1.0, 0.0);
      assertTrue(polygon.isPointInside(point4.getX(), point4.getY()));

      Point2D point5 = new Point2D(1.0, epsilon * 0.1);
      assertFalse(polygon.isPointInside(point5.getX(), point5.getY()));

      Point2D point6 = new Point2D(1.0, epsilon * 0.1);
      assertTrue(polygon.isPointInside(point6, epsilon));

      Point2D point7 = new Point2D(1.5, 0.0);
      assertTrue(polygon.isPointInside(point7, 0.5));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testIsPointInside3()
   {
      // triangle polygon
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(new Point2D(0.0, 0.0));
      polygon.addVertex(new Point2D(5.0, 0.0));
      polygon.addVertex(new Point2D(3.0, 5.0));
      polygon.update();

      Point2D point1 = new Point2D(0.3, 0.0);
      assertTrue(polygon.isPointInside(point1, epsilon));

      Point2D point2 = new Point2D(0.0, 0.0);
      assertTrue(polygon.isPointInside(point2, epsilon));

      Point2D point3 = new Point2D(2.0, 2.0);
      assertTrue(polygon.isPointInside(point3));

      Point2D point4 = new Point2D(1.0, 0.3);
      assertTrue(polygon.isPointInside(point4, epsilon));

      Point2D point5 = new Point2D(-1.0, 4.0);
      assertFalse(polygon.isPointInside(point5.getX(), point5.getY(), epsilon));

      Point2D point6 = new Point2D(6.0, 7.0);
      assertFalse(polygon.isPointInside(point6, epsilon));

      Point2D point7 = new Point2D(10.0, 0.0);
      assertFalse(polygon.isPointInside(point7, epsilon));

      Point2D point8 = new Point2D(0.1, 0.2);
      assertFalse(polygon.isPointInside(point8));

      Point2D point9 = new Point2D(3.5, 4.9);
      assertFalse(polygon.isPointInside(point9.getX(), point9.getY(), epsilon));

      Point2D point10 = new Point2D(3.5, -1.0);
      assertFalse(polygon.isPointInside(point10));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testIsPointInside4()
   {
      // empty polygon
      ConvexPolygon2D polygon = new ConvexPolygon2D();

      Point2D point1 = new Point2D(10.0, 0.0);
      assertFalse(polygon.isPointInside(point1, epsilon));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testIsPointInside5()
   {
      // foot polygon
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(new Point2D(-0.06, -0.08));
      polygon.addVertex(new Point2D(0.14, -0.08));
      polygon.addVertex(new Point2D(0.14, -0.19));
      polygon.addVertex(new Point2D(-0.06, -0.19));
      polygon.update();

      Point2D point1 = new Point2D(0.03, 0.0);
      assertFalse(polygon.isPointInside(point1, 0.02));

      Point2D point2 = new Point2D(0.03, -0.09);
      assertTrue(polygon.isPointInside(point2));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testDistancePoint2dConvexPolygon2d()
   {
      ArrayList<Point2D> points = new ArrayList<Point2D>();
      points.add(new Point2D());
      points.add(new Point2D());
      points.add(new Point2D());
      ConvexPolygon2D test = new ConvexPolygon2D(points);
      test.distance(new Point2D());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testExtremePointsSquare()
   {
      ReferenceFrame zUpFrame = ReferenceFrame.constructARootFrame("someFrame");
      double xMin = -.1;
      double xMax = 0.1;
      double yMin = -0.1;
      double yMax = 0.1;

      ArrayList<FramePoint2D> squareList = new ArrayList<FramePoint2D>();
      squareList.add(new FramePoint2D(zUpFrame, xMin, yMin));
      squareList.add(new FramePoint2D(zUpFrame, xMax, yMax));
      squareList.add(new FramePoint2D(zUpFrame, xMin, yMax));
      squareList.add(new FramePoint2D(zUpFrame, xMax, yMin));

      FrameConvexPolygon2d square = new FrameConvexPolygon2d(squareList);

      // Compute the extreme points:
      FramePoint2D frontmostLeft  = new FramePoint2D();
      FramePoint2D frontmostRight = new FramePoint2D();
      FramePoint2D backmostRight  = new FramePoint2D();
      FramePoint2D backmostLeft   = new FramePoint2D();

      square.getFrameVertex(square.getMinXMaxYIndex(), frontmostLeft);
      square.getFrameVertex(square.getMaxXMaxYIndex(), frontmostRight);
      square.getFrameVertex(square.getMaxXMinYIndex(), backmostRight);
      square.getFrameVertex(square.getMinXMinYIndex(), backmostLeft);

      for (int i = 0; i < square.getNumberOfVertices(); i++)
      {
         FramePoint2D point = square.getFrameVertexCopy(i);
         assertFalse("frontmostLeft wrong, frontMostLeft = " + frontmostLeft + ", point = " + point,
                     ((point.getX() < frontmostLeft.getX()) || ((point.getX() == frontmostLeft.getX()) && (point.getY() > frontmostLeft.getY()))));
         assertFalse("frontmostRight wrong, frontmostRight = " + frontmostRight + ", point = " + point,
                     ((point.getX() > frontmostRight.getX()) || ((point.getX() == frontmostRight.getX()) && (point.getY() > frontmostRight.getY()))));
         assertFalse("backmostRight wrong, backmostRight = " + backmostRight + ", point = " + point,
                     ((point.getX() > backmostRight.getX()) || ((point.getX() == backmostRight.getX()) && (point.getY() < backmostRight.getY()))));
         assertFalse("backmostLeft wrong, backmostLeft = " + backmostLeft + ", point = " + point,
                     ((point.getX() < backmostLeft.getX()) || ((point.getX() == backmostLeft.getX()) && (point.getY() < backmostLeft.getY()))));
      }

      if (PLOT_RESULTS)
      {
         // Plot:
         FrameGeometryTestFrame testFrame = new FrameGeometryTestFrame(2.0 * xMin, 2.0 * xMax, 2.0 * yMin, 2.0 * yMax);
         FrameGeometry2dPlotter plotter = testFrame.getFrameGeometry2dPlotter();
         plotter.setDrawPointsLarge();
         plotter.addPolygon(square);
         plotter.addFramePoint2d(frontmostLeft, Color.green);
         plotter.addFramePoint2d(frontmostRight, Color.orange);
         plotter.addFramePoint2d(backmostRight, Color.red);
         plotter.addFramePoint2d(backmostLeft, Color.black);

         waitForButtonOrPause(testFrame);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testInsideWithSimpleSquare()
   {
      double[][] vertices = new double[][] {{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}, {1.0, 1.0}};
      ReferenceFrame zUpFrame = ReferenceFrame.constructARootFrame("someFrame");
      FrameConvexPolygon2d polygon = ConvexPolygon2dTestHelpers.constructPolygon(zUpFrame, vertices);

      double[][] pointsOutside = new double[][] {{1.5, 0.5}, {1.1, 1.1}, {1.1, 0.5}, {-0.1, -0.1}, {-0.1, 0.5}};
      double[][] pointsInside = new double[][] {{0.5, 0.5}, {0.1, 0.7}, {0.3, 0.5}, {0.99, 0.99}, {0.01, 0.01}, {0.01, 0.99}, {0.99, 0.01}};
      double[][] boundaryPoints = new double[][] {{0.0, 0.5}, {0.5, 0.0}, {1.0, 0.5}, {0.5, 1.0}};

      for (double[] pointOutside : pointsOutside)
      {
         FramePoint2D testPoint = new FramePoint2D(zUpFrame, pointOutside);
         if (polygon.isPointInside(testPoint))
            throw new RuntimeException();
      }

      for (double[] pointInside : pointsInside)
      {
         FramePoint2D testPoint = new FramePoint2D(zUpFrame, pointInside);
         if (!polygon.isPointInside(testPoint))
            throw new RuntimeException();
      }

      for (double[] vertex : vertices)
      {
         FramePoint2D testPoint = new FramePoint2D(zUpFrame, vertex);
         if (!polygon.isPointInside(testPoint))
            throw new RuntimeException();
      }

      for (double[] boundaryPoint : boundaryPoints)
      {
         FramePoint2D testPoint = new FramePoint2D(zUpFrame, boundaryPoint);
         if (!polygon.isPointInside(testPoint))
            throw new RuntimeException();
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.8)
   @Test(timeout = 30000)
   public void testTiming()
   {
      double[][] vertices = new double[][] {{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}, {1.0, 1.0}, {1.7, 0.5}, {-0.6, 1.2}, {1.8, 1.1}, {0.5, 0.5}, {0.2, 1.56}};
      ReferenceFrame zUpFrame = ReferenceFrame.constructARootFrame("someFrame");

      int numberOfTests = 100000;

      long startTime = System.currentTimeMillis();
      for (int i = 0; i < numberOfTests; i++)
      {
         @SuppressWarnings("unused")
         FrameConvexPolygon2d polygon = ConvexPolygon2dTestHelpers.constructPolygon(zUpFrame, vertices);
      }

      long endTime = System.currentTimeMillis();

      double totalTime = (endTime - startTime) * 0.001;

      double timePerTest = totalTime / ((double) numberOfTests);

      System.out.println("Finding a convex polygon with " + vertices.length + " points to check took " + timePerTest * 1000.0 + " milliseconds per test.");
      double maxTimeAllowed = 0.1 * 0.001;

      assertTrue(timePerTest < maxTimeAllowed);

      FrameConvexPolygon2d polygon = ConvexPolygon2dTestHelpers.constructPolygon(zUpFrame, vertices);
      FramePoint2D pointToTest = new FramePoint2D(zUpFrame, 0.5, 0.5);

      numberOfTests = 100000;
      startTime = System.currentTimeMillis();

      for (int i = 0; i < numberOfTests; i++)
      {
         @SuppressWarnings("unused")
         boolean isInside = polygon.isPointInside(pointToTest);
      }

      endTime = System.currentTimeMillis();

      totalTime = (endTime - startTime) * 0.001;

      timePerTest = totalTime / ((double) numberOfTests);

      System.out.println("Checking inside took " + timePerTest * 1000.0 + " milliseconds per test.");
      maxTimeAllowed = 0.0025 * 0.001;

      assertTrue(timePerTest < maxTimeAllowed);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.6)
   @Test(timeout = 30000)
   public void testTimingTwo()
   {
      Random random = new Random(1776L);

      ReferenceFrame zUpFrame = ReferenceFrame.constructARootFrame("someFrame");

      double xMin = -1.0, xMax = 1.0, yMin = -1.0, yMax = 1.0;
      int numberOfPoints = 1000;

      ArrayList<FramePoint2D> vertices = ConvexPolygon2dTestHelpers.generateRandomCircularFramePoints(random, zUpFrame, xMin, xMax, yMin, yMax, numberOfPoints);

      int numberOfTests = 100;

      long startTime = System.currentTimeMillis();
      for (int i = 0; i < numberOfTests; i++)
      {
         @SuppressWarnings("unused")
         FrameConvexPolygon2d polygon = new FrameConvexPolygon2d(vertices);
      }

      long endTime = System.currentTimeMillis();

      double totalTime = (endTime - startTime) * 0.001;

      double timePerTest = totalTime / ((double) numberOfTests);

      System.out.println("Finding a convex polygon with " + vertices.size() + " points to check took " + timePerTest * 1000.0 + " milliseconds per test.");
      double maxTimeAllowed = 15 * 0.001;

      if (timePerTest > maxTimeAllowed)
         throw new RuntimeException();

      FrameConvexPolygon2d polygon = new FrameConvexPolygon2d(vertices);

      numberOfTests = 1000000;
      startTime = System.currentTimeMillis();

      for (int i = 0; i < numberOfTests; i++)
      {
         FramePoint2D pointToTest = EuclidFrameRandomTools.nextFramePoint2D(random, zUpFrame, 2.0 * xMin, 2.0 * xMax, 2.0 * yMin, 2.0 * yMax);
         @SuppressWarnings("unused")
         boolean isInside = polygon.isPointInside(pointToTest);
      }

      endTime = System.currentTimeMillis();

      totalTime = (endTime - startTime) * 0.001;

      timePerTest = totalTime / ((double) numberOfTests);

      System.out.println("Checking inside took " + timePerTest * 1000.0 + " milliseconds per test.");
      maxTimeAllowed = 0.003 * 0.001;

      if (timePerTest > maxTimeAllowed)
         throw new RuntimeException();
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testExtremePointsRandom()
   {
      Random random = new Random(1776L);

      // Create a polygon from 50 random points:
      ReferenceFrame zUpFrame = ReferenceFrame.constructARootFrame("someFrame");
      double xMin = -.1;
      double xMax = 0.1;
      double yMin = -0.1;
      double yMax = 0.1;

      ArrayList<FramePoint2D> randomPointList = ConvexPolygon2dTestHelpers.generateRandomRectangularFramePoints(random, zUpFrame, xMin, xMax, yMin, yMax, 50);
      FrameConvexPolygon2d randomPolygon = new FrameConvexPolygon2d(randomPointList);

      // Compute the extreme points:
      FramePoint2D frontmostLeft  = new FramePoint2D();
      FramePoint2D frontmostRight = new FramePoint2D();
      FramePoint2D backmostRight  = new FramePoint2D();
      FramePoint2D backmostLeft   = new FramePoint2D();

      randomPolygon.getFrameVertex(randomPolygon.getMinXMaxYIndex(), frontmostLeft);
      randomPolygon.getFrameVertex(randomPolygon.getMaxXMaxYIndex(), frontmostRight);
      randomPolygon.getFrameVertex(randomPolygon.getMaxXMinYIndex(), backmostRight);
      randomPolygon.getFrameVertex(randomPolygon.getMinXMinYIndex(), backmostLeft);

      for (int i = 0; i < randomPolygon.getNumberOfVertices(); i++)
      {
         FramePoint2D point = randomPolygon.getFrameVertexCopy(i);
         assertFalse("frontmostLeft wrong, frontMostLeft = " + frontmostLeft + ", point = " + point,
                     ((point.getX() < frontmostLeft.getX()) || ((point.getX() == frontmostLeft.getX()) && (point.getY() > frontmostLeft.getY()))));
         assertFalse("frontmostRight wrong, frontmostRight = " + frontmostRight + ", point = " + point,
                     ((point.getX() > frontmostRight.getX()) || ((point.getX() == frontmostRight.getX()) && (point.getY() > frontmostRight.getY()))));
         assertFalse("backmostRight wrong, backmostRight = " + backmostRight + ", point = " + point,
                     ((point.getX() > backmostRight.getX()) || ((point.getX() == backmostRight.getX()) && (point.getY() < backmostRight.getY()))));
         assertFalse("backmostLeft wrong, backmostLeft = " + backmostLeft + ", point = " + point,
                     ((point.getX() < backmostLeft.getX()) || ((point.getX() == backmostLeft.getX()) && (point.getY() < backmostLeft.getY()))));
      }

      if (PLOT_RESULTS)
      {
         // Plot:
         FrameGeometryTestFrame testFrame = new FrameGeometryTestFrame(xMin, xMax, yMin, yMax);
         FrameGeometry2dPlotter plotter = testFrame.getFrameGeometry2dPlotter();
         plotter.setDrawPointsLarge();
         plotter.addPolygon(randomPolygon);
         plotter.addFramePoint2d(frontmostLeft, Color.green);
         plotter.addFramePoint2d(frontmostRight, Color.orange);
         plotter.addFramePoint2d(backmostRight, Color.red);
         plotter.addFramePoint2d(backmostLeft, Color.black);
         waitForButtonOrPause(testFrame);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testOrthogonalProjection1()
   {
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(new Point2D(-1.0, -1.0));
      polygon.addVertex(new Point2D(1.0, -1.0));
      polygon.addVertex(new Point2D(-1.0, 1.0));
      polygon.update();

      Point2D point1 = new Point2D(0.5, 0.5);
      assertPointsEqual(new Point2D(0.0, 0.0), polygon.orthogonalProjectionCopy(point1));

      Point2D point2 = new Point2D(-0.25, -0.25);
      assertNull(polygon.orthogonalProjectionCopy(point2));

      Point2D point3 = new Point2D(-2.0, -2.0);
      assertPointsEqual(new Point2D(-1.0, -1.0), polygon.orthogonalProjectionCopy(point3));

      Point2D point4 = new Point2D(-0.9, -2.0);
      assertPointsEqual(new Point2D(-0.9, -1.0), polygon.orthogonalProjectionCopy(point4));

      Point2D point5 = new Point2D(-1.1, -2.0);
      assertPointsEqual(new Point2D(-1.0, -1.0), polygon.orthogonalProjectionCopy(point5));

      Point2D point6 = new Point2D(1.8, -1.0);
      assertPointsEqual(new Point2D(1.0, -1.0), polygon.orthogonalProjectionCopy(point6));

      Point2D point7 = new Point2D(1.8, -0.8);
      assertPointsEqual(new Point2D(1.0, -1.0), polygon.orthogonalProjectionCopy(point7));

      Point2D point8 = new Point2D(0.5, 0.0);
      assertPointsEqual(new Point2D(0.25, -0.25), polygon.orthogonalProjectionCopy(point8));

      Point2D point9 = new Point2D(0.0, 0.5);
      assertPointsEqual(new Point2D(-0.25, 0.25), polygon.orthogonalProjectionCopy(point9));

      Point2D point10 = new Point2D(0.0, 0.0);
      assertNull(polygon.orthogonalProjectionCopy(point10));

      Point2D point11 = new Point2D(1.0, -1.0);
      assertNull(polygon.orthogonalProjectionCopy(point11));

      Point2D point12 = new Point2D(-1.1, 0.0);
      assertPointsEqual(new Point2D(-1.0, 0.0), polygon.orthogonalProjectionCopy(point12));

      Point2D point13 = new Point2D(-1.5, 3.0);
      assertPointsEqual(new Point2D(-1.0, 1.0), polygon.orthogonalProjectionCopy(point13));

      Point2D point14 = new Point2D(3.0, -1.5);
      assertPointsEqual(new Point2D(1.0, -1.0), polygon.orthogonalProjectionCopy(point14));

      Point2D point15 = new Point2D(1.6, -1.5);
      assertPointsEqual(new Point2D(1.0, -1.0), polygon.orthogonalProjectionCopy(point15));

      Point2D point16 = new Point2D(-2.0, 0.9);
      assertPointsEqual(new Point2D(-1.0, 0.9), polygon.orthogonalProjectionCopy(point16));

      Point2D point17 = new Point2D(-2.0, -0.9);
      assertPointsEqual(new Point2D(-1.0, -0.9), polygon.orthogonalProjectionCopy(point17));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testOrthogonalProjection2()
   {
      // empty polygon
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.orthogonalProjectionCopy(new Point2D());
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testOrthogonalProjection3()
   {
      // single point polygon
      Point2D vertex = new Point2D(1.0, 2.0);
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(vertex);
      polygon.update();

      assertPointsEqual(vertex, polygon.orthogonalProjectionCopy(new Point2D(0.0, 0.0)));
      assertPointsEqual(vertex, polygon.orthogonalProjectionCopy(new Point2D(1.0, -0.2)));
      assertPointsEqual(vertex, polygon.orthogonalProjectionCopy(new Point2D(1.0, 2.0)));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testOrthogonalProjection4()
   {
      // line polygon
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(new Point2D(1.0, 2.0));
      polygon.addVertex(new Point2D(1.0, 1.0));
      polygon.update();

      Point2D point1 = new Point2D(1.0, -1.0);
      assertPointsEqual(new Point2D(1.0, 1.0), polygon.orthogonalProjectionCopy(point1));

      Point2D point2 = new Point2D(3.0, 2.1);
      assertPointsEqual(new Point2D(1.0, 2.0), polygon.orthogonalProjectionCopy(point2));

      Point2D point3 = new Point2D(0.2, 1.2);
      assertPointsEqual(new Point2D(1.0, 1.2), polygon.orthogonalProjectionCopy(point3));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetLineOfSightVerticesOne()
   {
      double[][] vertices = new double[][] {{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}, {1.0, 1.0}};
      ReferenceFrame zUpFrame = ReferenceFrame.constructARootFrame("someFrame");

      FrameConvexPolygon2d polygon = ConvexPolygon2dTestHelpers.constructPolygon(zUpFrame, vertices);

      double[][] pointsToTest = new double[][] {{0.5, -0.5}, {1.5, 0.5}, {0.5, 1.5}, {-0.5, 0.5}, {-0.5, -0.5}, {1.5, -0.5}, {1.5, 1.5}, {-0.5, 1.5}};

      for (double[] pointToTestDoubles : pointsToTest)
      {
         FramePoint2D pointToTest = new FramePoint2D(zUpFrame, pointToTestDoubles);
         performLineOfSightTest(polygon, pointToTest);
      }

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.6)
   @Test(timeout = 30000)
   public void testGetLineOfSightVerticesTwo()
   {
      Random random = new Random(1092L);

      ReferenceFrame zUpFrame = ReferenceFrame.constructARootFrame("someFrame");
      double xMin = -100.0, xMax = 100.0, yMin = -100.0, yMax = 100.0;

      ArrayList<FramePoint2D> points = ConvexPolygon2dTestHelpers.generateRandomCircularFramePoints(random, zUpFrame, xMin, xMax, yMin, yMax, 200);

      FrameConvexPolygon2d polygon = new FrameConvexPolygon2d(points);
      ConvexPolygon2dTestHelpers.verifyPointsAreClockwise(polygon);

      FrameGeometryTestFrame testFrame = null;
      FrameGeometry2dPlotter plotter = null;

      if (PLOT_RESULTS)
      {
         testFrame = new FrameGeometryTestFrame(xMin, xMax, yMin, yMax);

         plotter = testFrame.getFrameGeometry2dPlotter();
         plotter.setPolygonToCheckInside(polygon);

         testFrame.addTestPoints(points);
      }

      int numLineOfSightTests = 100000;

      ArrayList<FramePoint2D> randomOutsidePoints = new ArrayList<FramePoint2D>();

      for (int i = 0; i < numLineOfSightTests; i++)
      {
         FramePoint2D randomPoint = EuclidFrameRandomTools.nextFramePoint2D(random, zUpFrame, 2.0 * xMin, 2.0 * xMax, 2.0 * yMin, 2.0 * yMax);
         if (!polygon.isPointInside(randomPoint))
         {
            randomOutsidePoints.add(randomPoint);

            FramePoint2D[] lineOfSightVertices = polygon.getLineOfSightVerticesCopy(randomPoint);
            ConvexPolygon2dTestHelpers.verifyLineOfSightVertices(polygon, randomPoint, lineOfSightVertices);

            FrameLine2d frameLine1 = new FrameLine2d(randomPoint, lineOfSightVertices[0]);
            FrameLine2d frameLine2 = new FrameLine2d(randomPoint, lineOfSightVertices[1]);

            if (PLOT_RESULTS)
            {
               plotter.addFrameLine2d(frameLine1, Color.cyan);
               plotter.addFrameLine2d(frameLine2, Color.magenta);
            }
         }
         else
         {
         }
      }

      System.gc();
      try
      {
         Thread.sleep(100);
      }
      catch (InterruptedException e)
      {
      }
      long startTime = System.currentTimeMillis();
      for (FramePoint2D testPoint : randomOutsidePoints)
      {
         polygon.getLineOfSightVerticesCopy(testPoint);
      }

      long endTime = System.currentTimeMillis();

      double totalTime = (endTime - startTime) * 0.001;

      int numberOfLineOfSightTests = randomOutsidePoints.size();
      double millisPerTest = totalTime / ((double) numberOfLineOfSightTests) * 1000.0;

      System.out.println("Performed " + numberOfLineOfSightTests + " line of sight tests in " + totalTime + " seconds. Or " + millisPerTest
            + " milliseconds per test, with a polygon with " + polygon.getNumberOfVertices() + " vertices.");

      assertTrue(millisPerTest < 0.002);

      if (PLOT_RESULTS)
      {
         waitForButtonOrPause(testFrame);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.9)
   @Test(timeout = 30000)
   public void testPolygonShrinkInto()
   {
      Random random = new Random(2002L);

      ReferenceFrame zUpFrame = ReferenceFrame.constructARootFrame("someFrame");

      double xMin = 0.0, xMax = 1.0, yMin = 0.0, yMax = 1.0;
      double widthMax = 0.5, heightMax = 0.5;
      int numberOfPoints = 20;
      int numberOfPolygons = 30;

      ConvexPolygon2D randomPPolygon = ConvexPolygon2dTestHelpers.generateRandomPolygon(random, zUpFrame, -0.1, 0.1, -0.1, 0.1, 5).getConvexPolygon2dCopy();
      ArrayList<FrameConvexPolygon2d> randomQPolygons = ConvexPolygon2dTestHelpers.generateRandomPolygons(random, zUpFrame, xMin, xMax, yMin, yMax, widthMax,
                                                                                                          heightMax, numberOfPoints, numberOfPolygons);

      // Find the matrix of intersecting polygons:
      ConvexPolygon2D[] shrunkenPolygons = new ConvexPolygon2D[randomQPolygons.size()];
      Point2DReadOnly referencePointForP = randomPPolygon.getCentroid();

      for (int j = 0; j < randomQPolygons.size(); j++)
      {
         FrameConvexPolygon2d polygonQ = randomQPolygons.get(j);

         ConvexPolygon2D convexPolygonQ = polygonQ.getConvexPolygon2dCopy();

         try
         {
            shrunkenPolygons[j] = ConvexPolygonTools.shrinkInto(randomPPolygon, referencePointForP, convexPolygonQ);
         }
         catch (RuntimeException exception)
         {
         }

      }

      FrameGeometryTestFrame testFrame = null;
      FrameGeometry2dPlotter plotter = null;

      if (PLOT_RESULTS)
      {
         testFrame = new FrameGeometryTestFrame(xMin, xMax, yMin, yMax);
         plotter = testFrame.getFrameGeometry2dPlotter();

         plotter.addFrameConvexPolygons(randomQPolygons, Color.CYAN);
         plotter.addConvexPolygons(shrunkenPolygons, Color.BLACK);

         plotter.repaint();
      }

      // Generate a bunch of points. For each one, if it is in the shrunken polygon, make sure P moved to the point is fully inside Q.
      // If not, make sure that P moved to the point is not fully inside Q.

      ArrayList<FramePoint2D> testPoints = ConvexPolygon2dTestHelpers.generateRandomRectangularFramePoints(random, zUpFrame, xMin, xMax, yMin, yMax, 10000);

      for (FramePoint2D testPoint : testPoints)
      {
         boolean buggy = false;
         boolean insideAnyShrunkenPolygon = false;

         for (int j = 0; j < randomQPolygons.size(); j++)
         {
            ConvexPolygon2D polygonQ = randomQPolygons.get(j).getConvexPolygon2dCopy();

            ConvexPolygon2D shrunkenPolygon = shrunkenPolygons[j];

            boolean insideShrunkenPolygon = ((shrunkenPolygon != null) && shrunkenPolygon.isPointInside(testPoint));
            if (insideShrunkenPolygon)
               insideAnyShrunkenPolygon = true;

            // If point is inside, then polygonP when moved to this location should be fully inside Q.
            // Otherwise it shouldn't be fully inside Q.

            Vector2D translation = new Vector2D(testPoint);
            translation.sub(referencePointForP);
            ConvexPolygon2D translatedPolygon = randomPPolygon.translateCopy(translation);

            boolean completelyInside = ConvexPolygon2dCalculator.isPolygonInside(translatedPolygon, polygonQ);

            if (insideShrunkenPolygon && !completelyInside)
            {
               buggy = true;
               String errorMessage = "\n\ninsideShrunkenPolygon && !completelyInside. \nrandomPPolygon = " + randomPPolygon + ", \npolygonQ = " + polygonQ
                     + ", \ntestPoint = " + testPoint;
               System.err.println(errorMessage);

               throw new RuntimeException(errorMessage);
            }

            if (!insideShrunkenPolygon && completelyInside)
            {
               buggy = true;
               String errorMessage = "!insideShrunkenPolygon && completelyInside. \nrandomPPolygon = " + randomPPolygon + ", \npolygonQ = " + polygonQ
                     + ", \ntestPoint = " + testPoint;
               System.err.println(errorMessage);

               throw new RuntimeException(errorMessage);
            }
         }

         if (PLOT_RESULTS)
         {
            if (buggy)
            {
               plotter.addFramePoint2d(testPoint, Color.RED);
            }

            if (insideAnyShrunkenPolygon)
            {
               plotter.addFramePoint2d(testPoint, Color.GREEN);
            }
            else
            {
               plotter.addFramePoint2d(testPoint, Color.GRAY);
            }
         }

      }

      if (PLOT_RESULTS)
      {
         waitForButtonOrPause(testFrame);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.4)
   @Test(timeout = 30000)
   public void testPolygonIntersections()
   {
      Random random = new Random(1886L);

      ReferenceFrame zUpFrame = ReferenceFrame.constructARootFrame("someFrame");

      double xMin = 0.0, xMax = 1.0, yMin = 0.0, yMax = 1.0;
      double widthMax = 0.5, heightMax = 0.5;
      int numberOfPoints = 20;
      int numberOfPolygons = 30;

      ArrayList<FrameConvexPolygon2d> randomPolygons = ConvexPolygon2dTestHelpers.generateRandomPolygons(random, zUpFrame, xMin, xMax, yMin, yMax, widthMax,
                                                                                                         heightMax, numberOfPoints, numberOfPolygons);

      FrameGeometryTestFrame testFrame = null;
      FrameGeometry2dPlotter plotter = null;

      if (PLOT_RESULTS)
      {
         testFrame = new FrameGeometryTestFrame(xMin, xMax, yMin, yMax);
         plotter = testFrame.getFrameGeometry2dPlotter();
         plotter.setDrawPointsMedium();
      }

      // Find the matrix of intersecting polygons:
      ConvexPolygon2D[][] intersectingPolygons = new ConvexPolygon2D[randomPolygons.size()][randomPolygons.size()];

      int n = randomPolygons.size();
      for (int i = 0; i < n; i++)
      {
         for (int j = 0; j < n; j++)
         {
            FrameConvexPolygon2d polygon1 = randomPolygons.get(i);
            FrameConvexPolygon2d polygon2 = randomPolygons.get(j);

            ConvexPolygon2D convexPolygon1 = polygon1.getConvexPolygon2dCopy();
            ConvexPolygon2D convexPolygon2 = polygon2.getConvexPolygon2dCopy();

            ConvexPolygon2D intersectingPolygon = new ConvexPolygon2D();
            boolean success = ConvexPolygonTools.computeIntersectionOfPolygons(convexPolygon1, convexPolygon2, intersectingPolygon);
            if (!success)
               intersectingPolygon = null;
            intersectingPolygons[i][j] = intersectingPolygon;

            if ((success) && (i != j))
            {
               if (PLOT_RESULTS)
               {
                  plotter.addPolygon(new FrameConvexPolygon2d(zUpFrame, intersectingPolygon), Color.BLACK);
                  plotter.repaint();
               }
            }
         }
      }

      if (PLOT_RESULTS)
      {
         plotter.addFrameConvexPolygons(randomPolygons, Color.CYAN);
         plotter.repaint();
      }

      // Generate a bunch of points. For each one, if it is in the intersection of any intersecting polygon, make sure it is in both of the polygon's parents:
      ArrayList<FramePoint2D> testPoints = ConvexPolygon2dTestHelpers.generateRandomRectangularFramePoints(random, zUpFrame, xMin, xMax, yMin, yMax, 10000);

      for (FramePoint2D testPoint : testPoints)
      {
         boolean insideAnyIntersection = false;

         for (int i = 0; i < n; i++)
         {
            for (int j = 0; j < n; j++)
            {
               FrameConvexPolygon2d polygon1 = randomPolygons.get(i);
               FrameConvexPolygon2d polygon2 = randomPolygons.get(j);

               boolean inside1 = polygon1.isPointInside(testPoint);
               boolean inside2 = polygon2.isPointInside(testPoint);

               ConvexPolygon2D intersectionPolygon = intersectingPolygons[i][j];
               if (i == j)
               {
                  assertNotNull(intersectionPolygon);
               }

               boolean insideIntersection = ((intersectionPolygon != null)
                     && (intersectionPolygon.isPointInside(testPoint)));
               if (insideIntersection)
               {
                  insideAnyIntersection = true;
               }

               if (inside1 && inside2)
               {
                  assertTrue("inside1 and inside2, but not inside intersection", insideIntersection);
               }

               if (insideIntersection)
               {
                  assertTrue("insideIntersection, but not inside1", inside1);
                  assertTrue("insideIntersection, but not inside2", inside2);
               }
            }
         }

         if (PLOT_RESULTS)
         {
            if (insideAnyIntersection)
            {
               plotter.addFramePoint2d(testPoint, Color.GREEN);
            }
            else
            {
               plotter.addFramePoint2d(testPoint, Color.GRAY);
            }
         }

      }

      if (PLOT_RESULTS)
      {
         plotter.repaint();
         waitForButtonOrPause(testFrame);
      }
   }

   private void performLineOfSightTest(FrameConvexPolygon2d polygon, FramePoint2D pointToTest)
   {
      FramePoint2D[] lineOfSightVertices = polygon.getLineOfSightVerticesCopy(pointToTest);
      ConvexPolygon2dTestHelpers.verifyLineOfSightVertices(polygon, pointToTest, lineOfSightVertices);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testTroublesomeIntersection()
   {
      double[][] vertices = new double[][] {{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}, {1.0, 1.0}};

      ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      FrameConvexPolygon2d polygon = new FrameConvexPolygon2d(worldFrame, vertices);

      FrameLine2d line = new FrameLine2d(worldFrame, new Point2D(1.0, 0.5), new Point2D(1.5, 0.5));
      FramePoint2D[] intersections = polygon.intersectionWith(line);

      assertEquals(2, intersections.length);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIntersectionWithLinesOne()
   {
      double[][] vertices = new double[][] {{0.0, 0.0}, {1.0, 0.0}, {0.0, 1.0}, {1.0, 1.0}};
      ReferenceFrame zUpFrame = ReferenceFrame.constructARootFrame("someFrame");

      FrameConvexPolygon2d polygon = ConvexPolygon2dTestHelpers.constructPolygon(zUpFrame, vertices);
      ConvexPolygon2dTestHelpers.verifyPointsAreClockwise(polygon);

      double[][] pointsAndLinesToTest = new double[][] {{0.5, -0.5, 0.0, 1.0}, // Straight through the heart.
            {1.5, 0.5, -1.0, 0.0}, {0.5, 1.5, 0.0, -1.0}, {-0.5, 0.5, 1.0, 0.0}, {-0.5, -0.5, 1.0, 1.0}, // Diagonal through two vertices.
            {1.5, -0.5, -1.0, 1.0}, {1.5, 1.5, -1.0, -1.0}, {-0.5, 1.5, 1.0, -1.0}, {-0.5, -0.5, 0.0, 1.0}, // Don't intersect at all.
            {-0.5, -0.5, 1.0, 0.0}, {1.5, -0.5, -1.0, 0.0}, {1.5, -0.5, 0.0, 1.0}, {1.5, 1.5, -1.0, 0.0}, {1.5, 1.5, 0.0, -1.0}, {-0.5, 1.5, 0.0, -1.0},
            {-0.5, 1.5, 1.0, 0.0}, {-1.0, 0.0, 1.0, 0.0}, // Parallel to an edge. These should return null.
            {0.0, -1.0, 0.0, 1.0}, {1.0, -1.0, 0.0, 1.0}, {2.0, 0.0, -1.0, 0.0}, {1.0, 2.0, 0.0, -1.0}, {2.0, 1.0, -1.0, 0.0}, {0.0, 2.0, 0.0, -1.0},
            {-1.0, 1.0, 1.0, 0.0}, {0.5, -0.5, 0.0, -1.0}, // Line points away from Polygon. These should return null. We treat the line as a ray!
            {1.5, 0.5, 1.0, 0.0}, {0.5, 1.5, 0.0, 1.0}, {-0.5, 0.5, -1.0, 0.0}, {0.5, 0.5, 0.0, 1.0}, // Line starts inside the Polygon. Should only be one leaving point.
            {0.5, 0.5, 0.0, -1.0}, {0.5, 0.5, 1.0, 0.0}, {0.5, 0.5, -1.0, 0.0}, {0.5, 0.5, 1.0, 1.0}, {0.5, 0.5, 1.0, -1.0}, {0.5, 0.5, -1.0, 1.0},
            {0.5, 0.5, -1.0, -1.0}, {1.0, 0.5, 1.5, 0.5}};

      ArrayList<FrameLine2d> nonIntersectingLines = new ArrayList<FrameLine2d>();
      ArrayList<FrameLine2d> intersectingLines = new ArrayList<FrameLine2d>();
      ArrayList<FramePoint2D> intersectingPoints = new ArrayList<FramePoint2D>();

      for (double[] pointToTestDoubles : pointsAndLinesToTest)
      {
         FramePoint2D framePoint2d = new FramePoint2D(zUpFrame, pointToTestDoubles[0], pointToTestDoubles[1]);
         FrameVector2D frameVector2d = new FrameVector2D(zUpFrame, pointToTestDoubles[2], pointToTestDoubles[3]);

         FrameLine2d frameLine2d = new FrameLine2d(framePoint2d, frameVector2d);

         FrameLineSegment2d[] intersectingEdges = polygon.getIntersectingEdges(frameLine2d);

         if (intersectingEdges == null)
         {
            nonIntersectingLines.add(frameLine2d);
            ConvexPolygon2dTestHelpers.verifyLineDoesNotIntersectPolygon(frameLine2d, polygon);
         }
         else
         {
            intersectingLines.add(frameLine2d);
            ConvexPolygon2dTestHelpers.verifyLineIntersectsEdge(frameLine2d, intersectingEdges);
         }
      }

      if (PLOT_RESULTS)
      {
         FrameGeometryTestFrame testFrame = new FrameGeometryTestFrame(-1.0, 2.0, -1.0, 2.0);

         FrameGeometry2dPlotter plotter = testFrame.getFrameGeometry2dPlotter();
         plotter.setDrawPointsMedium();
         plotter.setPolygonToCheckInside(polygon);

         plotter.addFrameLines2d(intersectingLines, Color.green);
         plotter.addFrameLines2d(nonIntersectingLines, Color.red);

         testFrame.addTestPoints(intersectingPoints);

         waitForButtonOrPause(testFrame);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetOppositeMidEdgeWhenPublic()
   {
      int[][] leftRightNumExpecteds = new int[][] {{0, 0, 6, 3}, {1, 1, 6, 4}, {2, 2, 6, 5}, {3, 3, 6, 0}, {4, 4, 6, 1}, {5, 5, 6, 2}, {1, 0, 6, 1},
            {2, 0, 6, 1}, {3, 0, 6, 2}, {4, 0, 6, 2}, {5, 0, 6, 3}, {2, 1, 6, 2}, {3, 1, 6, 2}, {4, 1, 6, 3}, {5, 1, 6, 3}, {3, 2, 6, 3}, {4, 2, 6, 3},
            {5, 2, 6, 4}, {4, 3, 6, 4}, {5, 3, 6, 4}, {5, 4, 6, 5}, {0, 1, 6, 4}, {0, 2, 6, 4}, {0, 3, 6, 5}, {0, 4, 6, 5}, {0, 5, 6, 0}, {1, 2, 6, 5},
            {1, 3, 6, 5}, {1, 4, 6, 0}, {1, 5, 6, 0}, {2, 3, 6, 0}, {2, 4, 6, 0}, {2, 5, 6, 1}, {3, 4, 6, 1}, {3, 5, 6, 1}, {4, 5, 6, 2}, {0, 0, 3, 2},
            {1, 1, 3, 0}, {2, 2, 3, 1}, {0, 1, 3, 2}, {0, 2, 3, 0}, {1, 2, 3, 0}, {1, 0, 3, 1}, {2, 0, 3, 1}, {2, 1, 3, 2},};

      ConvexPolygon2D polygon = new ConvexPolygon2D();
      for (int[] leftRightNumExpected : leftRightNumExpecteds)
      {
         int numEdges = leftRightNumExpected[2];
         double radius = 1.0;

         polygon.clear();
         for (double angle = 0; angle < 2.0 * Math.PI; angle += 2.0 * Math.PI / numEdges)
         {
            double x = radius * Math.cos(angle);
            double y = radius * Math.sin(angle);
            polygon.addVertex(x, y);
         }
         polygon.update();

         int leftEdge = leftRightNumExpected[0];
         int rightEdge = leftRightNumExpected[1];
         int expectedAnswer = leftRightNumExpected[3];

         int answer = ConvexPolygon2dCalculator.getMiddleIndexCounterClockwise(leftEdge, rightEdge, polygon);

         if (answer != expectedAnswer)
            throw new RuntimeException("leftEdge = " + leftEdge + ", rightEdge = " + rightEdge + ", numEdges = " + numEdges + ", expectedAnswer = "
                  + expectedAnswer + ", answer = " + answer);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testCombineOne()
   {
      Random random = new Random(1992L);

      ReferenceFrame zUpFrame = ReferenceFrame.constructARootFrame("someFrame");
      double xMinA = -100.0, xMaxA = 100.0, yMinA = -100.0, yMaxA = 100.0;
      double xMinB = 100.0, xMaxB = 300.0, yMinB = 100.0, yMaxB = 300.0;

      double xMin = Math.min(xMinA, xMinB);
      double xMax = Math.max(xMaxA, xMaxB);
      double yMin = Math.min(yMinA, yMinB);
      double yMax = Math.max(yMaxA, yMaxB);

      ArrayList<FramePoint2D> pointsA = ConvexPolygon2dTestHelpers.generateRandomRectangularFramePoints(random, zUpFrame, xMinA, xMaxA, yMinA, yMaxA, 10000);
      ArrayList<FramePoint2D> pointsB = ConvexPolygon2dTestHelpers.generateRandomCircularFramePoints(random, zUpFrame, xMinB, xMaxB, yMinB, yMaxB, 10000);

      FrameConvexPolygon2d polygonA = new FrameConvexPolygon2d(pointsA);
      ConvexPolygon2dTestHelpers.verifyPointsAreClockwise(polygonA);

      FrameConvexPolygon2d polygonB = new FrameConvexPolygon2d(pointsB);
      ConvexPolygon2dTestHelpers.verifyPointsAreClockwise(polygonB);

      FrameConvexPolygon2d polygonC = new FrameConvexPolygon2d(polygonA, polygonB);
      ConvexPolygon2dTestHelpers.verifyPointsAreClockwise(polygonC);

      for (FramePoint2D point : pointsA)
      {
         if (!polygonC.isPointInside(point))
         {
            throw new RuntimeException();
         }
      }

      for (FramePoint2D point : pointsB)
      {
         if (!polygonC.isPointInside(point))
         {
            throw new RuntimeException();
         }
      }

      if (PLOT_RESULTS)
      {
         FrameGeometryTestFrame testFrame = new FrameGeometryTestFrame(xMin, xMax, yMin, yMax);

         FrameGeometry2dPlotter plotter = testFrame.getFrameGeometry2dPlotter();
         plotter.setPolygonToCheckInside(polygonC);

         testFrame.addTestPoints(pointsA);
         testFrame.addTestPoints(pointsB);
         waitForButtonOrPause(testFrame);
      }

   }

   private void waitForButtonOrPause(FrameGeometryTestFrame testFrame)
   {
      if (WAIT_FOR_BUTTON_PUSH)
         testFrame.waitForButtonPush();
      else
         pauseOneSecond();
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetOutSideFacingOrthoNormalVectors()
   {
      ConvexPolygon2D polygon = createSomeValidPolygon();

      double delta = 5e-3;
      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
      {
         Vector2D normalVector = new Vector2D();
         ConvexPolygon2dCalculator.getEdgeNormal(i, normalVector, polygon);
         assertEquals(1.0, normalVector.length(), 1e-9);

         Point2DReadOnly startPoint = polygon.getVertex(i);
         Point2DReadOnly endPoint = polygon.getNextVertex(i);

         double dx = endPoint.getX() - startPoint.getX();
         double dy = endPoint.getY() - startPoint.getY();

         Vector2D lineDirection = (new Vector2D(dx, dy));
         double dot = lineDirection.dot(normalVector);
         assertEquals(0.0, dot, delta);

         // test if outsideFacing
         LineSegment2D lineSegment = new LineSegment2D(startPoint, endPoint);
         Point2D testPoint = lineSegment.midpoint();

         assertTrue(polygon.isPointInside(testPoint, 1e-15));

         normalVector.scale(1e-9);
         testPoint.add(normalVector);

         assertFalse(polygon.isPointInside(testPoint, 1e-15));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testApplyTransformWithTranslations()
   {
      Random random = new Random(1776L);
      RigidBodyTransform transform = new RigidBodyTransform();

      // pure translation:
      Vector3D translation = new Vector3D(random.nextDouble(), random.nextDouble(), 0.0);
      transform.setTranslation(translation);

      final int listSize = 50;

      ArrayList<Point2D> pointList = new ArrayList<Point2D>();
      for (int i = 0; i < listSize; i++)
      {
         pointList.add(new Point2D(random.nextDouble(), random.nextDouble()));
      }

      ConvexPolygon2D convexPolygon = new ConvexPolygon2D(pointList);
      ConvexPolygon2D returnPolygon = new ConvexPolygon2D(convexPolygon);

      returnPolygon.applyTransformAndProjectToXYPlane(transform);

      if (PLOT_RESULTS)
      {
         ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
         FrameGeometryTestFrame testFrame = new FrameGeometryTestFrame(-1.0, 2.0, -1.0, 2.0);
         FrameGeometry2dPlotter plotter = testFrame.getFrameGeometry2dPlotter();
         plotter.addPolygon(new FrameConvexPolygon2d(worldFrame, convexPolygon));
         plotter.addPolygon(new FrameConvexPolygon2d(worldFrame, returnPolygon));
         waitForButtonOrPause(testFrame);
      }

      for (int i = 0; i < convexPolygon.getNumberOfVertices(); i++)
      {
         assertEquals("Translation not handled correctly", convexPolygon.getVertex(i).getX() + translation.getX(), returnPolygon.getVertex(i).getX(), 1e-7);
         assertEquals("Translation not handled correctly", convexPolygon.getVertex(i).getY() + translation.getY(), returnPolygon.getVertex(i).getY(), 1e-7);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testApplyTransformWithRotation()
   {
      Random random = new Random(1984L);
      RigidBodyTransform transform = new RigidBodyTransform();

      // pure translation:
      double yaw = 0.4;
      Vector3D eulerAngles = new Vector3D(0.0, 0.0, yaw);
      transform.setRotationEulerAndZeroTranslation(eulerAngles);

      final int listSize = 8;

      ArrayList<Point2D> pointList = new ArrayList<Point2D>();
      for (int i = 0; i < listSize; i++)
      {
         pointList.add(new Point2D(random.nextDouble(), random.nextDouble()));
      }

      ConvexPolygon2D convexPolygon = new ConvexPolygon2D(pointList);
      ConvexPolygon2D returnPolygon = new ConvexPolygon2D(convexPolygon);
      returnPolygon.applyTransformAndProjectToXYPlane(transform);

      if (PLOT_RESULTS)
      {
         ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
         FrameGeometryTestFrame testFrame = new FrameGeometryTestFrame(-1.0, 2.0, -1.0, 2.0);
         FrameGeometry2dPlotter plotter = testFrame.getFrameGeometry2dPlotter();
         plotter.addPolygon(new FrameConvexPolygon2d(worldFrame, convexPolygon));
         plotter.addPolygon(new FrameConvexPolygon2d(worldFrame, returnPolygon));
         waitForButtonOrPause(testFrame);
      }

      for (int i = 0; i < convexPolygon.getNumberOfVertices(); i++)
      {
         double expectedX = convexPolygon.getVertex(i).getX() * Math.cos(yaw) - convexPolygon.getVertex(i).getY() * Math.sin(yaw);
         double expectedY = convexPolygon.getVertex(i).getX() * Math.sin(yaw) + convexPolygon.getVertex(i).getY() * Math.cos(yaw);

         assertEquals("Rotation not handled correctly", expectedX, returnPolygon.getVertex(i).getX(), 1e-7);
         assertEquals("Rotation not handled correctly", expectedY, returnPolygon.getVertex(i).getY(), 1e-7);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetClosestEdge()
   {
      ConvexPolygon2D polygon = createSomeValidPolygon();
      Point2D point = new Point2D(1.314592, 6.0221415); // Useless Riddle: first number is easy, second one multiplied by 1e23 is called?

      assertFalse(polygon.isPointInside(point));

      LineSegment2D closestEdge = polygon.getClosestEdgeCopy(point);

      Point2D closestVertex = polygon.getClosestVertexCopy(point);

      int otherEdgeVertexIndex = 0;
      boolean isClosestVertexPartOfClosestEdge = false;
      for (int i = 0; i < 2; i++)
      {
         Point2DReadOnly segmentVertex = closestEdge.getEndpointsCopy()[i];
         if (arePointsAtExactlyEqualPosition(closestVertex, segmentVertex))
         {
            isClosestVertexPartOfClosestEdge = true;
            if (i == 0)
               otherEdgeVertexIndex = 1;
         }
      }

      assertTrue(isClosestVertexPartOfClosestEdge);

      int numberOfVertices = polygon.getNumberOfVertices();
      int prevIndex = numberOfVertices - 1;
      int nextIndex = 0;
      for (int i = 0; i < numberOfVertices; i++)
      {
         Point2DReadOnly currentPoint = polygon.getVertex(i);
         if (arePointsAtExactlyEqualPosition(closestVertex, currentPoint))
         {
            if (i < numberOfVertices - 1)
               nextIndex = i + 1;

            break;
         }

         prevIndex = i;
      }

      Point2DReadOnly[] neighbourPoints = new Point2D[2];
      neighbourPoints[0] = polygon.getVertex(prevIndex);
      neighbourPoints[1] = polygon.getVertex(nextIndex);
      int neighbourIndex = 1;
      int wrongNeighbourIndex = 0;
      boolean isClosestEdgeVertexThatIsNotClosestVertexNeighbourOfClosestVertex = false;
      for (int i = 0; i < 2; i++)
      {
         Point2DReadOnly neighbourPoint = neighbourPoints[i];
         Point2DReadOnly closestEdgeVertexThatIsNotClosest = closestEdge.getEndpointsCopy()[otherEdgeVertexIndex];
         if (arePointsAtExactlyEqualPosition(closestEdgeVertexThatIsNotClosest, neighbourPoint))
         {
            isClosestEdgeVertexThatIsNotClosestVertexNeighbourOfClosestVertex = true;
            neighbourIndex = i;
            if (i == 0)
               wrongNeighbourIndex = 1;
         }

      }

      assertTrue(isClosestEdgeVertexThatIsNotClosestVertexNeighbourOfClosestVertex);

      Line2D segmentLine = new Line2D(neighbourPoints[neighbourIndex], closestVertex);
      Line2D otherLine = new Line2D(neighbourPoints[wrongNeighbourIndex], closestVertex);

      Line2D interiorBiSector = segmentLine.interiorBisector(otherLine);

      boolean isPointBehindLine = interiorBiSector.isPointBehindLine(point);
      boolean isOtherEdgeVertexBehindLine = interiorBiSector.isPointBehindLine(closestEdge.getEndpointsCopy()[otherEdgeVertexIndex]);

      // TODO this may fail, if the point is really close to the "true" biSecotor and the biSector float calc error moves it to the wrong side...
      // TODO edge cases unsolved...
      assertEquals(isPointBehindLine, isOtherEdgeVertexBehindLine);

   }

   private boolean arePointsAtExactlyEqualPosition(Point2DReadOnly point1, Point2DReadOnly point2)
   {
      return ((point1.getX() == point2.getX()) && (point1.getY() == point2.getY()));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.1)
   @Test(timeout = 30000)
   public void testArea()
   {
      Random random = new Random(1888L);

      ReferenceFrame zUpFrame = ReferenceFrame.constructARootFrame("someFrame");
      double xMin1 = 0.0, xMax1 = 1.0, yMin1 = 0.0, yMax1 = 1.0;

      int numberOfPoints = 10000;
      ArrayList<FramePoint2D> points = ConvexPolygon2dTestHelpers.generateRandomCircularFramePoints(random, zUpFrame, xMin1, xMax1, yMin1, yMax1,
                                                                                                    numberOfPoints);

      FrameConvexPolygon2d polygon = new FrameConvexPolygon2d(points);
      double computedArea = polygon.getArea();

      double maximumPossibleArea = Math.PI * 0.5 * 0.5;

      assertTrue(maximumPossibleArea > computedArea);
      assertEquals("computedArea = " + computedArea + ", maximumPossibleArea = " + maximumPossibleArea, maximumPossibleArea, computedArea, 0.02);
   }

   private ConvexPolygon2D createSomeValidPolygon()
   {
      double[][] polygonPoints = new double[][] {{-0.05107802536335158, 0.04155594197133163}, {-0.05052044462374434, 0.1431544119584275},
            {0.12219695435431863, 0.14220652470109518}, {0.12219695435431865, -0.041946248489056696}, {0.12163937361471142, -0.1435447184761526},
            {-0.05107802536335154, -0.14259683121882027}};

      ConvexPolygon2D polygon = new ConvexPolygon2D(polygonPoints);

      return polygon;
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testLargeHullWithIntersections()
   {
      Random random = new Random(1776L);

      ReferenceFrame zUpFrame = ReferenceFrame.constructARootFrame("someFrame");
      double xMin = -100.0, xMax = 100.0, yMin = -100.0, yMax = 100.0;

      ArrayList<FramePoint2D> points = ConvexPolygon2dTestHelpers.generateRandomCircularFramePoints(random, zUpFrame, xMin, xMax, yMin, yMax, 100);

      FrameConvexPolygon2d polygon = new FrameConvexPolygon2d(points);
      ConvexPolygon2dTestHelpers.verifyPointsAreClockwise(polygon);

      // Generate random lines and test the intersections:

      int numberOfTests = 1000;

      ArrayList<FrameLine2d> nonIntersectingLines = new ArrayList<FrameLine2d>();
      ArrayList<FrameLine2d> intersectingLines = new ArrayList<FrameLine2d>();

      ArrayList<FrameLineSegment2d> nonIntersectingLineSegments = new ArrayList<FrameLineSegment2d>();
      ArrayList<FrameLineSegment2d> intersectingLineSegments = new ArrayList<FrameLineSegment2d>();

      ArrayList<FramePoint2D> intersectingPoints = new ArrayList<FramePoint2D>();

      for (int i = 0; i < numberOfTests; i++)
      {
         FrameLine2d testLine = FrameLine2d.generateRandomFrameLine2d(random, zUpFrame, 2.0 * xMin, 2.0 * xMax, 2.0 * yMin, 2.0 * yMax);
         FramePoint2D[] intersectionsWithLine = polygon.intersectionWith(testLine);

         if (intersectionsWithLine == null)
            nonIntersectingLines.add(testLine);
         else
         {
            intersectingLines.add(testLine);

            intersectingPoints.add(intersectionsWithLine[0]);
            if (intersectionsWithLine.length > 1)
               intersectingPoints.add(intersectionsWithLine[1]);
         }

         FrameLineSegment2d testLineSegment = FrameLineSegment2d.generateRandomFrameLineSegment2d(random, zUpFrame, 2.0 * xMin, 2.0 * xMax, 2.0 * yMin,
                                                                                                  2.0 * yMax);

         FramePoint2D[] intersectionsWithLineSegment = polygon.intersectionWith(testLineSegment);

         if (intersectionsWithLineSegment == null)
            nonIntersectingLineSegments.add(testLineSegment);
         else
         {
            intersectingLineSegments.add(testLineSegment);
            intersectingPoints.add(intersectionsWithLineSegment[0]);
            if (intersectionsWithLineSegment.length > 1)
               intersectingPoints.add(intersectionsWithLineSegment[1]);
         }
      }

      ConvexPolygon2dTestHelpers.verifyLinesIntersectPolygon(polygon, intersectingLines);
      ConvexPolygon2dTestHelpers.verifyLineSegmentsIntersectPolygon(polygon, intersectingLineSegments);
      ConvexPolygon2dTestHelpers.verifyLinesDoNotIntersectPolygon(polygon, nonIntersectingLines);
      ConvexPolygon2dTestHelpers.verifyLineSegmentsDoNotIntersectPolygon(polygon, nonIntersectingLineSegments);

      if (PLOT_RESULTS)
      {
         FrameGeometryTestFrame testFrame = new FrameGeometryTestFrame(xMin, xMax, yMin, yMax);

         FrameGeometry2dPlotter plotter = testFrame.getFrameGeometry2dPlotter();
         plotter.setPolygonToCheckInside(polygon);

         plotter.addFrameLines2d(intersectingLines, Color.green);
         plotter.addFrameLines2d(nonIntersectingLines, Color.red);

         plotter.addFrameLineSegments2d(intersectingLineSegments, Color.green);
         plotter.addFrameLineSegments2d(nonIntersectingLineSegments, Color.red);

         testFrame.addTestPoints(intersectingPoints);

         waitForButtonOrPause(testFrame);
      }

   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testOrthogonalProjectionPointConvexPolygon2d()
   {
      ArrayList<Point2D> pointList = new ArrayList<Point2D>();
      ConvexPolygon2D convexPolygon;
      Point2D testPoint = new Point2D();

      for (int i = 4; i < 10; i++) // polygon sizes
      {
         pointList.clear();

         for (int j = 0; j < i; j++) // points from which the polygon is constructed
         {
            pointList.add(new Point2D(random.nextDouble(), random.nextDouble()));
         }

         convexPolygon = new ConvexPolygon2D(pointList);

         for (int k = 0; k < 100; k++) // testPoints for isPointInside()
         {
            testPoint.set(random.nextDouble(), random.nextDouble());

            if (VERBOSE)
            {
               if ((i == 9) && (k == 69))
               {
                  System.out.println("convexPolygon = " + convexPolygon);
                  System.out.println("testPoint = " + testPoint);
               }
            }

            Point2D projectedPoint = convexPolygon.orthogonalProjectionCopy(testPoint);

            if (convexPolygon.isPointInside(testPoint))
               assertNull(projectedPoint);
            else
               assertTrue("Projected point was not inside the polygon for point\n" + projectedPoint + "\nand convex polygon \n" + convexPolygon,
                          convexPolygon.isPointInside(projectedPoint, 1.0E-10));
         }
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAllMethodsForPolygonWithOnePoint()
   {
      int numberOfTrials = 100;
      double epsilon = 1e-7;
      Random random = new Random(756483920L);

      for (int i = 0; i < numberOfTrials; i++)
      {
         ArrayList<Point2D> points = new ArrayList<Point2D>();
         Point2D pointThatDefinesThePolygon = new Point2D(random.nextDouble(), random.nextDouble());
         points.add(pointThatDefinesThePolygon);
         ConvexPolygon2D polygonWithOnePoint = new ConvexPolygon2D(points);
         points.clear();
         Point2D pointThatDefinesAnotherPolygon = new Point2D(random.nextDouble(), random.nextDouble());
         points.add(pointThatDefinesAnotherPolygon);
         ConvexPolygon2D anotherPolygonWithOnePoint = new ConvexPolygon2D(points);
         points.clear();
         points.add(new Point2D(random.nextDouble(), random.nextDouble()));
         points.add(new Point2D(random.nextDouble(), random.nextDouble()));
         points.add(new Point2D(random.nextDouble(), random.nextDouble()));
         ConvexPolygon2D sparePolygon = new ConvexPolygon2D(points);
         Point2D arbitraryPoint0 = new Point2D(random.nextDouble(), random.nextDouble());
         Point2D arbitraryPoint1 = new Point2D(random.nextDouble(), random.nextDouble());
         Line2D arbitraryLine = new Line2D(arbitraryPoint0, arbitraryPoint1);
         LineSegment2D arbitraryLineSegment = new LineSegment2D(arbitraryPoint0, arbitraryPoint1);

         assertEquals(pointThatDefinesThePolygon.distance(arbitraryPoint0),
                      polygonWithOnePoint.getClosestVertexCopy(arbitraryPoint0).distance(arbitraryPoint0), epsilon);
         assertEquals(0.0, polygonWithOnePoint.getArea(), epsilon);
         assertTrue(polygonWithOnePoint.getBoundingBoxCopy().getMaxPoint().equals(pointThatDefinesThePolygon));
         assertTrue(polygonWithOnePoint.getBoundingBoxCopy().getMinPoint().equals(pointThatDefinesThePolygon));
         assertTrue(polygonWithOnePoint.getCentroid().equals(pointThatDefinesThePolygon));
         assertEquals(1, polygonWithOnePoint.getNumberOfVertices());
         assertTrue(polygonWithOnePoint.getVertex(0).equals(pointThatDefinesThePolygon));
         assertTrue(polygonWithOnePoint.getClosestEdgeCopy(arbitraryPoint0) == null);
         assertTrue(polygonWithOnePoint.getClosestEdgeIndex(arbitraryPoint0) == -1);
         assertTrue(polygonWithOnePoint.getClosestVertexCopy(arbitraryLine).equals(pointThatDefinesThePolygon));
         assertTrue(polygonWithOnePoint.getClosestVertexCopy(arbitraryPoint0).equals(pointThatDefinesThePolygon));
         assertEquals(1, polygonWithOnePoint.getNumberOfVertices());
         assertTrue(polygonWithOnePoint.getVertexCCW(0).equals(pointThatDefinesThePolygon));
         assertTrue(ConvexPolygon2dCalculator.getIntersectingEdgesCopy(arbitraryLine, polygonWithOnePoint) == null);
         assertTrue(polygonWithOnePoint.getVertex(polygonWithOnePoint.lineOfSightStartIndex(arbitraryPoint0)).equals(pointThatDefinesThePolygon));
         assertTrue(polygonWithOnePoint.getVertex(polygonWithOnePoint.lineOfSightEndIndex(arbitraryPoint0)).equals(pointThatDefinesThePolygon));
         assertTrue(polygonWithOnePoint.getCentroid().equals(pointThatDefinesThePolygon));
         assertEquals(1, polygonWithOnePoint.getNumberOfVertices());
         assertEquals(1, polygonWithOnePoint.getNumberOfVertices());
         assertTrue(polygonWithOnePoint.getVertex(0).equals(pointThatDefinesThePolygon));
         assertTrue(ConvexPolygonTools.computeIntersectionOfPolygons(polygonWithOnePoint, sparePolygon) == null);
         assertTrue(polygonWithOnePoint.intersectionWith(arbitraryLine) == null);
         assertFalse(polygonWithOnePoint.isPointInside(arbitraryPoint0));
         assertFalse(ConvexPolygon2dCalculator.isPolygonInside(sparePolygon, polygonWithOnePoint));
         assertEquals(0, polygonWithOnePoint.getMaxXMaxYIndex());
         assertEquals(0, polygonWithOnePoint.getMaxXMinYIndex());
         assertEquals(0, polygonWithOnePoint.getMinXMaxYIndex());
         assertEquals(0, polygonWithOnePoint.getMinXMinYIndex());
         assertTrue(polygonWithOnePoint.orthogonalProjectionCopy(arbitraryPoint0).equals(pointThatDefinesThePolygon));
         assertTrue(polygonWithOnePoint.pointIsOnPerimeter(pointThatDefinesThePolygon));
         assertFalse(polygonWithOnePoint.pointIsOnPerimeter(arbitraryPoint0));

         ConvexPolygon2D polygonTranslation = polygonWithOnePoint.translateCopy(arbitraryPoint0);
         assertEquals(1, polygonTranslation.getNumberOfVertices());
         Point2D pointTranslation = new Point2D(pointThatDefinesThePolygon);
         pointTranslation.add(arbitraryPoint0);
         assertEquals(polygonTranslation.getVertex(0), pointTranslation);

         ConvexPolygon2D combinedPolygons = new ConvexPolygon2D(polygonWithOnePoint, anotherPolygonWithOnePoint);
         assertEquals(2, combinedPolygons.getNumberOfVertices());
         Point2DReadOnly point0 = combinedPolygons.getVertex(0);
         Point2DReadOnly point1 = combinedPolygons.getVertex(1);
         assertEqualsInEitherOrder(pointThatDefinesThePolygon, pointThatDefinesAnotherPolygon, point0, point1);

         ConvexPolygon2dAndConnectingEdges combinedDisjointPolygons = ConvexPolygonTools.combineDisjointPolygons(polygonWithOnePoint,
                                                                                                                 anotherPolygonWithOnePoint);
         assertEquals(2, combinedDisjointPolygons.getConvexPolygon2d().getNumberOfVertices());
         point0 = combinedDisjointPolygons.getConvexPolygon2d().getVertex(0);
         point1 = combinedDisjointPolygons.getConvexPolygon2d().getVertex(1);
         assertEqualsInEitherOrder(pointThatDefinesThePolygon, pointThatDefinesAnotherPolygon, point0, point1);

         assertTrue(ConvexPolygonTools.computeIntersectionOfPolygons(polygonWithOnePoint, anotherPolygonWithOnePoint, new ConvexPolygon2D()) == false);
         ConvexPolygon2D intersection = new ConvexPolygon2D();
         ConvexPolygonTools.computeIntersectionOfPolygons(polygonWithOnePoint, polygonWithOnePoint, intersection);
         assertEquals(1, intersection.getNumberOfVertices());
         ConvexPolygonTools.computeIntersectionOfPolygons(polygonWithOnePoint, polygonWithOnePoint, intersection);
         assertTrue(intersection.getVertex(0).equals(pointThatDefinesThePolygon));
         assertTrue(polygonWithOnePoint.intersectionWith(arbitraryLineSegment) == null);
         assertTrue(polygonWithOnePoint.intersectionWith(new LineSegment2D(pointThatDefinesThePolygon, arbitraryPoint0))[0].equals(pointThatDefinesThePolygon));

         ConvexPolygonScaler shrinker = new ConvexPolygonScaler();
         ConvexPolygon2D shrunkenOnePointPolygon = new ConvexPolygon2D();

         shrinker.scaleConvexPolygon(polygonWithOnePoint, random.nextDouble(), shrunkenOnePointPolygon);

         assertTrue(shrunkenOnePointPolygon.epsilonEquals(polygonWithOnePoint, 1e-7));

         assertEquals(1, ConvexPolygonTools.shrinkInto(sparePolygon, arbitraryPoint0, polygonWithOnePoint).getNumberOfVertices());
         assertTrue(ConvexPolygonTools.shrinkInto(sparePolygon, arbitraryPoint0, polygonWithOnePoint).getVertex(0).equals(pointThatDefinesThePolygon));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetSignedDistance1()
   {
      // single point polygon
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(new Point2D(0.0, 0.0));
      polygon.update();

      Point2D point = new Point2D(2.5, 1.0);
      double distance = polygon.signedDistance(point);
      assertDistanceCorrect(Math.sqrt(2.5 * 2.5 + 1.0 * 1.0), distance);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetSignedDistance2()
   {
      // line polygon
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(new Point2D(0.0, 0.0));
      polygon.addVertex(new Point2D(1.0, 0.0));
      polygon.update();

      Point2D point1 = new Point2D(2.5, 1.0);
      double distance1 = polygon.signedDistance(point1);
      assertDistanceCorrect(Math.sqrt(1.5 * 1.5 + 1.0 * 1.0), distance1);

      Point2D point2 = new Point2D(0.5, 1.0);
      double distance2 = polygon.signedDistance(point2);
      assertDistanceCorrect(1.0, distance2);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetSignedDistance3()
   {
      // triangle polygon
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(new Point2D(0.0, 0.0));
      polygon.addVertex(new Point2D(10.0, 0.0));
      polygon.addVertex(new Point2D(0.0, 10.0));
      polygon.update();

      Point2D point1 = new Point2D(10.0, 10.0);
      double distance1 = polygon.signedDistance(point1);
      assertDistanceCorrect(5.0 * Math.sqrt(2.0), distance1);

      Point2D point2 = new Point2D(1.2, 1.1);
      double distance2 = polygon.signedDistance(point2);
      assertDistanceCorrect(-1.1, distance2);

      Point2D point3 = new Point2D(0.05, 9.8);
      double distance3 = polygon.signedDistance(point3);
      assertDistanceCorrect(-0.05, distance3);

      Point2D point4 = new Point2D(9.8, 0.15);
      double distance4 = polygon.signedDistance(point4);
      assertDistanceCorrect(-0.5 * Math.sqrt(0.05 * 0.05 * 2.0), distance4);

      Point2D point5 = new Point2D(5.0, -0.15);
      double distance5 = polygon.signedDistance(point5);
      assertDistanceCorrect(0.15, distance5);

      Point2D point6 = new Point2D(15.0, -0.15);
      double distance6 = polygon.signedDistance(point6);
      assertDistanceCorrect(Math.sqrt(5.0 * 5.0 + 0.15 * 0.15), distance6);
   }

   private static void assertDistanceCorrect(double expected, double actual)
   {
      assertEquals("Distance does not equal expected.", expected, actual, epsilon);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testAllMethodsForPolygonWithTwoPoints()
   {
      int numberOfTrials = 100;
      double epsilon = 1e-7;
      Random random = new Random(756483920L);

      for (int i = 0; i < numberOfTrials; i++)
      {
         ArrayList<Point2D> points = new ArrayList<Point2D>();
         Point2D pointThatDefinesThePolygon0 = new Point2D(random.nextDouble(), random.nextDouble());
         Point2D pointThatDefinesThePolygon1 = new Point2D(random.nextDouble(), random.nextDouble());
         LineSegment2D lineSegmentThatDefinesThePolygon = new LineSegment2D(pointThatDefinesThePolygon0, pointThatDefinesThePolygon1);
         points.add(pointThatDefinesThePolygon0);
         points.add(pointThatDefinesThePolygon1);
         ConvexPolygon2D polygonWithTwoPoints = new ConvexPolygon2D(points);
         points.clear();
         Point2D pointThatDefinesAnotherPolygon = new Point2D(random.nextDouble(), random.nextDouble());
         points.add(pointThatDefinesAnotherPolygon);
         ConvexPolygon2D polygonWithOnePointx = new ConvexPolygon2D(points);
         points.clear();
         points.add(new Point2D(random.nextDouble(), random.nextDouble()));
         points.add(new Point2D(random.nextDouble(), random.nextDouble()));
         points.add(new Point2D(random.nextDouble(), random.nextDouble()));
         ConvexPolygon2D sparePolygon = new ConvexPolygon2D(points);
         Point2D arbitraryPoint0 = new Point2D(random.nextDouble(), random.nextDouble());
         Point2D arbitraryPoint1 = new Point2D(random.nextDouble(), random.nextDouble());
         Line2D arbitraryLine = new Line2D(arbitraryPoint0, arbitraryPoint1);
         LineSegment2D arbitraryLineSegment = new LineSegment2D(arbitraryPoint0, arbitraryPoint1);

         // one line tests
         assertEquals(Math.min(pointThatDefinesThePolygon0.distance(arbitraryPoint0), pointThatDefinesThePolygon1.distance(arbitraryPoint0)),
                      polygonWithTwoPoints.getClosestVertexCopy(arbitraryPoint0).distance(arbitraryPoint0), epsilon);
         assertEquals(0.0, polygonWithTwoPoints.getArea(), epsilon);
         Point2D minPoint = new Point2D(Math.min(pointThatDefinesThePolygon0.getX(), pointThatDefinesThePolygon1.getX()),
                                        Math.min(pointThatDefinesThePolygon0.getY(), pointThatDefinesThePolygon1.getY()));
         Point2D maxPoint = new Point2D(Math.max(pointThatDefinesThePolygon0.getX(), pointThatDefinesThePolygon1.getX()),
                                        Math.max(pointThatDefinesThePolygon0.getY(), pointThatDefinesThePolygon1.getY()));
         assertTrue(polygonWithTwoPoints.getBoundingBoxCopy().getMinPoint().equals(minPoint));
         assertTrue(polygonWithTwoPoints.getBoundingBoxCopy().getMaxPoint().equals(maxPoint));
         assertTrue(polygonWithTwoPoints.getCentroid().equals(lineSegmentThatDefinesThePolygon.midpoint()));
         assertEquals(2, polygonWithTwoPoints.getNumberOfVertices());
         assertEqualsInEitherOrder(pointThatDefinesThePolygon0, pointThatDefinesThePolygon1, polygonWithTwoPoints.getVertex(0),
                                   polygonWithTwoPoints.getVertex(1));
         assertFalse(polygonWithTwoPoints.isPointInside(arbitraryPoint0));
         assertFalse(ConvexPolygon2dCalculator.isPolygonInside(sparePolygon, polygonWithTwoPoints));
         assertEquals(2, polygonWithTwoPoints.getNumberOfVertices());
         assertTrue(polygonWithTwoPoints.getCentroid().getX() == 0.5 * (pointThatDefinesThePolygon0.getX() + pointThatDefinesThePolygon1.getX()));
         assertTrue(polygonWithTwoPoints.getCentroid().getY() == 0.5 * (pointThatDefinesThePolygon0.getY() + pointThatDefinesThePolygon1.getY()));
         assertEquals(2, polygonWithTwoPoints.getNumberOfVertices());

         // getClosestEdge
         Point2DReadOnly[] closestEdgeEndpoints = polygonWithTwoPoints.getClosestEdgeCopy(arbitraryPoint0).getEndpointsCopy();
         assertEqualsInEitherOrder(closestEdgeEndpoints[0], closestEdgeEndpoints[1], pointThatDefinesThePolygon0, pointThatDefinesThePolygon1);

         // getClosestEdgeVertexIndicesInClockwiseOrderedList
         int edgeIndex = polygonWithTwoPoints.getClosestEdgeIndex(arbitraryPoint0);
         assertEqualsInEitherOrder(edgeIndex, polygonWithTwoPoints.getNextVertexIndex(edgeIndex), 0, 1);

         // getCounterClockwiseOrderedListOfPointsCopy
         assertEqualsInEitherOrder(polygonWithTwoPoints.getVertexCCW(0), polygonWithTwoPoints.getVertexCCW(1), pointThatDefinesThePolygon0,
                                   pointThatDefinesThePolygon1);

         // getLineOfSightVertices
         Point2DReadOnly[] lineOfSightPoints = new Point2D[2];
         lineOfSightPoints[0] = polygonWithTwoPoints.getVertex(polygonWithTwoPoints.lineOfSightStartIndex(arbitraryPoint0));
         lineOfSightPoints[1] = polygonWithTwoPoints.getVertex(polygonWithTwoPoints.lineOfSightEndIndex(arbitraryPoint0));
         assertEqualsInEitherOrder(lineOfSightPoints[0], lineOfSightPoints[1], pointThatDefinesThePolygon0, pointThatDefinesThePolygon1);

         // orthoganolProjectionCopy
         Point2D expectedProjection = lineSegmentThatDefinesThePolygon.orthogonalProjectionCopy(arbitraryPoint0);
         Point2D actualProjection = polygonWithTwoPoints.orthogonalProjectionCopy(arbitraryPoint0);
         assertTrue(expectedProjection.epsilonEquals(actualProjection, epsilon));

         // getClosestVertexCopy
         Point2D closestVertexToLine = polygonWithTwoPoints.getClosestVertexCopy(arbitraryLine);
         if (arbitraryLine.distance(pointThatDefinesThePolygon0) < arbitraryLine.distance(pointThatDefinesThePolygon1))
            assertEquals(closestVertexToLine, pointThatDefinesThePolygon0);
         else
            assertEquals(closestVertexToLine, pointThatDefinesThePolygon1);

         Point2D closestVertexToPoint = polygonWithTwoPoints.getClosestVertexCopy(arbitraryPoint0);
         if (arbitraryPoint0.distance(pointThatDefinesThePolygon0) < arbitraryPoint0.distance(pointThatDefinesThePolygon1))
            assertEquals(closestVertexToPoint, pointThatDefinesThePolygon0);
         else
            assertEquals(closestVertexToPoint, pointThatDefinesThePolygon1);

         // getIntersectingEdges
         LineSegment2D[] intersectingEdges = ConvexPolygon2dCalculator.getIntersectingEdgesCopy(arbitraryLine, polygonWithTwoPoints);
         boolean isLineAbovePoint0 = ((pointThatDefinesThePolygon0.getX() - arbitraryLine.getPoint().getX()) * arbitraryLine.slope()
               + arbitraryLine.getPoint().getY()) >= pointThatDefinesThePolygon0.getY();
         boolean isLineAbovePoint1 = ((pointThatDefinesThePolygon1.getX() - arbitraryLine.getPoint().getX()) * arbitraryLine.slope()
               + arbitraryLine.getPoint().getY()) >= pointThatDefinesThePolygon1.getY();
         boolean lineCrossesThroughPolygon = isLineAbovePoint0 ^ isLineAbovePoint1;

         if (!lineCrossesThroughPolygon)
         {
            assertTrue(intersectingEdges == null);
         }
         else
         {
            for (int j : new int[] {0, 1})
            {
               Point2DReadOnly[] endPoints = intersectingEdges[j].getEndpointsCopy();
               assertEqualsInEitherOrder(endPoints[0], endPoints[1], pointThatDefinesThePolygon0, pointThatDefinesThePolygon1);
            }
         }

         // getStartingFromLeftMostClockwiseOrderedListOfPointsCopy
         assertEquals(2, polygonWithTwoPoints.getNumberOfVertices());
         Point2D leftPoint, rightPoint;
         if (pointThatDefinesThePolygon0.getX() <= pointThatDefinesThePolygon1.getX())
         {
            leftPoint = pointThatDefinesThePolygon0;
            rightPoint = pointThatDefinesThePolygon1;
         }
         else
         {
            leftPoint = pointThatDefinesThePolygon1;
            rightPoint = pointThatDefinesThePolygon0;
         }

         assertTrue((leftPoint.getX() == polygonWithTwoPoints.getVertex(0).getX()) && (leftPoint.getY() == polygonWithTwoPoints.getVertex(0).getY()));
         assertTrue((rightPoint.getX() == polygonWithTwoPoints.getVertex(1).getX()) && (rightPoint.getY() == polygonWithTwoPoints.getVertex(1).getY()));

         // maxXMaxYPointCopy, maxXMinYPointCopy, minXMaxYPointCopy, minXMinYPointCopy
         Point2D maxXPoint, minXPoint;
         if (pointThatDefinesThePolygon0.getX() > pointThatDefinesThePolygon1.getX())
         {
            maxXPoint = pointThatDefinesThePolygon0;
            minXPoint = pointThatDefinesThePolygon1;
         }
         else
         {
            maxXPoint = pointThatDefinesThePolygon1;
            minXPoint = pointThatDefinesThePolygon0;
         }

         assertTrue(polygonWithTwoPoints.getVertex(polygonWithTwoPoints.getMaxXMaxYIndex()).equals(maxXPoint));
         assertTrue(polygonWithTwoPoints.getVertex(polygonWithTwoPoints.getMaxXMinYIndex()).equals(maxXPoint));
         assertTrue(polygonWithTwoPoints.getVertex(polygonWithTwoPoints.getMinXMaxYIndex()).equals(minXPoint));
         assertTrue(polygonWithTwoPoints.getVertex(polygonWithTwoPoints.getMinXMinYIndex()).equals(minXPoint));

         // intersectionWith
         Point2D[] expectedIntersectionWithSparePolygon = sparePolygon.intersectionWith(new LineSegment2D(pointThatDefinesThePolygon0,
                                                                                                          pointThatDefinesThePolygon1));
         ConvexPolygon2D actualIntersectionWithSparePolygon = ConvexPolygonTools.computeIntersectionOfPolygons(sparePolygon, polygonWithTwoPoints);

         if (expectedIntersectionWithSparePolygon == null)
         {
            assertTrue(actualIntersectionWithSparePolygon == null);
         }
         else if (expectedIntersectionWithSparePolygon.length == 1)
         {
            assertTrue(actualIntersectionWithSparePolygon.getNumberOfVertices() == 1);
            assertTrue(expectedIntersectionWithSparePolygon[0].epsilonEquals(actualIntersectionWithSparePolygon.getVertex(0), epsilon));
         }
         else if (expectedIntersectionWithSparePolygon.length == 2)
         {
            assertTrue(actualIntersectionWithSparePolygon.getNumberOfVertices() == 2);
            assertEqualsInEitherOrder(expectedIntersectionWithSparePolygon[0], expectedIntersectionWithSparePolygon[1],
                                      actualIntersectionWithSparePolygon.getVertex(0), actualIntersectionWithSparePolygon.getVertex(1));
         }
         else
         {
            fail();
         }

         // pointIsOnPerimeter
         double randomFraction = random.nextDouble();
         Point2D scaledPoint0 = new Point2D(pointThatDefinesThePolygon0);
         Point2D scaledPoint1 = new Point2D(pointThatDefinesThePolygon1);
         scaledPoint0.scale(randomFraction);
         scaledPoint1.scale(1 - randomFraction);
         Point2D randomLinearCombination = new Point2D();
         randomLinearCombination.add(scaledPoint0, scaledPoint1);
         assertTrue(polygonWithTwoPoints.pointIsOnPerimeter(randomLinearCombination));

         // STATIC METHODS

         // translateCopy
         ConvexPolygon2D polygonTranslation = polygonWithTwoPoints.translateCopy(arbitraryPoint0);
         assertEquals(2, polygonTranslation.getNumberOfVertices());
         Point2D pointTranslation0 = new Point2D(pointThatDefinesThePolygon0);
         Point2D pointTranslation1 = new Point2D(pointThatDefinesThePolygon1);
         pointTranslation0.add(arbitraryPoint0);
         pointTranslation1.add(arbitraryPoint0);
         assertEqualsInEitherOrder(polygonTranslation.getVertex(0), polygonTranslation.getVertex(1), pointTranslation0, pointTranslation1);

         // combinePolygons
         ConvexPolygon2D combinedPolygons = new ConvexPolygon2D(polygonWithTwoPoints, polygonWithOnePointx);
         assertEquals(3, combinedPolygons.getNumberOfVertices());
         Point2DReadOnly point0 = combinedPolygons.getVertex(0);
         Point2DReadOnly point1 = combinedPolygons.getVertex(1);
         Point2DReadOnly point2 = combinedPolygons.getVertex(2);
         assertEqualsInAnyOrder(point0, point1, point2, pointThatDefinesThePolygon0, pointThatDefinesThePolygon1, pointThatDefinesAnotherPolygon);

         // computeIntersectionOfPolygons
         ConvexPolygon2D polygonIntersection = new ConvexPolygon2D();
         boolean success = ConvexPolygonTools.computeIntersectionOfPolygons(polygonWithTwoPoints, sparePolygon, polygonIntersection);
         if (!success)
            assertTrue(sparePolygon.intersectionWith(lineSegmentThatDefinesThePolygon) == null);
         else if (polygonIntersection.getNumberOfVertices() == 1)
            assertTrue(sparePolygon.intersectionWith(lineSegmentThatDefinesThePolygon)[0].epsilonEquals(polygonIntersection.getVertex(0), epsilon));
         else if (polygonIntersection.getNumberOfVertices() == 2)
            assertEqualsInEitherOrder(sparePolygon.intersectionWith(lineSegmentThatDefinesThePolygon)[0],
                                      sparePolygon.intersectionWith(lineSegmentThatDefinesThePolygon)[1], polygonIntersection.getVertex(0),
                                      polygonIntersection.getVertex(1));
         else
            fail();

         // intersection
         Point2D[] intersection = polygonWithTwoPoints.intersectionWith(arbitraryLineSegment);
         if (intersection == null)
            assertTrue(arbitraryLineSegment.intersectionWith(lineSegmentThatDefinesThePolygon) == null);
         else if (intersection.length == 1)
            assertTrue(intersection[0].distance(arbitraryLineSegment.intersectionWith(lineSegmentThatDefinesThePolygon)) < epsilon);
         else if (intersection.length == 2)
         {
            assertTrue(intersection[0].distance(arbitraryLineSegment.intersectionWith(lineSegmentThatDefinesThePolygon)) < epsilon);
            assertTrue(intersection[1].distance(arbitraryLineSegment.intersectionWith(lineSegmentThatDefinesThePolygon)) < epsilon);
            assertFalse(intersection[0].epsilonEquals(intersection[1], epsilon));
         }
         else
            fail();

         // shrinkConstantDistanceInto
         double shrinkDistance = random.nextDouble() * lineSegmentThatDefinesThePolygon.length() / 2.0;

         ConvexPolygonScaler shrinker = new ConvexPolygonScaler();

         ConvexPolygon2D shrunkenPolygon = new ConvexPolygon2D();

         shrinker.scaleConvexPolygon(polygonWithTwoPoints, shrinkDistance, shrunkenPolygon);
         shrinkDistance = lineSegmentThatDefinesThePolygon.length() / 2.0 + random.nextDouble();

         shrinker.scaleConvexPolygon(polygonWithTwoPoints, shrinkDistance, shrunkenPolygon);

         assertTrue(shrunkenPolygon.getNumberOfVertices() == 1);
         ConvexPolygon2D shrinkInto = ConvexPolygonTools.shrinkInto(sparePolygon, arbitraryPoint0, polygonWithTwoPoints);
         assertEquals(2, shrinkInto.getNumberOfVertices());
         shrinkInto = ConvexPolygonTools.shrinkInto(sparePolygon, arbitraryPoint0, polygonWithTwoPoints);
         assertEqualsInEitherOrder(shrinkInto.getVertex(0), shrinkInto.getVertex(1), pointThatDefinesThePolygon0, pointThatDefinesThePolygon1);
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetClosestEdge1()
   {
      Point2D vertex1 = new Point2D(0.0, 0.0);
      Point2D vertex2 = new Point2D(-1.0, 0.0);
      Point2D vertex3 = new Point2D(0.0, 1.0);
      Point2D vertex4 = new Point2D(1.0, 1.0);

      // add in order so vertices do not get changed when update is called.
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(vertex1);
      polygon.addVertex(vertex2);
      polygon.addVertex(vertex3);
      polygon.addVertex(vertex4);
      polygon.update();

      LineSegment2D edge1 = new LineSegment2D(vertex1, vertex2);
      LineSegment2D edge2 = new LineSegment2D(vertex2, vertex3);
      LineSegment2D edge3 = new LineSegment2D(vertex3, vertex4);
      LineSegment2D edge4 = new LineSegment2D(vertex4, vertex1);

      Point2D point1 = new Point2D(0.5, 0.1);
      assertEdgesEqual(edge4, polygon.getClosestEdgeCopy(point1));

      Point2D point2 = new Point2D(-0.5, -0.5);
      assertEdgesEqual(edge1, polygon.getClosestEdgeCopy(point2));

      Point2D point3 = new Point2D(-0.5, 0.5);
      assertEdgesEqual(edge2, polygon.getClosestEdgeCopy(point3));

      Point2D point4 = new Point2D(-0.5, 0.25);
      assertEdgesEqual(edge2, polygon.getClosestEdgeCopy(point4));

      Point2D point5 = new Point2D(-0.1, 3.0);
      assertEdgesEqual(edge2, polygon.getClosestEdgeCopy(point5));

      Point2D point6 = new Point2D(0.1, 0.8);
      assertEdgesEqual(edge3, polygon.getClosestEdgeCopy(point6));

      Point2D point7 = new Point2D(-0.11, 0.2);
      assertEdgesEqual(edge1, polygon.getClosestEdgeCopy(point7));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetClosestEdge2()
   {
      Point2D vertex1 = new Point2D(2.0, 2.0);
      Point2D vertex2 = new Point2D(3.0, 3.0);

      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(vertex1);
      polygon.addVertex(vertex2);
      polygon.update();

      LineSegment2D edge1 = new LineSegment2D(vertex1, vertex2);

      Point2D point1 = new Point2D(0.5, 0.1);
      assertEdgesEqual(edge1, polygon.getClosestEdgeCopy(point1));

      Point2D point2 = new Point2D(4.0, 4.0);
      assertEdgesEqual(edge1, polygon.getClosestEdgeCopy(point2));

      Point2D point3 = new Point2D(1.0, 1.0);
      assertEdgesEqual(edge1, polygon.getClosestEdgeCopy(point3));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetClosestEdge3()
   {
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(new Point2D());
      polygon.update();
      assertTrue(polygon.getClosestEdgeCopy(new Point2D()) == null);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetClosestEdge4()
   {
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      assertTrue(polygon.getClosestEdgeCopy(new Point2D()) == null);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetClosestVertexPoint1()
   {
      Point2D vertex1 = new Point2D(0.0, 0.0);
      Point2D vertex2 = new Point2D(10.0, 0.0);
      Point2D vertex3 = new Point2D(0.0, 10.0);

      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(vertex1);
      polygon.addVertex(vertex2);
      polygon.addVertex(vertex3);
      polygon.update();

      Point2D point1 = new Point2D(-1.0, -1.0);
      assertPointsEqual(vertex1, polygon.getClosestVertexCopy(point1));

      Point2D point2 = new Point2D(1.0, 1.0);
      assertPointsEqual(vertex1, polygon.getClosestVertexCopy(point2));

      Point2D point3 = new Point2D(10.0, 0.0);
      assertPointsEqual(vertex2, polygon.getClosestVertexCopy(point3));

      Point2D point4 = new Point2D(9.8, 0.0);
      assertPointsEqual(vertex2, polygon.getClosestVertexCopy(point4));

      Point2D point5 = new Point2D(10.0, 11.0);
      assertPointsEqual(vertex3, polygon.getClosestVertexCopy(point5));

      Point2D point6 = new Point2D(-3.0, 8.0);
      assertPointsEqual(vertex3, polygon.getClosestVertexCopy(point6));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testGetClosestVertexPoint2()
   {
      // make sure the method fails as expected with an empty polygon
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      Point2D closestVertex = new Point2D();

      assertFalse(polygon.getClosestVertex(new Point2D(), closestVertex));
      assertTrue(polygon.getClosestVertexCopy(new Point2D()) == null);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testCanObserverSeeEdge1()
   {
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(new Point2D(0.0, 0.0));
      polygon.addVertex(new Point2D(1.0, 0.0));
      polygon.addVertex(new Point2D(0.0, 1.0));
      polygon.addVertex(new Point2D(1.0, 1.0));
      polygon.update();

      // observer inside polygon can not see any outside edges
      Point2D observer1 = new Point2D(0.5, 0.5);
      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
         assertFalse(polygon.canObserverSeeEdge(i, observer1));

      // this observer should be able to see the edge starting at vertex (0.0, 0.0)
      Point2D observer2 = new Point2D(-0.5, 0.5);
      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
      {
         if (polygon.getVertex(i).epsilonEquals(new Point2D(0.0, 0.0), epsilon))
            assertTrue(polygon.canObserverSeeEdge(i, observer2));
         else
            assertFalse(polygon.canObserverSeeEdge(i, observer2));
      }

      // this observer should be able to see the edges starting at vertex (0.0, 1.0) and at (1.0, 1.0)
      Point2D observer3 = new Point2D(1.5, 1.5);
      for (int i = 0; i < polygon.getNumberOfVertices(); i++)
      {
         if (polygon.getVertex(i).epsilonEquals(new Point2D(0.0, 1.0), epsilon))
            assertTrue(polygon.canObserverSeeEdge(i, observer3));
         else if (polygon.getVertex(i).epsilonEquals(new Point2D(1.0, 1.0), epsilon))
            assertTrue(polygon.canObserverSeeEdge(i, observer3));
         else
            assertFalse(polygon.canObserverSeeEdge(i, observer3));
      }
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testCanObserverSeeEdge2()
   {
      // line polygon
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(new Point2D(1.0, 0.0));
      polygon.addVertex(new Point2D(0.0, 1.0));
      polygon.update();

      // should be able to see one edge
      Point2D observer1 = new Point2D(0.0, 0.0);
      boolean seeEdge1 = polygon.canObserverSeeEdge(0, observer1);
      boolean seeEdge2 = polygon.canObserverSeeEdge(1, observer1);
      assertTrue((seeEdge1 || seeEdge2) && !(seeEdge1 && seeEdge2));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testCanObserverSeeEdge3()
   {
      // point polygon
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(new Point2D(1.0, 1.0));
      polygon.update();

      Point2D observer1 = new Point2D(0.0, 0.0);
      assertFalse(polygon.canObserverSeeEdge(0, observer1));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testIntersectionWithLineSegment1()
   {
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(new Point2D(-1.0, -1.0));
      polygon.addVertex(new Point2D(1.0, -1.0));
      polygon.addVertex(new Point2D(-1.0, 1.0));
      polygon.addVertex(new Point2D(1.0, 1.0));
      polygon.update();

      Point2D result1 = new Point2D();
      Point2D result2 = new Point2D();

      LineSegment2D segment1 = new LineSegment2D(new Point2D(0.0, 0.0), new Point2D(2.0, 0.0));
      Point2D[] expected1 = new Point2D[] {new Point2D(1.0, 0.0)};
      assertPointsEqual(expected1, polygon.intersectionWith(segment1), false);
      assertTrue(polygon.intersectionWith(segment1, result1, result2) == 1);

      LineSegment2D segment2 = new LineSegment2D(new Point2D(-2.0, 0.0), new Point2D(2.0, 0.0));
      Point2D[] expected2 = new Point2D[] {new Point2D(-1.0, 0.0), new Point2D(1.0, 0.0)};
      assertPointsEqual(expected2, polygon.intersectionWith(segment2), false);
      assertTrue(polygon.intersectionWith(segment2, result1, result2) == 2);

      LineSegment2D segment3 = new LineSegment2D(new Point2D(-0.5, 0.0), new Point2D(0.5, 0.0));
      Point2D[] expected3 = null;
      assertPointsEqual(expected3, polygon.intersectionWith(segment3), false);
      assertTrue(polygon.intersectionWith(segment3, result1, result2) == 0);

      LineSegment2D segment4 = new LineSegment2D(new Point2D(-3.5, 0.0), new Point2D(-1.5, 0.0));
      Point2D[] expected4 = null;
      assertPointsEqual(expected4, polygon.intersectionWith(segment4), false);
      assertTrue(polygon.intersectionWith(segment4, result1, result2) == 0);

      LineSegment2D segment5 = new LineSegment2D(new Point2D(-1.5, 0.0), new Point2D(0.0, 1.5));
      Point2D[] expected5 = new Point2D[] {new Point2D(-1.0, 0.5), new Point2D(-0.5, 1.0)};
      assertPointsEqual(expected5, polygon.intersectionWith(segment5), false);
      assertTrue(polygon.intersectionWith(segment5, result1, result2) == 2);

      LineSegment2D segment6 = new LineSegment2D(new Point2D(-1.0, 0.5), new Point2D(-0.5, 1.0));
      Point2D[] expected6 = new Point2D[] {new Point2D(-1.0, 0.5), new Point2D(-0.5, 1.0)};
      assertPointsEqual(expected6, polygon.intersectionWith(segment6), false);
      assertTrue(polygon.intersectionWith(segment6, result1, result2) == 2);

      LineSegment2D segment7 = new LineSegment2D(new Point2D(-1.5, 1.0), new Point2D(1.5, 1.0));
      Point2D[] expected7 = new Point2D[] {new Point2D(-1.0, 1.0), new Point2D(1.0, 1.0)};
      assertPointsEqual(expected7, polygon.intersectionWith(segment7), false);
      assertTrue(polygon.intersectionWith(segment7, result1, result2) == 2);

      LineSegment2D segment8 = new LineSegment2D(new Point2D(-2.5, 1.0), new Point2D(-1.5, 1.0));
      Point2D[] expected8 = null;
      assertPointsEqual(expected8, polygon.intersectionWith(segment8), false);
      assertTrue(polygon.intersectionWith(segment8, result1, result2) == 0);

      LineSegment2D segment9 = new LineSegment2D(new Point2D(1.0, 0.0), new Point2D(1.0, 2.0));
      Point2D[] expected9 = new Point2D[] {new Point2D(1.0, 0.0), new Point2D(1.0, 1.0)};
      assertPointsEqual(expected9, polygon.intersectionWith(segment9), false);
      assertTrue(polygon.intersectionWith(segment9, result1, result2) == 2);

      LineSegment2D segment10 = new LineSegment2D(new Point2D(1.0, 0.0), new Point2D(1.0, 0.5));
      Point2D[] expected10 = new Point2D[] {new Point2D(1.0, 0.0), new Point2D(1.0, 0.5)};
      assertPointsEqual(expected10, polygon.intersectionWith(segment10), false);
      result1.set(expected10[0]);
      result2.set(expected10[0]);
      assertTrue(polygon.intersectionWith(segment10, result1, result2) == 2);
      result1.set(expected10[1]);
      result2.set(expected10[1]);
      assertTrue(polygon.intersectionWith(segment10, result1, result2) == 2);

      LineSegment2D segment11 = new LineSegment2D(new Point2D(-0.5, 1.0), new Point2D(-1.0, 0.5));
      Point2D[] expected11 = new Point2D[] {new Point2D(-1.0, 0.5), new Point2D(-0.5, 1.0)};
      assertPointsEqual(expected11, polygon.intersectionWith(segment11), false);
      assertTrue(polygon.intersectionWith(segment11, result1, result2) == 2);

      LineSegment2D segment12 = new LineSegment2D(new Point2D(-1.5, 0.5), new Point2D(1.5, 0.5));
      Point2D[] expected12 = new Point2D[] {new Point2D(-1.0, 0.5), new Point2D(1.0, 0.5)};
      assertPointsEqual(expected12, polygon.intersectionWith(segment12), false);
      result1.set(expected12[0]);
      result2.set(expected12[0]);
      assertTrue(polygon.intersectionWith(segment12, result1, result2) == 2);
      result1.set(expected12[1]);
      result2.set(expected12[1]);
      assertTrue(polygon.intersectionWith(segment12, result1, result2) == 2);

      LineSegment2D segment13 = new LineSegment2D(new Point2D(0.0, -1.5), new Point2D(1.5, -1.5));
      Point2D[] expected13 = null;
      assertPointsEqual(expected13, polygon.intersectionWith(segment13), false);
      assertTrue(polygon.intersectionWith(segment13, result1, result2) == 0);

      LineSegment2D segment14 = new LineSegment2D(new Point2D(0.0, 1.5), new Point2D(1.5, 1.5));
      Point2D[] expected14 = null;
      assertPointsEqual(expected14, polygon.intersectionWith(segment14), false);
      assertTrue(polygon.intersectionWith(segment14, result1, result2) == 0);

      LineSegment2D segment15 = new LineSegment2D(new Point2D(1.0, 1.0), new Point2D(0.5, 1.0));
      Point2D[] expected15 = new Point2D[] {new Point2D(1.0, 1.0), new Point2D(0.5, 1.0)};
      assertPointsEqual(expected15, polygon.intersectionWith(segment15), false);
      assertTrue(polygon.intersectionWith(segment15, result1, result2) == 2);

      LineSegment2D segment16 = new LineSegment2D(new Point2D(1.0, 1.0), new Point2D(1.0, 0.5));
      Point2D[] expected16 = new Point2D[] {new Point2D(1.0, 1.0), new Point2D(1.0, 0.5)};
      assertPointsEqual(expected16, polygon.intersectionWith(segment16), false);
      assertTrue(polygon.intersectionWith(segment16, result1, result2) == 2);

      LineSegment2D segment17 = new LineSegment2D(new Point2D(0.5, 1.0), new Point2D(1.0, 1.0));
      Point2D[] expected17 = new Point2D[] {new Point2D(1.0, 1.0), new Point2D(0.5, 1.0)};
      assertPointsEqual(expected17, polygon.intersectionWith(segment17), false);
      assertTrue(polygon.intersectionWith(segment17, result1, result2) == 2);

      LineSegment2D segment18 = new LineSegment2D(new Point2D(1.0, 0.5), new Point2D(1.0, 1.0));
      Point2D[] expected18 = new Point2D[] {new Point2D(1.0, 1.0), new Point2D(1.0, 0.5)};
      assertPointsEqual(expected18, polygon.intersectionWith(segment18), false);
      assertTrue(polygon.intersectionWith(segment18, result1, result2) == 2);

      LineSegment2D segment19 = new LineSegment2D(new Point2D(-1.5, 1.0), new Point2D(-0.5, 1.0));
      Point2D[] expected19 = new Point2D[] {new Point2D(-1.0, 1.0), new Point2D(-0.5, 1.0)};
      assertPointsEqual(expected19, polygon.intersectionWith(segment19), false);
      assertTrue(polygon.intersectionWith(segment19, result1, result2) == 2);

      LineSegment2D segment20 = new LineSegment2D(new Point2D(-1.5, 1.0), new Point2D(-1.0, 1.0));
      Point2D[] expected20 = new Point2D[] {new Point2D(-1.0, 1.0)};
      assertPointsEqual(expected20, polygon.intersectionWith(segment20), false);
      assertTrue(polygon.intersectionWith(segment20, result1, result2) == 1);

      LineSegment2D segment21 = new LineSegment2D(new Point2D(-1.0, 1.0), new Point2D(-1.5, 1.0));
      Point2D[] expected21 = new Point2D[] {new Point2D(-1.0, 1.0)};
      assertPointsEqual(expected21, polygon.intersectionWith(segment21), false);
      assertTrue(polygon.intersectionWith(segment21, result1, result2) == 1);

      LineSegment2D segment22 = new LineSegment2D(new Point2D(1.0, 1.0), new Point2D(1.5, 1.0));
      Point2D[] expected22 = new Point2D[] {new Point2D(1.0, 1.0)};
      assertPointsEqual(expected22, polygon.intersectionWith(segment22), false);
      assertTrue(polygon.intersectionWith(segment22, result1, result2) == 1);

      LineSegment2D segment23 = new LineSegment2D(new Point2D(1.5, 1.0), new Point2D(1.0, 1.0));
      Point2D[] expected23 = new Point2D[] {new Point2D(1.0, 1.0)};
      assertPointsEqual(expected23, polygon.intersectionWith(segment23), false);
      assertTrue(polygon.intersectionWith(segment23, result1, result2) == 1);

      LineSegment2D segment24 = new LineSegment2D(new Point2D(1.5, 1.5), new Point2D(1.0, 1.0));
      Point2D[] expected24 = new Point2D[] {new Point2D(1.0, 1.0)};
      assertPointsEqual(expected24, polygon.intersectionWith(segment24), false);
      assertTrue(polygon.intersectionWith(segment24, result1, result2) == 1);

      LineSegment2D segment25 = new LineSegment2D(new Point2D(0.5, 1.5), new Point2D(1.0, 1.0));
      Point2D[] expected25 = new Point2D[] {new Point2D(1.0, 1.0)};
      assertPointsEqual(expected25, polygon.intersectionWith(segment25), false);
      result1.set(expected25[0]);
      result2.set(expected25[0]);
      assertTrue(polygon.intersectionWith(segment25, result1, result2) == 1);

      LineSegment2D segment26 = new LineSegment2D(new Point2D(-1.0, -1.0), new Point2D(0.8, 1.0));
      Point2D[] expected26 = new Point2D[] {new Point2D(0.8, 1.0), new Point2D(-1.0, -1.0)};
      assertPointsEqual(expected26, polygon.intersectionWith(segment26), false);
      result1.set(expected26[0]);
      result2.set(expected26[0]);
      assertTrue(polygon.intersectionWith(segment26, result1, result2) == 2);
      result1.set(expected26[1]);
      result2.set(expected26[1]);
      assertTrue(polygon.intersectionWith(segment26, result1, result2) == 2);

      LineSegment2D segment27 = new LineSegment2D(new Point2D(1.0, 1.0), new Point2D(-1.0, -1.0));
      Point2D[] expected27 = new Point2D[] {new Point2D(1.0, 1.0), new Point2D(-1.0, -1.0)};
      assertPointsEqual(expected27, polygon.intersectionWith(segment27), false);
      result1.set(expected27[0]);
      result2.set(expected27[0]);
      assertTrue(polygon.intersectionWith(segment27, result1, result2) == 2);
      result1.set(expected27[1]);
      result2.set(expected27[1]);
      assertTrue(polygon.intersectionWith(segment27, result1, result2) == 2);

      LineSegment2D segment28 = new LineSegment2D(new Point2D(1.0, -0.5), new Point2D(1.0, 0.0));
      Point2D[] expected28 = new Point2D[] {new Point2D(1.0, 0.0), new Point2D(1.0, -0.5)};
      assertPointsEqual(expected28, polygon.intersectionWith(segment28), false);
      result1.set(expected28[0]);
      result2.set(expected28[0]);
      assertTrue(polygon.intersectionWith(segment28, result1, result2) == 2);
      result1.set(expected28[1]);
      result2.set(expected28[1]);
      assertTrue(polygon.intersectionWith(segment28, result1, result2) == 2);

      LineSegment2D segment29 = new LineSegment2D(new Point2D(1.0, -1.5), new Point2D(1.0, 0.5));
      Point2D[] expected29 = new Point2D[] {new Point2D(1.0, 0.5), new Point2D(1.0, -1.0)};
      assertPointsEqual(expected29, polygon.intersectionWith(segment29), false);
      result1.set(expected29[0]);
      result2.set(expected29[0]);
      assertTrue(polygon.intersectionWith(segment29, result1, result2) == 2);
      result1.set(expected29[1]);
      result2.set(expected29[1]);
      assertTrue(polygon.intersectionWith(segment29, result1, result2) == 2);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testIntersectionWithLineSegment2()
   {
      // empty polygon
      ConvexPolygon2D polygon = new ConvexPolygon2D();

      Point2D result1 = new Point2D();
      Point2D result2 = new Point2D();

      LineSegment2D segment1 = new LineSegment2D();
      Point2D[] expected1 = null;
      assertPointsEqual(expected1, polygon.intersectionWith(segment1), false);
      assertTrue(polygon.intersectionWith(segment1, result1, result2) == 0);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testIntersectionWithLineSegment3()
   {
      // point polygon
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(new Point2D(1.0, 1.0));
      polygon.update();

      Point2D result1 = new Point2D();
      Point2D result2 = new Point2D();

      LineSegment2D segment1 = new LineSegment2D(new Point2D(1.0, 0.0), new Point2D(2.0, 0.0));
      Point2D[] expected1 = null;
      assertPointsEqual(expected1, polygon.intersectionWith(segment1), false);
      assertTrue(polygon.intersectionWith(segment1, result1, result2) == 0);

      LineSegment2D segment2 = new LineSegment2D(new Point2D(1.0, 1.0), new Point2D(2.0, 0.0));
      Point2D[] expected2 = new Point2D[] {new Point2D(1.0, 1.0)};
      assertPointsEqual(expected2, polygon.intersectionWith(segment2), false);
      assertTrue(polygon.intersectionWith(segment2, result1, result2) == 1);

      LineSegment2D segment3 = new LineSegment2D(new Point2D(0.0, 0.0), new Point2D(1.0, 1.0));
      Point2D[] expected3 = new Point2D[] {new Point2D(1.0, 1.0)};
      assertPointsEqual(expected3, polygon.intersectionWith(segment3), false);
      assertTrue(polygon.intersectionWith(segment3, result1, result2) == 1);
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 3000)
   public void testIntersectionWithLineSegment4()
   {
      // line polygon
      ConvexPolygon2D polygon = new ConvexPolygon2D();
      polygon.addVertex(new Point2D(1.0, 1.0));
      polygon.addVertex(new Point2D(3.0, 3.0));
      polygon.update();

      Point2D result1 = new Point2D();
      Point2D result2 = new Point2D();

      LineSegment2D segment1 = new LineSegment2D(new Point2D(0.0, 0.0), new Point2D(3.0, 3.0));
      Point2D[] expected1 = new Point2D[] {new Point2D(1.0, 1.0), new Point2D(3.0, 3.0)};
      assertPointsEqual(expected1, polygon.intersectionWith(segment1), false);
      assertTrue(polygon.intersectionWith(segment1, result1, result2) == 2);

      LineSegment2D segment2 = new LineSegment2D(new Point2D(1.5, 1.5), new Point2D(2.5, 2.5));
      Point2D[] expected2 = new Point2D[] {new Point2D(1.5, 1.5), new Point2D(2.5, 2.5)};
      assertPointsEqual(expected2, polygon.intersectionWith(segment2), false);
      assertTrue(polygon.intersectionWith(segment2, result1, result2) == 2);

      LineSegment2D segment3 = new LineSegment2D(new Point2D(0.5, 0.5), new Point2D(3.5, 3.5));
      Point2D[] expected3 = new Point2D[] {new Point2D(1.0, 1.0), new Point2D(3.0, 3.0)};
      assertPointsEqual(expected3, polygon.intersectionWith(segment3), false);
      assertTrue(polygon.intersectionWith(segment3, result1, result2) == 2);

      LineSegment2D segment4 = new LineSegment2D(new Point2D(1.0, 1.0), new Point2D(3.0, 3.0));
      Point2D[] expected4 = new Point2D[] {new Point2D(1.0, 1.0), new Point2D(3.0, 3.0)};
      assertPointsEqual(expected4, polygon.intersectionWith(segment4), false);
      assertTrue(polygon.intersectionWith(segment4, result1, result2) == 2);

      LineSegment2D segment5 = new LineSegment2D(new Point2D(0.0, 0.0), new Point2D(0.5, 0.5));
      Point2D[] expected5 = null;
      assertPointsEqual(expected5, polygon.intersectionWith(segment5), false);
      assertTrue(polygon.intersectionWith(segment5, result1, result2) == 0);

      LineSegment2D segment6 = new LineSegment2D(new Point2D(0.5, 0.5), new Point2D(1.0, 1.0));
      Point2D[] expected6 = new Point2D[] {new Point2D(1.0, 1.0)};
      assertPointsEqual(expected6, polygon.intersectionWith(segment6), false);
      assertTrue(polygon.intersectionWith(segment6, result1, result2) == 1);

      LineSegment2D segment7 = new LineSegment2D(new Point2D(2.0, 0.5), new Point2D(2.0, 5.0));
      Point2D[] expected7 = new Point2D[] {new Point2D(2.0, 2.0)};
      assertPointsEqual(expected7, polygon.intersectionWith(segment7), false);
      assertTrue(polygon.intersectionWith(segment7, result1, result2) == 1);

      LineSegment2D segment8 = new LineSegment2D(new Point2D(2.0, 0.5), new Point2D(1.0, 1.0));
      Point2D[] expected8 = new Point2D[] {new Point2D(1.0, 1.0)};
      assertPointsEqual(expected8, polygon.intersectionWith(segment8), false);
      assertTrue(polygon.intersectionWith(segment8, result1, result2) == 1);

      LineSegment2D segment9 = new LineSegment2D(new Point2D(4.0, 4.0), new Point2D(2.0, 2.0));
      Point2D[] expected9 = new Point2D[] {new Point2D(3.0, 3.0), new Point2D(2.0, 2.0)};
      assertPointsEqual(expected9, polygon.intersectionWith(segment9), false);
      assertTrue(polygon.intersectionWith(segment9, result1, result2) == 2);
   }

   private void assertEqualsInEitherOrder(Point2DReadOnly expected0, Point2DReadOnly expected1, Point2DReadOnly actual0, Point2DReadOnly actual1)
   {
      if (expected0.epsilonEquals(actual0, epsilon))
         assertTrue(expected1.epsilonEquals(actual1, epsilon));
      else if (expected0.epsilonEquals(actual1, epsilon))
         assertTrue(expected1.epsilonEquals(actual0, epsilon));
      else
      {
         fail("Points are not equal in either order.");
      }
   }

   private void assertEqualsInAnyOrder(Point2DReadOnly expected0, Point2DReadOnly expected1, Point2DReadOnly expected2, Point2DReadOnly actual0,
                                       Point2DReadOnly actual1, Point2DReadOnly actual2)
   {
      if (expected0.equals(actual0) && expected1.equals(actual1))
         assertTrue(expected2.equals(actual2));
      else if (expected0.equals(actual0) && expected1.equals(actual2))
         assertTrue(expected2.equals(actual1));
      else if (expected0.equals(actual1) && expected1.equals(actual0))
         assertTrue(expected2.equals(actual2));
      else if (expected0.equals(actual1) && expected1.equals(actual2))
         assertTrue(expected2.equals(actual0));
      else if (expected0.equals(actual2) && expected1.equals(actual0))
         assertTrue(expected2.equals(actual1));
      else if (expected0.equals(actual2) && expected1.equals(actual1))
         assertTrue(expected2.equals(actual0));
      else
         fail("Points are not equal in any order");
   }

   private void assertEqualsInEitherOrder(double expected0, double expected1, double actual0, double actual1)
   {
      if (expected0 == actual0)
         assertTrue(expected1 == actual1);
      else if (expected0 == actual1)
         assertTrue(expected1 == actual0);
      else
      {
         System.out.println(expected0);
         System.out.println(expected1);
         System.out.println(actual0);
         System.out.println(actual1);
         fail("Doubles are not equal in either order.");
      }
   }

   private void pauseOneSecond()
   {
      try
      {
         Thread.sleep(1000);
      }
      catch (InterruptedException ex)
      {
      }
   }

   private static void assertEdgesEqual(LineSegment2D expected, LineSegment2D actual)
   {
      assertTrue("Edge did not match expected.", expected.epsilonEquals(actual, epsilon) || expected.epsilonEquals(actual.flipDirectionCopy(), epsilon));
   }

   private static void assertPointsEqual(Point2D[] expected, Point2D[] actual, boolean enforceOrder)
   {
      if (expected == null || actual == null)
      {
         assertTrue("Expected did not equal actual. One of them was null.", expected == actual);
         return;
      }

      assertEquals("Array lengths are not equal.", expected.length, actual.length);
      int points = expected.length;
      for (int i = 0; i < points; i++)
      {
         if (enforceOrder)
         {
            assertPointsEqual(expected[i], actual[i]);
            continue;
         }

         boolean foundPoint = false;
         for (int j = 0; j < points; j++)
         {
            if (expected[i].epsilonEquals(actual[j], epsilon))
               foundPoint = true;
         }
         assertTrue("Did not find point.", foundPoint);
      }
   }

   private static void assertPointsEqual(Point2D expected, Point2D actual)
   {
      if (expected == null && actual == null)
         return;

      double localEpsilon = epsilon * expected.distance(new Point2D());
      assertTrue("Point does not match expected.", expected.epsilonEquals(actual, localEpsilon));
   }

   private void verifyEpsilonEquals(FramePoint2D point1, FramePoint2D point2)
   {
      if (point1.distance(point2) > 1e-7)
         throw new RuntimeException("point1 = " + point1 + ", point2 = " + point2);
   }

   public static void main(String[] args)
   {
      MutationTestFacilitator.facilitateMutationTestForClass(ConvexPolygon2D.class, ConvexPolygon2dTest.class);
   }
}
