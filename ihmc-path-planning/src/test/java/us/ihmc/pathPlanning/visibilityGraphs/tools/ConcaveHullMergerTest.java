package us.ihmc.pathPlanning.visibilityGraphs.tools;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.awt.Color;
import java.util.ArrayList;

import org.apache.commons.lang3.tuple.Pair;
import org.junit.jupiter.api.Test;

import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.LineSegment2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.robotics.geometry.FrameGeometry2dPlotter;
import us.ihmc.robotics.geometry.FrameGeometryTestFrame;
import us.ihmc.robotics.geometry.PlanarRegion;

public class ConcaveHullMergerTest
{
   @Test
   public void testMergePlanarRegions()
   {
      Point2D pointA0 = new Point2D(0.0, 0.0);
      Point2D pointA1 = new Point2D(0.0, 1.0);
      Point2D pointA2 = new Point2D(1.0, 1.0);
      Point2D pointA3 = new Point2D(1.0, 0.0);

      Point2D pointB0 = new Point2D(-0.5, -0.5);
      Point2D pointB1 = new Point2D(-0.49, 0.5);
      Point2D pointB2 = new Point2D(0.5, 0.5);
      Point2D pointB3 = new Point2D(0.5, -0.5);

      ConvexPolygon2D polygonA = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(pointA0, pointA1, pointA2, pointA3));
      ConvexPolygon2D polygonB = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(pointB0, pointB1, pointB2, pointB3));

      PlanarRegion regionA = new PlanarRegion(new RigidBodyTransform(), polygonA);
      PlanarRegion regionB = new PlanarRegion(new RigidBodyTransform(), polygonB);

      PlanarRegion mergedPlanarRegion = ConcaveHullMerger.mergePlanarRegions(regionA, regionB);

      Point2D[] concaveHull = mergedPlanarRegion.getConcaveHull();
      assertEquals(8, concaveHull.length);

      int i = 0;
      double epsilon = 1e-7;
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointB0, concaveHull[i++], epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointB1, concaveHull[i++], epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(0.0, 0.5), concaveHull[i++], epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointA1, concaveHull[i++], epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointA2, concaveHull[i++], epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointA3, concaveHull[i++], epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(0.5, 0.0), concaveHull[i++], epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointB3, concaveHull[i++], epsilon);

      assertEquals(3, mergedPlanarRegion.getNumberOfConvexPolygons());
   }

   @Test
   public void testMergeConcaveHullsSimpleSquares()
   {
      ArrayList<Point2D> hullAVertices = new ArrayList<Point2D>();

      Point2D pointA0 = new Point2D(0.0, 0.0);
      Point2D pointA1 = new Point2D(0.0, 1.0);
      Point2D pointA2 = new Point2D(1.0, 1.0);
      Point2D pointA3 = new Point2D(1.0, 0.0);

      hullAVertices.add(pointA0);
      hullAVertices.add(pointA1);
      hullAVertices.add(pointA2);
      hullAVertices.add(pointA3);

      ArrayList<Point2D> hullBVertices = new ArrayList<Point2D>();

      Point2D pointB0 = new Point2D(-0.5, -0.5);
      Point2D pointB1 = new Point2D(-0.5, 0.5);
      Point2D pointB2 = new Point2D(0.5, 0.5);
      Point2D pointB3 = new Point2D(0.5, -0.49);

      hullBVertices.add(pointB0);
      hullBVertices.add(pointB1);
      hullBVertices.add(pointB2);
      hullBVertices.add(pointB3);

      Point2D[] hullA = new Point2D[hullAVertices.size()];
      Point2D[] hullB = new Point2D[hullBVertices.size()];

      hullAVertices.toArray(hullA);
      hullBVertices.toArray(hullB);

      ArrayList<Point2D> mergedHulls = ConcaveHullMerger.mergeConcaveHulls(hullA, hullB);

      assertEquals(8, mergedHulls.size());

      int i = 0;
      double epsilon = 1e-7;
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointB0, mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointB1, mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(0.0, 0.5), mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointA1, mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointA2, mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointA3, mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(0.5, 0.0), mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointB3, mergedHulls.get(i++), epsilon);
   }

   @Test
   public void testMergeConcaveHullsSmallSquareInsideLargeSquare()
   {
      boolean visualize = false;

      ArrayList<Point2D> hullAVertices = new ArrayList<Point2D>();

      Point2D pointA0 = new Point2D(0.0, 0.0);
      Point2D pointA1 = new Point2D(0.0, 1.0);
      Point2D pointA2 = new Point2D(1.0, 1.0);
      Point2D pointA3 = new Point2D(1.0, 0.0);

      hullAVertices.add(pointA0);
      hullAVertices.add(pointA1);
      hullAVertices.add(pointA2);
      hullAVertices.add(pointA3);

      ArrayList<Point2D> hullBVertices = new ArrayList<Point2D>();

      Point2D pointB0 = new Point2D(0.5, 0.5);
      Point2D pointB1 = new Point2D(0.5, 0.6);
      Point2D pointB2 = new Point2D(0.6, 0.6);
      Point2D pointB3 = new Point2D(0.6, 0.5);

      hullBVertices.add(pointB0);
      hullBVertices.add(pointB1);
      hullBVertices.add(pointB2);
      hullBVertices.add(pointB3);

      Point2D[] hullA = new Point2D[hullAVertices.size()];
      Point2D[] hullB = new Point2D[hullBVertices.size()];

      hullAVertices.toArray(hullA);
      hullBVertices.toArray(hullB);

      ConcaveHullMergerListener listener = (visualize ? new ConcaveHullMergerListener() : null);
      ArrayList<Point2D> mergedHulls = ConcaveHullMerger.mergeConcaveHulls(hullA, hullB, listener);

      assertEquals(4, mergedHulls.size());

      int i = 0;
      double epsilon = 1e-7;
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointA0, mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointA1, mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointA2, mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointA3, mergedHulls.get(i++), epsilon);

      if (visualize)
         ThreadTools.sleepForever();
   }

   @Test
   public void testMergeConcaveHullsInteriorHole()
   {
      boolean visualize = false;

      ArrayList<Point2D> hullAVertices = new ArrayList<Point2D>();

      Point2D pointA0 = new Point2D(0.0, 0.0);
      Point2D pointA1 = new Point2D(0.0, 1.0);
      Point2D pointA2 = new Point2D(1.0, 1.0);
      Point2D pointA3 = new Point2D(1.0, 0.0);

      hullAVertices.add(pointA0);
      hullAVertices.add(pointA1);
      hullAVertices.add(pointA2);
      hullAVertices.add(pointA3);

      ArrayList<Point2D> hullBVertices = new ArrayList<Point2D>();

      Point2D pointB0 = new Point2D(0.1, -0.5);
      Point2D pointB1 = new Point2D(0.1, 0.5);
      Point2D pointB2 = new Point2D(0.2, 0.5);
      Point2D pointB3 = new Point2D(0.2, -0.4);
      Point2D pointB4 = new Point2D(0.3, -0.4);
      Point2D pointB5 = new Point2D(0.3, 0.5);
      Point2D pointB6 = new Point2D(0.4, 0.5);
      Point2D pointB7 = new Point2D(0.4, -0.5);

      hullBVertices.add(pointB0);
      hullBVertices.add(pointB1);
      hullBVertices.add(pointB2);
      hullBVertices.add(pointB3);
      hullBVertices.add(pointB4);
      hullBVertices.add(pointB5);
      hullBVertices.add(pointB6);
      hullBVertices.add(pointB7);

      Point2D[] hullA = new Point2D[hullAVertices.size()];
      Point2D[] hullB = new Point2D[hullBVertices.size()];

      hullAVertices.toArray(hullA);
      hullBVertices.toArray(hullB);

      ConcaveHullMergerListener listener = (visualize ? new ConcaveHullMergerListener() : null);
      ArrayList<Point2D> mergedHulls = ConcaveHullMerger.mergeConcaveHulls(hullA, hullB, listener);

      assertEquals(8, mergedHulls.size());

      int i = 0;
      double epsilon = 1e-7;
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointA0, mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointA1, mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointA2, mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointA3, mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(0.4, 0.0), mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointB7, mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointB0, mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(new Point2D(0.1, 0.0), mergedHulls.get(i++), epsilon);

      if (visualize)
         ThreadTools.sleepForever();
   }

   @Test
   public void testMergeExactSame()
   {
      boolean visualize = false;

      ArrayList<Point2D> hullAVertices = new ArrayList<Point2D>();

      Point2D pointA0 = new Point2D(0.0, 0.0);
      Point2D pointA1 = new Point2D(0.0, 1.0);
      Point2D pointA2 = new Point2D(1.0, 1.0);
      Point2D pointA3 = new Point2D(1.0, 0.0);

      hullAVertices.add(pointA0);
      hullAVertices.add(pointA1);
      hullAVertices.add(pointA2);
      hullAVertices.add(pointA3);

      ArrayList<Point2D> hullBVertices = new ArrayList<Point2D>();

      Point2D pointB0 = new Point2D(0.0, 0.0);
      Point2D pointB1 = new Point2D(0.0, 1.0);
      Point2D pointB2 = new Point2D(1.0, 1.0);
      Point2D pointB3 = new Point2D(1.0, 0.0);

      hullBVertices.add(pointB0);
      hullBVertices.add(pointB1);
      hullBVertices.add(pointB2);
      hullBVertices.add(pointB3);

      Point2D[] hullA = new Point2D[hullAVertices.size()];
      Point2D[] hullB = new Point2D[hullBVertices.size()];

      hullAVertices.toArray(hullA);
      hullBVertices.toArray(hullB);

      ConcaveHullMergerListener listener = (visualize ? new ConcaveHullMergerListener() : null);
      ArrayList<Point2D> mergedHulls = ConcaveHullMerger.mergeConcaveHulls(hullA, hullB, listener);

      assertEquals(4, mergedHulls.size());

      int i = 0;
      double epsilon = 1e-7;
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointA0, mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointA1, mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointA2, mergedHulls.get(i++), epsilon);
      EuclidCoreTestTools.assertPoint2DGeometricallyEquals(pointA3, mergedHulls.get(i++), epsilon);

      if (visualize)
         ThreadTools.sleepForever();
   }

   @Test
   public void testMergeConcaveHullsTroublesomeOne()
   {
      boolean visualize = false;

      double[][] hullOne = new double[][] {{-0.32080844044685364, 0.9086798429489136}, {-0.28522926568984985, 0.9880890846252441},
            {-0.1836146116256714, 1.0075507164001465}, {-0.1378334015607834, 0.9325186014175415}, {-0.12150357663631439, 0.8489445447921753},
            {-0.11247798800468445, 0.7848914265632629}, {-0.10055802017450333, 0.7219595313072205}, {-0.0858231782913208, 0.6531859636306763},
            {-0.08974666893482208, 0.5877435803413391}, {-0.07057010382413864, 0.44702088832855225}, {-0.05137510970234871, 0.35009393095970154},
            {-0.048323068767786026, 0.2116333544254303}, {-0.029583362862467766, 0.06909555941820145}, {-0.012792153283953667, -0.005410400684922934},
            {-0.002692134352400899, -0.07482614368200302}, {0.015383324585855007, -0.17355236411094666}, {0.02966991625726223, -0.3129369914531708},
            {0.030642254278063774, -0.38066574931144714}, {0.09247396886348724, -0.41485705971717834}, {0.19452345371246338, -0.4152769446372986},
            {0.23610325157642365, -0.414505273103714}, {0.3529823124408722, -0.41112226247787476}, {0.41674885153770447, -0.41055598855018616},
            {0.5968952775001526, -0.41316646337509155}, {0.6718282699584961, -0.4079112112522125}, {0.7358959913253784, -0.37659206986427307},
            {0.7940598726272583, -0.39256760478019714}, {0.8539779782295227, -0.36987099051475525}, {0.8775563836097717, -0.44087880849838257},
            {0.8951427340507507, -0.497694730758667}, {0.8463701009750366, -0.5413227081298828}, {0.7558804750442505, -0.5653964281082153},
            {0.6759657263755798, -0.551409125328064}, {0.6368259787559509, -0.549574077129364}, {0.5739794373512268, -0.550038754940033},
            {0.5343534350395203, -0.588975727558136}, {0.45960354804992676, -0.5972794890403748}, {0.3683496415615082, -0.587111234664917},
            {0.2744791507720947, -0.6093615889549255}, {0.14267343282699585, -0.6075680255889893}, {0.0530564971268177, -0.6351839900016785},
            {-0.0165009256452322, -0.6289142370223999}, {-0.08789733052253723, -0.6491257548332214}, {-0.14297418296337128, -0.5095702409744263},
            {-0.1441502869129181, -0.4497186839580536}, {-0.1623517870903015, -0.36984652280807495}, {-0.16642220318317413, -0.31222599744796753},
            {-0.17461782693862915, -0.2546726167201996}, {-0.19403919577598572, -0.13188748061656952}, {-0.20356975495815277, -0.03135628625750542},
            {-0.22276757657527924, 0.028552822768688202}, {-0.23816534876823425, 0.12956836819648743}, {-0.2233646810054779, 0.20830659568309784},
            {-0.23749388754367828, 0.26624444127082825}, {-0.2414117306470871, 0.36849892139434814}, {-0.24244770407676697, 0.3880820572376251},
            {-0.2581060230731964, 0.46913930773735046}, {-0.2633146047592163, 0.5846882462501526}, {-0.2798686921596527, 0.6706967353820801},
            {-0.3030450940132141, 0.8290156126022339}};

      double[][] hullTwo = new double[][] {{-0.15936496272840953, 0.08901430953161665}, {-0.047907207824508136, 0.11249011839155193},
            {-0.009886369055515487, 0.0780884316128021}, {-0.004574789669821888, -0.03442622045027294}, {0.022450792800197822, -0.1683871873291485},
            {0.012758309221879843, -0.24440572474261255}, {0.01789889409201989, -0.3444630600706097}, {0.055439442362793676, -0.41044137718121043},
            {0.12838851032560822, -0.4200984524054827}, {0.18588113170475543, -0.40900853006219284}, {0.2616279871221178, -0.40034939313016116},
            {0.3728812252385902, -0.40692794894670714}, {0.5067275085573423, -0.3890745762402008}, {0.5853448326672742, -0.3998915611253164},
            {0.8214038080362078, -0.4249131537925772}, {0.8878281581325169, -0.4934313704624567}, {0.8562573850357904, -0.5638341162746424},
            {0.6972782956386417, -0.556293100791231}, {0.6306328030329531, -0.5724172189201938}, {0.5776880238324902, -0.567426514063733},
            {0.4619108600260218, -0.5413079293009101}, {0.38840872535166443, -0.5895591884394169}, {0.29777405386267325, -0.584678478243929},
            {0.21465674088276535, -0.5958315076800703}, {0.13354830510427668, -0.6164547163407885}, {0.023641897671828227, -0.6240701572006659},
            {-0.02899821040360062, -0.6240849919270718}, {-0.12330590277407918, -0.5602205030858636}, {-0.11745372007422264, -0.4666361629461157},
            {-0.14178226448362094, -0.32370311833003684}, {-0.140508233857459, -0.2346025872800086}, {-0.17330307409554194, -0.149159839926986},
            {-0.19239786762148312, -0.09986922600319931}, {-0.19854388436229184, -8.514394122500213E-4}, {-0.22962867228965547, 0.05809039869701854}};

      ConcaveHullMergerListener listener = (visualize ? new ConcaveHullMergerListener() : null);
      ArrayList<Point2D> mergedHulls = ConcaveHullMerger.mergeConcaveHulls(convertToPointArray(hullOne), convertToPointArray(hullTwo), listener);

      assertEquals(74, mergedHulls.size());

      if (visualize)
         ThreadTools.sleepForever();

   }

   private Point2D[] convertToPointArray(double[][] hull)
   {
      Point2D[] points = new Point2D[hull.length];

      for (int i = 0; i < hull.length; i++)
      {
         points[i] = new Point2D(hull[i][0], hull[i][1]);
      }

      return points;
   }

   @Test
   public void testFindIntersection()
   {
      ArrayList<Point2D> hullAVertices = new ArrayList<Point2D>();

      Point2D pointA0 = new Point2D(0.0, 0.0);
      Point2D pointA1 = new Point2D(0.0, 1.0);
      Point2D pointA2 = new Point2D(1.0, 1.0);
      Point2D pointA3 = new Point2D(1.0, 0.0);

      hullAVertices.add(pointA0);
      hullAVertices.add(pointA1);
      hullAVertices.add(pointA2);
      hullAVertices.add(pointA3);

      Point2D[] hullA = new Point2D[hullAVertices.size()];
      hullAVertices.toArray(hullA);

      Point2D firstEndPoint = new Point2D(0.5, -0.5);
      Point2D secondEndPoint = new Point2D(0.5, 0.5);
      LineSegment2D edge = new LineSegment2D(firstEndPoint, secondEndPoint);

      Pair<Integer, Point2D> intersection = ConcaveHullMerger.findClosestIntersection(edge, hullA, -1);

      assertTrue(intersection.getRight().epsilonEquals(new Point2D(0.5, 0.0), 1e-7));
      assertEquals(0, intersection.getLeft());
   }

}
