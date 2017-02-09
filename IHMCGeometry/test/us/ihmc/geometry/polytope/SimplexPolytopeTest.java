package us.ihmc.geometry.polytope;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertTrue;

import javax.vecmath.Point3d;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.tools.testing.JUnitTools;

public class SimplexPolytopeTest
{

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testGetClosestPointToOriginOnConvexHull()
   {
      SimplexPolytope simplex = new SimplexPolytope();

      // Single Point
      Point3d pointOne = new Point3d(1.0, 2.0, 3.0);
      simplex.setPoints(pointOne);

      Point3d closestPointToOrigin = new Point3d();
      simplex.getClosestPointToOriginOnConvexHullAndRemoveUnusedVertices(closestPointToOrigin);
      assertLambdasOnSimplex(simplex);
      assertEquals(1, simplex.getNumberOfPoints());

      JUnitTools.assertPoint3dEquals("", new Point3d(1.0, 2.0, 3.0), closestPointToOrigin, 1e-7);

      // Two Points
      Point3d pointTwo = new Point3d(1.0, 2.0, 4.0);
      simplex.setPoints(pointOne, pointTwo);

      simplex.getClosestPointToOriginOnConvexHullAndRemoveUnusedVertices(closestPointToOrigin);
      assertLambdasOnSimplex(simplex);
      JUnitTools.assertPoint3dEquals("", new Point3d(1.0, 2.0, 3.0), closestPointToOrigin, 1e-7);
      assertEquals(1, simplex.getNumberOfPoints());
      assertTrue(simplex.containsPoint(pointOne));
      assertFalse(simplex.containsPoint(pointTwo));

      pointTwo = new Point3d(1.0, 2.0, 2.5);
      simplex.setPoints(pointOne, pointTwo);

      simplex.getClosestPointToOriginOnConvexHullAndRemoveUnusedVertices(closestPointToOrigin);
      assertLambdasOnSimplex(simplex);
      JUnitTools.assertPoint3dEquals("", new Point3d(1.0, 2.0, 2.5), closestPointToOrigin, 1e-7);
      assertEquals(1, simplex.getNumberOfPoints());
      assertFalse(simplex.containsPoint(pointOne));
      assertTrue(simplex.containsPoint(pointTwo));

      pointTwo = new Point3d(1.0, 2.0, -12.9);
      simplex.setPoints(pointOne, pointTwo);

      simplex.getClosestPointToOriginOnConvexHullAndRemoveUnusedVertices(closestPointToOrigin);
      assertLambdasOnSimplex(simplex);
      JUnitTools.assertPoint3dEquals("", new Point3d(1.0, 2.0, 0.0), closestPointToOrigin, 1e-7);
      assertEquals(2, simplex.getNumberOfPoints());
      assertTrue(simplex.containsPoint(pointOne));
      assertTrue(simplex.containsPoint(pointTwo));

      pointTwo = new Point3d(1.0, 2.0, 4.0);
      simplex.setPoints(pointOne, pointTwo);

      // Three Points. Start at (1, 2, 3) and (1, 2, 4). Add (4, 2, 3)
      Point3d pointThree = new Point3d(4.0, 2.0, 3.0);
      simplex.setPoints(pointOne, pointTwo, pointThree);

      simplex.getClosestPointToOriginOnConvexHullAndRemoveUnusedVertices(closestPointToOrigin);
      assertLambdasOnSimplex(simplex);
      JUnitTools.assertPoint3dEquals("", new Point3d(1.0, 2.0, 3.0), closestPointToOrigin, 1e-7);
      assertEquals(1, simplex.getNumberOfPoints());
      assertTrue(simplex.containsPoint(pointOne));
      assertFalse(simplex.containsPoint(pointTwo));
      assertFalse(simplex.containsPoint(pointThree));

      pointTwo = new Point3d(1.0, 2.0, -12.9);
      simplex.setPoints(pointOne, pointTwo, pointThree);

      simplex.getClosestPointToOriginOnConvexHullAndRemoveUnusedVertices(closestPointToOrigin);
      assertLambdasOnSimplex(simplex);
      JUnitTools.assertPoint3dEquals("", new Point3d(1.0, 2.0, 0.0), closestPointToOrigin, 1e-7);
      assertEquals(2, simplex.getNumberOfPoints());
      assertTrue(simplex.containsPoint(pointOne));
      assertTrue(simplex.containsPoint(pointTwo));
      assertFalse(simplex.containsPoint(pointThree));

      pointOne = new Point3d(-20.0, 2.0, 3.0);
      simplex.setPoints(pointOne, pointTwo, pointThree);

      simplex.getClosestPointToOriginOnConvexHullAndRemoveUnusedVertices(closestPointToOrigin);
      assertLambdasOnSimplex(simplex);

      JUnitTools.assertPoint3dEquals("", new Point3d(0.0, 2.0, 0.0), closestPointToOrigin, 1e-7);
      assertEquals(3, simplex.getNumberOfPoints());
      assertTrue(simplex.containsPoint(pointOne));
      assertTrue(simplex.containsPoint(pointTwo));
      assertTrue(simplex.containsPoint(pointThree));

      // Four Points. Start at (1, 1, 1), (1, 2, 1), (2, 1, 1), and (1, 1, 2)
      simplex = new SimplexPolytope();

      pointOne = new Point3d(1.0, 1.0, 1.0);
      pointTwo = new Point3d(1.0, 2.0, 1.0);
      pointThree = new Point3d(2.0, 1.0, 1.0);
      Point3d pointFour = new Point3d(1.0, 1.0, 2.0);
      simplex.setPoints(pointOne, pointTwo, pointThree, pointFour);

      simplex.getClosestPointToOriginOnConvexHullAndRemoveUnusedVertices(closestPointToOrigin);
      assertLambdasOnSimplex(simplex);
      JUnitTools.assertPoint3dEquals("", new Point3d(1.0, 1.0, 1.0), closestPointToOrigin, 1e-7);
      assertEquals(1, simplex.getNumberOfPoints());
      assertTrue(simplex.containsPoint(pointOne));
      assertFalse(simplex.containsPoint(pointTwo));
      assertFalse(simplex.containsPoint(pointThree));
      assertFalse(simplex.containsPoint(pointFour));

      pointOne = new Point3d(1.0, 1.0, -20.0);
      simplex.setPoints(pointOne, pointTwo, pointThree, pointFour);

      simplex.getClosestPointToOriginOnConvexHullAndRemoveUnusedVertices(closestPointToOrigin);
      assertLambdasOnSimplex(simplex);
      JUnitTools.assertPoint3dEquals("", new Point3d(1.0, 1.0, 0.0), closestPointToOrigin, 1e-7);
      assertEquals(2, simplex.getNumberOfPoints());
      assertTrue(simplex.containsPoint(pointOne));
      assertFalse(simplex.containsPoint(pointTwo));
      assertFalse(simplex.containsPoint(pointThree));
      assertTrue(simplex.containsPoint(pointFour));

      pointOne = new Point3d(1.0, -20.0, -20.0);
      simplex.setPoints(pointOne, pointTwo, pointThree, pointFour);

      simplex.getClosestPointToOriginOnConvexHullAndRemoveUnusedVertices(closestPointToOrigin);
      assertLambdasOnSimplex(simplex);
      JUnitTools.assertPoint3dEquals("", new Point3d(1.0, 0.0, 0.0), closestPointToOrigin, 1e-7);
      assertEquals(3, simplex.getNumberOfPoints());
      assertTrue(simplex.containsPoint(pointOne));
      assertTrue(simplex.containsPoint(pointTwo));
      assertFalse(simplex.containsPoint(pointThree));
      assertTrue(simplex.containsPoint(pointFour));

      pointOne = new Point3d(-20.0, -20.0, -20.0);
      simplex.setPoints(pointOne, pointTwo, pointThree, pointFour);

      simplex.getClosestPointToOriginOnConvexHullAndRemoveUnusedVertices(closestPointToOrigin);
      assertLambdasOnSimplex(simplex);
      JUnitTools.assertPoint3dEquals("", new Point3d(0.0, 0.0, 0.0), closestPointToOrigin, 1e-7);
      assertEquals(4, simplex.getNumberOfPoints());
      assertTrue(simplex.containsPoint(pointOne));
      assertTrue(simplex.containsPoint(pointTwo));
      assertTrue(simplex.containsPoint(pointThree));
      assertTrue(simplex.containsPoint(pointFour));
   }

   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testVornoiRegionChecks()
   {
      SimplexPolytope simplex = new SimplexPolytope();

      // Two Points:
      Point3d pointOne = new Point3d(1.0, 0.0, 0.0);
      Point3d pointTwo = new Point3d(2.0, 0.0, 0.0);
      simplex.setPoints(pointOne, pointTwo);

      assertTrue(simplex.isInVoronoiRegionOfVertex(pointOne, pointTwo));
      assertFalse(simplex.isInVoronoiRegionOfVertex(pointTwo, pointOne));

      pointOne = new Point3d(3.0, 4.0, 7.0);
      pointTwo = new Point3d(2.0, 3.0, 5.0);
      simplex.setPoints(pointOne, pointTwo);

      assertFalse(simplex.isInVoronoiRegionOfVertex(pointOne, pointTwo));
      assertTrue(simplex.isInVoronoiRegionOfVertex(pointTwo, pointOne));

      pointOne = new Point3d(-1.0, 0.01, 0.0);
      pointTwo = new Point3d(2.0, 0.01, 0.0);
      simplex.setPoints(pointOne, pointTwo);

      assertFalse(simplex.isInVoronoiRegionOfVertex(pointOne, pointTwo));
      assertFalse(simplex.isInVoronoiRegionOfVertex(pointTwo, pointOne));

      // Three Points:
      pointOne = new Point3d(1.0, 0.0, 0.0);
      pointTwo = new Point3d(2.0, 0.0, 0.0);
      Point3d pointThree = new Point3d(1.5, 1.0, 0.0);
      simplex.setPoints(pointOne, pointTwo, pointThree);

      assertTrue(simplex.isInVoronoiRegionOfVertex(pointOne, pointTwo, pointThree));
      assertTrue(simplex.isInVoronoiRegionOfVertex(pointOne, pointThree, pointTwo));
      assertFalse(simplex.isInVoronoiRegionOfVertex(pointTwo, pointOne, pointThree));
      assertFalse(simplex.isInVoronoiRegionOfVertex(pointTwo, pointThree, pointOne));
      assertFalse(simplex.isInVoronoiRegionOfVertex(pointThree, pointOne, pointTwo));
      assertFalse(simplex.isInVoronoiRegionOfVertex(pointThree, pointTwo, pointOne));

      assertFalse(simplex.isInVoronoiRegionOfEdge(pointOne, pointTwo, pointThree));
      assertFalse(simplex.isInVoronoiRegionOfEdge(pointOne, pointThree, pointTwo));
      assertFalse(simplex.isInVoronoiRegionOfEdge(pointTwo, pointOne, pointThree));
      assertFalse(simplex.isInVoronoiRegionOfEdge(pointTwo, pointThree, pointOne));
      assertFalse(simplex.isInVoronoiRegionOfEdge(pointThree, pointOne, pointTwo));
      assertFalse(simplex.isInVoronoiRegionOfEdge(pointThree, pointTwo, pointOne));

      pointOne = new Point3d(3.0, 1.0, 0.0);
      pointTwo = new Point3d(3.0, -1.0, 0.0);
      pointThree = new Point3d(5.0, 0.0, 0.0);
      simplex.setPoints(pointOne, pointTwo, pointThree);

      assertFalse(simplex.isInVoronoiRegionOfVertex(pointOne, pointTwo, pointThree));
      assertFalse(simplex.isInVoronoiRegionOfVertex(pointOne, pointThree, pointTwo));
      assertFalse(simplex.isInVoronoiRegionOfVertex(pointTwo, pointOne, pointThree));
      assertFalse(simplex.isInVoronoiRegionOfVertex(pointTwo, pointThree, pointOne));
      assertFalse(simplex.isInVoronoiRegionOfVertex(pointThree, pointOne, pointTwo));
      assertFalse(simplex.isInVoronoiRegionOfVertex(pointThree, pointTwo, pointOne));

      assertTrue(simplex.isInVoronoiRegionOfEdge(pointOne, pointTwo, pointThree));
      assertTrue(simplex.isInVoronoiRegionOfEdge(pointTwo, pointOne, pointThree));
      assertFalse(simplex.isInVoronoiRegionOfEdge(pointOne, pointThree, pointTwo));
      assertFalse(simplex.isInVoronoiRegionOfEdge(pointThree, pointOne, pointTwo));
      assertFalse(simplex.isInVoronoiRegionOfEdge(pointTwo, pointThree, pointOne));
      assertFalse(simplex.isInVoronoiRegionOfEdge(pointThree, pointTwo, pointOne));

      double epsilon = 1e-7;
      pointOne = new Point3d(3.0, 0.0 + epsilon, 0.0);
      pointTwo = new Point3d(3.0, -1.0, 0.0);
      pointThree = new Point3d(5.0, 0.0, 0.0);
      simplex.setPoints(pointOne, pointTwo, pointThree);

      assertFalse(simplex.isInVoronoiRegionOfVertex(pointOne, pointTwo, pointThree));
      assertFalse(simplex.isInVoronoiRegionOfVertex(pointOne, pointThree, pointTwo));
      assertFalse(simplex.isInVoronoiRegionOfVertex(pointTwo, pointOne, pointThree));
      assertFalse(simplex.isInVoronoiRegionOfVertex(pointTwo, pointThree, pointOne));
      assertFalse(simplex.isInVoronoiRegionOfVertex(pointThree, pointOne, pointTwo));
      assertFalse(simplex.isInVoronoiRegionOfVertex(pointThree, pointTwo, pointOne));

      assertTrue(simplex.isInVoronoiRegionOfEdge(pointOne, pointTwo, pointThree));
      assertTrue(simplex.isInVoronoiRegionOfEdge(pointTwo, pointOne, pointThree));
      assertFalse(simplex.isInVoronoiRegionOfEdge(pointOne, pointThree, pointTwo));
      assertFalse(simplex.isInVoronoiRegionOfEdge(pointThree, pointOne, pointTwo));
      assertFalse(simplex.isInVoronoiRegionOfEdge(pointTwo, pointThree, pointOne));
      assertFalse(simplex.isInVoronoiRegionOfEdge(pointThree, pointTwo, pointOne));

      pointOne = new Point3d(3.0, 0.0 - epsilon, 0.0);
      pointTwo = new Point3d(3.0, -1.0, 0.0);
      pointThree = new Point3d(5.0, 0.0, 0.0);
      simplex.setPoints(pointOne, pointTwo, pointThree);

      assertTrue(simplex.isInVoronoiRegionOfVertex(pointOne, pointTwo, pointThree));
      assertTrue(simplex.isInVoronoiRegionOfVertex(pointOne, pointThree, pointTwo));
      assertFalse(simplex.isInVoronoiRegionOfVertex(pointTwo, pointOne, pointThree));
      assertFalse(simplex.isInVoronoiRegionOfVertex(pointTwo, pointThree, pointOne));
      assertFalse(simplex.isInVoronoiRegionOfVertex(pointThree, pointOne, pointTwo));
      assertFalse(simplex.isInVoronoiRegionOfVertex(pointThree, pointTwo, pointOne));

      assertFalse(simplex.isInVoronoiRegionOfEdge(pointOne, pointTwo, pointThree));
      assertFalse(simplex.isInVoronoiRegionOfEdge(pointTwo, pointOne, pointThree));
      assertFalse(simplex.isInVoronoiRegionOfEdge(pointOne, pointThree, pointTwo));
      assertFalse(simplex.isInVoronoiRegionOfEdge(pointThree, pointOne, pointTwo));
      assertFalse(simplex.isInVoronoiRegionOfEdge(pointTwo, pointThree, pointOne));
      assertFalse(simplex.isInVoronoiRegionOfEdge(pointThree, pointTwo, pointOne));

      pointOne = new Point3d(3.0, 0.0, 0.0);
      pointTwo = new Point3d(3.0, -1.0, 0.0);
      pointThree = new Point3d(5.0, 0.0, 0.0);
      simplex.setPoints(pointOne, pointTwo, pointThree);

      assertTrue(simplex.isInVoronoiRegionOfVertex(pointOne, pointTwo, pointThree));
      assertTrue(simplex.isInVoronoiRegionOfVertex(pointOne, pointThree, pointTwo));
      assertFalse(simplex.isInVoronoiRegionOfVertex(pointTwo, pointOne, pointThree));
      assertFalse(simplex.isInVoronoiRegionOfVertex(pointTwo, pointThree, pointOne));
      assertFalse(simplex.isInVoronoiRegionOfVertex(pointThree, pointOne, pointTwo));
      assertFalse(simplex.isInVoronoiRegionOfVertex(pointThree, pointTwo, pointOne));

      assertTrue(simplex.isInVoronoiRegionOfEdge(pointOne, pointTwo, pointThree));
      assertTrue(simplex.isInVoronoiRegionOfEdge(pointTwo, pointOne, pointThree));
      assertFalse(simplex.isInVoronoiRegionOfEdge(pointOne, pointThree, pointTwo));
      assertFalse(simplex.isInVoronoiRegionOfEdge(pointThree, pointOne, pointTwo));
      assertFalse(simplex.isInVoronoiRegionOfEdge(pointTwo, pointThree, pointOne));
      assertFalse(simplex.isInVoronoiRegionOfEdge(pointThree, pointTwo, pointOne));

      // Four Points:
      pointOne = new Point3d(3.0, 0.0, 0.0);
      pointTwo = new Point3d(5.0, 0.0, 0.0);
      pointThree = new Point3d(4.0, 2.0, 0.0);
      Point3d pointFour = new Point3d(4.0, 1.0, 1.0);
      simplex.setPoints(pointOne, pointTwo, pointThree, pointFour);

      assertTrue(simplex.isInVoronoiRegionOfVertex(pointOne, pointTwo, pointThree, pointFour));
      assertFalse(simplex.isInVoronoiRegionOfVertex(pointTwo, pointOne, pointThree, pointFour));
      assertFalse(simplex.isInVoronoiRegionOfVertex(pointThree, pointOne, pointTwo, pointFour));
      assertFalse(simplex.isInVoronoiRegionOfVertex(pointFour, pointOne, pointTwo, pointThree));

      assertFalse(simplex.isInVoronoiRegionOfEdge(pointOne, pointTwo, pointThree, pointFour));
      assertFalse(simplex.isInVoronoiRegionOfEdge(pointOne, pointThree, pointTwo, pointFour));
      assertFalse(simplex.isInVoronoiRegionOfEdge(pointOne, pointFour, pointTwo, pointThree));
      assertFalse(simplex.isInVoronoiRegionOfEdge(pointTwo, pointThree, pointOne, pointFour));
      assertFalse(simplex.isInVoronoiRegionOfEdge(pointTwo, pointFour, pointOne, pointThree));
      assertFalse(simplex.isInVoronoiRegionOfEdge(pointThree, pointFour, pointOne, pointTwo));

      assertFalse(simplex.isInVoronoiRegionOfFace(pointOne, pointTwo, pointThree, pointFour));
      assertFalse(simplex.isInVoronoiRegionOfFace(pointOne, pointTwo, pointFour, pointThree));
      assertTrue(simplex.isInVoronoiRegionOfFace(pointOne, pointThree, pointFour, pointTwo));
      assertFalse(simplex.isInVoronoiRegionOfFace(pointTwo, pointThree, pointFour, pointOne));

      // A Troublesome one:
      //      pointOne = new Point3d(-99.63099726377406, -101.05538475479693, -300.5);
      //      pointTwo = new Point3d(101.80371491802498, 97.55120182650874, -299.5);
      //      pointThree = new Point3d(-100.32770397312123, 98.22725915430355, -299.5);
      //      pointFour = new Point3d(-98.89299179132217, -103.16615426439077, -299.5);

      pointOne = new Point3d(-0.996, -1.01, -3.01);
      pointTwo = new Point3d(1.00, 0.98, -3.0);
      pointThree = new Point3d(-1.00, 0.98, -3.0);
      pointFour = new Point3d(-0.988, -1.01, -3.0);
      simplex.setPoints(pointOne, pointTwo, pointThree, pointFour);

      assertFalse(simplex.hasFourCoplanarPoints());

      assertFalse(simplex.isInVoronoiRegionOfVertex(pointOne, pointTwo, pointThree, pointFour));
      assertFalse(simplex.isInVoronoiRegionOfVertex(pointTwo, pointOne, pointThree, pointFour));
      assertFalse(simplex.isInVoronoiRegionOfVertex(pointThree, pointOne, pointTwo, pointFour));
      assertFalse(simplex.isInVoronoiRegionOfVertex(pointFour, pointOne, pointTwo, pointThree));

      assertFalse(simplex.isInVoronoiRegionOfEdge(pointOne, pointTwo, pointThree, pointFour));
      assertFalse(simplex.isInVoronoiRegionOfEdge(pointOne, pointThree, pointTwo, pointFour));
      assertFalse(simplex.isInVoronoiRegionOfEdge(pointOne, pointFour, pointTwo, pointThree));
      assertFalse(simplex.isInVoronoiRegionOfEdge(pointTwo, pointThree, pointOne, pointFour));
      assertFalse(simplex.isInVoronoiRegionOfEdge(pointTwo, pointFour, pointOne, pointThree));
      assertFalse(simplex.isInVoronoiRegionOfEdge(pointThree, pointFour, pointOne, pointTwo));

      assertFalse(simplex.isInVoronoiRegionOfFace(pointOne, pointTwo, pointThree, pointFour));
      assertFalse(simplex.isInVoronoiRegionOfFace(pointOne, pointTwo, pointFour, pointThree));
      assertTrue(simplex.isInVoronoiRegionOfFace(pointOne, pointThree, pointFour, pointTwo));
      assertTrue(simplex.isInVoronoiRegionOfFace(pointTwo, pointThree, pointFour, pointOne));
   }


   @ContinuousIntegrationTest(estimatedDuration = 0.0)
   @Test(timeout = 30000)
   public void testTroublesomeOnes()
   {
      SimplexPolytope simplex = new SimplexPolytope();

      Point3d pointOne = new Point3d(-99.63099726377406, -101.05538475479693, -300.5);
      Point3d pointTwo = new Point3d(-100.32770397312123, 98.22725915430355, -299.5);
      Point3d pointThree = new Point3d(-98.89299179132217, -103.16615426439077, -299.5);
      simplex.setPoints(pointOne, pointTwo, pointThree);

      Point3d closestPointToOrigin = new Point3d();
      simplex.getClosestPointToOriginOnConvexHullAndRemoveUnusedVertices(closestPointToOrigin);
      assertLambdasOnSimplex(simplex);

      simplex = new SimplexPolytope();
      pointOne = new Point3d(-0.5379411195816246, -0.38484462547765474, -1.1060106195237154);
      pointTwo = new Point3d(0.020587720117710262, -0.6621396208363777, 1.3677196242027705);
      pointThree = new Point3d(0.19546617376320796, -0.10118334510837457, -0.1912468320707319);
      Point3d pointFour = new Point3d(-1.277478438460875, -0.22650524035534403, 0.7366990117925241);
      simplex.setPoints(pointOne, pointTwo, pointThree, pointFour);

      simplex.getClosestPointToOriginOnConvexHullAndRemoveUnusedVertices(closestPointToOrigin);
      assertLambdasOnSimplex(simplex);

      simplex = new SimplexPolytope();
      pointOne = new Point3d(-99.63099726377406, -101.05538475479693, -300.5);
      pointTwo = new Point3d(101.80371491802498, 97.55120182650874, -299.5);
      pointThree = new Point3d(-100.32770397312123, 98.22725915430355, -299.5);
      pointFour = new Point3d(-98.89299179132217, -103.16615426439077, -299.5);
      simplex.setPoints(pointOne, pointTwo, pointThree, pointFour);

      simplex.getClosestPointToOriginOnConvexHullAndRemoveUnusedVertices(closestPointToOrigin);
      assertLambdasOnSimplex(simplex);

      simplex = new SimplexPolytope();
      pointOne = new Point3d(-0.2644512361529179, -0.32337063955868706, 0.0);
      pointTwo = new Point3d(0.06360108118380214, 0.07099989145805807, -3.1598253882005345);
      pointThree = new Point3d(0.06360108118382168, 0.07099989145807761, 0.0);
      pointFour = new Point3d(-0.10486116850266392, -0.12249526582978199, 0.0);
      simplex.setPoints(pointOne, pointTwo, pointThree, pointFour);

      simplex.getClosestPointToOriginOnConvexHullAndRemoveUnusedVertices(closestPointToOrigin);
      assertLambdasOnSimplex(simplex);

      simplex = new SimplexPolytope();
      pointOne = new Point3d(8.642810956093754, 1.3207856735513748, -62.923956016892326);
      pointTwo = new Point3d(8.642810956093754, 1.3207856735513748, -62.923956016892326);
      simplex.setPoints(pointOne, pointTwo);

      simplex.getClosestPointToOriginOnConvexHullAndRemoveUnusedVertices(closestPointToOrigin);
      assertLambdasOnSimplex(simplex);
   }

   private void assertLambdasOnSimplex(SimplexPolytope simplex)
   {
      int numberOfPoints = simplex.getNumberOfPoints();

      double lambdaTotal = 0.0;

      for (int i = 0; i < numberOfPoints; i++)
      {
         Point3d point = simplex.getPoint(i);
         double lambda = simplex.getLambda(point);
         assertTrue(lambda >= 0.0);
         assertTrue(lambda <= 1.0);

         lambdaTotal = lambdaTotal + lambda;
      }

      assertEquals(1.0, lambdaTotal, 1e-7);
   }

}
