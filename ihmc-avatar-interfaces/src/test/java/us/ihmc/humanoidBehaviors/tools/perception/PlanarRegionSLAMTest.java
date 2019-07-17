package us.ihmc.humanoidBehaviors.tools.perception;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertTrue;

import java.util.ArrayList;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.junit.jupiter.api.Test;

import us.ihmc.euclid.geometry.ConvexPolygon2D;
import us.ihmc.euclid.geometry.interfaces.Vertex2DSupplier;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.robotics.geometry.PlanarRegion;
import us.ihmc.robotics.geometry.PlanarRegionsList;
import us.ihmc.tools.lists.PairList;

class PlanarRegionSLAMTest
{

   @Test
   void testFindHighConfidencePairsWithSomeSimpleSquares()
   {
      // Two identical squares.
      PlanarRegion squareOne = createASingleSquareCenteredAtOrigin(1.0, 1.0);
      PlanarRegion squareTwo = createASingleSquareCenteredAtOrigin(1.0, 1.0);
      assertHighConfidencePairingBothWays(squareOne, squareTwo);

      // One small square inside a larger square.
      squareOne = createASingleSquareCenteredAtOrigin(1.0, 1.0);
      squareTwo = createASingleSquareCenteredAtOrigin(0.5, 0.5);
      assertHighConfidencePairingBothWays(squareOne, squareTwo);

      Vector3D translationOne = new Vector3D(10.1, -20.3, 7.6);

      // One large, one small translated same amounts.
      squareOne = createASingleTranslatedSquare(translationOne, 1.0, 1.0);
      squareTwo = createASingleTranslatedSquare(translationOne, 0.5, 0.5);
      assertHighConfidencePairingBothWays(squareOne, squareTwo);

      // One large, one small, but translated a significant amount in z, so should not be a pair.
      Vector3D largeZOffset = new Vector3D(0.0, 0.0, 0.2);
      Vector3D translationTwo = new Vector3D(translationOne);
      translationTwo.add(largeZOffset);
      squareOne = createASingleTranslatedSquare(translationOne, 1.0, 1.0);
      squareTwo = createASingleTranslatedSquare(translationTwo, 0.5, 0.5);
      assertNotAHighConfidencePairing(squareOne, squareTwo);

      // One large, one small, shifted a bit but still overlapping, with a small z delta.
      double epsilon = 1e-3;
      Vector3D smallZOffset = new Vector3D(0.0, 0.0, epsilon);
      Vector3D overlappingHorizontalOffset = new Vector3D(0.1, 0.2, 0.0);

      translationTwo = new Vector3D(translationOne);
      translationTwo.add(smallZOffset);
      translationTwo.add(overlappingHorizontalOffset);
      squareOne = createASingleTranslatedSquare(translationOne, 1.0, 1.0);
      squareTwo = createASingleTranslatedSquare(translationTwo, 0.5, 0.5);
      assertHighConfidencePairingBothWays(squareOne, squareTwo);

      translationTwo = new Vector3D(translationOne);
      translationTwo.sub(smallZOffset);
      translationTwo.sub(overlappingHorizontalOffset);
      squareOne = createASingleTranslatedSquare(translationOne, 1.0, 1.0);
      squareTwo = createASingleTranslatedSquare(translationTwo, 0.5, 0.5);
      assertHighConfidencePairingBothWays(squareOne, squareTwo);

      // One large, one small, both rotated differently in yaw should still overlap.
      translationTwo = new Vector3D(translationOne);
      squareOne = createASingleTranslatedAndYawedSquare(translationOne, Math.PI / 2.0, 1.0, 1.0);
      squareTwo = createASingleTranslatedAndYawedSquare(translationTwo, -Math.PI / 3.0, 0.5, 0.5);
      assertHighConfidencePairingBothWays(squareOne, squareTwo);

      // Two pitched squares
      double yaw = 0.0;
      double pitch = Math.PI;
      double roll = 0.0;

      squareOne = createASingleSquare(translationOne, yaw, pitch, roll, 1.0, 1.0);
      squareOne = createASingleSquare(translationOne, yaw, pitch, roll, 1.0, 1.0);
      assertHighConfidencePairingBothWays(squareOne, squareTwo);

      // Two randomly oriented squares
      yaw = Math.PI * 0.5;
      pitch = Math.PI * 0.3;
      roll = -Math.PI * 0.7;

      squareOne = createASingleSquare(translationOne, yaw, pitch, roll, 0.5, 0.5);
      squareOne = createASingleSquare(translationOne, yaw, pitch, roll, 1.0, 1.0);
      assertHighConfidencePairingBothWays(squareOne, squareTwo);

   }

   private void assertHighConfidencePairingBothWays(PlanarRegion regionOne, PlanarRegion regionTwo)
   {
      assertHighConfidencePairing(regionOne, regionTwo);
      assertHighConfidencePairing(regionTwo, regionOne);
   }

   private void assertHighConfidencePairing(PlanarRegion regionOne, PlanarRegion regionTwo)
   {
      PlanarRegionsList map = new PlanarRegionsList();
      PlanarRegionsList newData = new PlanarRegionsList();

      map.addPlanarRegion(regionOne);
      newData.addPlanarRegion(regionTwo);

      PairList<PlanarRegion, PlanarRegion> highConfidencePairs = PlanarRegionSLAM.findHighConfidencePairs(map, newData);

      assertEquals(1, highConfidencePairs.size());
      ImmutablePair<PlanarRegion, PlanarRegion> pair = highConfidencePairs.get(0);

      assertTrue(regionOne == pair.getLeft());
      assertTrue(regionTwo == pair.getRight());
   }

   private void assertNotAHighConfidencePairing(PlanarRegion regionOne, PlanarRegion regionTwo)
   {
      PlanarRegionsList map = new PlanarRegionsList();
      PlanarRegionsList newData = new PlanarRegionsList();

      map.addPlanarRegion(regionOne);
      newData.addPlanarRegion(regionTwo);

      PairList<PlanarRegion, PlanarRegion> highConfidencePairs = PlanarRegionSLAM.findHighConfidencePairs(map, newData);

      assertTrue(highConfidencePairs.isEmpty());
   }

   private PlanarRegion createASingleTranslatedAndYawedSquare(Vector3D translation, double yaw, double xSize, double ySize)
   {
      Point2D minimumPoint = new Point2D(-xSize / 2.0, -ySize / 2.0);
      Point2D maximumPoint = new Point2D(xSize / 2.0, ySize / 2.0);

      return createASingleSquare(translation, yaw, 0.0, 0.0, minimumPoint, maximumPoint);
   }

   private PlanarRegion createASingleTranslatedSquare(Vector3D translation, double xSize, double ySize)
   {
      Point2D minimumPoint = new Point2D(-xSize / 2.0, -ySize / 2.0);
      Point2D maximumPoint = new Point2D(xSize / 2.0, ySize / 2.0);

      return createASingleSquare(translation, 0.0, 0.0, 0.0, minimumPoint, maximumPoint);
   }

   private PlanarRegion createASingleSquareCenteredAtOrigin(double xSize, double ySize)
   {
      Point2D minimumPoint = new Point2D(-xSize / 2.0, -ySize / 2.0);
      Point2D maximumPoint = new Point2D(xSize / 2.0, ySize / 2.0);

      return createASingleSquare(new Vector3D(), 0.0, 0.0, 0.0, minimumPoint, maximumPoint);
   }

   private PlanarRegion createASingleSquare(Vector3D translation, double yaw, double pitch, double roll, double xSize, double ySize)
   {
      Point2D minimumPoint = new Point2D(-xSize / 2.0, -ySize / 2.0);
      Point2D maximumPoint = new Point2D(xSize / 2.0, ySize / 2.0);

      return createASingleSquare(translation, yaw, pitch, roll, minimumPoint, maximumPoint);
   }

   private PlanarRegion createASingleSquare(Vector3D translation, double yaw, double pitch, double roll, Point2D minimumPoint, Point2D maximumPoint)
   {
      RigidBodyTransform transformToWorld = new RigidBodyTransform();
      transformToWorld.setTranslation(translation);
      transformToWorld.setRotationYawPitchRoll(yaw, pitch, roll);

      ArrayList<Point2D> vertices = new ArrayList<Point2D>();
      vertices.add(new Point2D(minimumPoint));
      vertices.add(new Point2D(minimumPoint.getX(), maximumPoint.getY()));
      vertices.add(new Point2D(maximumPoint));
      vertices.add(new Point2D(maximumPoint.getX(), minimumPoint.getY()));

      ConvexPolygon2D convexPolygon = new ConvexPolygon2D(Vertex2DSupplier.asVertex2DSupplier(vertices));
      PlanarRegion square = new PlanarRegion(transformToWorld, convexPolygon);
      return square;

      //      List<ConvexPolygon2D> polygonsRegion1 = new ArrayList<>();
      //
      //      ConvexPolygon2D polygon11 = new ConvexPolygon2D();
      //      polygon11.addVertex(0.0, 0.0);
      //      polygon11.addVertex(2.0, 0.0);
      //      polygon11.addVertex(2.0, 0.5);
      //      polygon11.addVertex(0.0, 0.5);
      //      polygon11.update();
      //      polygonsRegion1.add(polygon11);
      //
      //      ConvexPolygon2D polygon12 = new ConvexPolygon2D();
      //      polygon12.addVertex(1.0, 0.0);
      //      polygon12.addVertex(2.0, 0.0);
      //      polygon12.addVertex(2.0, -1.5);
      //      polygon12.addVertex(1.0, -1.5);
      //      polygon12.update();
      //      polygonsRegion1.add(polygon12);
      //
      //      RigidBodyTransform transform1 = new RigidBodyTransform();
      //      PlanarRegion region1 = new PlanarRegion(transform1, polygonsRegion1);
      //
      //      List<ConvexPolygon2D> polygonsRegion2 = new ArrayList<>();
      //      ConvexPolygon2D polygon21 = new ConvexPolygon2D();
      //      polygon21.addVertex(-1.0, 0.1);
      //      polygon21.addVertex(1.0, 0.1);
      //      polygon21.addVertex(1.0, -0.1);
      //      polygon21.addVertex(-1.0, -0.1);
      //      polygon21.update();
      //      polygonsRegion2.add(polygon21);
      //
      //      ConvexPolygon2D polygon22 = new ConvexPolygon2D();
      //      polygon22.addVertex(1.5, 0.1);
      //      polygon22.addVertex(2.0, 0.1);
      //      polygon22.addVertex(2.0, -0.1);
      //      polygon22.addVertex(1.5, -0.1);
      //      polygon22.update();
      //      polygonsRegion2.add(polygon22);
      //
      //      RigidBodyTransform transform2 = new RigidBodyTransform();
      //      transform2.setTranslation(0.5, 0.0, 0.0);
      //      transform2.appendYawRotation(-Math.PI / 4.0);
      //      transform2.appendRollRotation(Math.PI / 2.0);
      //      PlanarRegion region2 = new PlanarRegion(transform2, polygonsRegion2);
   }

}
