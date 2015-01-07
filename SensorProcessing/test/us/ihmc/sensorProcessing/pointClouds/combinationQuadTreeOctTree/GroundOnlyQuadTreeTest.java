package us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;
import static org.junit.Assert.fail;

import java.util.Random;

import org.junit.Test;

import us.ihmc.utilities.dataStructures.AbstractHeightMapTest;
import us.ihmc.utilities.math.dataStructures.HeightMap;

public class GroundOnlyQuadTreeTest extends AbstractHeightMapTest
{
   private static final float ALTERNATE_HEIGHT_THRESHOLD = 0.01f;
   private double epsilon = 1e-6;
   private static final float HEIGHT_THRESHOLD = 0.05f;

   @Test(timeout=300000)
   public void testSimpleThings()
   {
      float xMin = 0.0f;
      float yMin = 0.0f;
      float xMax = 1.0f;
      float yMax = 1.0f;

      GroundOnlyQuadTree quadTree1 = new GroundOnlyQuadTree(xMin, yMin, xMax, yMax, 1.0, 0.1, 100000);
      GroundOnlyQuadTree quadTree = quadTree1;
      assertNull(quadTree.get(0.2f, 0.2f));
      quadTree.addToQuadtree(0.5f, 0.5f, 20.0f);
      assertEquals(20.0f, quadTree.get(0.5f, 0.5f), epsilon);
      quadTree.addToQuadtree(0.5f, 0.5f, 1.0f);
      assertEquals(1.0f, quadTree.get(0.5f, 0.5f), epsilon);
      assertEquals(true, quadTree.getLeafNodeAtLocation(0.5f, 0.5f).getMetaData().getIsStuffAboveMe());
   }

   @Test(timeout=300000)
   public void testGettingAreas()
   {
      super.testGettingAreas();
   }

   @Test(timeout=300000)
   public void testSomethingAbove() throws Exception
   {
      float xMin = 0.0f;
      float yMin = 0.0f;
      float xMax = 2.0f;
      float yMax = 2.0f;

      GroundOnlyQuadTree quadTree1 = new GroundOnlyQuadTree(xMin, yMin, xMax, yMax, 1.0, 0.1, 100000);
      GroundOnlyQuadTree quadTree = quadTree1;

      float ground = 0.1f;
      float higher = 1.0f;
      float unseen = Float.NaN;
      float bottom = -5.0f;
      float tiptop = 5.0f;

      float[][] testLocations = new float[][]
      {
         {0.5f, 0.5f},    //
         {0.5f, 1.5f},    //
         {1.5f, 0.5f},    //
         {1.5f, 1.5f}
      };

      int[] indexesToSet = new int[]
      {
         0, 1, 2, 3,    //
         0, 1, 2, 3,    //
         0,    //
         0
      };

      float[] valuesToSet = new float[]
      {
         ground, higher, ground, higher,    //
         higher, ground, higher, ground,    //
         tiptop,    //
         bottom
      };

      int[] expectedLeafNumbers = new int[]
      {
         1, 4, 4, 4,    //
         4, 4, 4, 1,    //
         1,    //
         4
      };

      float[][] expectedAtAllLocations = new float[][]
      {
         {ground, ground, ground, ground}, {ground, higher, unseen, unseen}, {ground, higher, ground, unseen}, {ground, higher, ground, higher},    //
         {ground, higher, ground, higher}, {ground, ground, ground, higher}, {ground, ground, ground, higher}, {ground, ground, ground, ground},    //
         {ground, ground, ground, ground},    //
         {bottom, ground, ground, ground}
      };

      boolean[][] expectedStuffAboveMeFlagAtAllLocations = new boolean[][]
      {
         {false, false, false, false}, {false, false, false, false}, {false, false, false, false}, {false, false, false, false},    //
         {true, false, false, false}, {true, true, false, false}, {true, true, true, false}, {true, true, true, true},    //
         {true, true, true, true},    //
         {true, true, true, true}
      };

      int numTests = expectedLeafNumbers.length;
      for (int i = 0; i < numTests; i++)
      {
         int index = indexesToSet[i];
         float[] point = testLocations[index];
         float x = point[0];
         float y = point[1];
         float value = valuesToSet[i];
         System.out.println("Running test " + i + " putting a point at (" + x + "," + y + ") which is " + value);
         quadTree.addToQuadtree(x, y, value);
         String wrongQuantityString = "test index " + i;
         for (int j = 0; j < testLocations.length; j++)
         {
            float expectedAtLocation = expectedAtAllLocations[i][j];
            float xToTest = testLocations[j][0];
            float yToTest = testLocations[j][1];
            String testString = "test " + i + " location " + j + " with (x,y)=(" + xToTest + "," + yToTest + ") and value = " + expectedAtLocation + "";
            Float float1 = quadTree.get(xToTest, yToTest);
            if (null == float1)
               if (Float.isNaN(expectedAtLocation))
                  continue;
               else
                  fail(testString + "value is unexpectedly null!");
            else
               assertEquals(testString, expectedAtLocation, float1, epsilon);

            assertEquals(expectedStuffAboveMeFlagAtAllLocations[i][j], quadTree.getLeafNodeAtLocation(xToTest, yToTest).getMetaData().getIsStuffAboveMe());
         }

         assertEquals(wrongQuantityString, expectedLeafNumbers[i], quadTree.listAllLeafNodes().size());
      }
   }

/*   @Test(timeout=300000)
   public void testUnhandledPoints()
   {
      super.testUnhandledPoints();
   }*/

   @Override
   public HeightMap getHeightMap(double minX, double minY, double maxX, double maxY, double resolution)
   {
      return new GroundOnlyQuadTree(minX, minY, maxX, maxY, resolution, HEIGHT_THRESHOLD, 100000);
   }

   protected GroundOnlyQuadTree createDefaultQuadTree(float quadTreeResolution)
   {
      float xMin = 0.0f;
      float yMin = 0.0f;
      float xMax = 4.0f;
      float yMax = 4.0f;

      float heightThreshold = ALTERNATE_HEIGHT_THRESHOLD;

      return new GroundOnlyQuadTree(xMin, yMin, xMax, yMax, quadTreeResolution, heightThreshold, 100000);
   }

   @Test(timeout=300000)
   public void testPuttingDifferentHeightPointsAtDifferentLocations() throws Exception
   {
      GroundOnlyQuadTree quadTree = createDefaultQuadTree(0.5f);
      testEmptyTree(quadTree);

      // first point
      // should remain a single quad and should all have the same value
      double expected = 1.0;
      quadTree.addToQuadtree(1.0f, 1.0f, (float) expected);
      verifyLevelOneAtHeight(quadTree, expected);
      assertTrue(quadTree.listAllLeafNodes().size() == 1);

      // second point
      // should become four quads and each has a different value
      expected = 0.5;
      quadTree.addToQuadtree(1.0f, 3.0f, (float) expected);
      assertTrue(quadTree.listAllLeafNodes().size() == 4);
      double actual = quadTree.get(1.0, 1.0);
      expected = 1.0;
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(1.0, 3.0);
      expected = 0.5;
      assertEquals(expected, actual, epsilon);
   }

   @Test(timeout=300000)
   public void testPuttingSimilarHeightPoints() throws Exception
   {
      GroundOnlyQuadTree quadTree = createDefaultQuadTree(0.5f);

      // no points
      testEmptyTree(quadTree);

      // first point
      // should remain a single quad and should all have the same value
      double expected = 1.0;
      quadTree.addToQuadtree(1.0f, 1.0f, (float) expected);
      assertTrue(quadTree.listAllLeafNodes().size() == 1);
      verifyLevelOneAtHeight(quadTree, expected);

      // more points at height within threshold
      // should remain one quad
      int numberOfPoints = 50;
      Random random = new Random(777);
      for (int i = 0; i < numberOfPoints; i++)
      {
         quadTree.addToQuadtree(1.0f, 3.0f, (float) expected + (ALTERNATE_HEIGHT_THRESHOLD * random.nextFloat()));
         assertTrue(quadTree.listAllLeafNodes().size() == 1);
         verifyLevelOneAtHeight(quadTree, expected);
      }
   }

   @Test(timeout=300000)
   public void testAll() throws Exception
   {
      GroundOnlyQuadTree quadTree = createDefaultQuadTree(0.5f);

      // no points
      testEmptyTree(quadTree);

      // first point
      double expected = 1.0;
      quadTree.addToQuadtree(1.0f, 1.0f, (float) expected);
      assertTrue(quadTree.listAllLeafNodes().size() == 1);
      double actual = quadTree.get(1.0, 1.0);
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(1.0, 3.0);
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(3.0, 1.0);
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(3.0, 3.0);
      assertEquals(expected, actual, epsilon);

      // second point in different quad
      quadTree.addToQuadtree(1.0f, 3.0f, (float) 0.5);
      assertTrue(quadTree.listAllLeafNodes().size() == 4);
      actual = quadTree.get(1.0, 1.0);
      expected = 1.0;
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(1.0, 3.0);
      expected = 0.5;
      assertEquals(expected, actual, epsilon);
      assertNull(quadTree.get(3.0, 1.0));
      assertNull(quadTree.get(3.0, 3.0));

      // third point in different quad
      quadTree.addToQuadtree(3.0f, 3.0f, (float) 1.5);
      assertTrue(quadTree.listAllLeafNodes().size() == 4);
      actual = quadTree.get(1.0, 1.0);
      expected = 1.0;
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(1.0, 3.0);
      expected = 0.5;
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(3.0, 3.0);
      expected = 1.5;
      assertEquals(expected, actual, epsilon);
      assertNull(quadTree.get(3.0, 1.0));

      // fourth point in different quad
      quadTree.addToQuadtree(3.0f, 1.0f, (float) 0.0);
      assertTrue(quadTree.listAllLeafNodes().size() == 4);
      actual = quadTree.get(1.0, 1.0);
      expected = 1.0;
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(1.0, 3.0);
      expected = 0.5;
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(3.0, 3.0);
      expected = 1.5;
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(3.0, 1.0);
      expected = 0.0;
      assertEquals(expected, actual, epsilon);

      // fifth point (second point in same quad)
      quadTree.addToQuadtree(0.5f, 0.5f, (float) 0.5);
      assertTrue(quadTree.listAllLeafNodes().size() == 7);
      actual = quadTree.get(1.0, 1.0);
      expected = 1.0;
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(1.0, 3.0);
      expected = 0.5;
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(3.0, 3.0);
      expected = 1.5;
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(3.0, 1.0);
      expected = 0.0;
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(0.5, 0.5);
      expected = 0.5;
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(1.5, 1.5);
      expected = 1.0;
      assertEquals(expected, actual, epsilon);
      assertNull(quadTree.get(0.5, 1.5));
      assertNull(quadTree.get(1.5, 0.5));

      // sixth point (second point in same sub-quad)
      quadTree.addToQuadtree(0.25f, 0.25f, (float) 0.25);
      assertTrue(quadTree.listAllLeafNodes().size() == 10);
      actual = quadTree.get(1.0, 1.0);
      expected = 1.0;
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(1.0, 3.0);
      expected = 0.5;
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(3.0, 3.0);
      expected = 1.5;
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(3.0, 1.0);
      expected = 0.0;
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(0.5, 0.5);
      expected = 0.5;
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(1.5, 1.5);
      expected = 1.0;
      assertEquals(expected, actual, epsilon);
      assertNull(quadTree.get(0.5, 1.5));
      assertNull(quadTree.get(1.5, 0.5));
      actual = quadTree.get(0.25, 0.25);
      expected = 0.25;
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(0.5, 0.5);
      expected = 0.5;
      assertEquals(expected, actual, epsilon);
      assertNull(quadTree.get(0.25, 0.75));
      assertNull(quadTree.get(0.75, 0.25));

      // seventh point (third point in same sub-sub-quad and below resolution)
      quadTree.addToQuadtree(0.1f, 0.1f, (float) -7.0);
      assertTrue(quadTree.listAllLeafNodes().size() == 10);
      actual = quadTree.get(0.25, 0.25);
      expected = -7.0;
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(1.0, 1.0);
      expected = 1.0;
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(1.0, 3.0);
      expected = 0.5;
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(3.0, 3.0);
      expected = 1.5;
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(3.0, 1.0);
      expected = 0.0;
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(0.5, 0.5);
      expected = 0.5;
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(1.5, 1.5);
      expected = 1.0;
      assertEquals(expected, actual, epsilon);
      assertNull(quadTree.get(0.5, 1.5));
      assertNull(quadTree.get(1.5, 0.5));
      actual = quadTree.get(0.5, 0.5);
      expected = 0.5;
      assertEquals(expected, actual, epsilon);
      assertNull(quadTree.get(0.25, 0.75));
      assertNull(quadTree.get(0.75, 0.25));

      // point close in height
      quadTree.addToQuadtree(1.1f, 3.1f, (float) 0.495);
      assertTrue(quadTree.listAllLeafNodes().size() == 10);
      actual = quadTree.get(1.0, 3.0);
      expected = 0.5;
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(0.5, 2.2);
      expected = 0.5;
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(1.0, 1.0);
      expected = 1.0;
      assertEquals(expected, actual, epsilon);

      /*
       * Removal is not yet tested in GroundOnlyQuadTree, since it is not used.
       *
       *
       * // remove smallest point
       *   quadTree.remove(0.2f, 0.2f);
       *   assertEquals(7,quadTree.listAllLeafNodes().size());
       *   actual = quadTree.get(1.0, 1.0);
       *   expected = 1.0;
       *   assertEquals(expected, actual, epsilon);
       *   actual = quadTree.get(1.0, 3.0);
       *   expected = 0.5;
       *   assertEquals(expected, actual, epsilon);
       *   actual = quadTree.get(3.0, 3.0);
       *   expected = 1.5;
       *   assertEquals(expected, actual, epsilon);
       *   actual = quadTree.get(3.0, 1.0);
       *   expected = 0.0;
       *   assertEquals(expected, actual, epsilon);
       *   actual = quadTree.get(0.5, 0.5);
       *   expected = 0.5;
       *   assertEquals(expected, actual, epsilon);
       *   actual = quadTree.get(1.5, 1.5);
       *   expected = 1.0;
       *   assertEquals(expected, actual, epsilon);
       *   assertNull(quadTree.get(0.5, 1.5));
       *   assertNull(quadTree.get(1.5, 0.5));
       *   actual = quadTree.get(0.25, 0.25);
       *   expected = 0.5;
       *   assertEquals(expected, actual, epsilon);
       *   actual = quadTree.get(0.25, 0.75);
       *   expected = 0.5;
       *   assertEquals(expected, actual, epsilon);
       *   actual = quadTree.get(0.75, 0.25);
       *   expected = 0.5;
       *   assertEquals(expected, actual, epsilon);
       *
       *   // remove null leaf
       *   quadTree.remove(0.25f, 0.75f);
       *   assertTrue(quadTree.listAllLeafNodes().size() == 4);
       *   actual = quadTree.get(1.0, 1.0);
       *   expected = 1.0;
       *   assertEquals(expected, actual, epsilon);
       *   actual = quadTree.get(1.0, 3.0);
       *   expected = 0.5;
       *   assertEquals(expected, actual, epsilon);
       *   actual = quadTree.get(3.0, 3.0);
       *   expected = 1.5;
       *   assertEquals(expected, actual, epsilon);
       *   actual = quadTree.get(3.0, 1.0);
       *   expected = 0.0;
       *   assertEquals(expected, actual, epsilon);
       *   actual = quadTree.get(0.5, 0.5);
       *   expected = 1.0;
       *   assertEquals(expected, actual, epsilon);
       *   actual = quadTree.get(1.5, 1.5);
       *   expected = 1.0;
       *   assertEquals(expected, actual, epsilon);
       *   actual = quadTree.get(0.5, 1.5);
       *   expected = 1.0;
       *   assertEquals(expected, actual, epsilon);
       *   actual = quadTree.get(1.5, 0.5);
       *   expected = 1.0;
       *   assertEquals(expected, actual, epsilon);
       *   actual = quadTree.get(0.5, 0.5);
       *   expected = 1.0;
       *   assertEquals(expected, actual, epsilon);
       *   actual = quadTree.get(0.25, 0.25);
       *   expected = 1.0;
       *   assertEquals(expected, actual, epsilon);
       *   actual = quadTree.get(0.25, 0.75);
       *   expected = 1.0;
       *   assertEquals(expected, actual, epsilon);
       *   actual = quadTree.get(0.75, 0.25);
       *   expected = 1.0;
       *   assertEquals(expected, actual, epsilon);
       *
       *   // remove another leaf
       *   quadTree.remove(0.5f, 0.5f);
       *   assertTrue(quadTree.listAllLeafNodes().size() == 4);
       *   assertNull(quadTree.get(1.0, 1.0));
       *   actual = quadTree.get(1.0, 3.0);
       *   expected = 0.5;
       *   assertEquals(expected, actual, epsilon);
       *   actual = quadTree.get(3.0, 3.0);
       *   expected = 1.5;
       *   assertEquals(expected, actual, epsilon);
       *   actual = quadTree.get(3.0, 1.0);
       *   expected = 0.0;
       *   assertEquals(expected, actual, epsilon);
       *   assertNull(quadTree.get(1.5, 1.5));
       *   assertNull(quadTree.get(0.5, 1.5));
       *   assertNull(quadTree.get(1.5, 0.5));
       *   assertNull(quadTree.get(1.5, 0.5));
       *   assertNull(quadTree.get(0.5, 0.5));
       *   assertNull(quadTree.get(0.25, 0.25));
       *
       *   // remove another leaf
       *   quadTree.remove(3.0, 1.0f);
       *   assertTrue(quadTree.listAllLeafNodes().size() == 4);
       *   actual = quadTree.get(1.0, 3.0);
       *   expected = 0.5;
       *   assertEquals(expected, actual, epsilon);
       *   actual = quadTree.get(3.0, 3.0);
       *   expected = 1.5;
       *   assertEquals(expected, actual, epsilon);
       *   assertNull(quadTree.get(3.0, 1.0));
       *   assertNull(quadTree.get(1.0, 1.0));
       *
       *   // remove another leaf
       *   quadTree.remove(3.0, 3.0f);
       *   assertTrue(quadTree.listAllLeafNodes().size() == 1);
       *   actual = quadTree.get(1.0, 3.0);
       *   expected = 0.5;
       *   assertEquals(expected, actual, epsilon);
       *   actual = quadTree.get(3.0, 3.0);
       *   expected = 0.5;
       *   assertEquals(expected, actual, epsilon);
       *   actual = quadTree.get(3.0, 1.0);
       *   expected = 0.5;
       *   assertEquals(expected, actual, epsilon);
       *   actual = quadTree.get(1.0, 1.0);
       *   expected = 0.5;
       *   assertEquals(expected, actual, epsilon);
       *
       *   // remove another leaf
       *   quadTree.remove(1.0, 3.0f);
       *   assertTrue(quadTree.listAllLeafNodes().size() == 1);
       *   assertNull(quadTree.get(1.0, 3.0));
       *   assertNull(quadTree.get(3.0, 3.0));
       *   assertNull(quadTree.get(3.0, 1.0));
       *   assertNull(quadTree.get(1.0, 1.0));
       */
   }

   @Test(timeout=300000)
   public void testMerging() throws Exception
   {
      /*
       * Merging in GroundOnlyQuadTrees differs from merging in QuadTrees.
       */

      GroundOnlyQuadTree quadTree = createDefaultQuadTree(2.0f);
      int actualCount = quadTree.countNodes();
      int expectedCount = 1;
      assertEquals(expectedCount, actualCount);

      // add point
      quadTree.addToQuadtree(1.0f, 1.0f, (float) 1.0);
      actualCount = quadTree.countNodes();
      expectedCount = 1;
      assertEquals(expectedCount, actualCount);

      // add point
      quadTree.addToQuadtree(1.0f, 3.0f, (float) 0.5);
      actualCount = quadTree.countNodes();
      expectedCount = 5;
      assertEquals(expectedCount, actualCount);

      // add point
      quadTree.addToQuadtree(3.0f, 3.0f, (float) 1.5);
      actualCount = quadTree.countNodes();
      expectedCount = 5;
      assertEquals(expectedCount, actualCount);

      // add point
      quadTree.addToQuadtree(3.0f, 1.0f, (float) 0.75);
      actualCount = quadTree.countNodes();
      expectedCount = 5;
      assertEquals(expectedCount, actualCount);

      // test values
      double expected = 1.0;
      double actual = quadTree.get(1.0, 1.0);
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(1.0, 3.0);
      expected = 0.5;
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(3.0, 3.0);
      expected = 1.5;
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(3.0, 1.0);
      expected = 0.75;
      assertEquals(expected, actual, epsilon);

      // change values
      quadTree.addToQuadtree(3.0f, 3.0f, (float) 0.5);
      actualCount = quadTree.countNodes();
      expectedCount = 5;
      assertEquals(expectedCount, actualCount);

      // test values
      actual = quadTree.get(1.0, 1.0);
      expected = 1.0;
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(1.0, 3.0);
      expected = 0.5;
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(3.0, 3.0);
      expected = 0.5;
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(3.0, 1.0);
      expected = 0.75;
      assertEquals(expected, actual, epsilon);

      // change values
      quadTree.addToQuadtree(3.0f, 1.0f, (float) 0.5);
      actualCount = quadTree.countNodes();
      expectedCount = 5;
      assertEquals(expectedCount, actualCount);

      // test values
      actual = quadTree.get(1.0, 1.0);
      expected = 1.0;
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(1.0, 3.0);
      expected = 0.5;
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(3.0, 3.0);
      expected = 0.5;
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(3.0, 1.0);
      expected = 0.5;
      assertEquals(expected, actual, epsilon);

      // change values
      quadTree.addToQuadtree(1.0f, 1.0f, (float) 0.5);

      // test values
      expected = 0.5;
      verifyLevelOneAtHeight(quadTree, expected);

      // won't merge because one has not seen stuff above
      actualCount = quadTree.countNodes();
      expectedCount = 5;
      assertEquals(expectedCount, actualCount);

      // change values
      quadTree.addToQuadtree(1.0f, 3.0f, (float) 1.5);

      // shouldn'e change because it is higher
      assertEquals(expectedCount, actualCount);

      // should have merged because all have stuff above now
      actualCount = quadTree.countNodes();
      expectedCount = 1;
      assertEquals(expectedCount, actualCount);

      // change value to divide
      quadTree.addToQuadtree(1.0f, 1.0f, (float) 0.45);
      actualCount = quadTree.countNodes();
      expectedCount = 5;
      assertEquals(expectedCount, actualCount);

      // test values
      actual = quadTree.get(1.0, 1.0);
      expected = 0.45;
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(1.0, 3.0);
      expected = 0.5;
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(3.0, 3.0);
      expected = 0.5;
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(3.0, 1.0);
      expected = 0.5;
      assertEquals(expected, actual, epsilon);

   }

   @Test(timeout=300000)
   public void testClear() throws Exception
   {
      GroundOnlyQuadTree quadTree = createDefaultQuadTree(0.5f);

      // no points
      double expected = 0.0;
      testEmptyTree(quadTree);

      // first point
      expected = 1.0;
      quadTree.addToQuadtree(0.9f, 0.90f, (float) expected);
      assertTrue(quadTree.listAllLeafNodes().size() == 1);
      verifyLevelOneAtHeight(quadTree, expected);

      // clear tree
      quadTree.clear();
      assertTrue(quadTree.listAllLeafNodes().size() == 1);
      assertNull(quadTree.get(1.0, 1.0));

      // first point again
      expected = 1.0;
      quadTree.addToQuadtree(0.9f, 0.90f, (float) expected);
      assertTrue(quadTree.listAllLeafNodes().size() == 1);
      verifyLevelOneAtHeight(quadTree, expected);
   }

   private void testEmptyTree(GroundOnlyQuadTree quadTree)
   {
      assertNull(quadTree.get(1.0f, 1.0f));
      assertTrue(quadTree.listAllLeafNodes().size() == 1);
   }

   private void verifyLevelOneAtHeight(GroundOnlyQuadTree quadTree, double expected)
   {
      double actual = quadTree.get(1.0f, 1.0f);
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(1.0f, 3.0f);
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(3.0f, 1.0f);
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(3.0f, 3.0f);
      assertEquals(expected, actual, epsilon);
   }

/*   private void verifyLevelOneAtDefaultHeight(GroundOnlyQuadTree quadTree)
   {
      double actual = quadTree.get(1.0, 1.0);
      double expected = 1.0;
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(1.0, 3.0);
      expected = 0.5;
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(3.0, 3.0);
      expected = 1.5;
      assertEquals(expected, actual, epsilon);
      actual = quadTree.get(3.0, 1.0);
      expected = 0.0;
      assertEquals(expected, actual, epsilon);
   }*/

   @Test(timeout=300000)
   public void testGetMinX() throws Exception
   {
      GroundOnlyQuadTree quadTree = createDefaultQuadTree(0.5f);

      double expected = 0.0f;
      double actual = quadTree.getMinX();
      assertEquals(expected, actual, epsilon);
   }

   @Test(timeout=300000)
   public void testGetMaxX() throws Exception
   {
      GroundOnlyQuadTree quadTree = createDefaultQuadTree(0.5f);

      double expected = 4.0f;
      double actual = quadTree.getMaxX();
      assertEquals(expected, actual, epsilon);
   }

   @Test(timeout=300000)
   public void testGetMinY() throws Exception
   {
      GroundOnlyQuadTree quadTree = createDefaultQuadTree(0.5f);

      double expected = 0.0f;
      double actual = quadTree.getMinY();
      assertEquals(expected, actual, epsilon);
   }

   @Test(timeout=300000)
   public void testGetMaxY() throws Exception
   {
      GroundOnlyQuadTree quadTree = createDefaultQuadTree(0.5f);

      double expected = 4.0f;
      double actual = quadTree.getMaxY();
      assertEquals(expected, actual, epsilon);
   }

}
