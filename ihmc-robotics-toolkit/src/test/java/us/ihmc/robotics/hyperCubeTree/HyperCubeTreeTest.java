package us.ihmc.robotics.hyperCubeTree;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;

import java.util.concurrent.locks.LockSupport;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class HyperCubeTreeTest
{

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPutGetNoSplit()
   {
      int dimensionality = 3;
      HyperCubeTree<Double, Void> tree = setupTree(dimensionality, 1.0);
      double[] point = new double[] {0.25, 0.6, 0.6};
      Double testValue1 = 0.25;
      Double testValue2 = 0.7;

      assertNull(tree.get(point));
      tree.put(point, testValue1);
      assertEquals(testValue1, tree.get(point).getValue());
      assertEquals(1, tree.listAllLeafNodes().size());

      point = new double[] {0.25, 0.25, 0.25};
      assertEquals(testValue1, tree.get(point).getValue());

      point = new double[] {0.25, 0.25, 0.75};
      assertEquals(testValue1, tree.get(point).getValue());
      point = new double[] {0.25, 0.75, 0.25};
      assertEquals(testValue1, tree.get(point).getValue());
      point = new double[] {0.75, 0.25, 0.25};
      assertEquals(testValue1, tree.get(point).getValue());

      point = new double[] {0.25, 0.25, 0.25};
      tree.put(point, testValue2);
      assertFalse(tree.getRootNode().hasChildren());
      assertEquals(1, tree.listAllLeafNodes().size());
      assertEquals(testValue2, tree.get(point).getValue());

      point = new double[] {0.25, 0.25, 0.75};
      assertEquals(testValue2, tree.get(point).getValue());
      point = new double[] {0.25, 0.75, 0.25};
      assertEquals(testValue2, tree.get(point).getValue());
      point = new double[] {0.75, 0.25, 0.25};
      assertEquals(testValue2, tree.get(point).getValue());

   }

   private HyperCubeTree<Double, Void> setupTree(int dimensionality, double resolution)
   {
      OneDimensionalBounds[] bounds = new OneDimensionalBounds[dimensionality];
      for (int i = 0; i < dimensionality; i++)
      {
         bounds[i] = new OneDimensionalBounds(0.0, 1.0);
      }

      SimpleLossyDoubleTree tree = new SimpleLossyDoubleTree(bounds, resolution, 1e-3);

      return tree;
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPutGetRemove2D()
   {
      int dimensionality = 2;
      HyperCubeTree<Double, Void> tree = setupTree(dimensionality, 0.5);
      double low = 0.25;
      double high = 0.75;
      double[] point = new double[] {low, high};
      Double testValue1 = 0.1;
      Double testValue2 = 0.6;

      assertNull(tree.get(point));
      tree.put(point, testValue1);
      assertEquals(testValue1, tree.get(point).getValue());

      testQuadNodeFourPointsAreTheSame(tree, low, high, testValue1);
      point = new double[] {high, high};
      tree.put(point, testValue2);
      assertEquals(4, tree.listAllLeafNodes().size());
      HyperCubeLeaf<Double> leafToRemove = tree.get(point);

      point = new double[] {low, low};
      assertNull(tree.get(point));
      point = new double[] {low, high};
      assertEquals(testValue1, tree.get(point).getValue());
      point = new double[] {high, low};
      assertNull(tree.get(point));
      point = new double[] {high, high};
      assertEquals(testValue2, tree.get(point).getValue());

      tree.remove(leafToRemove);

      assertEquals(1, tree.listAllLeafNodes().size());
      testQuadNodeFourPointsAreTheSame(tree, low, high, testValue1);
   }

   private static void testQuadNodeFourPointsAreTheSame(HyperCubeTree<Double, Void> tree, double low, double high, Double testValue1)
   {
      double[] point;
      point = new double[] {low, low};
      assertEquals(testValue1, tree.get(point).getValue());
      point = new double[] {low, high};
      assertEquals(testValue1, tree.get(point).getValue());
      point = new double[] {high, low};
      assertEquals(testValue1, tree.get(point).getValue());
      point = new double[] {high, high};
      assertEquals(testValue1, tree.get(point).getValue());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPutGetWithSplit2()
   {
      int dimensionality = 2;
      HyperCubeTree<Double, Void> tree = setupTree(dimensionality, 0.5);
      double low = 0.25;
      double high = 0.75;
      double[] point = new double[] {low, high};
      Double testValue1 = 0.1;
      Double testValue2 = 0.6;

      assertNull(tree.get(point));

      tree.put(point, testValue1);

      assertEquals(testValue1, tree.get(point).getValue());
      assertEquals(1, tree.listAllLeafNodes().size());
      point = new double[] {low, low};
      assertEquals(testValue1, tree.get(point).getValue());

      point = new double[] {low, low};
      assertEquals(testValue1, tree.get(point).getValue());
      point = new double[] {low, high};
      assertEquals(testValue1, tree.get(point).getValue());
      point = new double[] {high, low};
      assertEquals(testValue1, tree.get(point).getValue());
      point = new double[] {high, high};
      assertEquals(testValue1, tree.get(point).getValue());

      tree.put(point, testValue2);
      assertEquals(4, tree.listAllLeafNodes().size());

      point = new double[] {low, low};
      assertNull(tree.get(point));
      point = new double[] {low, high};
      assertEquals(testValue1, tree.get(point).getValue());
      point = new double[] {high, low};
      assertNull(tree.get(point));
      point = new double[] {high, high};
      assertEquals(testValue2, tree.get(point).getValue());

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPutGetWithSplit3()
   {
      int dimensionality = 3;
      HyperCubeTree<Double, Void> tree = setupTree(dimensionality, 0.5);
      double low = 0.25;
      double high = 0.75;
      double[] point = new double[] {low, high, high};
      Double testValue1 = 0.1;
      Double testValue2 = 0.6;

      assertNull(tree.get(point));
      tree.put(point, testValue1);
      assertEquals(testValue1, tree.get(point).getValue());

      point = new double[] {low, low, low};
      assertEquals(testValue1, tree.get(point).getValue());

      point = new double[] {low, low, high};
      assertEquals(testValue1, tree.get(point).getValue());
      point = new double[] {low, high, low};
      assertEquals(testValue1, tree.get(point).getValue());
      point = new double[] {high, low, low};
      assertEquals(testValue1, tree.get(point).getValue());

      tree.put(point, testValue2);
      assertEquals(8, tree.listAllLeafNodes().size());

      point = new double[] {low, low, low};
      assertNull(tree.get(point));
      point = new double[] {low, low, high};
      assertNull(tree.get(point));
      point = new double[] {high, low, low};
      assertEquals(testValue2, tree.get(point).getValue());
      point = new double[] {low, high, high};
      assertEquals(testValue1, tree.get(point).getValue());

   }


// @Test(timeout=300000)
// public void testGatherLeavesWithinBounds()
// {
//    int dimensionality = 2;
//    HyperCubeNode<Double> node = setupDoubleNode(dimensionality);
//    OneDimensionalBounds[] gatherBounds = new OneDimensionalBounds[dimensionality];
//    for (int i = 0; i < dimensionality; i++)
//    {
//       gatherBounds[i] = new OneDimensionalBounds(0.2, 0.45);
//    }
//    List<HyperCubeLeaf<Double>> leaves = node.gatherLeavesWithinBounds(gatherBounds);
// }


   public static final double eps = 1e-13;

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testLocationSpecificRecursions()
   {
      int dimensionality = 3;
      HyperCubeTree<Double, Void> tree = setupTree(dimensionality, 0.25);
      RecursableHyperTreeNode<Double, Void> node = tree.getRootNode();
      tree.put(new double[] {0.25, 0.25, 0.25}, 1.6);
      assertEquals(1.6, node.getLeaf().getValue(), eps);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSimplePutting()
   {
      int dimensionality = 3;
      HyperCubeTree<Double, Void> tree = setupTree(dimensionality, 0.25);
      RecursableHyperTreeNode<Double, Void> node = tree.getRootNode();
      node.setLeaf(new HyperCubeLeaf<Double>(1.6, (new double[] {0.25, 0.25, 0.25})));
      assertEquals(1.6, node.getLeaf().getValue(), eps);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testGatherAllLeaves()
   {
      int dimensionality = 2;
      HyperCubeTree<Double, Void> tree = setupTree(dimensionality, 0.25);
      OneDimensionalBounds[] gatherBounds = new OneDimensionalBounds[dimensionality];
      for (int i = 0; i < dimensionality; i++)
      {
         gatherBounds[i] = new OneDimensionalBounds(0.2, 0.45);
      }

      double[] xVals = new double[]
      {
         0.0, 0.1, 0.4, 0.4, 0.4, 0.9, 1.0
      };
      double[] yVals = new double[]
      {
         0.0, 0.1, 0.4, 0.0, 0.4, 0.9, 0.0
      };
      double[] zVals = new double[]
      {
         0.1, 0.2, 0.3, 0.5, 0.5, 0.5, 0.6
      };
      for (int i = 0; i < xVals.length; i++)
      {
         tree.put(new double[] {xVals[i], yVals[i]}, zVals[i]);
      }

//    List<HyperCubeLeaf<Double>> leaves = tree.gatherAllLeaves();
//    assertEquals(5, leaves.size());
//    double[] xExpectedVals = new double[] { 0.1, 0.4, 0.4, 0.9, 1.0 };
//    double[] yExpectedVals = new double[] { 0.1, 0.0, 0.4, 0.9, 0.0 };
//    double[] zExpectedVals = new double[] { 0.2, 0.5, 0.5, 0.5, 0.6 };
//    for (int i = 0; i < 5; i++)
//    {
//       int valueIndex = -1;
//       for (int j = 0; j < 5; j++)
//       {
//          if ((leaves.get(j).getLocation()[0] == xExpectedVals[i]) && (leaves.get(j).getLocation()[1] == yExpectedVals[i])
//                && (leaves.get(j).getValue() == zExpectedVals[i]))
//             valueIndex = j;
//       }
//       assertFalse("point not found: <" + xExpectedVals[i] + ", " + yExpectedVals[i] + ", " + zExpectedVals[i] + ">", -1 == valueIndex);
//    }
   }

   private long startTime;
   private long averageTime;
   private double averageSeconds;
   private final static int alpha = 9;
   private final static double DIVISOR = 1 / ((double) (1 << alpha));

	@ContinuousIntegrationTest(estimatedDuration = 1.1)
	@Test(timeout = 30000)
   public void testTimingMeasurement()
   {
      for (int i = 0; i < 1000; i++)
      {
         this.startTime = System.nanoTime();
         LockSupport.parkNanos(1000000);
         averageTime = (averageTime >> alpha) * ((1 << alpha) - 1) + System.nanoTime() - startTime;
         averageSeconds = (double) averageTime * DIVISOR * 1e-9;
      }

      System.out.println("ThreadTools.sleep(1L) takes on average " + averageSeconds + " seconds to store a point.");
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testTimer()
   {
      LowPassTimingReporter time = new LowPassTimingReporter(7);
      int numTimes = 1;
//      long[] times = new long[numTimes];
      for (int j = 0; j < 30; j++)
      {
         time = new LowPassTimingReporter(6);

         for (int i = 0; i < 10000; i++)
         {
            time.startTime();

//          for (int j=0; j<numTimes;j++)
//             times[j]=System.nanoTime();
            time.endTime();
         }

         System.out.println(time.generateMessage("the timer itself", "do basically nothing"));
      }

      // 0.0010669192539062501
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.1)
	@Test(timeout = 30000)
   public void testMinimumMeasurableTime()
   {
      LowPassTimingReporter time = new LowPassTimingReporter(7);
      int numTimes = 1;
      long[] times = new long[numTimes];
      for (int j = 0; j < 50; j++)
      {
         time = new LowPassTimingReporter(6);

         for (int i = 0; i < 10000; i++)
         {
            time.startTime();

          for (int k=0; k<numTimes;k++)
             times[k]=System.nanoTime();
            time.endTime();
         }
         for (int k=1;k<numTimes;k++)
            times[k]=times[k]%times[k-1];
         System.out.println(time.generateMessage("testMinimumMeasurableTime", "record several calls to system.nanoTime(). hash = "+times[times.length-1]));
         
      }

      // 0.0010669192539062501
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSystemTimeNano()
   {
      long[] times = new long[1000];
      long[] deltaTimes = new long[999];
      for (int i = 0; i < 1000; i++)
      {
         times[i] = (System.nanoTime());

//       times[i] = (System.nanoTime());
      }

      for (int i = 0; i < 999; i++)
      {
         deltaTimes[i] = times[i + 1] - times[i];
      }

      for (int i = 0; i < 1000; i++)
      {
//         System.out.println(times[i]);
      }

      for (int i = 0; i < 999; i++)
      {
//         System.out.println(deltaTimes[i]);
      }

   }
}
