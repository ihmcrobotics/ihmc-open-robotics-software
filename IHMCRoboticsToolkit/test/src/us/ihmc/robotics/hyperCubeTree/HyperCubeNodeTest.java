package us.ihmc.robotics.hyperCubeTree;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;

public class HyperCubeNodeTest
{
   public static final double eps = 1.0e-6;

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testAssumptions()
   {
      assertEquals(6, 3 << 1);
      assertEquals(7, (3 << 1) + 1);
      int index = 0;
      boolean[] sides = new boolean[] { true, true, true };
      for (boolean side : sides)
      {
         index = (index << 1) + (side ? 1 : 0);
      }
      assertTrue(index == 7);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testToIndex()
   {
      int dimensionality = 3;
      HyperCubeNode<Double,Void> node  = setupDoubleNode(dimensionality);
      boolean[] sides = new boolean[] { true, true, true };
      int index = node.toIndex(sides);
      assertEquals(7, index);
      sides = new boolean[] { false, true, true };
      index = node.toIndex(sides);
      assertEquals(3, index);
      sides = new boolean[] { false, true, false };
      index = node.toIndex(sides);
      assertEquals(2, index);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testToBooleanArray()
   {
      int dimensionality = 3;
      HyperCubeNode<Boolean,Void> node  = setupBooleanNode(dimensionality);
      boolean[] sides = new boolean[] { true, true, false };
      int index = node.toIndex(sides);
      assertEquals(6, index);
      boolean[] recreatedSides = node.toBooleanArray(index);
      for (int i = 0; i < dimensionality; i++)
      {
         assertEquals(sides[i], recreatedSides[i]);
      }
      sides = new boolean[] { false, true, true };
      index = node.toIndex(sides);
      assertEquals(3, index);
      sides = new boolean[] { false, true, false };
      index = node.toIndex(sides);
      assertEquals(2, index);
   }


   private HyperCubeNode<Double,Void> setupDoubleNode(int dimensionality)
   {
      OneDimensionalBounds[] bounds = new OneDimensionalBounds[dimensionality];
      for (int i = 0; i < dimensionality; i++)
         bounds[i] = new OneDimensionalBounds(0.0, 1.0);
      HyperCubeNode<Double,Void> node = new HyperCubeNode<Double,Void>(bounds, "Node", new NullListener<Double,Void>());
      return node;
   }
   
   private HyperCubeNode<Boolean,Void> setupBooleanNode(int dimensionality)
   {
      OneDimensionalBounds[] bounds = new OneDimensionalBounds[dimensionality];
      for (int i = 0; i < dimensionality; i++)
         bounds[i] = new OneDimensionalBounds(0.0, 1.0);
      HyperCubeNode<Boolean,Void> node = new HyperCubeNode<Boolean,Void>(bounds, "Node", new NullListener<Boolean,Void>());
      return node;
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testLocatePoint()
   {
      int dimensionality = 3;
      HyperCubeNode<Double,Void> node = setupDoubleNode(dimensionality);
      double[] point = new double[] { 0.25, 0.6, 0.6 };
      int expectation = 3;
      boolean[] sides = node.locatePoint(point);
      int index = node.toIndex(sides);
      assertEquals(expectation, index);

      point = new double[] { 0.5, 0.5, 1.0 };
      expectation = 7;
      sides = node.locatePoint(point);
      index = node.toIndex(sides);
      assertEquals(expectation, index);

      point = new double[] { 0.499, 0.499, 0.499 };
      expectation = 0;
      sides = node.locatePoint(point);
      index = node.toIndex(sides);
      assertEquals(expectation, index);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testIndexing()
   {
      int dimensionality = 12;
      HyperCubeNode<Double,Void> node = setupDoubleNode(dimensionality);
      for (int i = 0; i < (1 << dimensionality); i++)
      {
         boolean[] sides = node.toBooleanArray(i);
         int index = node.toIndex(sides);
         assertEquals(i, index);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testWithinBounds()
   {
      OneDimensionalBounds[] bounds = new OneDimensionalBounds[] { new OneDimensionalBounds(-0.35, -0.1), new OneDimensionalBounds(57.6, 65.3) };
      double[] test1 = new double[] { 2.3, 59.9 };
      double[] test2 = new double[] { -0.2, 60.5 };
      double[] test3 = new double[] { -99.2, 69.5 };
      assertFalse(HyperCubeNode.withinBounds(bounds, test1));
      assertTrue(HyperCubeNode.withinBounds(bounds, test2));
      assertFalse(HyperCubeNode.withinBounds(bounds, test3));
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSubdivideBounds()
   {
      int dimensionality = 3;
      HyperCubeNode<Double,Void> node = setupDoubleNode(dimensionality);
      OneDimensionalBounds[] subBounds = node.subdivideBounds(new boolean[] { true, false, false });
      assertEquals(0.5, subBounds[0].min(), eps);
      assertEquals(1.0, subBounds[0].max(), eps);
      assertEquals(0.0, subBounds[1].min(), eps);
      assertEquals(0.5, subBounds[1].max(), eps);
      assertEquals(0.0, subBounds[2].min(), eps);
      assertEquals(0.5, subBounds[2].max(), eps);
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSplit()
   {
      for (int i=1;i<11; i++)
      {
         HyperCubeNode<Double,Void> node = setupDoubleNode(i);
         node.split();
         assertEquals(true, node.hasChildren());
         assertEquals(1<<i, node.getChildNumber());
         for (int j=0;j<1<<i;j++)
            assertTrue(node.getChild(j)!=null);
      }
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testReplaceLeaf()
   {
      int dimensionality = 3;
      HyperCubeNode<Double,Void> node = setupDoubleNode(dimensionality);
      double leafValue = 0.78;
      assertNull(node.getLeaf());
      node.setLeaf(new HyperCubeLeaf<Double>(leafValue, new double[]{0.0,0.5,0.5}));
      assertEquals(leafValue,node.getLeaf().getValue(), eps);
   }

   public static class NullListener<T,D> implements HyperCubeTreeListener<T,D>
   {
      public void nodeAdded(String id, OneDimensionalBounds[] bounds, HyperCubeLeaf<T> leaf)
      {
      }

      public void nodeRemoved(String id)
      { 
      }

      public void leafAdded(HyperCubeLeaf<T> leaf)
      {
      }

      public void treeCleared()
      {
         
      }

      public void metaDataUpdated(String id, OneDimensionalBounds[] bounds, D data)
      {
         
      }
   }

}
