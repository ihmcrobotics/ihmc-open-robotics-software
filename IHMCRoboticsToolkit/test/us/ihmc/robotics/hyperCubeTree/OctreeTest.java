package us.ihmc.robotics.hyperCubeTree;

import static org.junit.Assert.assertEquals;
import static org.junit.Assert.assertFalse;
import static org.junit.Assert.assertNull;
import static org.junit.Assert.assertTrue;

import java.util.List;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations.ContinuousIntegrationTest;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.geometry.LineSegment3d;

public class OctreeTest
{
   private static final String[] axes = { "x", "y", "z" };

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testSimplePutGetOctree()
   {
      HyperCubeTree<Boolean, Void> tree = setupUnitCubeOctree(0.5);
      tree.put(new double[] { 0.0, 0.0, 0.0}, true);
      assertEquals(true,tree.getRootNode().getLeaf().getValue());
      assertEquals(false,tree.getRootNode().hasChildren());
      
      tree.put(new double[] { 0.0, 0.0, 0.0}, true);
      assertEquals(true,tree.getRootNode().hasChildren());
      assertEquals(true,tree.get(new double[] {0.0,0.0,0.0}).getValue());
      assertEquals(8, tree.listAllLeafNodes().size());
      assertEquals(1, tree.listAllLeaves().size());
      
      tree.put(new double[] { 0.0, 0.0, 0.0}, false);
      assertEquals(true,tree.getRootNode().hasChildren());
      assertEquals(false,tree.get(new double[] {0.0,0.0,0.0}).getValue());
      assertNull(tree.get(new double[] {1.0,0.0,0.0}));
      assertEquals(8, tree.listAllLeafNodes().size());
      assertEquals(1, tree.listAllLeaves().size());
      
      tree.put(new double[] {1.0,0.0,0.0}, false);
      assertEquals(true,tree.getRootNode().hasChildren());
      assertEquals(false,tree.get(new double[] {1.0,0.0,0.0}).getValue());
      assertNull(tree.get(new double[] {0.0,1.0,0.0}));
      assertNull(tree.get(new double[] {0.0,0.0,1.0}));
      assertEquals(8, tree.listAllLeafNodes().size());
      assertEquals(2, tree.listAllLeaves().size());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void test3DPutGetRemoveTwoLevel()
   {
      HyperCubeTree<Boolean, Void> tree = setupUnitCubeOctree(0.25);
      double d1 = 0.125;
      double d2 = 0.375;
      double d3 = 0.625;
      reportLeafCount(0, tree);
      tree.put(new double[] { d1, d1, d1 }, true);
      tree.put(new double[] { d1, d1, d1 }, true);
      tree.put(new double[] { d1, d2, d1 }, true);
      tree.put(new double[] { d1, d1, d2 }, true);
      tree.put(new double[] { d1, d2, d2 }, true);
      tree.put(new double[] { d2, d1, d1 }, true);
      tree.put(new double[] { d2, d2, d1 }, true);
      tree.put(new double[] { d2, d1, d2 }, true);
      tree.put(new double[] { d2, d2, d2 }, true);

      reportLeafCount(8, tree);

      tree.put(new double[] { d1, d3, d3 }, false);
      reportLeafCount(9, tree);
      tree.put(new double[] { d1, d3, d1 }, false);
      tree.put(new double[] { d1, d1, d3 }, false);
      reportLeafCount(11, tree);

//      assertTrue(tree.getRootNode().getHasChildren());
      assertTrue(tree.get(new double[] { d1, d1, d1 }).getValue());
      assertFalse(tree.get(new double[] { d1, d3, d3 }).getValue());
      assertFalse(tree.get(new double[] { d1, d3, d1 }).getValue());
      assertFalse(tree.get(new double[] { d1, d1, d3 }).getValue());

      HyperCubeLeaf<Boolean> lastRemainingTrueNode = tree.get(new double[] { d2, d2, d2 });

      tree.remove(lastRemainingTrueNode);

      reportLeafCount(4, tree);

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testOctreeInternalBoarders()
   {
      HyperCubeTree<Boolean, Void> tree = setupUnitCubeOctree(0.25);
      tree.put(new double[] { 0.0, 0.0, 0.0 }, false);
      tree.put(new double[] { 0.5, 0.5, 0.5 }, true);
      tree.put(new double[] { 0.5, 0.5, 0.5 }, true);
      tree.put(new double[] { 0.0, 0.0, 0.0 }, false);
      assertEquals(2, tree.listAllLeaves().size());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testOctreeLineSearch()
   {

      HyperCubeTree<Boolean, Void> tree = createStandardTestOctree();
      reportLeafCount(35, tree);


      List<RecursableHyperTreeNode<Boolean,Void>> nodes = getNodesIntersectingLine(tree, new Point3D(0.0, 0.0, 0.0), new Point3D(1.0, 1.0, 0.0));
      assertEquals(4, nodes.size());

      nodes = getNodesIntersectingLine(tree, new Point3D(0.0, 0.0, 0.0), new Point3D(0.0, 0.0, 0.0));
      assertEquals(1, nodes.size());

      nodes = getNodesIntersectingLine(tree, new Point3D(1.0, 0.0, 0.0), new Point3D(1.0, 0.0, 1.0));
      //      listBounds(nodes);
      assertEquals(3, nodes.size());

      nodes = getNodesIntersectingLine(tree, new Point3D(0.9, 0.1, 0.9), new Point3D(0.9, 0.9, 0.9));
      assertEquals(4, nodes.size());

   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPrint()
   {
      HyperCubeTree<Boolean, Void> tree = createStandardTestOctree();
      System.out.println(tree.toString());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.0)
	@Test(timeout = 30000)
   public void testPutLidarInOctree()
   {
      Octree tree = createStandardTestOctree();
      assertNull(tree.get(new double[] { 0.0, 0.0, 0.0 }));
      assertNull(tree.get(new double[] { 0.375, 0.0, 0.0 }));
      tree.putLidarAtGraduallyMoreAccurateResolution(new Point3D(0.0, 0.0, 0.0), new Point3D(0.375, 0.0, 0.0));

      assertEquals(true, tree.get(new double[] { 0.0, 0.0, 0.0 }).getValue());
      assertEquals(true, tree.get(new double[] { 0.375, 0.0, 0.0 }).getValue());
      tree.putLidarAtGraduallyMoreAccurateResolution(new Point3D(0.0, 0.0, 0.0), new Point3D(0.375, 0.0, 0.0));

      assertEquals(false, tree.get(new double[] { 0.0, 0.0, 0.0 }).getValue());
      assertEquals(true, tree.get(new double[] { 0.375, 0.0, 0.0 }).getValue());
   }

	@ContinuousIntegrationTest(estimatedDuration = 0.4)
	@Test(timeout = 30000)
   public void testShootOctreeIntoSphereWithLidarBullets()
   {
      long time = System.currentTimeMillis();
      Octree tree = setupUnitCubeOctree(0.0000125);
      HyperCubeTreeStaticTestingUtilities.shootTreeIntoSphere(tree, 0.4, 10.5, 0.45, 0.45, 0.45, 10000);
      System.out.println(System.currentTimeMillis() - time);
      time = System.currentTimeMillis();
      System.out.println("Octree has " + tree.listAllLeaves().size() + " leaves");
      System.out.println(System.currentTimeMillis() - time);
      time = System.currentTimeMillis();
   }

   private Octree createStandardTestOctree()
   {
      Octree tree = setupUnitCubeOctree(0.25);
      reportLeafCount(0, tree);

      // if the first two hits are not alternating true false in each recursive level of the octree than data will be lost.

      double d1 = 0.125;
      double d2 = 0.375;
      double d3 = 0.625;
      double d4 = 0.875;
      //LHL Oct                 
      HyperCubeTreeStaticTestingUtilities.fillOctreeDefaultTest(tree, d1, d2, d3, d4);

      return tree;
   }

   private static <T,D> List<RecursableHyperTreeNode<T,D>>  getNodesIntersectingLine(HyperCubeTree<T,D> tree, Point3D startPoint, Point3D endPoint)
   {
      LineSegment3d lineSegment = new LineSegment3d(startPoint, endPoint);
      LineSegmentSearchVolume hyperVolume = new LineSegmentSearchVolume(lineSegment);
      List<RecursableHyperTreeNode<T,D>> nodes = tree.getHyperVolumeIntersection(hyperVolume);
      return nodes;
   }


   private static <T,D> void reportLeafCount(int expected, HyperCubeTree<T,D> tree)
   {
      int numberOfLeaves = tree.listAllLeaves().size();
      System.out.println("node has this many leaves: " + numberOfLeaves);
      assertEquals(expected, numberOfLeaves);
   }

   private Octree setupUnitCubeOctree(double resolution)
   {
      OneDimensionalBounds[] bounds = new OneDimensionalBounds[3];
      for (int i = 0; i < 3; i++)
         bounds[i] = new OneDimensionalBounds(0.0, 1.0);
      Octree tree = new Octree(bounds, resolution);
      return tree;
   }
}
