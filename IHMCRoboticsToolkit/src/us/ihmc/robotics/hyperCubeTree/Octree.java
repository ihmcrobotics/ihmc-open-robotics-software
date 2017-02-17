package us.ihmc.robotics.hyperCubeTree;

import java.util.List;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.geometry.LineSegment3d;

public class Octree extends HyperCubeTree<Boolean, Void>
{
   private final ResolutionProvider constantResolution;

   public Octree(OneDimensionalBounds[] bounds, double resolution)
   {
      this(bounds, new ConstantResolutionProvider(resolution));
   }

   public Octree(OneDimensionalBounds[] bounds, ResolutionProvider resolutionProvider)
   {
      super(bounds);
      this.constantResolution = resolutionProvider;
      if (bounds.length != 3)
         throw new DimensionalityMismatchException();
   }

   public void mergeIfPossible()
   {
      lock();
      mergeWherePossibleRecursively(getRootNode());
      unlock();
   }

   private void mergeTruth(RecursableHyperTreeNode<Boolean, Void> node)
   {
      if (!node.hasChildren())
         return;
      HyperCubeLeaf<Boolean> firstLeaf = null;
      for (int i = 0; i < node.getChildNumber(); i++)
      {
         RecursableHyperTreeNode<Boolean, Void> lowerLevelNode = node.getChild(i);
         if (lowerLevelNode.hasChildren())
            return;
         if (null == lowerLevelNode.getLeaf())
            continue;

         if (null == firstLeaf)
         {
            firstLeaf = lowerLevelNode.getLeaf();

            continue;
         }

         if (false == lowerLevelNode.getLeaf().getValue())
            return;
      }

      if (null == firstLeaf)
         return;
      node.clear();
      node.setLeaf(firstLeaf);
   }

   public void nodeAdded(String id, OneDimensionalBounds[] bounds, HyperCubeLeaf<Boolean> leaf)
   {
      super.nodeAdded(id, bounds, leaf);
   }

   public void nodeRemoved(String id)
   {
      super.nodeRemoved(id);
   }

   public void putLidarAtGraduallyMoreAccurateResolution(Point3D start, Point3D end)
   {
      HyperVolume line = new LineSegmentSearchVolume(new LineSegment3d(start, end));
      double[] location = new double[] {end.getX(), end.getY(), end.getZ()};
      this.put(location, true);
      this.leafAdded(new HyperCubeLeaf<Boolean>(true, location));
      RecursableHyperTreeNode<Boolean, Void> endNode = this.getLeafNodeAtLocation(location);
      List<RecursableHyperTreeNode<Boolean, Void>> lineNodes = this.getHyperVolumeIntersection(line);
      lineNodes.remove(endNode);

      for (RecursableHyperTreeNode<Boolean, Void> node : lineNodes)
      {
         double[] intersectionWithBounds = line.intersectionWithBounds(new BoundsGetter(node).get());
         HyperCubeLeaf<Boolean> hyperCubeLeaf = new HyperCubeLeaf<Boolean>(false, intersectionWithBounds);
         this.replacementPut(hyperCubeLeaf);
      }
   }

   public void putLidarAtMinimumResolution(Point3D start, Point3D end)
   {
      HyperVolume line = new LineSegmentSearchVolume(new LineSegment3d(start, end));
      double[] location = new double[] {end.getX(), end.getY(), end.getZ()};
      this.upRezz(location);
      this.put(location, true);
      this.leafAdded(new HyperCubeLeaf<Boolean>(true, location));
      RecursableHyperTreeNode<Boolean, Void> endNode = this.getLeafNodeAtLocation(location);
      List<RecursableHyperTreeNode<Boolean, Void>> lineNodes = this.getHyperVolumeIntersection(line);
      lineNodes.remove(endNode);

      for (RecursableHyperTreeNode<Boolean, Void> node : lineNodes)
      {
         double[] intersectionWithBounds = line.intersectionWithBounds(new BoundsGetter(node).get());
         HyperCubeLeaf<Boolean> hyperCubeLeaf = new HyperCubeLeaf<Boolean>(false, intersectionWithBounds);
         this.replacementPut(hyperCubeLeaf);
      }

      lock();
      this.mergeParentRecursively(getRootNode(), location);
      unlock();
   }

   protected boolean canMergeLeaves(HyperCubeLeaf<Boolean> firstLeaf, HyperCubeLeaf<Boolean> secondLeaf)
   {
      return firstLeaf.getValue() == secondLeaf.getValue();
   }

   protected boolean canSplit(RecursableHyperTreeNode<Boolean, Void> node)
   {
      for (int i = 0; i < node.getDimensionality(); i++)
      {
         if (node.getBounds(i).size() <= constantResolution.getResolution(node.getMidpoint()))
            return false;
      }

      return true;
   }

   protected HyperCubeLeaf<Boolean> mergeLeaves(HyperCubeLeaf<Boolean> oldLeaf, HyperCubeLeaf<Boolean> newLeaf)
   {
      return newLeaf;
   }

   private void mergeParentRecursively(RecursableHyperTreeNode<Boolean, Void> node, double[] location)
   {
      if (null == node)
         return;

      if (node.hasChildren())
      {
         mergeParentRecursively(node.getChildAtLocation(location), location);
         mergeTruth(node);
      }
   }

   public static class BoundsGetter
   {
      RecursableHyperTreeNode<?, ?> node;

      public BoundsGetter(RecursableHyperTreeNode<?, ?> node)
      {
         this.node = node;
      }

      public OneDimensionalBounds[] get()
      {
         OneDimensionalBounds[] ret = new OneDimensionalBounds[node.getDimensionality()];
         for (int i = 0; i < node.getDimensionality(); i++)
         {
            ret[i] = node.getBounds(i);
         }

         return ret;
      }
   }


   public void treeCleared()
   {

   }

   public boolean put(HyperCubeLeaf<Boolean> leaf)
   {
      lock();
      boolean success = putRecursively(getRootNode(), leaf);
      unlock();
      return success;
   }

   private boolean putRecursively(RecursableHyperTreeNode<Boolean, Void> node, final HyperCubeLeaf<Boolean> leaf)
   {
      if (node.hasChildren())
      {
         return putRecursively(node.getChildAtLocation(leaf.getLocation()), leaf);
      }

      if (node.getLeaf() != null)
      {
         if (canSplit(node))
         {
            HyperCubeLeaf<Boolean> oldLeaf = node.getLeaf();
            node.setLeaf(null);
            node.split();
            node.getChildAtLocation(oldLeaf.getLocation()).setLeaf(oldLeaf);
            RecursableHyperTreeNode<Boolean, Void> childAtLocation = node.getChildAtLocation(leaf.getLocation());
            childAtLocation.setLeaf(mergeLeaves(childAtLocation.getLeaf(), leaf));

            return true;
         }
         else
         {
            if (node.getLeaf().getValue().booleanValue() == leaf.getValue().booleanValue())
            {
               return false;
            }
            else
            {
               node.setLeaf(leaf);

               return true;
            }
         }
      }
      else
      {
         node.setLeaf(leaf);

         return true;
      }
   }

}
