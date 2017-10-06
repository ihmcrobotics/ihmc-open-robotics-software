package us.ihmc.robotics.hyperCubeTree;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.locks.ReentrantLock;

public abstract class HyperCubeTree<T, D> implements HyperCubeTreeListener<T, D>
{
   private final RecursableHyperTreeNode<T, D> rootNode;
   private List<HyperCubeTreeListener<T, D>> treeListeners = new ArrayList<HyperCubeTreeListener<T, D>>();
   private final ReentrantLock lock = new ReentrantLock();

   public HyperCubeTree(OneDimensionalBounds[] bounds)
   {
      rootNode = new HyperCubeNode<T, D>(bounds, "root", this);

      for (int i = 0; i < bounds.length; i++)
      {
         if (bounds[i].isInfinite())
         {
            throw new RuntimeException("Infinite ranges are unbisectable.");
         }
      }
   }

   public void addListener(HyperCubeTreeListener<T, D> listener)
   {
      treeListeners.add(listener);
   }

   public void checkDimensionality(double[] array)
   {
      checkDimensionality(array, rootNode.getDimensionality());
   }

   public void clearTree()
   {
      lock();
      {
         clearRecursively(this.getRootNode());

         for (HyperCubeTreeListener<T, D> listener : this.treeListeners)
         {
            listener.treeCleared();
         }
      }
   }

   public int countNodes()
   {
      lock();
      int count = countNodesRecursively(this.getRootNode());
      unlock();
      return count;
   }

   public HyperCubeLeaf<T> get(double[] location)
   {
      lock();
      checkDimensionality(location);
      OneDimensionalBounds[] boundsCopy = rootNode.getBoundsCopy();
      for (int i = 0; i < location.length; i++)
      {
         if (!boundsCopy[i].contains(location[i]))
            return null;
      }

      HyperCubeLeaf<T> leaves = getRecursively(getRootNode(), location);
      unlock();
      return leaves;

   }

   public List<RecursableHyperTreeNode<T, D>> getHyperVolumeIntersection(HyperVolume volume)
   {
      lock();
      List<RecursableHyperTreeNode<T, D>> nodes = new ArrayList<RecursableHyperTreeNode<T, D>>();
      getHyperVolumeIntersectionRecursively(getRootNode(), volume, nodes);
      unlock();
      return nodes;

   }

   public RecursableHyperTreeNode<T, D> getLeafNodeAtLocation(double[] location)
   {
      lock();
      RecursableHyperTreeNode<T, D> node = getNode(getRootNode(), location);
      unlock();
      return node;

   }

   public RecursableHyperTreeNode<T, D> getNode(double[] location)
   {
      lock();
      RecursableHyperTreeNode<T, D> node = getNode(this.getRootNode(), location);
      unlock();
      return node;
   }

   public RecursableHyperTreeNode<T, D> getRootNode()
   {
      return this.rootNode;
   }

   public void leafAdded(HyperCubeLeaf<T> leaf)
   {
      for (HyperCubeTreeListener<T, D> listener : this.treeListeners)
      {
         listener.leafAdded(leaf);
      }

   }

   public List<RecursableHyperTreeNode<T, D>> listAllLeafNodes()
   {
      lock();
      List<RecursableHyperTreeNode<T, D>> leafNodes = new ArrayList<RecursableHyperTreeNode<T, D>>();
      listAllLeafNodesRecursively(getRootNode(), leafNodes);
      unlock();
      return leafNodes;
   }

   public List<HyperCubeLeaf<T>> listAllLeaves()
   {
      lock();
      List<HyperCubeLeaf<T>> leaves = new ArrayList<HyperCubeLeaf<T>>();
      listAllLeavesRecursively(getRootNode(), leaves);
      unlock();
      return leaves;
   }

   protected void unSynchronizedMergeOneLevel(RecursableHyperTreeNode<T, D> node)
   {
      if (!node.hasChildren())
         return;
      HyperCubeLeaf<T> firstLeaf = null;
      for (int i = 0; i < node.getChildNumber(); i++)
      {
         RecursableHyperTreeNode<T, D> lowerLevelNode = node.getChild(i);
         if (lowerLevelNode.hasChildren())
            return;
         if (null == lowerLevelNode.getLeaf())
            continue;

         if (null == firstLeaf)
         {
            firstLeaf = lowerLevelNode.getLeaf();

            continue;
         }

         if (!canMergeLeaves(firstLeaf, lowerLevelNode.getLeaf()))
            return;
      }

      if (null == firstLeaf)
         return;
      node.clear();
      node.setLeaf(firstLeaf);
   }

   public void nodeAdded(String id, OneDimensionalBounds[] bounds, HyperCubeLeaf<T> leaf)
   {
      for (HyperCubeTreeListener<T, D> listener : this.treeListeners)
      {
         listener.nodeAdded(id, bounds, leaf);
      }
   }

   public void nodeRemoved(String id)
   {
      for (HyperCubeTreeListener<T, D> listener : this.treeListeners)
      {
         listener.nodeRemoved(id);
      }
   }

   public void metaDataUpdated(String id, OneDimensionalBounds[] bounds, D data)
   {
      for (HyperCubeTreeListener<T, D> listener : this.treeListeners)
      {
         listener.metaDataUpdated(id, bounds, data);
      }
   }

   public boolean put(double[] location, T value)
   {
      checkDimensionality(location);
      HyperCubeLeaf<T> leaf = new HyperCubeLeaf<T>(value, location);

      return this.put(leaf);

   }

   public boolean put(HyperCubeLeaf<T> leaf)
   {
      lock();
      boolean success = putRecursively(getRootNode(), leaf);
      unlock();
      return success;
   }

   public void remove(HyperCubeLeaf<T> leaf)
   {
      lock();
      removeRecursively(getRootNode(), leaf);
      unlock();
   }

   public void removeListener(HyperCubeTreeListener<T, D> listener)
   {
      treeListeners.remove(listener);
   }

   public void replacementPut(HyperCubeLeaf<T> leaf)
   {
      lock();
      replacementPutRecursively(this.getRootNode(), leaf);
      unlock();
   }

   public void upRezz(double[] location)
   {
      lock();
      this.upRezzRecursively(getRootNode(), location);
      unlock();
   }

   protected abstract boolean canMergeLeaves(HyperCubeLeaf<T> firstLeaf, HyperCubeLeaf<T> secondLeaf);

   protected abstract boolean canSplit(RecursableHyperTreeNode<T, D> node);

   protected abstract HyperCubeLeaf<T> mergeLeaves(HyperCubeLeaf<T> oldLeaf, HyperCubeLeaf<T> newLeaf);

   protected void mergeWherePossibleRecursively(RecursableHyperTreeNode<T, D> node)
   {
      if (node.hasChildren())
      {
         for (int i = 0; i < node.getChildNumber(); i++)
         {
            mergeWherePossibleRecursively(node.getChild(i));
         }

         unSynchronizedMergeOneLevel(node);
      }
   }

   private void clearRecursively(RecursableHyperTreeNode<T, D> node)
   {
      if (node.hasChildren())
      {
         for (int i = 0; i < node.getChildNumber(); i++)
         {
            clearRecursively(node.getChild(i));
         }

         node.clear();
      }
      else
      {
         node.clear();
      }
   }

   private int countNodesRecursively(RecursableHyperTreeNode<T, D> node)
   {
      if (node.hasChildren())
      {
         int nodes = 1;
         for (int i = 0; i < node.getChildNumber(); i++)
         {
            nodes += countNodesRecursively(node.getChild(i));
         }

         return nodes;
      }

      return 1;
   }

   private boolean putRecursively(RecursableHyperTreeNode<T, D> node, final HyperCubeLeaf<T> leaf)
   {
      if (node.hasChildren())
      {
         return putRecursively(node.getChildAtLocation(leaf.getLocation()), leaf);
      }
      else
      {
         if (node.getLeaf() != null)
         {
            if (canSplit(node))
            {
               HyperCubeLeaf<T> oldLeaf = node.getLeaf();
               node.setLeaf(null);
               node.split();
               node.getChildAtLocation(oldLeaf.getLocation()).setLeaf(oldLeaf);
               RecursableHyperTreeNode<T, D> childAtLocation = node.getChildAtLocation(leaf.getLocation());
               childAtLocation.setLeaf(mergeLeaves(childAtLocation.getLeaf(), leaf));
            }
            else
               node.setLeaf(mergeLeaves(node.getLeaf(), leaf));
         }
         else
         {
            node.setLeaf(leaf);
         }
      }

      return true;
   }

   private void removeRecursively(RecursableHyperTreeNode<T, D> node, HyperCubeLeaf<T> leaf)
   {
      if (node.hasChildren())
      {
         removeRecursively(node.getChildAtLocation(leaf.getLocation()), leaf);
         unSynchronizedMergeOneLevel(node);
      }
      else
      {
         if (node.getLeaf() != null)
            node.clear();
      }
   }

   private void upRezzRecursively(RecursableHyperTreeNode<T, D> node, double[] location)
   {
      if (!node.hasChildren() && this.canSplit(node))
      {
         HyperCubeLeaf<T> oldLeaf = node.getLeaf();
         node.setLeaf(null);
         node.split();
         if (null != oldLeaf)
            node.getChildAtLocation(oldLeaf.getLocation()).setLeaf(oldLeaf);
      }

      if (node.hasChildren())
         upRezzRecursively(node.getChildAtLocation(location), location);
   }

   private static void checkDimensionality(double[] array, int dimensionality)
   {
      if (array.length != dimensionality)
         throw new DimensionalityMismatchException();
   }

   private static <T, D> void getHyperVolumeIntersectionRecursively(RecursableHyperTreeNode<T, D> node, HyperVolume volume,
         List<RecursableHyperTreeNode<T, D>> nodes)
   {
      if (volume.intersectsBounds(node.getBoundsCopy()))
      {
         if (node.hasChildren())
         {
            for (int i = 0; i < node.getChildNumber(); i++)
            {
               getHyperVolumeIntersectionRecursively(node.getChild(i), volume, nodes);
            }
         }
         else
         {
            nodes.add(node);
         }
      }
   }

   private static <T, D> RecursableHyperTreeNode<T, D> getNode(RecursableHyperTreeNode<T, D> node, double[] location)
   {
      if (node.hasChildren())
         return getNode(node.getChildAtLocation(location), location);

      return node;
   }

   private static <T, D> HyperCubeLeaf<T> getRecursively(RecursableHyperTreeNode<T, D> node, double[] location)
   {
      if (node.hasChildren())
      {
         return getRecursively(node.getChildAtLocation(location), location);
      }
      else
      {
         return node.getLeaf();
      }
   }

   private static <T, D> void listAllLeafNodesRecursively(RecursableHyperTreeNode<T, D> node, List<RecursableHyperTreeNode<T, D>> leafNodes)
   {
      if (node.hasChildren())
      {
         for (int i = 0; i < node.getChildNumber(); i++)
         {
            listAllLeafNodesRecursively(node.getChild(i), leafNodes);
         }
      }
      else
      {
         leafNodes.add(node);
      }
   }

   private static <T, D> void listAllLeavesRecursively(RecursableHyperTreeNode<T, D> node, List<HyperCubeLeaf<T>> leaves)
   {
      if (node.hasChildren())
      {
         for (int i = 0; i < node.getChildNumber(); i++)
         {
            listAllLeavesRecursively(node.getChild(i), leaves);
         }
      }
      else
      {
         HyperCubeLeaf<T> leaf = node.getLeaf();
         if (leaf != null)
            leaves.add(leaf);
      }
   }

   private static <T, D> void replacementPutRecursively(RecursableHyperTreeNode<T, D> node, HyperCubeLeaf<T> leaf)
   {
      if (node.hasChildren())
      {
         replacementPutRecursively(node.getChildAtLocation(leaf.getLocation()), leaf);
      }
      else
      {
         node.setLeaf(leaf);
      }
   }

   public void lock()
   {
      lock.lock();
   }

   public void unlock()
   {
      lock.unlock();
   }
}
