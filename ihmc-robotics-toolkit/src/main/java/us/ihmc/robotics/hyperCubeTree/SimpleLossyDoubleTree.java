package us.ihmc.robotics.hyperCubeTree;

public class SimpleLossyDoubleTree extends HyperCubeTree<Double,Void>
{
   private final double constantResolution;
   private final double eps;

   public SimpleLossyDoubleTree(OneDimensionalBounds[] bounds, double constantResolution, double eps)
   {
      super(bounds);
      this.constantResolution = constantResolution;
      this.eps = eps;
   }

   public double getResolution(OneDimensionalBounds[] bounds, int i)
   {
      return constantResolution;
   }

   public boolean shouldSubdivide(HyperCubeLeaf<Double> leaf1, HyperCubeLeaf<Double> leaf2)
   {
      double diff = (leaf1.getValue() - leaf2.getValue());
      return (diff > eps) || (diff < -eps);
   }

   protected boolean canSplit(RecursableHyperTreeNode<Double,Void> node)
   {
      for (int i = 0; i < node.getDimensionality(); i++)
      {
         if (node.getBounds(i).size() <= constantResolution)
            return false;
      }
      return true;
   }

   protected HyperCubeLeaf<Double> mergeLeaves(HyperCubeLeaf<Double> oldLeaf, HyperCubeLeaf<Double> newLeaf)
   {
      return newLeaf;
   }

   protected boolean canMergeLeaves(HyperCubeLeaf<Double> firstLeaf, HyperCubeLeaf<Double> secondLeaf)
   {
      double diff = (firstLeaf.getValue() - secondLeaf.getValue());
      return (diff > eps) || (diff < -eps);
   }

   public void treeCleared()
   {
      // TODO Auto-generated method stub
      
   }

}