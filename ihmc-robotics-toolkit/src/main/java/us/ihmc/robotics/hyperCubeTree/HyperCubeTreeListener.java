package us.ihmc.robotics.hyperCubeTree;

public interface HyperCubeTreeListener<T,D>
{
   public void nodeAdded(String id, OneDimensionalBounds[] bounds, HyperCubeLeaf<T> leaf);

   public void nodeRemoved(String id);

   public void leafAdded(HyperCubeLeaf<T> leaf);
   
   public void metaDataUpdated(String id, OneDimensionalBounds[] bounds, D data);
   
   public void treeCleared();


}
