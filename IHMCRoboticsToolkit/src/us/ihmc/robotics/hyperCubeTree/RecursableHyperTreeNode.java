package us.ihmc.robotics.hyperCubeTree;

public interface RecursableHyperTreeNode<T,D> 
{
   public void split();
   public void clear();
   public OneDimensionalBounds getBounds(int i);
   public OneDimensionalBounds[] getBoundsCopy();
   public double[] getMidpoint();
   public HyperCubeLeaf<T> getLeaf();
   public void setLeaf(HyperCubeLeaf<T> leaf);
   public int getDimensionality();
   public int getChildNumber();
   public boolean hasChildren();
   public RecursableHyperTreeNode<T,D> getChild(int number);
   public RecursableHyperTreeNode<T,D> getChildAtLocation(double[] location);
   public void setMetaData(D metaData);
   public D getMetaData();
   public void updateMetaDataListeners();

}