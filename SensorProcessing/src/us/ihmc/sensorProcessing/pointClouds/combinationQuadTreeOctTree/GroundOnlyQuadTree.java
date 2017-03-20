package us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree;

import java.util.ArrayList;
import java.util.Collection;
import java.util.List;

import us.ihmc.euclid.geometry.BoundingBox2D;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.geometry.InclusionFunction;
import us.ihmc.robotics.hyperCubeTree.ConstantResolutionProvider;
import us.ihmc.robotics.hyperCubeTree.HyperCubeLeaf;
import us.ihmc.robotics.hyperCubeTree.HyperCubeTree;
import us.ihmc.robotics.hyperCubeTree.OneDimensionalBounds;
import us.ihmc.robotics.hyperCubeTree.RecursableHyperTreeNode;
import us.ihmc.robotics.hyperCubeTree.ResolutionProvider;
import us.ihmc.robotics.quadTree.QuadTreeForGroundListener;

public class GroundOnlyQuadTree extends HyperCubeTree<GroundAirDescriptor, GroundOnlyQuadTreeData> implements QuadTreeHeightMapInterface
{
   
   private final ResolutionProvider constantResolution;

   private double heightThreshold;
   private int maxNodes = 1;

   private int numberOfNodes = 0;
   
   private  double defaultHeightWhenNoPoints=Double.NaN;

   //================================================================================
   // Constructors
   //================================================================================

   public GroundOnlyQuadTree(BoundingBox2D bounds, double resolution, double heightThreshold, int maxNodes)
   {
      this(toBounds(bounds), new ConstantResolutionProvider(resolution), heightThreshold, maxNodes);
   }

   public GroundOnlyQuadTree(double minX, double minY, double maxX, double maxY, double resolution, double heightThreshold, int maxNodes)
   {
      this(toBounds(minX, maxX, minY, maxY), new ConstantResolutionProvider(resolution), heightThreshold, maxNodes);
   }

   public GroundOnlyQuadTree(double minX, double minY, double maxX, double maxY, ResolutionProvider resolution, double heightThreshold, int maxNodes)
   {
      this(toBounds(minX, maxX, minY, maxY), resolution, heightThreshold, maxNodes);
   }

   private GroundOnlyQuadTree(OneDimensionalBounds[] bounds, ResolutionProvider resolution, double heightThreshold, int maxNodes)
   {
      super(bounds);
      this.constantResolution = resolution;
      this.heightThreshold = heightThreshold;
      this.getRootNode().setMetaData(new GroundOnlyQuadTreeData());
      this.maxNodes = maxNodes;
      numberOfNodes = 1;
   }

   //================================================================================
   // External Interface
   //================================================================================
   
   public boolean addPoint(double x, double y, double z)
   {
      return addToQuadtree(x, y, z);
   }
   

   public boolean addToQuadtree(double x, double y, double z)
   {
      double[] location = toLocation(x, y);
      Float value = Float.valueOf((float)z);
      
      checkDimensionality(location);
      HyperCubeLeaf<GroundAirDescriptor> leaf = new HyperCubeLeaf<GroundAirDescriptor>(new GroundAirDescriptor(value, null), location);
      lock();
      boolean success = this.putRecursively(this.getRootNode(), leaf);
      unlock();
      return success;
   }
   
   //================================================================================
   // Core functions
   //================================================================================


   private boolean putRecursively(RecursableHyperTreeNode<GroundAirDescriptor, GroundOnlyQuadTreeData> node, final HyperCubeLeaf<GroundAirDescriptor> leaf)
   {
      if (node.hasChildren())
      {
         boolean treeChanged = putRecursively(node.getChildAtLocation(leaf.getLocation()), leaf);
         if (treeChanged)
            this.unSynchronizedMergeOneLevel(node);

         return treeChanged;
      }

      if (node.getLeaf() == null)
      {
         replaceLeaf(node, leaf);

         return true;
      }

      boolean newLeafIsSignificantyAboveOldLeaf = (leaf.getValue().getHeight() - node.getLeaf().getValue().getHeight()) > heightThreshold;
      boolean newLeafIsSignificantyBelowOldLeaf = (leaf.getValue().getHeight() - node.getLeaf().getValue().getHeight()) < -heightThreshold;

      if (node.getMetaData().getIsStuffAboveMe() && newLeafIsSignificantyAboveOldLeaf)
      {
         return false;
      }

      if (!newLeafIsSignificantyAboveOldLeaf && !newLeafIsSignificantyBelowOldLeaf)
      {
         return false;
      }

      if (canSplit(node))
      {
         HyperCubeLeaf<GroundAirDescriptor> oldLeaf = node.getLeaf();

         node.setLeaf(null);
         node.split();

         node.getChildAtLocation(oldLeaf.getLocation()).setLeaf(oldLeaf);

         //       replaceLeaf(node.getChildAtLocation(leaf.getLocation()), leaf); Only in a slowly increasing resolution quadtree, incompatible with height map.

         initializeMetaDataForChildren(node, node.getMetaData().getIsStuffAboveMe(), Math.min(oldLeaf.getValue().getHeight(), leaf.getValue().getHeight()));
         if (node.getMetaData().getIsStuffAboveMe())
            setDefaultLeafInAllEmptyChildren(node, oldLeaf.getValue());

         putRecursively(node, leaf);

         return true;
      }

      if (newLeafIsSignificantyAboveOldLeaf)
      {
         node.getMetaData().setIsStuffAboveMe(true);

         return true; // true because metadata changed
      }

      if (newLeafIsSignificantyBelowOldLeaf)
      {
         node.setLeaf(leaf);
         node.getMetaData().setHeight(leaf.getValue().getHeight());
         node.getMetaData().setIsStuffAboveMe(true);

         return true;
      }

      replaceLeaf(node, leaf);

      return true;
   }
   
   //================================================================================
   // Private Utility Functions
   //================================================================================

   private void setDefaultLeafInAllEmptyChildren(RecursableHyperTreeNode<GroundAirDescriptor, GroundOnlyQuadTreeData> node, GroundAirDescriptor value)
   {
      for (int i = 0; i < node.getChildNumber(); i++)
      {
         RecursableHyperTreeNode<GroundAirDescriptor, GroundOnlyQuadTreeData> child = node.getChild(i);
         if (child.getLeaf() == null)
         {
            double[] childMiddle = new double[node.getDimensionality()];
            for (int j = 0; j < node.getDimensionality(); j++)
            {
               childMiddle[j] = child.getBounds(j).midpoint();
            }

            child.setLeaf(new HyperCubeLeaf<GroundAirDescriptor>(value, childMiddle));
         }
      }
   }

   private void initializeMetaDataForChildren(RecursableHyperTreeNode<GroundAirDescriptor, GroundOnlyQuadTreeData> node, boolean hasStuffAbove, double height)
   {
      for (int i = 0; i < node.getChildNumber(); i++)
      {
         node.getChild(i).setMetaData(new GroundOnlyQuadTreeData());
         node.getChild(i).getMetaData().setIsStuffAboveMe(hasStuffAbove);
         node.getChild(i).getMetaData().setHeight(height);
         node.getChild(i).updateMetaDataListeners();
      }
   }

   private void replaceLeaf(RecursableHyperTreeNode<GroundAirDescriptor, GroundOnlyQuadTreeData> node, HyperCubeLeaf<GroundAirDescriptor> leaf)
   {
      HyperCubeLeaf<GroundAirDescriptor> oldLeaf = node.getLeaf();
      node.setLeaf(mergeLeaves(oldLeaf, leaf));
   }

   private static double[] toLocation(double x, double y)
   {
      return new double[] { x, y };
   }

   private static OneDimensionalBounds[] toBounds(double xMin, double xMax, double yMin, double yMax)
   {
      return new OneDimensionalBounds[] { new OneDimensionalBounds(xMin, xMax), new OneDimensionalBounds(yMin, yMax) };
   }
   
   private static OneDimensionalBounds[] toBounds(BoundingBox2D bounds)
   {
      return new OneDimensionalBounds[] { new OneDimensionalBounds(bounds.getMinPoint().getX(), bounds.getMaxPoint().getX()), new OneDimensionalBounds(bounds.getMinPoint().getY(), bounds.getMaxPoint().getY()) };
   }

   //================================================================================
   // Inherited from HeightMap
   //================================================================================

   public void clear()
   {
      this.clearTree();
   }

   public boolean containsPoint(double x, double y)
   {
      return !Double.isNaN(getHeightAtPoint(x, y));
   }

   public double getHeightAtPoint(double x, double y)
   {
      HyperCubeLeaf<GroundAirDescriptor> hyperCubeLeaf = this.get(new double[] { x, y });
      RecursableHyperTreeNode<GroundAirDescriptor, GroundOnlyQuadTreeData> node = this.getLeafNodeAtLocation(new double[] { x, y });

      if (hyperCubeLeaf == null)
         return getMetaDataHeight(node);
      if (hyperCubeLeaf.getValue() == null)
         return getMetaDataHeight(node);
      if (hyperCubeLeaf.getValue().getHeight() == null)
         return getMetaDataHeight(node);

      return hyperCubeLeaf.getValue().getHeight();
   }

   private double getMetaDataHeight(RecursableHyperTreeNode<GroundAirDescriptor, GroundOnlyQuadTreeData> node)
   {
      if (node.getMetaData() == null)
      {
         System.err.println("GroundOnlyQuadTree: node has null metadata");

         return Double.NaN;
      }

      return node.getMetaData().getHeight();
   }

   public List<Point3D> getAllPointsWithinArea(double xCenter, double yCenter, double xExtent, double yExtent)
   {
      return getAllPointsUsingGrid(xCenter, yCenter, xExtent, yExtent, constantResolution.getMinResolution());
   }

   private List<Point3D> getAllPointsUsingGrid(double centerX, double centerY, double extentX, double extentY, double resolution)
   {
      ArrayList<Point3D> points = new ArrayList<Point3D>();

      for (double x = centerX - extentX * 0.5; x <= centerX + extentX * 0.5; x += resolution)
      {
         for (double y = centerY - extentY * 0.5; y <= centerY + extentY * 0.5; y += resolution)
         {
            Float f = get(x, y);
            if (f != null)
               points.add(new Point3D(x, y, f));

         }
      }

      return points;
   }

   public List<Point3D> getAllPointsWithinArea(double xCenter, double yCenter, double xExtent, double yExtent,
         InclusionFunction<Point3D> maskFunctionAboutCenter)
   {
      ArrayList<Point3D> points = new ArrayList<Point3D>();

      for (double x = xCenter - xExtent * 0.5; x <= xCenter + xExtent * 0.5; x += constantResolution.getMinResolution())
      {
         for (double y = yCenter - yExtent * 0.5; y <= yCenter + yExtent * 0.5; y += constantResolution.getMinResolution())
         {
            double height = this.getHeightAtPoint(x, y);
            if (!Double.isNaN(height))
            {
               Point3D point3d = new Point3D(x, y, height);
               if (maskFunctionAboutCenter.isIncluded(point3d))
                  points.add(point3d);
            }
         }
      }

      return points;
   }

   //================================================================================
   // Inherited from HyperCubeTree
   //================================================================================

   protected void unSynchronizedMergeOneLevel(RecursableHyperTreeNode<GroundAirDescriptor, GroundOnlyQuadTreeData> node)
   {
      if (!node.hasChildren())
         return;
      HyperCubeLeaf<GroundAirDescriptor> firstLeaf = null;
      boolean stuffAboveToMatch = node.getChild(0).getMetaData().getIsStuffAboveMe();
      for (int i = 0; i < node.getChildNumber(); i++)
      {
         RecursableHyperTreeNode<GroundAirDescriptor, GroundOnlyQuadTreeData> lowerLevelNode = node.getChild(i);
         if (lowerLevelNode.hasChildren())
            return;
         boolean childHasIncompatibleMetaData = stuffAboveToMatch != lowerLevelNode.getMetaData().getIsStuffAboveMe();
         if (childHasIncompatibleMetaData)
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
      node.getMetaData().setHeight(firstLeaf.getValue().getHeight());
      node.getMetaData().setIsStuffAboveMe(stuffAboveToMatch);
   }

   protected boolean canMergeLeaves(HyperCubeLeaf<GroundAirDescriptor> firstLeaf, HyperCubeLeaf<GroundAirDescriptor> secondLeaf)
   {
      float heightDifference = Math.abs(firstLeaf.getValue().getHeight() - secondLeaf.getValue().getHeight());

      return heightDifference < heightThreshold;
   }

   protected boolean canSplit(RecursableHyperTreeNode<GroundAirDescriptor, GroundOnlyQuadTreeData> node)
   {
      if (this.numberOfNodes >= this.maxNodes)
         return false;

      for (int i = 0; i < node.getDimensionality(); i++)
      {
         if (node.getBounds(i).size() <= constantResolution.getResolution(node.getMidpoint()))
            return false;
      }

      return true;
   }

   protected HyperCubeLeaf<GroundAirDescriptor> mergeLeaves(HyperCubeLeaf<GroundAirDescriptor> oldLeaf, HyperCubeLeaf<GroundAirDescriptor> newLeaf)
   {
      // PclTODO: this needs work.
      return newLeaf;
   }

   public void nodeAdded(String id, OneDimensionalBounds[] bounds, HyperCubeLeaf<GroundAirDescriptor> leaf)
   {
      super.nodeAdded(id, bounds, leaf);
      this.numberOfNodes++;
   }

   public void nodeRemoved(String id)
   {
      super.nodeRemoved(id);
      this.numberOfNodes--;
   }

   public void treeCleared()
   {

   }

   //================================================================================
   // Getters used for Test
   //================================================================================

   protected Float get(double xToTest, double yToTest)
   {
      HyperCubeLeaf<GroundAirDescriptor> resultLeaf = this.get(toLocation(xToTest, yToTest));
      if (null == resultLeaf)
         return null;

      return resultLeaf.getValue().getHeight();
   }

   protected RecursableHyperTreeNode<GroundAirDescriptor, GroundOnlyQuadTreeData> getLeafNodeAtLocation(float xToTest, float yToTest)
   {
      return getLeafNodeAtLocation(toLocation(xToTest, yToTest));
   }

   protected double getMaxY()
   {
      return this.getRootNode().getBounds(1).max();
   }

   protected double getMinY()
   {
      return this.getRootNode().getBounds(1).min();
   }

   protected double getMaxX()
   {
      return this.getRootNode().getBounds(0).max();
   }

   protected double getMinX()
   {
      return this.getRootNode().getBounds(0).min();
   }

   //================================================================================
   // Setters
   //================================================================================


   public void setHeightThreshold(double threshold)
   {
      this.heightThreshold = threshold;
   }

   @Override
   public void addQuadTreeListener(QuadTreeForGroundListener jmeGroundONlyQuadTreeVisualizer)
   {      
   }

   @Override
   /**
    * Get points filtered by the Quadtree
    */
   public void getStoredPoints(Collection<Point3D> points)
   {
      getCellAverageStoredPoints(points);
   }
   @Override
   public void getCellAverageStoredPoints(Collection<Point3D> points)
   {
      List<RecursableHyperTreeNode<GroundAirDescriptor, GroundOnlyQuadTreeData>> leaves =listAllLeafNodes();
      for(RecursableHyperTreeNode<GroundAirDescriptor, GroundOnlyQuadTreeData> leaf :leaves)
      {
         HyperCubeLeaf<GroundAirDescriptor> leafData = leaf.getLeaf();
         if(leafData!=null)
            points.add(new Point3D(leafData.getLocation()[0], leafData.getLocation()[1], leafData.getValue().getHeight()));
      }
      
   }
   

   public void clearTree(double defaultGroundHeight)
   {
      super.clearTree();
      this.defaultHeightWhenNoPoints = defaultGroundHeight;
   }
   
   public double getDefaultHeightWhenNoPoints()
   {
      return defaultHeightWhenNoPoints;
   }

   @Override
   public boolean hasPoints()
   {
      return true;
   }


}
