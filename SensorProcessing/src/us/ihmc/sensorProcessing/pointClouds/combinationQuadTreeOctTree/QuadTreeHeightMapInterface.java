package us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree;

import us.ihmc.utilities.dataStructures.hyperCubeTree.HyperCubeTreeListener;
import us.ihmc.utilities.dataStructures.hyperCubeTree.Octree;
import us.ihmc.utilities.dataStructures.quadTree.QuadTreeListener;
import us.ihmc.utilities.math.dataStructures.HeightMap;

public interface QuadTreeHeightMapInterface extends HeightMap
{

	public abstract void setOctree(Octree octree);

	public abstract boolean addPointToOctree(double x, double y, double z);

	public abstract void setUpdateOctree(boolean b);

	public abstract void clearTree();

	public abstract void setHeightThreshold(double quadtreeHeightThreshold);

   public abstract void addListener(HyperCubeTreeListener<GroundAirDescriptor, GroundOnlyQuadTreeData> jmeGroundONlyQuadTreeVisualizer);
   public abstract void addQuadTreeListener(QuadTreeListener jmeGroundONlyQuadTreeVisualizer);

	public abstract boolean addToQuadtree(double x, double y, double z);

	public abstract void setUpdateQuadtree(boolean update);
}
