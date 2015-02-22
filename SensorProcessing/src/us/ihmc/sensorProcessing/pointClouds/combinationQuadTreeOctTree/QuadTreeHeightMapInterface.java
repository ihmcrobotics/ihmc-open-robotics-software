package us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree;

import java.util.Collection;

import javax.vecmath.Point3d;

import us.ihmc.utilities.dataStructures.hyperCubeTree.HyperCubeTreeListener;
import us.ihmc.utilities.dataStructures.hyperCubeTree.Octree;
import us.ihmc.utilities.dataStructures.quadTree.QuadTreeForGroundListener;
import us.ihmc.utilities.math.dataStructures.HeightMapWithPoints;

public interface QuadTreeHeightMapInterface extends HeightMapWithPoints
{

	public abstract void setOctree(Octree octree);

	public abstract boolean addPointToOctree(double x, double y, double z);

	public abstract void setUpdateOctree(boolean b);

	public abstract void clearTree();

	public abstract void setHeightThreshold(double quadtreeHeightThreshold);

   public abstract void addListener(HyperCubeTreeListener<GroundAirDescriptor, GroundOnlyQuadTreeData> jmeGroundONlyQuadTreeVisualizer);
   public abstract void addQuadTreeListener(QuadTreeForGroundListener jmeGroundONlyQuadTreeVisualizer);

	public abstract boolean addToQuadtree(double x, double y, double z);

	public abstract void setUpdateQuadtree(boolean update);
	
   public void getStoredPoints(Collection<Point3d> points);

   public abstract void lock();

   public abstract void unlock();
}
