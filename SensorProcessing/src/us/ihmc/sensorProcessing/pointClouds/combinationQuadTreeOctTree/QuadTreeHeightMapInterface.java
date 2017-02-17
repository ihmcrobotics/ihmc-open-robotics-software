package us.ihmc.sensorProcessing.pointClouds.combinationQuadTreeOctTree;

import java.util.Collection;

import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.robotics.dataStructures.HeightMapWithPoints;
import us.ihmc.robotics.hyperCubeTree.HyperCubeTreeListener;
import us.ihmc.robotics.quadTree.QuadTreeForGroundListener;

public interface QuadTreeHeightMapInterface extends HeightMapWithPoints
{

	/**
	 * 
	 * @param defaultGroundHeight, set to Double.NaN so it would take the value from first point.z
	 */
	public abstract void clearTree(double defaultGroundHeight); 
   public abstract double getDefaultHeightWhenNoPoints();


	public abstract void setHeightThreshold(double quadtreeHeightThreshold);

   public abstract void addListener(HyperCubeTreeListener<GroundAirDescriptor, GroundOnlyQuadTreeData> jmeGroundONlyQuadTreeVisualizer);
   public abstract void addQuadTreeListener(QuadTreeForGroundListener jmeGroundONlyQuadTreeVisualizer);

	public abstract boolean addToQuadtree(double x, double y, double z);
	
   public void getStoredPoints(Collection<Point3D> points);
   public void getCellAverageStoredPoints(Collection<Point3D> points);

   public abstract void lock();

   public abstract void unlock();
   public abstract boolean hasPoints();
}
