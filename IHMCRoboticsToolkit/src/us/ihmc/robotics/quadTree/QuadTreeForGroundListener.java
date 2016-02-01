package us.ihmc.robotics.quadTree;

import javax.vecmath.Point3f;

public interface QuadTreeForGroundListener
{
   public void nodeAdded(String id, Box bounds, float x, float y, float height);

   public void nodeRemoved(String id);

   public void RawPointAdded(float x, float y, float z);
   
   public void PopToOctree(Point3f location);
   public void PopToOctree(Point3f location, Point3f LidarHeadLocation);
   
   
//   public void metaDataUpdated(String id, OneDimensionalBounds[] bounds, float height);
//
//   public void treeCleared();

}
