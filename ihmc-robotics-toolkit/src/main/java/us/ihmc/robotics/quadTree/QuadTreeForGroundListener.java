package us.ihmc.robotics.quadTree;

import us.ihmc.euclid.tuple3D.Point3D32;

public interface QuadTreeForGroundListener
{
   public void nodeAdded(String id, Box bounds, float x, float y, float height);

   public void nodeRemoved(String id);

   public void RawPointAdded(float x, float y, float z);
   
   public void PopToOctree(Point3D32 location);
   public void PopToOctree(Point3D32 location, Point3D32 LidarHeadLocation);
   
   
//   public void metaDataUpdated(String id, OneDimensionalBounds[] bounds, float height);
//
//   public void treeCleared();

}
