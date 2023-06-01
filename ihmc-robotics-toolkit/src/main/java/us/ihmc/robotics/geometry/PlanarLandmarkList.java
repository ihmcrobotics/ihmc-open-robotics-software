package us.ihmc.robotics.geometry;

import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;

import java.util.ArrayList;

public class PlanarLandmarkList
{
   private ArrayList<PlanarLandmark> planes = new ArrayList<>();

   public void addPlane(PlanarLandmark plane)
   {
      planes.add(plane);
   }

   public PlanarLandmark getPlanarLandmarkById(int id)
   {
      return planes.get(id);
   }

   public PlanarLandmarkList()
   {
   }

   public PlanarLandmarkList(PlanarRegionsList planarRegionsList)
   {
      for (PlanarRegion planarRegion : planarRegionsList.getPlanarRegionsAsList())
      {
         PlanarLandmark plane = new PlanarLandmark(planarRegion.getRegionId(),
                                                   planarRegion.getPlane(),
                                                   planarRegion.getBoundingBox3dInWorld(),
                                                   planarRegion.getTransformToWorld(),
                                                   planarRegion.getArea());
         planes.add(plane);
      }
   }

   public ArrayList<PlanarLandmark> getPlanarLandmarksAsList()
   {
      return planes;
   }

   public PlanarLandmarkList copy()
   {
      PlanarLandmarkList landmarksListCopy = new PlanarLandmarkList();
      for (PlanarLandmark landmark : planes)
      {
         landmarksListCopy.addPlane(new PlanarLandmark(landmark));
      }
      return landmarksListCopy;
   }

   public void applyTransform(RigidBodyTransformReadOnly transform)
   {
      for (PlanarLandmark plane : planes)
      {
         plane.applyTransform(transform);
      }
   }

   public void clear()
   {
      planes.clear();
   }

   public void addAll(PlanarLandmarkList other)
   {
      planes.addAll(other.getPlanarLandmarksAsList());
   }

   public void addAll(PlanarRegionsList regions)
   {
      for (PlanarRegion region : regions.getPlanarRegionsAsList())
      {
         PlanarLandmark plane = new PlanarLandmark(region.getRegionId(),
                                                   region.getPlane(),
                                                   region.getBoundingBox3dInWorld(),
                                                   region.getTransformToWorld(),
                                                   region.getArea());
         planes.add(plane);
      }
   }
}
