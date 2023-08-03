package us.ihmc.humanoidRobotics.bipedSupportPolygons;

import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;

import java.util.List;
import java.util.Vector;

public class StepConstraintRegionsList
{
   private final RecyclingArrayList<StepConstraintRegion> stepConstraintRegions = new RecyclingArrayList<>(StepConstraintRegion::new);

   public void clear()
   {
      stepConstraintRegions.clear();
   }

   public StepConstraintRegion getNextConstraintRegion()
   {
      StepConstraintRegion region = stepConstraintRegions.add();
      region.clear();
      return region;
   }

   public void set(StepConstraintRegionsList other)
   {
      stepConstraintRegions.clear();
      for (int i = 0; i < other.stepConstraintRegions.size(); i++)
      {
         stepConstraintRegions.add().set(other.stepConstraintRegions.get(i));
      }
   }

   public List<StepConstraintRegion> getAsList()
   {
      return stepConstraintRegions;
   }

   public void addOffset(Vector3DReadOnly offset)
   {
      for (int i = 0; i < stepConstraintRegions.size(); i++)
      {
         stepConstraintRegions.get(i).addOffset(offset);
      }
   }
}
