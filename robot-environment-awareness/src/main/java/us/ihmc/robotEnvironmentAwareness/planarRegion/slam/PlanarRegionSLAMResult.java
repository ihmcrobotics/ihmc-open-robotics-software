package us.ihmc.robotEnvironmentAwareness.planarRegion.slam;

import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.robotics.geometry.PlanarRegionsList;

public class PlanarRegionSLAMResult
{
   private final RigidBodyTransform transformFromIncomingToMap;
   private final PlanarRegionsList mergedMap;
   
   public PlanarRegionSLAMResult(RigidBodyTransform transformFromIncomingToMap, PlanarRegionsList mergedMap)
   {
      this.transformFromIncomingToMap = transformFromIncomingToMap;
      this.mergedMap = mergedMap;
   }

   public RigidBodyTransform getTransformFromIncomingToMap()
   {
      return transformFromIncomingToMap;
   }

   public PlanarRegionsList getMergedMap()
   {
      return mergedMap;
   }
}
