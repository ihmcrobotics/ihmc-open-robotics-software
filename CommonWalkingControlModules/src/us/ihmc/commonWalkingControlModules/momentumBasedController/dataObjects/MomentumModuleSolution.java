package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects;

import java.util.Map;

import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialForceVector;
import us.ihmc.utilities.screwTheory.Wrench;

public class MomentumModuleSolution
{
   public final SpatialForceVector centroidalMomentumRateSolution;
   public final Map<RigidBody, Wrench> externalWrenchSolution;

   public MomentumModuleSolution(SpatialForceVector centroidalMomentumRateSolution, Map<RigidBody, Wrench> externalWrenchSolution)
   {
      this.centroidalMomentumRateSolution = centroidalMomentumRateSolution;
      this.externalWrenchSolution = externalWrenchSolution;
   }

   public SpatialForceVector getCentroidalMomentumRateSolution()
   {
      return centroidalMomentumRateSolution;
   }

   public Map<RigidBody, Wrench> getExternalWrenchSolution()
   {
      return externalWrenchSolution;
   }
}
