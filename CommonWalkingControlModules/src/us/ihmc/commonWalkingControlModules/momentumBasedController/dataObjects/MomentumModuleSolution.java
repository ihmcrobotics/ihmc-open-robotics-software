package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects;

import java.util.Map;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialForceVector;
import us.ihmc.utilities.screwTheory.Wrench;

public class MomentumModuleSolution
{
   public final InverseDynamicsJoint[] jointsToOptimizeFor;
   public final DenseMatrix64F jointAccelerations;
   public final SpatialForceVector centroidalMomentumRateSolution;
   public final Map<RigidBody, Wrench> externalWrenchSolution;

   public MomentumModuleSolution(InverseDynamicsJoint[] jointsToOptimizeFor, DenseMatrix64F jointAccelerations, SpatialForceVector centroidalMomentumRateSolution, Map<RigidBody, Wrench> externalWrenchSolution)
   {
      this.jointsToOptimizeFor = jointsToOptimizeFor;
      this.jointAccelerations = jointAccelerations;
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
