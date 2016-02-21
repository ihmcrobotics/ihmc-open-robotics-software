package us.ihmc.commonWalkingControlModules.momentumBasedController.dataObjects.solver;

import java.util.List;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.robotics.screwTheory.Wrench;

public class MomentumModuleSolution
{
   private final InverseDynamicsJoint[] jointsToOptimizeFor;
   private final DenseMatrix64F jointAccelerations;
   private final SpatialForceVector centroidalMomentumRateSolution;
   private final Map<RigidBody, Wrench> externalWrenchSolution;
   private final List<RigidBody> rigidBodiesWithExternalWrench;

   public MomentumModuleSolution(InverseDynamicsJoint[] jointsToOptimizeFor, DenseMatrix64F jointAccelerations,
         SpatialForceVector centroidalMomentumRateSolution, Map<RigidBody, Wrench> externalWrenchSolution, List<RigidBody> rigidBodiesWithExternalWrench)
   {
      this.jointsToOptimizeFor = jointsToOptimizeFor;
      this.jointAccelerations = jointAccelerations;
      this.centroidalMomentumRateSolution = centroidalMomentumRateSolution;
      this.externalWrenchSolution = externalWrenchSolution;
      this.rigidBodiesWithExternalWrench = rigidBodiesWithExternalWrench;
   }

   public SpatialForceVector getCentroidalMomentumRateSolution()
   {
      return centroidalMomentumRateSolution;
   }

   public Map<RigidBody, Wrench> getExternalWrenchSolution()
   {
      return externalWrenchSolution;
   }

   public List<RigidBody> getRigidBodiesWithExternalWrench()
   {
      return rigidBodiesWithExternalWrench;
   }

   public InverseDynamicsJoint[] getJointsToOptimizeFor()
   {
      return jointsToOptimizeFor;
   }

   public DenseMatrix64F getJointAccelerations()
   {
      return jointAccelerations;
   }
}
