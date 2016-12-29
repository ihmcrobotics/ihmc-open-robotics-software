package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

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
   private final DenseMatrix64F rhoSolution;
   private final SpatialForceVector centroidalMomentumRateSolution;
   private final Map<RigidBody, Wrench> externalWrenchSolution;
   private final List<RigidBody> rigidBodiesWithExternalWrench;

   public MomentumModuleSolution(InverseDynamicsJoint[] jointsToOptimizeFor, DenseMatrix64F jointAccelerations, DenseMatrix64F rhoSolution,
         SpatialForceVector centroidalMomentumRateSolution, Map<RigidBody, Wrench> externalWrenchSolution, List<RigidBody> rigidBodiesWithExternalWrench)
   {
      this.jointsToOptimizeFor = jointsToOptimizeFor;
      this.jointAccelerations = jointAccelerations;
      this.rhoSolution = rhoSolution;
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

   public DenseMatrix64F getRhoSolution()
   {
      return rhoSolution;
   }
}
