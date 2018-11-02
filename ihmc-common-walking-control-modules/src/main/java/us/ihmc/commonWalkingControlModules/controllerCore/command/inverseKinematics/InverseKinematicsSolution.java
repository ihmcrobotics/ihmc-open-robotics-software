package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.mecano.spatial.Momentum;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;

public class InverseKinematicsSolution
{
   private final InverseDynamicsJoint[] jointsToOptimizeFor;
   private final DenseMatrix64F jointVelocities;

   private Momentum centroidalMomentumSolution;

   public InverseKinematicsSolution(InverseDynamicsJoint[] jointsToOptimizeFor, DenseMatrix64F jointVelocities)
   {
      this.jointsToOptimizeFor = jointsToOptimizeFor;
      this.jointVelocities = jointVelocities;
   }

   public InverseDynamicsJoint[] getJointsToOptimizeFor()
   {
      return jointsToOptimizeFor;
   }

   public DenseMatrix64F getJointVelocities()
   {
      return jointVelocities;
   }

   public void setCentroidalMomentumSolution(Momentum centroidalMomentumSolution)
   {
      this.centroidalMomentumSolution = centroidalMomentumSolution;
   }

   public Momentum getCentroidalMomentumSolution()
   {
      return centroidalMomentumSolution;
   }
}
