package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.Momentum;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.robotics.screwTheory.SpatialMotionVector;

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
