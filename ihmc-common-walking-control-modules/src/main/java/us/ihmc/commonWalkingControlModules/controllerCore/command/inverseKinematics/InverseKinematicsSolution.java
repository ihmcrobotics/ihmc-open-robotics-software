package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.mecano.spatial.interfaces.MomentumReadOnly;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;

public class InverseKinematicsSolution
{
   private final InverseDynamicsJoint[] jointsToOptimizeFor;
   private final DenseMatrix64F jointVelocities;

   private MomentumReadOnly centroidalMomentumSolution;

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

   public void setCentroidalMomentumSolution(MomentumReadOnly centroidalMomentumSolution)
   {
      this.centroidalMomentumSolution = centroidalMomentumSolution;
   }

   public MomentumReadOnly getCentroidalMomentumSolution()
   {
      return centroidalMomentumSolution;
   }
}
