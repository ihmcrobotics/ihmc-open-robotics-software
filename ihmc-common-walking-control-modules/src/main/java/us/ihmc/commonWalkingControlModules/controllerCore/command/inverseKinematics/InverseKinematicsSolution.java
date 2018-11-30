package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.spatial.interfaces.MomentumReadOnly;

public class InverseKinematicsSolution
{
   private final JointBasics[] jointsToOptimizeFor;
   private final DenseMatrix64F jointVelocities;

   private MomentumReadOnly centroidalMomentumSolution;

   public InverseKinematicsSolution(JointBasics[] jointsToOptimizeFor, DenseMatrix64F jointVelocities)
   {
      this.jointsToOptimizeFor = jointsToOptimizeFor;
      this.jointVelocities = jointVelocities;
   }

   public JointBasics[] getJointsToOptimizeFor()
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
