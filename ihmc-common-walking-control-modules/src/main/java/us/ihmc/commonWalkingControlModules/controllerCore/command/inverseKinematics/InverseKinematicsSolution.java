package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.spatial.interfaces.MomentumReadOnly;

public class InverseKinematicsSolution
{
   private final JointBasics[] jointsToOptimizeFor;
   private DenseMatrix64F jointVelocities;
   private MomentumReadOnly centroidalMomentumSolution;

   public InverseKinematicsSolution(JointBasics[] jointsToOptimizeFor)
   {
      this.jointsToOptimizeFor = jointsToOptimizeFor;
   }

   public JointBasics[] getJointsToOptimizeFor()
   {
      return jointsToOptimizeFor;
   }

   public void setJointVelocities(DenseMatrix64F jointVelocities)
   {
      this.jointVelocities = jointVelocities;
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
