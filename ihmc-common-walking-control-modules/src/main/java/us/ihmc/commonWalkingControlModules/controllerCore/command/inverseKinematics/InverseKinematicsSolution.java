package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.spatial.interfaces.MomentumReadOnly;

public class InverseKinematicsSolution
{
   private final JointBasics[] jointsToOptimizeFor;
   private DMatrixRMaj jointVelocities;
   private MomentumReadOnly centroidalMomentumSolution;

   public InverseKinematicsSolution(JointBasics[] jointsToOptimizeFor)
   {
      this.jointsToOptimizeFor = jointsToOptimizeFor;
   }

   public JointBasics[] getJointsToOptimizeFor()
   {
      return jointsToOptimizeFor;
   }

   public void setJointVelocities(DMatrixRMaj jointVelocities)
   {
      this.jointVelocities = jointVelocities;
   }

   public DMatrixRMaj getJointVelocities()
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
