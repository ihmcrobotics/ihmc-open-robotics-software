package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import java.util.List;
import java.util.Map;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.SpatialForceReadOnly;

public class MomentumModuleSolution
{
   private JointBasics[] jointsToOptimizeFor;
   private DMatrixRMaj jointAccelerations;
   private DMatrixRMaj rhoSolution;
   private SpatialForceReadOnly centroidalMomentumRateSolution;
   private Map<RigidBodyBasics, Wrench> externalWrenchSolution;
   private List<RigidBodyBasics> rigidBodiesWithExternalWrench;

   public void setJointsToOptimizeFor(JointBasics[] jointsToOptimizeFor)
   {
      this.jointsToOptimizeFor = jointsToOptimizeFor;
   }

   public void setJointAccelerations(DMatrixRMaj jointAccelerations)
   {
      this.jointAccelerations = jointAccelerations;
   }
   public void setRhoSolution(DMatrixRMaj rhoSolution)
   {
      this.rhoSolution = rhoSolution;
   }

   public void setCentroidalMomentumRateSolution(SpatialForceReadOnly centroidalMomentumRateSolution)
   {
      this.centroidalMomentumRateSolution = centroidalMomentumRateSolution;
   }

   public void setExternalWrenchSolution(Map<RigidBodyBasics, Wrench> externalWrenchSolution)
   {
      this.externalWrenchSolution = externalWrenchSolution;
   }

   public void setRigidBodiesWithExternalWrench(List<RigidBodyBasics> rigidBodiesWithExternalWrench)
   {
      this.rigidBodiesWithExternalWrench = rigidBodiesWithExternalWrench;
   }

   public SpatialForceReadOnly getCentroidalMomentumRateSolution()
   {
      return centroidalMomentumRateSolution;
   }

   public Map<RigidBodyBasics, Wrench> getExternalWrenchSolution()
   {
      return externalWrenchSolution;
   }

   public List<RigidBodyBasics> getRigidBodiesWithExternalWrench()
   {
      return rigidBodiesWithExternalWrench;
   }

   public JointBasics[] getJointsToOptimizeFor()
   {
      return jointsToOptimizeFor;
   }

   public DMatrixRMaj getJointAccelerations()
   {
      return jointAccelerations;
   }

   public DMatrixRMaj getRhoSolution()
   {
      return rhoSolution;
   }
}
