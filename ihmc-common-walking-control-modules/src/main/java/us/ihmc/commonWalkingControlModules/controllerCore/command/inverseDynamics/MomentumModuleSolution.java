package us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics;

import java.util.List;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.SpatialForceReadOnly;

public class MomentumModuleSolution
{
   private JointBasics[] jointsToOptimizeFor;
   private DenseMatrix64F jointAccelerations;
   private DenseMatrix64F rhoSolution;
   private SpatialForceReadOnly centroidalMomentumRateSolution;
   private Map<RigidBodyBasics, Wrench> externalWrenchSolution;
   private List<RigidBodyBasics> rigidBodiesWithExternalWrench;

   public void setJointsToOptimizeFor(JointBasics[] jointsToOptimizeFor)
   {
      this.jointsToOptimizeFor = jointsToOptimizeFor;
   }

   public void setJointAccelerations(DenseMatrix64F jointAccelerations)
   {
      this.jointAccelerations = jointAccelerations;
   }
   public void setRhoSolution(DenseMatrix64F rhoSolution)
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

   public DenseMatrix64F getJointAccelerations()
   {
      return jointAccelerations;
   }

   public DenseMatrix64F getRhoSolution()
   {
      return rhoSolution;
   }
}
