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
   private InverseDynamicsJoint[] jointsToOptimizeFor;
   private DenseMatrix64F jointAccelerations;
   private SpatialForceVector centroidalMomentumRateSolution;
   private Map<RigidBody, Wrench> externalWrenchSolution;
   private List<RigidBody> rigidBodiesWithExternalWrench;
   public void setJointsToOptimizeFor(InverseDynamicsJoint[] jointsToOptimizeFor)
   {
      this.jointsToOptimizeFor = jointsToOptimizeFor;
   }

   public void setJointAccelerations(DenseMatrix64F jointAccelerations)
   {
      this.jointAccelerations = jointAccelerations;
   }

   public void setCentroidalMomentumRateSolution(SpatialForceVector centroidalMomentumRateSolution)
   {
      this.centroidalMomentumRateSolution = centroidalMomentumRateSolution;
   }

   public void setExternalWrenchSolution(Map<RigidBody, Wrench> externalWrenchSolution)
   {
      this.externalWrenchSolution = externalWrenchSolution;
   }

   public void setRigidBodiesWithExternalWrench(List<RigidBody> rigidBodiesWithExternalWrench)
   {
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
