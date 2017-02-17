package us.ihmc.commonWalkingControlModules.virtualModelControl;

import java.util.List;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.SpatialForceVector;
import us.ihmc.robotics.screwTheory.Wrench;

public class VirtualModelControlSolution
{
   private InverseDynamicsJoint[] jointsToCompute;
   private Map<InverseDynamicsJoint, Double> jointTorques;
   private SpatialForceVector centroidalMomentumRateSolution;
   private DenseMatrix64F centroidalMomentumSelectionMatrix;
   private Map<RigidBody, Wrench> externalWrenchSolution;
   private List<RigidBody> rigidBodiesWithExternalWrench;
   private List<RigidBody> bodiesInContact;

   public VirtualModelControlSolution()
   {
   }

   public void setJointsToCompute(InverseDynamicsJoint[] jointsToCompute)
   {
      this.jointsToCompute = jointsToCompute;
   }

   public void setJointTorques(Map<InverseDynamicsJoint, Double> jointTorques)
   {
      this.jointTorques = jointTorques;
   }

   public void setCentroidalMomentumRateSolution(SpatialForceVector centroidalMomentumRateSolution)
   {
      this.centroidalMomentumRateSolution = centroidalMomentumRateSolution;
   }

   public void setExternalWrenchSolution(List<RigidBody> rigidBodiesWithExternalWrench, Map<RigidBody, Wrench> externalWrenchSolution)
   {
      this.rigidBodiesWithExternalWrench = rigidBodiesWithExternalWrench;
      this.externalWrenchSolution = externalWrenchSolution;
   }

   public void setBodiesInContact(List<RigidBody> bodiesInContact)
   {
      this.bodiesInContact = bodiesInContact;
   }

   public void setCentroidalMomentumSelectionMatrix(DenseMatrix64F selectionMatrix)
   {
      this.centroidalMomentumSelectionMatrix = selectionMatrix;
   }

   public List<RigidBody> getRigidBodiesWithExternalWrench()
   {
      return rigidBodiesWithExternalWrench;
   }

   public Map<RigidBody, Wrench> getExternalWrenchSolution()
   {
      return externalWrenchSolution;
   }

   public List<RigidBody> getBodiesInContact()
   {
      return bodiesInContact;
   }

   public InverseDynamicsJoint[] getJointsToCompute()
   {
      return jointsToCompute;
   }

   public Map<InverseDynamicsJoint, Double> getJointTorques()
   {
      return jointTorques;
   }

   public SpatialForceVector getCentroidalMomentumRateSolution()
   {
      return centroidalMomentumRateSolution;
   }

   public DenseMatrix64F getCentroidalMomentumSelectionMatrix()
   {
      return centroidalMomentumSelectionMatrix;
   }
}
