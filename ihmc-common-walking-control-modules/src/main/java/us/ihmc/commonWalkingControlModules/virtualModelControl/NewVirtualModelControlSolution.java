package us.ihmc.commonWalkingControlModules.virtualModelControl;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.screwTheory.*;

import java.util.List;
import java.util.Map;

public class NewVirtualModelControlSolution
{
   private InverseDynamicsJoint[] jointsToCompute;
   private DenseMatrix64F jointTorques;
   private SpatialForceVector centroidalMomentumRateSolution;
   private SelectionMatrix6D centroidalMomentumSelectionMatrix;
   private Map<RigidBody, Wrench> externalWrenchSolution;
   private List<RigidBody> rigidBodiesWithExternalWrench;

   public NewVirtualModelControlSolution()
   {
   }

   public void setJointsToCompute(InverseDynamicsJoint[] jointsToCompute)
   {
      this.jointsToCompute = jointsToCompute;
   }

   public void setJointTorques(DenseMatrix64F jointTorques)
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

   public void setCentroidalMomentumSelectionMatrix(SelectionMatrix6D selectionMatrix)
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

   public InverseDynamicsJoint[] getJointsToCompute()
   {
      return jointsToCompute;
   }

   public DenseMatrix64F getJointTorques()
   {
      return jointTorques;
   }

   public SpatialForceVector getCentroidalMomentumRateSolution()
   {
      return centroidalMomentumRateSolution;
   }

   public SelectionMatrix6D getCentroidalMomentumSelectionMatrix()
   {
      return centroidalMomentumSelectionMatrix;
   }
}
