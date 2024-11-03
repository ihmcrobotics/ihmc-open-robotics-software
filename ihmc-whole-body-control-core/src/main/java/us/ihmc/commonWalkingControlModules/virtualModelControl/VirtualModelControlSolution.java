package us.ihmc.commonWalkingControlModules.virtualModelControl;

import java.util.List;
import java.util.Map;

import org.ejml.data.DMatrixRMaj;

import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.SpatialForceReadOnly;

public class VirtualModelControlSolution
{
   private DMatrixRMaj jointTorques;

   private SpatialForceReadOnly centroidalMomentumRateSolution;
   private Map<RigidBodyBasics, Wrench> externalWrenchSolution;
   private List<RigidBodyBasics> rigidBodiesWithExternalWrench;

   public void setCentroidalMomentumRateSolution(SpatialForceReadOnly centroidalMomentumRateSolution)
   {
      this.centroidalMomentumRateSolution = centroidalMomentumRateSolution;
   }

   public void setExternalWrenchSolution(List<RigidBodyBasics> rigidBodiesWithExternalWrench, Map<RigidBodyBasics, Wrench> externalWrenchSolution)
   {
      this.rigidBodiesWithExternalWrench = rigidBodiesWithExternalWrench;
      this.externalWrenchSolution = externalWrenchSolution;
   }

   public List<RigidBodyBasics> getRigidBodiesWithExternalWrench()
   {
      return rigidBodiesWithExternalWrench;
   }

   public Map<RigidBodyBasics, Wrench> getExternalWrenchSolution()
   {
      return externalWrenchSolution;
   }

   public SpatialForceReadOnly getCentroidalMomentumRateSolution()
   {
      return centroidalMomentumRateSolution;
   }

   public void setJointTorques(DMatrixRMaj jointTorques)
   {
      this.jointTorques = jointTorques;
   }

   public DMatrixRMaj getJointTorques()
   {
      return jointTorques;
   }
}
