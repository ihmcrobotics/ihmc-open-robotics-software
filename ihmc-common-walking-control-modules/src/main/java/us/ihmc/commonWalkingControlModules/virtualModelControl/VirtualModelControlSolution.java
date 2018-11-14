package us.ihmc.commonWalkingControlModules.virtualModelControl;

import java.util.List;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.SpatialForceReadOnly;

public class VirtualModelControlSolution
{
   private DenseMatrix64F jointTorques;

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

   public void setJointTorques(DenseMatrix64F jointTorques)
   {
      this.jointTorques = jointTorques;
   }

   public DenseMatrix64F getJointTorques()
   {
      return jointTorques;
   }
}
