package us.ihmc.commonWalkingControlModules.virtualModelControl;

import java.util.List;
import java.util.Map;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.mecano.spatial.SpatialForce;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.robotics.screwTheory.RigidBody;

public class VirtualModelControlSolution
{
   private DenseMatrix64F jointTorques;

   private SpatialForce centroidalMomentumRateSolution;
   private Map<RigidBody, Wrench> externalWrenchSolution;
   private List<RigidBody> rigidBodiesWithExternalWrench;

   public void setCentroidalMomentumRateSolution(SpatialForce centroidalMomentumRateSolution)
   {
      this.centroidalMomentumRateSolution = centroidalMomentumRateSolution;
   }

   public void setExternalWrenchSolution(List<RigidBody> rigidBodiesWithExternalWrench, Map<RigidBody, Wrench> externalWrenchSolution)
   {
      this.rigidBodiesWithExternalWrench = rigidBodiesWithExternalWrench;
      this.externalWrenchSolution = externalWrenchSolution;
   }

   public List<RigidBody> getRigidBodiesWithExternalWrench()
   {
      return rigidBodiesWithExternalWrench;
   }

   public Map<RigidBody, Wrench> getExternalWrenchSolution()
   {
      return externalWrenchSolution;
   }

   public SpatialForce getCentroidalMomentumRateSolution()
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
