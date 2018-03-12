package us.ihmc.commonWalkingControlModules.virtualModelControl;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.robotics.screwTheory.*;

import java.util.List;
import java.util.Map;

public class NewVirtualModelControlSolution
{
   private DenseMatrix64F jointTorques;

   private SpatialForceVector centroidalMomentumRateSolution;
   private Map<RigidBody, Wrench> externalWrenchSolution;
   private List<RigidBody> rigidBodiesWithExternalWrench;

   public void setCentroidalMomentumRateSolution(SpatialForceVector centroidalMomentumRateSolution)
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

   public SpatialForceVector getCentroidalMomentumRateSolution()
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
