package us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.groundContactForce;

import us.ihmc.robotics.screwTheory.*;

import java.util.List;
import java.util.Map;

public class GroundContactForceSolution
{
   protected SpatialForceVector centroidalMomentumRateSolution;
   protected SelectionMatrix6D centroidalMomentumSelectionMatrix;
   protected Map<RigidBody, Wrench> externalWrenchSolution;
   protected List<RigidBody> rigidBodiesWithExternalWrench;

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

   public SpatialForceVector getCentroidalMomentumRateSolution()
   {
      return centroidalMomentumRateSolution;
   }

   public SelectionMatrix6D getCentroidalMomentumSelectionMatrix()
   {
      return centroidalMomentumSelectionMatrix;
   }
}
