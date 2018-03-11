package us.ihmc.commonWalkingControlModules.virtualModelControl;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.groundContactForce.GroundContactForceSolution;
import us.ihmc.robotics.screwTheory.*;

import java.util.List;
import java.util.Map;

public class NewVirtualModelControlSolution extends GroundContactForceSolution
{
   private InverseDynamicsJoint[] jointsToCompute;
   private DenseMatrix64F jointTorques;

   public void setGroundContactForceSolution(GroundContactForceSolution other)
   {
      this.centroidalMomentumRateSolution = other.getCentroidalMomentumRateSolution();
      this.centroidalMomentumSelectionMatrix = other.getCentroidalMomentumSelectionMatrix();
      this.externalWrenchSolution = other.getExternalWrenchSolution();
      this.rigidBodiesWithExternalWrench = other.getRigidBodiesWithExternalWrench();
   }

   public void setJointsToCompute(InverseDynamicsJoint[] jointsToCompute)
   {
      this.jointsToCompute = jointsToCompute;
   }

   public void setJointTorques(DenseMatrix64F jointTorques)
   {
      this.jointTorques = jointTorques;
   }

   public InverseDynamicsJoint[] getJointsToCompute()
   {
      return jointsToCompute;
   }

   public DenseMatrix64F getJointTorques()
   {
      return jointTorques;
   }
}
