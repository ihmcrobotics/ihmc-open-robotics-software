package us.ihmc.commonWalkingControlModules.momentumBasedController;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.robotics.screwTheory.GeometricJacobian;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.SpatialForceVector;

/**
 * @author twan
 *         Date: 4/22/13
 */
public interface MomentumSolverInterface
{
   void initialize();

   void reset();

   void setDesiredJointAcceleration(InverseDynamicsJoint joint, DenseMatrix64F jointAcceleration);

   void setDesiredSpatialAcceleration(GeometricJacobian jacobian, TaskspaceConstraintData taskspaceConstraintData);

   void setDesiredSpatialAcceleration(InverseDynamicsJoint[] constrainedJoints, GeometricJacobian jacobian,
                                      TaskspaceConstraintData taskspaceConstraintData);

   void compute();

   void solve(SpatialForceVector momentumRateOfChange);

   void solve(DenseMatrix64F accelerationSubspace, DenseMatrix64F accelerationMultipliers, DenseMatrix64F momentumSubspace,
              DenseMatrix64F momentumMultipliers);

   void getRateOfChangeOfMomentum(SpatialForceVector rateOfChangeOfMomentumToPack);
}
