package us.ihmc.commonWalkingControlModules.momentumBasedController;

import java.util.Map;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactableCylinderBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.wrenchDistribution.CylindricalContactState;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.utilities.screwTheory.SpatialForceVector;
import us.ihmc.utilities.screwTheory.Wrench;

/**
 * @author twan
 *         Date: 4/25/13
 */
public interface MomentumControlModule
{
   public abstract void initialize();

   public abstract void reset();

   public abstract void compute(Map<ContactablePlaneBody, ? extends PlaneContactState> contactStates, Map<ContactableCylinderBody, ? extends CylindricalContactState> cylinderContactStates, RobotSide upcomingSupportSide);

   public abstract void resetGroundReactionWrenchFilter();

   public abstract void setDesiredJointAcceleration(InverseDynamicsJoint joint, DenseMatrix64F jointAcceleration);

   public abstract void setDesiredSpatialAcceleration(GeometricJacobian jacobian, TaskspaceConstraintData taskspaceConstraintData);

   public abstract void setDesiredRateOfChangeOfMomentum(MomentumRateOfChangeData momentumRateOfChangeData);

   public abstract SpatialForceVector getDesiredCentroidalMomentumRate();

   public abstract Map<RigidBody, Wrench> getExternalWrenches();

   public abstract void setExternalWrenchToCompensateFor(RigidBody rigidBody, Wrench wrench);

   public abstract void setDesiredPointAcceleration(GeometricJacobian jacobian, FramePoint bodyFixedPoint, FrameVector desiredAccelerationWithRespectToBase);

   public abstract void setDesiredJointAcceleration(InverseDynamicsJoint joint, DenseMatrix64F jointAcceleration, double weight);

   public abstract void setDesiredSpatialAcceleration(GeometricJacobian jacobian, TaskspaceConstraintData taskspaceConstraintData, double weight);
}
