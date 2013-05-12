package us.ihmc.commonWalkingControlModules.momentumBasedController;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumRateOfChangeData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.RootJointAccelerationData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.screwTheory.*;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * @author twan
 *         Date: 4/25/13
 */
public interface MomentumControlModule
{
   void initialize();

   void reset();

   void compute(Map<ContactablePlaneBody, ? extends PlaneContactState> contactStates, RobotSide upcomingSupportSide);

   void resetGroundReactionWrenchFilter();

   void setDesiredJointAcceleration(InverseDynamicsJoint joint, DenseMatrix64F jointAcceleration);

   void setDesiredSpatialAcceleration(GeometricJacobian jacobian, TaskspaceConstraintData taskspaceConstraintData);

   void setDesiredRateOfChangeOfMomentum(MomentumRateOfChangeData momentumRateOfChangeData);

   void setExternalWrenchToCompensateFor(RigidBody rigidBody, Wrench wrench);

   SpatialForceVector getDesiredCentroidalMomentumRate();

   Map<RigidBody, Wrench> getExternalWrenches();
}
