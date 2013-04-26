package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl;

import org.ejml.data.DenseMatrix64F;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.PlaneContactState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumRateOfChangeData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.RootJointAccelerationData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.TaskspaceConstraintData;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.screwTheory.GeometricJacobian;
import us.ihmc.utilities.screwTheory.InverseDynamicsJoint;
import us.ihmc.utilities.screwTheory.SpatialForceVector;
import us.ihmc.utilities.screwTheory.Wrench;

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

   void compute(RootJointAccelerationData rootJointAccelerationData, MomentumRateOfChangeData
         momentumRateOfChangeData, LinkedHashMap<ContactablePlaneBody, ? extends PlaneContactState> contactStates, RobotSide upcomingSupportLeg);

   void resetGroundReactionWrenchFilter();

   FramePoint2d getCoP(ContactablePlaneBody contactablePlaneBody);

   void setDesiredJointAcceleration(InverseDynamicsJoint joint, DenseMatrix64F jointAcceleration);

   void setDesiredSpatialAcceleration(GeometricJacobian spineJacobian, TaskspaceConstraintData taskspaceConstraintData);

   SpatialForceVector getDesiredCentroidalMomentumRate();

   Map<ContactablePlaneBody, Wrench> getExternalWrenches();
}
