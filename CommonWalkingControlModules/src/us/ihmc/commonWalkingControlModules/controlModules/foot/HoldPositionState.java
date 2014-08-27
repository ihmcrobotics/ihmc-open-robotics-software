package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.util.controller.GainCalculator;

public class HoldPositionState extends AbstractFootControlState
{
   private final FrameVector holdPositionNormalContactVector = new FrameVector();
   private final BooleanYoVariable requestHoldPosition;
   private final FrameVector fullyConstrainedNormalContactVector;
   private final EnumYoVariable<ConstraintType> requestedState;

   private double holdZeta, holdKpx, holdKpy, holdKpz, /* holdKdz, */holdKpRoll, holdKpPitch, holdKpYaw;

   public HoldPositionState(RigidBodySpatialAccelerationControlModule accelerationControlModule, MomentumBasedController momentumBasedController,
         ContactablePlaneBody contactableBody, BooleanYoVariable requestHoldPosition, EnumYoVariable<ConstraintType> requestedState, int jacobianId,
         DoubleYoVariable nullspaceMultiplier, BooleanYoVariable jacobianDeterminantInRange, BooleanYoVariable doSingularityEscape,
         FrameVector fullyConstrainedNormalContactVector, RobotSide robotSide, YoVariableRegistry registry)
   {
      super(ConstraintType.HOLD_POSITION, accelerationControlModule, momentumBasedController, contactableBody, jacobianId, nullspaceMultiplier,
            jacobianDeterminantInRange, doSingularityEscape, robotSide, registry);

      this.requestHoldPosition = requestHoldPosition;
      this.fullyConstrainedNormalContactVector = fullyConstrainedNormalContactVector;
      this.requestedState = requestedState;
   }

   public void doTransitionIntoAction()
   {
      // Remember the previous contact normal, in case the foot leaves the ground and rotates
      holdPositionNormalContactVector.setIncludingFrame(fullyConstrainedNormalContactVector);
      holdPositionNormalContactVector.changeFrame(worldFrame);
      momentumBasedController.setPlaneContactStateNormalContactVector(contactableBody, holdPositionNormalContactVector);

      desiredPosition.setToZero(contactableBody.getFrameAfterParentJoint());
      desiredPosition.changeFrame(worldFrame);

      desiredOrientation.setToZero(contactableBody.getFrameAfterParentJoint());
      desiredOrientation.changeFrame(worldFrame);

      desiredLinearVelocity.setToZero(worldFrame);
      desiredAngularVelocity.setToZero(worldFrame);

      desiredLinearAcceleration.setToZero(worldFrame);
      desiredAngularAcceleration.setToZero(worldFrame);
   }

   public void doSpecificAction()
   {
      setHoldPositionStateGains();
      determineCoPOnEdge();

      if (!isCoPOnEdge && (requestHoldPosition == null || !requestHoldPosition.getBooleanValue()))
         requestedState.set(ConstraintType.FULL);

      accelerationControlModule.doPositionControl(desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity,
            desiredLinearAcceleration, desiredAngularAcceleration, rootBody);
      accelerationControlModule.packAcceleration(footAcceleration);

      setTaskspaceConstraint(footAcceleration);
   }

   @Override
   public void doTransitionOutOfAction()
   {
   }

   public void setHoldGains(double holdZeta, double holdKpx, double holdKpy, double holdKpz, double holdKdz, double holdKpRoll, double holdKpPitch,
         double holdKpYaw)
   {
      this.holdZeta = holdZeta;
      this.holdKpx = holdKpx;
      this.holdKpy = holdKpy;
      this.holdKpz = holdKpz;
      //      this.holdKdz = holdKdz;
      this.holdKpRoll = holdKpRoll;
      this.holdKpPitch = holdKpPitch;
      this.holdKpYaw = holdKpYaw;
   }

   private void setHoldPositionStateGains()
   {
      double dxPosition = GainCalculator.computeDerivativeGain(holdKpx, holdZeta);
      double dyPosition = GainCalculator.computeDerivativeGain(holdKpy, holdZeta);
      double dzPosition = GainCalculator.computeDerivativeGain(holdKpz, holdZeta);
      double dxOrientation = GainCalculator.computeDerivativeGain(holdKpRoll, holdZeta);
      double dyOrientation = GainCalculator.computeDerivativeGain(holdKpPitch, holdZeta);
      double dzOrientation = GainCalculator.computeDerivativeGain(holdKpYaw, holdZeta);

      accelerationControlModule.setPositionProportionalGains(holdKpx, holdKpy, holdKpz);
      accelerationControlModule.setPositionDerivativeGains(dxPosition, dyPosition, dzPosition);
      accelerationControlModule.setOrientationProportionalGains(holdKpRoll, holdKpPitch, holdKpYaw);
      accelerationControlModule.setOrientationDerivativeGains(dxOrientation, dyOrientation, dzOrientation);
   }
}
