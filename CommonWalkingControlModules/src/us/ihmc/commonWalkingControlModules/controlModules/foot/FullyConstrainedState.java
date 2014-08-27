package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.yoUtilities.YoVariableRegistry;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;

public class FullyConstrainedState extends AbstractFootControlState
{
   private static final boolean USE_SUPPORT_FOOT_HOLD_POSITION_STATE = true;
   private static final boolean USE_SUPPORT_DAMPING = true;

   private final double supportKdRoll = 20.0;
   private final double supportKdPitch = 0.0;
   private final double supportKdYaw = 0.0;

   private final BooleanYoVariable requestHoldPosition;
   private final FrameVector fullyConstrainedNormalContactVector;
   private final BooleanYoVariable doFancyOnToesControl;

   private final EnumYoVariable<ConstraintType> requestedState;

   public FullyConstrainedState(RigidBodySpatialAccelerationControlModule accelerationControlModule, MomentumBasedController momentumBasedController,
         ContactablePlaneBody contactableBody, BooleanYoVariable requestHoldPosition, EnumYoVariable<ConstraintType> requestedState, int jacobianId,
         DoubleYoVariable nullspaceMultiplier, BooleanYoVariable jacobianDeterminantInRange, BooleanYoVariable doSingularityEscape,
         FrameVector fullyConstrainedNormalContactVector, BooleanYoVariable doFancyOnToesControl, RobotSide robotSide, YoVariableRegistry registry)
   {
      super(ConstraintType.FULL, accelerationControlModule, momentumBasedController,
            contactableBody, jacobianId, nullspaceMultiplier, jacobianDeterminantInRange, doSingularityEscape, robotSide, registry);

      this.requestHoldPosition = requestHoldPosition;
      this.fullyConstrainedNormalContactVector = fullyConstrainedNormalContactVector;
      this.doFancyOnToesControl = doFancyOnToesControl;
      this.requestedState = requestedState;
   }

   public void doTransitionIntoAction()
   {
      momentumBasedController.setPlaneContactStateNormalContactVector(contactableBody, fullyConstrainedNormalContactVector);
   }

   private void setFullyConstrainedStateGains()
   {
      accelerationControlModule.setPositionProportionalGains(0.0, 0.0, 0.0);
      accelerationControlModule.setPositionDerivativeGains(0.0, 0.0, 0.0);
      accelerationControlModule.setOrientationProportionalGains(0.0, 0.0, 0.0);
      accelerationControlModule.setOrientationDerivativeGains(supportKdRoll, supportKdPitch, supportKdYaw);
   }

   public void doSpecificAction()
   {
      // is false on the real robot anyway
      if (doFancyOnToesControl.getBooleanValue())
         determineCoPOnEdge();

      if (USE_SUPPORT_FOOT_HOLD_POSITION_STATE)
      {
         if (isCoPOnEdge && doFancyOnToesControl.getBooleanValue())
            requestedState.set(ConstraintType.HOLD_POSITION);
         else if (requestHoldPosition != null && requestHoldPosition.getBooleanValue())
            requestedState.set(ConstraintType.HOLD_POSITION);
      }

      if (!USE_SUPPORT_DAMPING)
      {
         footAcceleration.setToZero(contactableBody.getFrameAfterParentJoint(), rootBody.getBodyFixedFrame(), contactableBody.getFrameAfterParentJoint());
      }
      else
      {
         setFullyConstrainedStateGains();

         desiredPosition.setToZero(contactableBody.getFrameAfterParentJoint());
         desiredPosition.changeFrame(worldFrame);

         desiredOrientation.setToZero(contactableBody.getFrameAfterParentJoint());
         desiredOrientation.changeFrame(worldFrame);

         desiredLinearVelocity.setToZero(worldFrame);
         desiredAngularVelocity.setToZero(worldFrame);

         desiredLinearAcceleration.setToZero(worldFrame);
         desiredAngularAcceleration.setToZero(worldFrame);

         accelerationControlModule.doPositionControl(desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity,
               desiredLinearAcceleration, desiredAngularAcceleration, rootBody);
         accelerationControlModule.packAcceleration(footAcceleration);
      }

      setTaskspaceConstraint(footAcceleration);
   }

   @Override
   public void doTransitionOutOfAction()
   {

   }
}
