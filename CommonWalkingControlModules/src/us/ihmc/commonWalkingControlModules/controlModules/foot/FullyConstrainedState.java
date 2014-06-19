package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.math.geometry.FrameVector;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.EnumYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;

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

   public FullyConstrainedState(YoFramePoint yoDesiredPosition, YoFrameVector yoDesiredLinearVelocity, YoFrameVector yoDesiredLinearAcceleration,
         RigidBodySpatialAccelerationControlModule accelerationControlModule, MomentumBasedController momentumBasedController,
         ContactablePlaneBody contactableBody, BooleanYoVariable requestHoldPosition, EnumYoVariable<ConstraintType> requestedState, int jacobianId,
         DoubleYoVariable nullspaceMultiplier, BooleanYoVariable jacobianDeterminantInRange, BooleanYoVariable doSingularityEscape,
         FrameVector fullyConstrainedNormalContactVector, BooleanYoVariable doFancyOnToesControl,
         LegSingularityAndKneeCollapseAvoidanceControlModule legSingularityAndKneeCollapseAvoidanceControlModule, RobotSide robotSide,
         YoVariableRegistry registry)
   {
      super(ConstraintType.FULL, yoDesiredPosition, yoDesiredLinearVelocity, yoDesiredLinearAcceleration, accelerationControlModule, momentumBasedController,
            contactableBody, requestedState, jacobianId, nullspaceMultiplier, jacobianDeterminantInRange, doSingularityEscape,
            legSingularityAndKneeCollapseAvoidanceControlModule, robotSide, registry);

      this.requestHoldPosition = requestHoldPosition;
      this.fullyConstrainedNormalContactVector = fullyConstrainedNormalContactVector;
      this.doFancyOnToesControl = doFancyOnToesControl;
   }

   public void doTransitionIntoAction()
   {
      momentumBasedController.setPlaneContactStateNormalContactVector(contactableBody, fullyConstrainedNormalContactVector);
   }

   private void setFullyConstrainedStateGains()
   {
      setGains(0.0, 0.0, 0.0);
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
         footAcceleration.setToZero(contactableBody.getBodyFrame(), rootBody.getBodyFixedFrame(), contactableBody.getBodyFrame());
      }
      else
      {
         setFullyConstrainedStateGains();

         desiredPosition.setToZero(contactableBody.getBodyFrame());
         desiredPosition.changeFrame(worldFrame);

         desiredOrientation.setToZero(contactableBody.getBodyFrame());
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
