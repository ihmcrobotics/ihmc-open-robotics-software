package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.yoUtilities.controllers.YoSE3PIDGains;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.EnumYoVariable;

public class HoldPositionState extends AbstractFootControlState
{
   private static final boolean CONTROL_WRT_PELVIS = false;

   private final FrameVector holdPositionNormalContactVector = new FrameVector();
   private final BooleanYoVariable requestHoldPosition;
   private final FrameVector fullyConstrainedNormalContactVector;
   private final EnumYoVariable<ConstraintType> requestedState;

   private final YoSE3PIDGains gains;

   private final RigidBody pelvisBody;
   private final FramePoint2d cop = new FramePoint2d();
   private final PartialFootholdControlModule partialFootholdControlModule;

   public HoldPositionState(FootControlHelper footControlHelper, BooleanYoVariable requestHoldPosition, EnumYoVariable<ConstraintType> requestedState,
         int jacobianId, DoubleYoVariable nullspaceMultiplier, BooleanYoVariable jacobianDeterminantInRange, BooleanYoVariable doSingularityEscape,
         FrameVector fullyConstrainedNormalContactVector, YoSE3PIDGains gains, YoVariableRegistry registry)
   {
      super(ConstraintType.HOLD_POSITION, footControlHelper, jacobianId, nullspaceMultiplier, jacobianDeterminantInRange, doSingularityEscape, registry);

      this.requestHoldPosition = requestHoldPosition;
      this.fullyConstrainedNormalContactVector = fullyConstrainedNormalContactVector;
      this.requestedState = requestedState;
      this.gains = gains;
      this.pelvisBody = momentumBasedController.getFullRobotModel().getPelvis();
      this.partialFootholdControlModule = footControlHelper.getPartialFootholdControlModule();
   }

   @Override
   public void doTransitionIntoAction()
   {
      super.doTransitionIntoAction();
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

      accelerationControlModule.setGains(gains);
   }

   @Override
   public void doSpecificAction()
   {
      momentumBasedController.getFootSwitches().get(robotSide).computeAndPackCoP(cop);
      FramePoint2d desiredCoP = momentumBasedController.getDesiredCoP(contactableBody);
      partialFootholdControlModule.compute(desiredCoP, cop);
      YoPlaneContactState contactState = momentumBasedController.getContactState(contactableBody);
      partialFootholdControlModule.applyShrunkPolygon(contactState);

      accelerationControlModule.setGains(gains);
      determineCoPOnEdge();

      if (!isCoPOnEdge && (requestHoldPosition == null || !requestHoldPosition.getBooleanValue()))
         requestedState.set(ConstraintType.FULL);
      if (!FootControlModule.USE_SUPPORT_FOOT_HOLD_POSITION_STATE)
         requestedState.set(ConstraintType.FULL);

      RigidBody baseForControl = CONTROL_WRT_PELVIS ? pelvisBody : rootBody;
      accelerationControlModule.doPositionControl(desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity,
            desiredLinearAcceleration, desiredAngularAcceleration, baseForControl);
      accelerationControlModule.packAcceleration(footAcceleration);

      setTaskspaceConstraint(footAcceleration);
   }
}
