package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.robotics.controllers.YoSE3PIDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;

public class FullyConstrainedState extends AbstractFootControlState
{
   private final FrameVector fullyConstrainedNormalContactVector;

   private final YoSE3PIDGains gains;
   private final FramePoint2d cop = new FramePoint2d();
   private final PartialFootholdControlModule partialFootholdControlModule;

   private final FootSwitchInterface footSwitch;

   public FullyConstrainedState(FootControlHelper footControlHelper, YoSE3PIDGains gains, YoVariableRegistry registry)
   {
      super(ConstraintType.FULL, footControlHelper, registry);

      fullyConstrainedNormalContactVector = footControlHelper.getFullyConstrainedNormalContactVector();
      this.gains = gains;
      partialFootholdControlModule = footControlHelper.getPartialFootholdControlModule();
      footSwitch = momentumBasedController.getFootSwitches().get(robotSide);
   }

   @Override
   public void doTransitionIntoAction()
   {
      super.doTransitionIntoAction();
      momentumBasedController.setPlaneContactStateNormalContactVector(contactableFoot, fullyConstrainedNormalContactVector);
      ConstraintType previousStateEnum = getPreviousState().getStateEnum();
      boolean resetCurrentFootShrink = previousStateEnum != ConstraintType.FULL && previousStateEnum != ConstraintType.HOLD_POSITION;
      footControlHelper.initializeParametersForSupportFootShrink(resetCurrentFootShrink);
   }

   @Override
   public void doTransitionOutOfAction()
   {
      super.doTransitionOutOfAction();
      footControlHelper.restoreFootContactPoints();
   }

   @Override
   public void doSpecificAction()
   {
      footSwitch.computeAndPackCoP(cop);
      FramePoint2d desiredCoP = momentumBasedController.getDesiredCoP(contactableFoot);
      partialFootholdControlModule.compute(desiredCoP, cop);
      YoPlaneContactState contactState = momentumBasedController.getContactState(contactableFoot);
      partialFootholdControlModule.applyShrunkPolygon(contactState);

      footControlHelper.shrinkSupportFootContactPointsToToesIfNecessary();

      if (gains == null)
      {
         footAcceleration.setToZero(contactableFoot.getFrameAfterParentJoint(), rootBody.getBodyFixedFrame(), contactableFoot.getFrameAfterParentJoint());
      }
      else
      {
         footControlHelper.setGains(gains);

         desiredPosition.setToZero(contactableFoot.getFrameAfterParentJoint());
         desiredPosition.changeFrame(worldFrame);

         desiredOrientation.setToZero(contactableFoot.getFrameAfterParentJoint());
         desiredOrientation.changeFrame(worldFrame);

         desiredLinearVelocity.setToZero(worldFrame);
         desiredAngularVelocity.setToZero(worldFrame);

         desiredLinearAcceleration.setToZero(worldFrame);
         desiredAngularAcceleration.setToZero(worldFrame);

         RigidBodySpatialAccelerationControlModule accelerationControlModule = footControlHelper.getAccelerationControlModule();
         accelerationControlModule.doPositionControl(desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity,
               desiredLinearAcceleration, desiredAngularAcceleration, rootBody);
         accelerationControlModule.packAcceleration(footAcceleration);
      }

      footControlHelper.submitTaskspaceConstraint(footAcceleration);
   }
}
