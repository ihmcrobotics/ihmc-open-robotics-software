package us.ihmc.commonWalkingControlModules.controlModules.foot;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.screwTheory.RigidBody;
import us.ihmc.yoUtilities.controllers.YoSE3PIDGains;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;

public class HoldPositionState extends AbstractFootControlState
{
   private static final boolean CONTROL_WRT_PELVIS = true;

   private final FrameVector holdPositionNormalContactVector = new FrameVector();
   private final FrameVector fullyConstrainedNormalContactVector;

   private final YoSE3PIDGains gains;

   private final RigidBody pelvisBody;
   private final FramePoint2d cop = new FramePoint2d();
   private final PartialFootholdControlModule partialFootholdControlModule;

   private final FootSwitchInterface footSwitch;

   public HoldPositionState(FootControlHelper footControlHelper, YoSE3PIDGains gains, YoVariableRegistry registry)
   {
      super(ConstraintType.HOLD_POSITION, footControlHelper, registry);

      this.gains = gains;
      fullyConstrainedNormalContactVector = footControlHelper.getFullyConstrainedNormalContactVector();
      pelvisBody = momentumBasedController.getFullRobotModel().getPelvis();
      partialFootholdControlModule = footControlHelper.getPartialFootholdControlModule();
      footSwitch = momentumBasedController.getFootSwitches().get(robotSide);
   }

   @Override
   public void doTransitionIntoAction()
   {
      super.doTransitionIntoAction();
      // Remember the previous contact normal, in case the foot leaves the ground and rotates
      holdPositionNormalContactVector.setIncludingFrame(fullyConstrainedNormalContactVector);
      holdPositionNormalContactVector.changeFrame(worldFrame);
      momentumBasedController.setPlaneContactStateNormalContactVector(contactableFoot, holdPositionNormalContactVector);

      desiredPosition.setToZero(contactableFoot.getFrameAfterParentJoint());
      desiredPosition.changeFrame(worldFrame);

      desiredOrientation.setToZero(contactableFoot.getFrameAfterParentJoint());
      desiredOrientation.changeFrame(worldFrame);

      desiredLinearVelocity.setToZero(worldFrame);
      desiredAngularVelocity.setToZero(worldFrame);

      desiredLinearAcceleration.setToZero(worldFrame);
      desiredAngularAcceleration.setToZero(worldFrame);

      footControlHelper.setGains(gains);
   }

   @Override
   public void doSpecificAction()
   {
      footSwitch.computeAndPackCoP(cop);
      FramePoint2d desiredCoP = momentumBasedController.getDesiredCoP(contactableFoot);
      partialFootholdControlModule.compute(desiredCoP, cop);
      YoPlaneContactState contactState = momentumBasedController.getContactState(contactableFoot);
      partialFootholdControlModule.applyShrunkPolygon(contactState);

      footControlHelper.setGains(gains);

      RigidBody baseForControl = CONTROL_WRT_PELVIS ? pelvisBody : rootBody;
      RigidBodySpatialAccelerationControlModule accelerationControlModule = footControlHelper.getAccelerationControlModule();
      accelerationControlModule.doPositionControl(desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity,
            desiredLinearAcceleration, desiredAngularAcceleration, baseForControl);
      accelerationControlModule.packAcceleration(footAcceleration);

      footControlHelper.submitTaskspaceConstraint(footAcceleration);
   }
}
