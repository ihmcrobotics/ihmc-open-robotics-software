package us.ihmc.commonWalkingControlModules.controlModules.foot;

import javax.vecmath.AxisAngle4d;
import javax.vecmath.Vector3d;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.sensors.footSwitch.FootSwitchInterface;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.geometry.FrameConvexPolygon2d;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.yoUtilities.controllers.YoSE3PIDGains;
import us.ihmc.yoUtilities.math.frames.YoFrameOrientation;

public class HoldPositionState extends AbstractFootControlState
{
   private static final boolean CONTROL_WRT_PELVIS = false;

   private static final double EPSILON = 0.010;

   private final FrameVector holdPositionNormalContactVector = new FrameVector();
   private final FrameVector fullyConstrainedNormalContactVector;

   private final YoSE3PIDGains gains;

   private final RigidBody pelvisBody;
   private final FramePoint2d cop = new FramePoint2d();
   private final PartialFootholdControlModule partialFootholdControlModule;

   private final FootSwitchInterface footSwitch;
   private final FrameConvexPolygon2d footPolygon = new FrameConvexPolygon2d();
   private final FrameLineSegment2d closestEdgeToCoP = new FrameLineSegment2d();
   private final FrameVector2d edgeVector2d = new FrameVector2d();
   private final FrameVector edgeVector = new FrameVector();
   private final FrameOrientation desiredOrientationCopy = new FrameOrientation();
   private final AxisAngle4d desiredAxisAngle = new AxisAngle4d();
   private final Vector3d desiredRotationVector = new Vector3d();

   private final BooleanYoVariable doSmartHoldPosition;
   private final YoFrameOrientation desiredHoldOrientation;
   
   public HoldPositionState(FootControlHelper footControlHelper, YoSE3PIDGains gains, YoVariableRegistry registry)
   {
      super(ConstraintType.HOLD_POSITION, footControlHelper, registry);

      this.gains = gains;
      fullyConstrainedNormalContactVector = footControlHelper.getFullyConstrainedNormalContactVector();
      pelvisBody = momentumBasedController.getFullRobotModel().getPelvis();
      partialFootholdControlModule = footControlHelper.getPartialFootholdControlModule();
      footSwitch = momentumBasedController.getFootSwitches().get(robotSide);
      footPolygon.setIncludingFrameAndUpdate(footControlHelper.getContactableFoot().getContactPoints2d());
      String namePrefix = footControlHelper.getContactableFoot().getName();
      desiredHoldOrientation = new YoFrameOrientation(namePrefix + "DesiredHoldOrientation", worldFrame, registry);
      doSmartHoldPosition = new BooleanYoVariable(namePrefix + "DoSmartHoldPosition", registry);

      doSmartHoldPosition.set(true);
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
      correctDesiredOrientationForSmartHoldPosition();
      desiredHoldOrientation.set(desiredOrientation);
      FramePoint2d desiredCoP = momentumBasedController.getDesiredCoP(contactableFoot);
      partialFootholdControlModule.compute(desiredCoP, cop);
      YoPlaneContactState contactState = momentumBasedController.getContactState(contactableFoot);
      partialFootholdControlModule.applyShrunkPolygon(contactState);

      footControlHelper.setGains(gains);

      footControlHelper.shrinkSupportFootContactPointsToToesIfNecessary();

      RigidBody baseForControl = CONTROL_WRT_PELVIS ? pelvisBody : rootBody;
      RigidBodySpatialAccelerationControlModule accelerationControlModule = footControlHelper.getAccelerationControlModule();
      accelerationControlModule.doPositionControl(desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity,
            desiredLinearAcceleration, desiredAngularAcceleration, baseForControl);
      accelerationControlModule.packAcceleration(footAcceleration);

      footControlHelper.submitTaskspaceConstraint(footAcceleration);
   }

   /**
    * Correct the desired orientation such that if the foot landed on the edge, the foot is still able to rotate towards the ground and eventually be in full support.
    */
   private void correctDesiredOrientationForSmartHoldPosition()
   {
      if (!doSmartHoldPosition.getBooleanValue())
         return;

      if (cop.containsNaN())
         return;

      boolean isCoPOnEdge = !footPolygon.isPointInside(cop, -EPSILON);
      
      if (!isCoPOnEdge)
         return;
      
      footPolygon.getClosestEdge(closestEdgeToCoP, cop);
      closestEdgeToCoP.getFrameVector(edgeVector2d);
      edgeVector.setXYIncludingFrame(edgeVector2d);
      edgeVector.normalize();
      desiredOrientationCopy.setIncludingFrame(desiredOrientation);
      desiredOrientationCopy.changeFrame(footPolygon.getReferenceFrame());
      desiredOrientationCopy.getAxisAngle(desiredAxisAngle);
      desiredRotationVector.set(desiredAxisAngle.getX(), desiredAxisAngle.getY(), desiredAxisAngle.getZ());
      desiredRotationVector.scale(desiredAxisAngle.getAngle());

      boolean holdRotationAroundEdge = true;
      double rotationOnEdge = edgeVector.dot(desiredRotationVector);

      if (closestEdgeToCoP.isPointOnLeftSideOfLineSegment(footPolygon.getCentroid()))
      {
         if (rotationOnEdge > 0.0)
            holdRotationAroundEdge = false;
      }
      else
      {
         if (rotationOnEdge < 0.0)
            holdRotationAroundEdge = false;
      }
      
      if (holdRotationAroundEdge)
         return;

      edgeVector.scale(rotationOnEdge);
      desiredRotationVector.sub(edgeVector.getVector());
      double angle = desiredRotationVector.length();
      desiredRotationVector.scale(1.0 / angle);
      desiredAxisAngle.set(desiredRotationVector, angle);
      desiredOrientationCopy.set(desiredAxisAngle);
      desiredOrientationCopy.changeFrame(worldFrame);
      desiredOrientation.set(desiredOrientationCopy);
   }
}
