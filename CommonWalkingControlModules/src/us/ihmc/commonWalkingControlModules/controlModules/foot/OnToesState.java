package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Matrix3d;

import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculatorTools;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.utilities.humanoidRobot.partNames.LegJointName;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.robotSide.RobotSide;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.SpatialMotionVector;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.yoUtilities.controllers.YoPositionPIDGains;
import us.ihmc.yoUtilities.controllers.YoSE3PIDGains;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.humanoidRobot.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;
import us.ihmc.yoUtilities.math.trajectories.providers.YoVariableDoubleProvider;

public class OnToesState extends AbstractFootControlState
{
   private static final int NUMBER_OF_CONTACTS_POINTS_TO_ROTATE_ABOUT = 2;
   private static final boolean USE_TOEOFF_FOOT_HOLD_POSITION = true;
   private static final boolean CONTROL_SINGLE_POINT = true;

   protected final List<FramePoint2d> edgeContactPoints;
   private final List<FramePoint> desiredEdgeContactPositions;
   private final YoVariableDoubleProvider maximumToeOffAngleProvider;

   private final Twist footTwist = new Twist();

   private double desiredYawToHold = 0.0;
   private double desiredRollToHold = 0.0;
   private final double[] tempYawPitchRoll = new double[3];

   private final FramePoint contactPointPosition = new FramePoint();
   private final FrameVector contactPointLinearVelocity = new FrameVector();
   private final FramePoint proportionalPart = new FramePoint();
   private final FrameVector derivativePart = new FrameVector();

   private final int rootToFootJacobianId;

   private final YoPlaneContactState contactState = momentumBasedController.getContactState(contactableBody);
   private final List<YoContactPoint> contactPoints = contactState.getContactPoints();
   private final List<FramePoint> originalContactPointPositions;
   
   private final List<YoFrameVector> contactPointPositionErrors = new ArrayList<YoFrameVector>();
   private final List<YoFrameVector> contactPointDesiredAccelerations = new ArrayList<YoFrameVector>();

   private final FramePoint2d singleToeContactPoint;

   double alphaShrinkFootSizeForToeOff = 0.0;
   double kneeToeOffGain = 0.0;
   double kneeToeOffQDesired = 0.7;
   double kneeToeOffDamp = 0.0;
   double controlledKneeToeOffQdd;

   private final OneDoFJoint kneeJoint;

   private final YoSE3PIDGains gains;
   private final Matrix3d proportionalGainMatrix;
   private final Matrix3d derivativeGainMatrix;

   public OnToesState(WalkingControllerParameters walkingControllerParameters, RigidBodySpatialAccelerationControlModule accelerationControlModule,
         MomentumBasedController momentumBasedController, ContactablePlaneBody contactableBody, int jacobianId, DoubleYoVariable nullspaceMultiplier,
         BooleanYoVariable jacobianDeterminantInRange, BooleanYoVariable doSingularityEscape, YoSE3PIDGains gains, RobotSide robotSide,
         YoVariableRegistry registry)
   {
      super(ConstraintType.TOES, accelerationControlModule, momentumBasedController, contactableBody, jacobianId, nullspaceMultiplier,
            jacobianDeterminantInRange, doSingularityEscape, robotSide, registry);

      rootToFootJacobianId = momentumBasedController.getOrCreateGeometricJacobian(rootBody, jacobian.getEndEffector(), rootBody.getBodyFixedFrame());

      String namePrefix = contactableBody.getName();
      maximumToeOffAngleProvider = new YoVariableDoubleProvider(namePrefix + "MaximumToeOffAngle", registry);
      maximumToeOffAngleProvider.set(walkingControllerParameters.getMaximumToeOffAngle());

      //      if (edgeContactPoints.size() != NUMBER_OF_CONTACTS_POINTS_TO_ROTATE_ABOUT)
      //         throw new RuntimeException("Number of contacts not handled for OnEdgeState: " + edgeContactPoints.size());

      this.edgeContactPoints = getEdgeContactPoints2d();
      desiredEdgeContactPositions = new ArrayList<FramePoint>();
      for (int i = 0; i < 2; i++)
      {
         desiredEdgeContactPositions.add(edgeContactPoints.get(i).toFramePoint());
         
         String index = String.valueOf(i);
         
         YoFrameVector contactPointPositionError = new YoFrameVector(namePrefix + "ToeOffContactPoint" + index + "PositionError", worldFrame, registry);
         contactPointPositionErrors.add(contactPointPositionError);

         YoFrameVector contactPointDesiredAcceleration = new YoFrameVector(namePrefix + "ToeOffContactPoint" + index + "DesiredAcceleration", worldFrame, registry);
         contactPointDesiredAccelerations.add(contactPointDesiredAcceleration);
      }
      singleToeContactPoint = new FramePoint2d(edgeContactPoints.get(0).getReferenceFrame());
      singleToeContactPoint.interpolate(edgeContactPoints.get(0), edgeContactPoints.get(1), 0.5);

      originalContactPointPositions = new ArrayList<FramePoint>(contactPoints.size());
      copyOriginalContactPointPositions();

      kneeJoint = momentumBasedController.getFullRobotModel().getLegJoint(robotSide, LegJointName.KNEE);

      this.gains = gains;
      YoPositionPIDGains positionGains = gains.getPositionGains();
      proportionalGainMatrix = positionGains.createProportionalGainMatrix();
      derivativeGainMatrix = positionGains.createDerivativeGainMatrix();
   }

   @Override
   public void doSpecificAction()
   {
      desiredOrientation.setToZero(contactableBody.getFrameAfterParentJoint());
      desiredOrientation.changeFrame(worldFrame);
      desiredOrientation.getYawPitchRoll(tempYawPitchRoll);

      momentumBasedController.getTwistCalculator().packRelativeTwist(footTwist, rootBody, contactableBody.getRigidBody());
      footTwist.changeFrame(contactableBody.getFrameAfterParentJoint());

      boolean blockToMaximumPitch = tempYawPitchRoll[1] > maximumToeOffAngleProvider.getValue();

      double footPitch = 0.0, footPitchd = 0.0, footPitchdd = 0.0;

      desiredPosition.setToZero(contactableBody.getFrameAfterParentJoint());
      desiredPosition.changeFrame(worldFrame);

      if (blockToMaximumPitch)
      {
         footPitch = maximumToeOffAngleProvider.getValue();
         footPitchd = 0.0;
         footPitchdd = 0.0;
      }
      else
      {
         footPitch = desiredOrientation.getPitch();
         footPitchd = footTwist.getAngularPartY();
         footPitchdd = 0.0;
      }

      if (USE_TOEOFF_FOOT_HOLD_POSITION)
         desiredOrientation.setYawPitchRoll(desiredYawToHold, footPitch, desiredRollToHold);
      else
         desiredOrientation.setYawPitchRoll(tempYawPitchRoll[0], footPitch, tempYawPitchRoll[2]);

      desiredLinearVelocity.setToZero(worldFrame);
      desiredAngularVelocity.setIncludingFrame(contactableBody.getFrameAfterParentJoint(), 0.0, footPitchd, 0.0);
      desiredAngularVelocity.changeFrame(worldFrame);

      desiredLinearAcceleration.setToZero(worldFrame);
      desiredAngularAcceleration.setIncludingFrame(contactableBody.getFrameAfterParentJoint(), 0.0, footPitchdd, 0.0);
      desiredAngularAcceleration.changeFrame(worldFrame);

      accelerationControlModule.doPositionControl(desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity,
            desiredLinearAcceleration, desiredAngularAcceleration, rootBody);
      accelerationControlModule.packAcceleration(footAcceleration);

      int numberOfContactPointsToControl = CONTROL_SINGLE_POINT ? 1 : edgeContactPoints.size();
      for (int i = 0; i < numberOfContactPointsToControl; i++)
      {
         FramePoint2d contactPoint2d = edgeContactPoints.get(i);
         contactPointPosition.setIncludingFrame(contactPoint2d.getReferenceFrame(), contactPoint2d.getX(), contactPoint2d.getY(), 0.0);

         contactPointPosition.changeFrame(footTwist.getBaseFrame());
         footTwist.changeFrame(footTwist.getBaseFrame());
         footTwist.packVelocityOfPointFixedInBodyFrame(contactPointLinearVelocity, contactPointPosition);
         contactPointPosition.changeFrame(rootBody.getBodyFixedFrame());

         proportionalPart.changeFrame(rootBody.getBodyFixedFrame());
         proportionalPart.sub(desiredEdgeContactPositions.get(i), contactPointPosition);
         contactPointPositionErrors.get(i).setAndMatchFrame(proportionalPart);
         proportionalGainMatrix.transform(proportionalPart.getPoint());

         derivativePart.setToZero(rootBody.getBodyFixedFrame());
         derivativePart.sub(contactPointLinearVelocity);
         derivativeGainMatrix.transform(derivativePart.getVector());

         desiredLinearAcceleration.setToZero(rootBody.getBodyFixedFrame());
         desiredLinearAcceleration.add(proportionalPart);
         desiredLinearAcceleration.add(derivativePart);
         contactPointDesiredAccelerations.get(i).setAndMatchFrame(desiredLinearAcceleration);

         momentumBasedController.setDesiredPointAcceleration(rootToFootJacobianId, contactPointPosition, desiredLinearAcceleration);
      }

      FrameVector2d axisOfRotation2d = edgeToRotateAbout.getVectorCopy();
      FrameVector axisOfRotation = new FrameVector(axisOfRotation2d.getReferenceFrame(), axisOfRotation2d.getX(), axisOfRotation2d.getY(), 0.0);
      axisOfRotation.normalize();
      axisOfRotation.changeFrame(footAcceleration.getExpressedInFrame());

      selectionMatrix.reshape(2, SpatialMotionVector.SIZE);
      selectionMatrix.set(0, 0, axisOfRotation.getX());
      selectionMatrix.set(0, 1, axisOfRotation.getY());
      selectionMatrix.set(0, 2, axisOfRotation.getZ());
      selectionMatrix.set(1, 0, 0.0);
      selectionMatrix.set(1, 1, 0.0);
      selectionMatrix.set(1, 2, 1.0);

      // Just to make sure we're not trying to do singularity escape
      // (the MotionConstraintHandler crashes when using point jacobian and singularity escape)
      nullspaceMultipliers.reshape(0, 1);
      setTaskspaceConstraint(footAcceleration);

      shrinkFootSizeToMidToe();

      // TODO Hack to do singularity escape since the motion constraint handler doesn't handle it with the given selection matrix 
      controlledKneeToeOffQdd = kneeToeOffGain * Math.max(0.0, kneeToeOffQDesired - kneeJoint.getQ()) - kneeToeOffDamp * kneeJoint.getQd();
      momentumBasedController.setOneDoFJointAcceleration(kneeJoint, controlledKneeToeOffQdd);
   }

   private void copyOriginalContactPointPositions()
   {
      for (int i = 0; i < contactPoints.size(); i++)
      {
         originalContactPointPositions.add(new FramePoint(contactPoints.get(i).getPosition()));
      }
   }

   private void resetContactPointPositions()
   {
      for (int i = 0; i < contactPoints.size(); i++)
      {
         contactPoints.get(i).getPosition().set(originalContactPointPositions.get(i));
      }
   }

   private void shrinkFootSizeToMidToe()
   {
      double alphaShrink = alphaShrinkFootSizeForToeOff;
      for (int i = 0; i < contactPoints.size(); i++)
      {
         FramePoint position = contactPoints.get(i).getPosition();
         position.setX(alphaShrink * position.getX() + (1.0 - alphaShrink) * singleToeContactPoint.getX());
         position.setY(alphaShrink * position.getY() + (1.0 - alphaShrink) * singleToeContactPoint.getY());
      }
   }

   @Override
   public void doTransitionIntoAction()
   {
      super.doTransitionIntoAction();
      edgeToRotateAbout.set(edgeContactPoints.get(0), edgeContactPoints.get(1));
      for (int i = 0; i < 2; i++)
      {
         desiredEdgeContactPositions.get(i).setIncludingFrame(edgeContactPoints.get(i).getReferenceFrame(), edgeContactPoints.get(i).getX(),
               edgeContactPoints.get(i).getY(), 0.0);
         desiredEdgeContactPositions.get(i).changeFrame(rootBody.getBodyFixedFrame());
      }

      desiredOrientation.setToZero(contactableBody.getFrameAfterParentJoint());
      desiredOrientation.changeFrame(worldFrame);
      desiredYawToHold = desiredOrientation.getYaw();
      desiredRollToHold = desiredOrientation.getRoll();

      setOnToesFreeMotionGains();
   }

   @Override
   public void doTransitionOutOfAction()
   {
      super.doTransitionOutOfAction();
      // TODO: kind of a hack
      selectionMatrix.reshape(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);
      CommonOps.setIdentity(selectionMatrix);

      resetContactPointPositions();

      controlledKneeToeOffQdd = 0.0;
   }

   private void setOnToesFreeMotionGains()
   {
      // TODO Pretty hackish there clean that up
      // We use the RigidBodySpatialAccelerationControlModule only for the orientation, the position control part is done in this class
      accelerationControlModule.setPositionProportionalGains(0.0, 0.0, 0.0);
      accelerationControlModule.setPositionDerivativeGains(0.0, 0.0, 0.0);
      accelerationControlModule.setPositionIntegralGains(0.0, 0.0, 0.0, 0.0);
      accelerationControlModule.setPositionMaxAccelerationAndJerk(0.0, 0.0);
      accelerationControlModule.setOrientationGains(gains.getOrientationGains());
   }

   protected List<FramePoint2d> getEdgeContactPoints2d()
   {
      FrameVector direction = new FrameVector(contactableBody.getFrameAfterParentJoint(), 1.0, 0.0, 0.0);

      List<FramePoint> contactPoints = DesiredFootstepCalculatorTools.computeMaximumPointsInDirection(contactableBody.getContactPointsCopy(), direction,
            NUMBER_OF_CONTACTS_POINTS_TO_ROTATE_ABOUT);

      List<FramePoint2d> contactPoints2d = new ArrayList<FramePoint2d>(contactPoints.size());
      for (FramePoint contactPoint : contactPoints)
      {
         contactPoint.changeFrame(contactableBody.getSoleFrame());
         contactPoints2d.add(contactPoint.toFramePoint2d());
      }

      return contactPoints2d;
   }
}
