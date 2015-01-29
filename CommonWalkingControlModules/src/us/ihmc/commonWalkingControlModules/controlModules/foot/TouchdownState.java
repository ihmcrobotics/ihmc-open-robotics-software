package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Vector3d;

import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculatorTools;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.math.geometry.RigidBodyTransform;
import us.ihmc.utilities.math.trajectories.providers.ConstantDoubleProvider;
import us.ihmc.utilities.math.trajectories.providers.DoubleProvider;
import us.ihmc.utilities.math.trajectories.providers.VectorProvider;
import us.ihmc.utilities.screwTheory.SpatialMotionVector;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.yoUtilities.controllers.GainCalculator;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.BooleanYoVariable;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.trajectories.ConstantVelocityTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.DoubleTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.providers.YoVariableDoubleProvider;

public class TouchdownState extends AbstractFootControlState
{
   private static final int NUMBER_OF_CONTACTS_POINTS_TO_ROTATE_ABOUT = 2;

   private final DoubleTrajectoryGenerator onEdgePitchAngleTrajectoryGenerator;
   private final YoVariableDoubleProvider touchdownInitialPitchProvider;
   private final YoVariableDoubleProvider touchdownInitialAngularVelocityProvider;

   private final List<FramePoint2d> edgeContactPoints;
   private final List<FramePoint> desiredEdgeContactPositions;

   private final Twist footTwist = new Twist();

   private final double[] tempYawPitchRoll = new double[3];

   private final FramePoint contactPointPosition = new FramePoint();
   private final FrameVector contactPointLinearVelocity = new FrameVector();
   private final FramePoint proportionalPart = new FramePoint();
   private final FrameVector derivativePart = new FrameVector();

   private final int rootToFootJacobianId;

   private final WalkingControllerParameters walkingControllerParameters;

   private final VectorProvider touchdownVelocityProvider;

   public TouchdownState(ConstraintType stateEnum, FootControlHelper footControlHelper, VectorProvider touchdownVelocityProvider, DoubleYoVariable nullspaceMultiplier,
         BooleanYoVariable jacobianDeterminantInRange, BooleanYoVariable doSingularityEscape, YoVariableRegistry registry)
   {
      super(stateEnum, footControlHelper, nullspaceMultiplier, jacobianDeterminantInRange, doSingularityEscape, registry);

      this.walkingControllerParameters = footControlHelper.getWalkingControllerParameters();
      this.touchdownVelocityProvider = touchdownVelocityProvider;
      rootToFootJacobianId = momentumBasedController.getOrCreateGeometricJacobian(rootBody, jacobian.getEndEffector(), rootBody.getBodyFixedFrame());

      String namePrefix = contactableBody.getName();

      if (stateEnum == ConstraintType.TOES_TOUCHDOWN)
         namePrefix += "Toe";
      else
         namePrefix += "Heel";

      touchdownInitialPitchProvider = new YoVariableDoubleProvider(namePrefix + "TouchdownPitch", registry);

      if (stateEnum == ConstraintType.TOES_TOUCHDOWN)
      {
         touchdownInitialPitchProvider.set(walkingControllerParameters.getToeTouchdownAngle());
      }
      else
      {
         touchdownInitialPitchProvider.set(walkingControllerParameters.getHeelTouchdownAngle());
      }

      touchdownInitialAngularVelocityProvider = new YoVariableDoubleProvider(namePrefix + "TouchdownPitchAngularVelocity", registry);
      DoubleProvider touchdownTrajectoryTime = new ConstantDoubleProvider(walkingControllerParameters.getDefaultTransferTime() / 3.0);

      this.onEdgePitchAngleTrajectoryGenerator = new ConstantVelocityTrajectoryGenerator(namePrefix + "FootTouchdownPitchTrajectoryGenerator",
            touchdownInitialPitchProvider, touchdownInitialAngularVelocityProvider, touchdownTrajectoryTime, registry);

      this.edgeContactPoints = getEdgeContactPoints2d();
      desiredEdgeContactPositions = new ArrayList<FramePoint>();
      for (int i = 0; i < 2; i++)
         desiredEdgeContactPositions.add(edgeContactPoints.get(i).toFramePoint());
   }

   @Override
   public void doTransitionIntoAction()
   {
      super.doTransitionIntoAction();
      updateTouchdownInitialAngularVelocity();
      setTouchdownOnEdgeGains();

      edgeToRotateAbout.set(edgeContactPoints.get(0), edgeContactPoints.get(1));
      for (int i = 0; i < 2; i++)
      {
         desiredEdgeContactPositions.get(i).setIncludingFrame(edgeContactPoints.get(i).getReferenceFrame(), edgeContactPoints.get(i).getX(),
               edgeContactPoints.get(i).getY(), 0.0);
         desiredEdgeContactPositions.get(i).changeFrame(rootBody.getBodyFixedFrame());
      }

      if (onEdgePitchAngleTrajectoryGenerator != null)
      {
         onEdgePitchAngleTrajectoryGenerator.initialize();
      }

      desiredOrientation.setToZero(contactableBody.getFrameAfterParentJoint());
      desiredOrientation.changeFrame(worldFrame);
   }

   @Override
   public void doSpecificAction()
   {
      desiredOrientation.setToZero(contactableBody.getFrameAfterParentJoint());
      desiredOrientation.changeFrame(worldFrame);
      desiredOrientation.getYawPitchRoll(tempYawPitchRoll);

      momentumBasedController.getTwistCalculator().packRelativeTwist(footTwist, rootBody, contactableBody.getRigidBody());
      footTwist.changeFrame(contactableBody.getFrameAfterParentJoint());

      double footPitch = 0.0, footPitchd = 0.0, footPitchdd = 0.0;

      desiredPosition.setToZero(contactableBody.getFrameAfterParentJoint());
      desiredPosition.changeFrame(worldFrame);

      onEdgePitchAngleTrajectoryGenerator.compute(getTimeInCurrentState());
      footPitch = onEdgePitchAngleTrajectoryGenerator.getValue();
      footPitchd = onEdgePitchAngleTrajectoryGenerator.getVelocity();
      footPitchdd = onEdgePitchAngleTrajectoryGenerator.getAcceleration();
      //      desiredOnEdgeAngle.set(footPitch);

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

      for (int i = 0; i < edgeContactPoints.size(); i++)
      {
         FramePoint2d contactPoint2d = edgeContactPoints.get(i);
         contactPointPosition.setIncludingFrame(contactPoint2d.getReferenceFrame(), contactPoint2d.getX(), contactPoint2d.getY(), 0.0);

         contactPointPosition.changeFrame(footTwist.getBaseFrame());
         footTwist.changeFrame(footTwist.getBaseFrame());
         footTwist.packVelocityOfPointFixedInBodyFrame(contactPointLinearVelocity, contactPointPosition);
         contactPointPosition.changeFrame(rootBody.getBodyFixedFrame());

         proportionalPart.changeFrame(rootBody.getBodyFixedFrame());
         proportionalPart.sub(desiredEdgeContactPositions.get(i), contactPointPosition);
         proportionalPart.scale(0.0, 0.0, 0.0);

         derivativePart.setToZero(rootBody.getBodyFixedFrame());
         derivativePart.sub(contactPointLinearVelocity);
         derivativePart.scale(0.0, 0.0, 0.0);

         desiredLinearAcceleration.setToZero(rootBody.getBodyFixedFrame());
         desiredLinearAcceleration.add(proportionalPart);
         desiredLinearAcceleration.add(derivativePart);

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
   }

   @Override
   public void doTransitionOutOfAction()
   {
      super.doTransitionOutOfAction();
      // TODO: kind of a hack
      selectionMatrix.reshape(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);
      CommonOps.setIdentity(selectionMatrix);
   }

   private List<FramePoint2d> getEdgeContactPoints2d()
   {
      FrameVector direction = new FrameVector(contactableBody.getFrameAfterParentJoint(), 1.0, 0.0, 0.0);
      if (getStateEnum() == ConstraintType.HEEL_TOUCHDOWN)
         direction.scale(-1.0);

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

   public double getTouchdownInitialPitchAngle()
   {
      return touchdownInitialPitchProvider.getValue();
   }

   private void updateTouchdownInitialAngularVelocity()
   {
      Vector3d edgeToAnkle;

      if (getStateEnum() == ConstraintType.TOES_TOUCHDOWN)
         edgeToAnkle = new Vector3d(-walkingControllerParameters.getFootForwardOffset(), 0.0, walkingControllerParameters.getAnkleHeight());
      else
         edgeToAnkle = new Vector3d(walkingControllerParameters.getFootBackwardOffset(), 0.0, walkingControllerParameters.getAnkleHeight());

      RigidBodyTransform rotationByPitch = new RigidBodyTransform();
      rotationByPitch.rotY(-touchdownInitialPitchProvider.getValue());
      rotationByPitch.transform(edgeToAnkle);

      Vector3d orthonormalToHeelToAnkle = new Vector3d(edgeToAnkle);
      RigidBodyTransform perpendicularRotation = new RigidBodyTransform();
      perpendicularRotation.rotY(-Math.PI / 2.0);
      perpendicularRotation.transform(orthonormalToHeelToAnkle);
      orthonormalToHeelToAnkle.normalize();

      FrameVector linearVelocity = new FrameVector();
      touchdownVelocityProvider.get(linearVelocity);
      double angularSpeed = -orthonormalToHeelToAnkle.dot(linearVelocity.getVector()) / edgeToAnkle.length();

      touchdownInitialAngularVelocityProvider.set(angularSpeed);
   }

   private void setTouchdownOnEdgeGains()
   {
      double kPosition = 0.0;
      double dPosition = GainCalculator.computeDerivativeGain(kPosition, 1.0);
      double kxzOrientation = 300.0;
      double kyOrientation = 0.0;
      double dxzOrientation = GainCalculator.computeDerivativeGain(kxzOrientation, 0.4);
      double dyOrientation = GainCalculator.computeDerivativeGain(kxzOrientation, 0.4); //Only damp the pitch velocity

      accelerationControlModule.setPositionProportionalGains(kPosition, kPosition, kPosition);
      accelerationControlModule.setPositionDerivativeGains(dPosition, dPosition, dPosition);
      accelerationControlModule.setOrientationProportionalGains(kxzOrientation, kyOrientation, kxzOrientation);
      accelerationControlModule.setOrientationDerivativeGains(dxzOrientation, dyOrientation, dxzOrientation);
   }
}
