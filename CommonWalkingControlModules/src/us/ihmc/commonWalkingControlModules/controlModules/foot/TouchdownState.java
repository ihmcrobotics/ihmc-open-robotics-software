package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.util.ArrayList;
import java.util.List;

import javax.vecmath.Vector3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculatorTools;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.trajectories.providers.ConstantDoubleProvider;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.robotics.trajectories.providers.VectorProvider;
import us.ihmc.utilities.screwTheory.SpatialMotionVector;
import us.ihmc.utilities.screwTheory.Twist;
import us.ihmc.yoUtilities.controllers.YoSE3PIDGains;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
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
   private final FrameLineSegment2d edgeToRotateAbout;

   private final YoSE3PIDGains gains;

   public TouchdownState(ConstraintType stateEnum, FootControlHelper footControlHelper, VectorProvider touchdownVelocityProvider, YoSE3PIDGains gains, YoVariableRegistry registry)
   {
      super(stateEnum, footControlHelper, registry);

      this.gains = gains;
      walkingControllerParameters = footControlHelper.getWalkingControllerParameters();
      this.touchdownVelocityProvider = touchdownVelocityProvider;
      rootToFootJacobianId = momentumBasedController.getOrCreateGeometricJacobian(rootBody, contactableFoot.getRigidBody(), rootBody.getBodyFixedFrame());

      String namePrefix = contactableFoot.getName();

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

      onEdgePitchAngleTrajectoryGenerator = new ConstantVelocityTrajectoryGenerator(namePrefix + "FootTouchdownPitchTrajectoryGenerator",
            touchdownInitialPitchProvider, touchdownInitialAngularVelocityProvider, touchdownTrajectoryTime, registry);

      edgeContactPoints = getEdgeContactPoints2d();
      desiredEdgeContactPositions = new ArrayList<FramePoint>();
      for (int i = 0; i < 2; i++)
         desiredEdgeContactPositions.add(edgeContactPoints.get(i).toFramePoint());

      edgeToRotateAbout = new FrameLineSegment2d(contactableFoot.getSoleFrame());
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

      desiredOrientation.setToZero(contactableFoot.getFrameAfterParentJoint());
      desiredOrientation.changeFrame(worldFrame);
   }

   private final FrameVector axisOfRotation = new FrameVector();
   private final FrameVector2d axisOfRotation2d = new FrameVector2d();

   @Override
   public void doSpecificAction()
   {
      desiredOrientation.setToZero(contactableFoot.getFrameAfterParentJoint());
      desiredOrientation.changeFrame(worldFrame);
      desiredOrientation.getYawPitchRoll(tempYawPitchRoll);

      momentumBasedController.getTwistCalculator().packRelativeTwist(footTwist, rootBody, contactableFoot.getRigidBody());
      footTwist.changeFrame(contactableFoot.getFrameAfterParentJoint());

      double footPitch = 0.0, footPitchd = 0.0, footPitchdd = 0.0;

      desiredPosition.setToZero(contactableFoot.getFrameAfterParentJoint());
      desiredPosition.changeFrame(worldFrame);

      onEdgePitchAngleTrajectoryGenerator.compute(getTimeInCurrentState());
      footPitch = onEdgePitchAngleTrajectoryGenerator.getValue();
      footPitchd = onEdgePitchAngleTrajectoryGenerator.getVelocity();
      footPitchdd = onEdgePitchAngleTrajectoryGenerator.getAcceleration();
      //      desiredOnEdgeAngle.set(footPitch);

//      desiredOrientation.setYawPitchRoll(tempYawPitchRoll[0], footPitch, tempYawPitchRoll[2]); // Only damp the foot

      desiredLinearVelocity.setToZero(worldFrame);
      desiredAngularVelocity.setIncludingFrame(contactableFoot.getFrameAfterParentJoint(), 0.0, footPitchd, 0.0);
      desiredAngularVelocity.changeFrame(worldFrame);

      desiredLinearAcceleration.setToZero(worldFrame);
      desiredAngularAcceleration.setIncludingFrame(contactableFoot.getFrameAfterParentJoint(), 0.0, footPitchdd, 0.0);
      desiredAngularAcceleration.changeFrame(worldFrame);

      RigidBodySpatialAccelerationControlModule accelerationControlModule = footControlHelper.getAccelerationControlModule();
      accelerationControlModule.doPositionControl(desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity, desiredLinearAcceleration, desiredAngularAcceleration, rootBody);
      accelerationControlModule.packAcceleration(footAcceleration);

      edgeToRotateAbout.getFrameVector(axisOfRotation2d);
      axisOfRotation.setXYIncludingFrame(axisOfRotation2d);
      axisOfRotation.normalize();
      axisOfRotation.changeFrame(footAcceleration.getExpressedInFrame());

      DenseMatrix64F selectionMatrix = footControlHelper.getSelectionMatrix();
      selectionMatrix.reshape(2, SpatialMotionVector.SIZE);
      selectionMatrix.set(0, 0, axisOfRotation.getX());
      selectionMatrix.set(0, 1, axisOfRotation.getY());
      selectionMatrix.set(0, 2, axisOfRotation.getZ());
      selectionMatrix.set(1, 0, 0.0);
      selectionMatrix.set(1, 1, 0.0);
      selectionMatrix.set(1, 2, 1.0);

      // Just to make sure we're not trying to do singularity escape
      // (the MotionConstraintHandler crashes when using point jacobian and singularity escape)
      footControlHelper.resetNullspaceMultipliers();
      footControlHelper.submitTaskspaceConstraint(footAcceleration);

      for (int i = 0; i < edgeContactPoints.size(); i++)
      {
         FramePoint2d contactPoint2d = edgeContactPoints.get(i);
         contactPointPosition.setIncludingFrame(contactPoint2d.getReferenceFrame(), contactPoint2d.getX(), contactPoint2d.getY(), 0.0);

         contactPointPosition.changeFrame(footTwist.getBaseFrame());
         footTwist.changeFrame(footTwist.getBaseFrame());
         footTwist.packLinearVelocityOfPointFixedInBodyFrame(contactPointLinearVelocity, contactPointPosition);
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
   }

   @Override
   public void doTransitionOutOfAction()
   {
      super.doTransitionOutOfAction();
      footControlHelper.resetSelectionMatrix();
   }

   private List<FramePoint2d> getEdgeContactPoints2d()
   {
      FrameVector direction = new FrameVector(contactableFoot.getFrameAfterParentJoint(), 1.0, 0.0, 0.0);
      if (getStateEnum() == ConstraintType.HEEL_TOUCHDOWN)
         direction.scale(-1.0);

      List<FramePoint> contactPoints = DesiredFootstepCalculatorTools.computeMaximumPointsInDirection(contactableFoot.getContactPointsCopy(), direction,
            NUMBER_OF_CONTACTS_POINTS_TO_ROTATE_ABOUT);

      List<FramePoint2d> contactPoints2d = new ArrayList<FramePoint2d>(contactPoints.size());
      for (FramePoint contactPoint : contactPoints)
      {
         contactPoint.changeFrame(contactableFoot.getSoleFrame());
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
      footControlHelper.setGainsToZero();
      footControlHelper.setOrientationGains(gains.getOrientationGains());
   }
}
