package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import javax.vecmath.Matrix3d;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculatorTools;
import us.ihmc.robotics.humanoidRobot.partNames.LegJointName;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.FrameLineSegment2d;
import us.ihmc.robotics.geometry.FrameOrientation;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FramePoint2d;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.FrameVector2d;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.trajectories.providers.DoubleProvider;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SpatialMotionVector;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.robotics.screwTheory.TwistCalculator;
import us.ihmc.yoUtilities.controllers.YoPositionPIDGains;
import us.ihmc.yoUtilities.controllers.YoSE3PIDGains;
import us.ihmc.yoUtilities.dataStructure.registry.YoVariableRegistry;
import us.ihmc.yoUtilities.dataStructure.variable.DoubleYoVariable;
import us.ihmc.yoUtilities.math.frames.YoFrameVector;
import us.ihmc.yoUtilities.math.trajectories.ThirdOrderPolynomialTrajectoryGenerator;
import us.ihmc.yoUtilities.math.trajectories.providers.YoVariableDoubleProvider;

public class OnToesState extends AbstractFootControlState
{
   /** If true: toes contact points are merged into one position at mid toe. */
   private static final boolean CONTROL_SINGLE_POINT = true;
   private static final boolean USE_TOEOFF_TRAJECTORY = false;
   private static final double MIN_TRAJECTORY_TIME = 0.1;

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

   private final YoPlaneContactState contactState = momentumBasedController.getContactState(contactableFoot);
   private final List<YoContactPoint> contactPoints = contactState.getContactPoints();

   private final List<YoFrameVector> contactPointPositionErrors = new ArrayList<YoFrameVector>();
   private final List<YoFrameVector> contactPointDesiredAccelerations = new ArrayList<YoFrameVector>();

   private final DoubleYoVariable toeOffDesiredPitchAngle, toeOffDesiredPitchVelocity, toeOffDesiredPitchAcceleration;
   private final DoubleYoVariable toeOffCurrentPitchAngle, toeOffCurrentPitchVelocity;

   private final DoubleYoVariable toeOffInitialAngle;
   private final DoubleYoVariable toeOffInitialVelocity;
   private final DoubleYoVariable toeOffFinalAngle;
   private final DoubleYoVariable toeOffTrajectoryTime;
   private final ThirdOrderPolynomialTrajectoryGenerator toeOffTrajectory;

   private final FramePoint2d singleToeContactPoint;

   private final YoSE3PIDGains gains;
   private final Matrix3d proportionalGainMatrix;
   private final Matrix3d derivativeGainMatrix;

   private final FrameLineSegment2d edgeToRotateAbout;

   private final TwistCalculator twistCalculator;

   private final OneDoFJoint kneeJoint;

   public OnToesState(FootControlHelper footControlHelper, YoSE3PIDGains gains, YoVariableRegistry registry)
   {
      super(ConstraintType.TOES, footControlHelper, registry);

      kneeJoint = momentumBasedController.getFullRobotModel().getLegJoint(robotSide, LegJointName.KNEE);
      rootToFootJacobianId = createRootToFootJacobian();
      twistCalculator = momentumBasedController.getTwistCalculator();

      String namePrefix = contactableFoot.getName();
      maximumToeOffAngleProvider = new YoVariableDoubleProvider(namePrefix + "MaximumToeOffAngle", registry);
      maximumToeOffAngleProvider.set(footControlHelper.getWalkingControllerParameters().getMaximumToeOffAngle());

      this.edgeContactPoints = getEdgeContactPoints2d();
      desiredEdgeContactPositions = new ArrayList<FramePoint>();
      for (int i = 0; i < 2; i++)
      {
         desiredEdgeContactPositions.add(edgeContactPoints.get(i).toFramePoint());

         String index = String.valueOf(i);

         YoFrameVector contactPointPositionError = new YoFrameVector(namePrefix + "ToeOffContactPoint" + index + "PositionError", worldFrame, registry);
         contactPointPositionErrors.add(contactPointPositionError);

         YoFrameVector contactPointDesiredAcceleration = new YoFrameVector(namePrefix + "ToeOffContactPoint" + index + "DesiredAcceleration", worldFrame,
               registry);
         contactPointDesiredAccelerations.add(contactPointDesiredAcceleration);
      }
      singleToeContactPoint = new FramePoint2d(edgeContactPoints.get(0).getReferenceFrame());
      singleToeContactPoint.interpolate(edgeContactPoints.get(0), edgeContactPoints.get(1), 0.5);

      toeOffDesiredPitchAngle = new DoubleYoVariable(namePrefix + "ToeOffDesiredPitchAngle", registry);
      toeOffDesiredPitchVelocity = new DoubleYoVariable(namePrefix + "ToeOffDesiredPitchVelocity", registry);
      toeOffDesiredPitchAcceleration = new DoubleYoVariable(namePrefix + "ToeOffDesiredPitchAcceleration", registry);

      toeOffCurrentPitchAngle = new DoubleYoVariable(namePrefix + "ToeOffCurrentPitchAngle", registry);
      toeOffCurrentPitchVelocity = new DoubleYoVariable(namePrefix + "ToeOffCurrentPitchVelocity", registry);

      this.gains = gains;
      YoPositionPIDGains positionGains = gains.getPositionGains();
      proportionalGainMatrix = positionGains.createProportionalGainMatrix();
      derivativeGainMatrix = positionGains.createDerivativeGainMatrix();

      toeOffInitialAngle = new DoubleYoVariable(namePrefix + "ToeOffInitialAngle", registry);
      toeOffInitialVelocity = new DoubleYoVariable(namePrefix + "ToeOffInitialVelocity", registry);
      toeOffFinalAngle = new DoubleYoVariable(namePrefix + "ToeOffFinalAngle", registry);
      toeOffTrajectoryTime = new DoubleYoVariable(namePrefix + "ToeOffTrajectoryTime", registry);
      toeOffTrajectoryTime.set(Double.NaN);
      DoubleProvider initialPositionProvider = new YoVariableDoubleProvider(toeOffInitialAngle);
      DoubleProvider initialVelocityProvider = new YoVariableDoubleProvider(toeOffInitialVelocity);
      DoubleProvider finalPositionProvider = new YoVariableDoubleProvider(toeOffFinalAngle);
      DoubleProvider trajectoryTimeProvider = new YoVariableDoubleProvider(toeOffTrajectoryTime);
      toeOffTrajectory = new ThirdOrderPolynomialTrajectoryGenerator(namePrefix + "ToeOffTrajectory", initialPositionProvider, initialVelocityProvider,
            finalPositionProvider, trajectoryTimeProvider, registry);

      toeOffInitialAngle.set(Double.NaN);
      toeOffInitialVelocity.set(Double.NaN);
      toeOffTrajectoryTime.set(Double.NaN);

      toeOffDesiredPitchAngle.set(Double.NaN);
      toeOffDesiredPitchVelocity.set(Double.NaN);
      toeOffDesiredPitchAcceleration.set(Double.NaN);

      toeOffCurrentPitchAngle.set(Double.NaN);
      toeOffCurrentPitchVelocity.set(Double.NaN);

      edgeToRotateAbout = new FrameLineSegment2d(contactableFoot.getSoleFrame());
   }

   private int createRootToFootJacobian()
   {
      int jacobianId;
      RigidBody foot = contactableFoot.getRigidBody();
      ReferenceFrame jacobianFrame = rootBody.getBodyFixedFrame();

      if (USE_TOEOFF_TRAJECTORY)
      {
         jacobianId = momentumBasedController.getOrCreateGeometricJacobian(rootBody, foot, jacobianFrame);
      }
      else
      {
         ArrayList<InverseDynamicsJoint> jointPathWithoutKnee = new ArrayList<>(Arrays.asList(ScrewTools.createJointPath(rootBody, foot)));
         jointPathWithoutKnee.remove(kneeJoint);
         jacobianId = momentumBasedController.getOrCreateGeometricJacobian(jointPathWithoutKnee.toArray(new InverseDynamicsJoint[0]), jacobianFrame);
      }
      return jacobianId;
   }

   private final FrameVector axisOfRotation = new FrameVector();
   private final FrameVector2d axisOfRotation2d = new FrameVector2d();

   @Override
   public void doSpecificAction()
   {
      desiredOrientation.setToZero(contactableFoot.getFrameAfterParentJoint());
      desiredOrientation.changeFrame(worldFrame);
      desiredOrientation.getYawPitchRoll(tempYawPitchRoll);

      twistCalculator.packRelativeTwist(footTwist, rootBody, contactableFoot.getRigidBody());
      footTwist.changeFrame(contactableFoot.getFrameAfterParentJoint());

      toeOffCurrentPitchAngle.set(tempYawPitchRoll[1]);
      toeOffCurrentPitchVelocity.set(footTwist.getAngularPartY());

      desiredPosition.setToZero(contactableFoot.getFrameAfterParentJoint());
      desiredPosition.changeFrame(worldFrame);

      if (USE_TOEOFF_TRAJECTORY)
         computeDesiredsForTrajectoryBasedMotion();
      else
         computeDesiredsForFreeMotion();

      desiredOrientation.setYawPitchRoll(desiredYawToHold, toeOffDesiredPitchAngle.getDoubleValue(), desiredRollToHold);

      desiredLinearVelocity.setToZero(worldFrame);
      desiredAngularVelocity.setIncludingFrame(contactableFoot.getFrameAfterParentJoint(), 0.0, toeOffDesiredPitchVelocity.getDoubleValue(), 0.0);
      desiredAngularVelocity.changeFrame(worldFrame);

      desiredLinearAcceleration.setToZero(worldFrame);
      desiredAngularAcceleration.setIncludingFrame(contactableFoot.getFrameAfterParentJoint(), 0.0, toeOffDesiredPitchAcceleration.getDoubleValue(), 0.0);
      desiredAngularAcceleration.changeFrame(worldFrame);

      RigidBodySpatialAccelerationControlModule accelerationControlModule = footControlHelper.getAccelerationControlModule();
      accelerationControlModule.doPositionControl(desiredPosition, desiredOrientation, desiredLinearVelocity, desiredAngularVelocity,
            desiredLinearAcceleration, desiredAngularAcceleration, rootBody);
      accelerationControlModule.packAcceleration(footAcceleration);

      edgeToRotateAbout.getFrameVector(axisOfRotation2d);
      axisOfRotation.setXYIncludingFrame(axisOfRotation2d);
      axisOfRotation.normalize();
      axisOfRotation.changeFrame(footAcceleration.getExpressedInFrame());

      DenseMatrix64F selectionMatrix = footControlHelper.getSelectionMatrix();
      if (CONTROL_SINGLE_POINT)
      {
         // Need to control the whole orientation of the foot as only one contact point is position controlled.
         selectionMatrix.reshape(3, SpatialMotionVector.SIZE);
         footControlHelper.updateSelectionMatrixToHandleAnkleRollAndHipYawAlignment();
      }
      else
      {
         selectionMatrix.reshape(1, SpatialMotionVector.SIZE);
         // Hack to control the y axis of the foot in the root frame
         selectionMatrix.set(0, 0, axisOfRotation.getX());
         selectionMatrix.set(0, 1, axisOfRotation.getY());
         selectionMatrix.set(0, 2, axisOfRotation.getZ());
      }
      // Just to make sure we're not trying to do singularity escape
      // (the MotionConstraintHandler crashes when using point jacobian and singularity escape)
      footControlHelper.resetNullspaceMultipliers();
      footControlHelper.submitTaskspaceConstraint(footAcceleration);

      int numberOfContactPointsToControl = CONTROL_SINGLE_POINT ? 1 : edgeContactPoints.size();
      for (int i = 0; i < numberOfContactPointsToControl; i++)
      {
         FramePoint2d contactPoint2d = edgeContactPoints.get(i);
         contactPointPosition.setIncludingFrame(contactPoint2d.getReferenceFrame(), contactPoint2d.getX(), contactPoint2d.getY(), 0.0);

         contactPointPosition.changeFrame(footTwist.getBaseFrame());
         footTwist.changeFrame(footTwist.getBaseFrame());
         footTwist.packLinearVelocityOfPointFixedInBodyFrame(contactPointLinearVelocity, contactPointPosition);
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

      if (!USE_TOEOFF_TRAJECTORY)
         momentumBasedController.setOneDoFJointAcceleration(kneeJoint, 0.0);

      if (CONTROL_SINGLE_POINT)
         setupMidToeContactPoint();
   }

   private void computeDesiredsForFreeMotion()
   {
      boolean blockToMaximumPitch = tempYawPitchRoll[1] > maximumToeOffAngleProvider.getValue();

      if (blockToMaximumPitch)
      {
         toeOffDesiredPitchAngle.set(maximumToeOffAngleProvider.getValue());
         toeOffDesiredPitchVelocity.set(0.0);
      }
      else
      {
         toeOffDesiredPitchAngle.set(desiredOrientation.getPitch());
         toeOffDesiredPitchVelocity.set(footTwist.getAngularPartY());
      }

      toeOffDesiredPitchAcceleration.set(0.0);
   }

   private void computeDesiredsForTrajectoryBasedMotion()
   {
      if (toeOffTrajectoryTime.isNaN() || toeOffTrajectoryTime.getDoubleValue() < MIN_TRAJECTORY_TIME)
      {
         computeDesiredsForFreeMotion();
         return;
      }

      double time = MathTools.clipToMinMax(getTimeInCurrentState(), 0.0, toeOffTrajectoryTime.getDoubleValue());
      toeOffTrajectory.compute(time);

      boolean isToeOffAngleBehindTrajectory = tempYawPitchRoll[1] < toeOffTrajectory.getValue();
      if (isToeOffAngleBehindTrajectory)
         toeOffDesiredPitchAngle.set(toeOffTrajectory.getValue());
      else
         toeOffDesiredPitchAngle.set(desiredOrientation.getPitch());

      boolean isToeOffTooSlow = footTwist.getAngularPartY() < toeOffTrajectory.getVelocity();
      if (isToeOffTooSlow)
         toeOffDesiredPitchVelocity.set(toeOffTrajectory.getVelocity());
      else
         toeOffDesiredPitchVelocity.set(footTwist.getAngularPartY());

      toeOffDesiredPitchAcceleration.set(Math.max(0.0, toeOffTrajectory.getAcceleration()));
   }

   public void getDesireds(FrameOrientation desiredOrientationToPack, FrameVector desiredAngularVelocityToPack)
   {
      desiredOrientationToPack.setIncludingFrame(desiredOrientation);
      desiredAngularVelocityToPack.setIncludingFrame(desiredAngularVelocity);
   }

   private void setupMidToeContactPoint()
   {
      for (int i = 0; i < contactPoints.size(); i++)
      {
         contactPoints.get(i).setPosition(singleToeContactPoint);
      }
   }

   @Override
   public void doTransitionIntoAction()
   {
      super.doTransitionIntoAction();
      edgeToRotateAbout.set(edgeContactPoints.get(0), edgeContactPoints.get(1));
      for (int i = 0; i < 2; i++)
      {
         desiredEdgeContactPositions.get(i).setXYIncludingFrame(edgeContactPoints.get(i));
         desiredEdgeContactPositions.get(i).changeFrame(rootBody.getBodyFixedFrame());
      }

      desiredOrientation.setToZero(contactableFoot.getFrameAfterParentJoint());
      desiredOrientation.changeFrame(worldFrame);
      desiredYawToHold = desiredOrientation.getYaw();
      desiredRollToHold = desiredOrientation.getRoll();

      if (toeOffTrajectoryTime.getDoubleValue() > MIN_TRAJECTORY_TIME) // Returns false if the trajectory time is NaN
      {
         twistCalculator.packRelativeTwist(footTwist, rootBody, contactableFoot.getRigidBody());
         footTwist.changeFrame(contactableFoot.getFrameAfterParentJoint());

         toeOffInitialAngle.set(desiredOrientation.getPitch());
         toeOffInitialVelocity.set(footTwist.getAngularPartY());

         toeOffTrajectory.initialize();
      }
      else
      {
         toeOffTrajectoryTime.set(Double.NaN);
      }

      setOnToesFreeMotionGains();
      footControlHelper.saveContactPointsFromContactState();
   }

   @Override
   public void doTransitionOutOfAction()
   {
      super.doTransitionOutOfAction();

      toeOffInitialAngle.set(Double.NaN);
      toeOffInitialVelocity.set(Double.NaN);
      toeOffTrajectoryTime.set(Double.NaN);

      toeOffDesiredPitchAngle.set(Double.NaN);
      toeOffDesiredPitchVelocity.set(Double.NaN);
      toeOffDesiredPitchAcceleration.set(Double.NaN);

      toeOffCurrentPitchAngle.set(Double.NaN);
      toeOffCurrentPitchVelocity.set(Double.NaN);

      // TODO: kind of a hack
      footControlHelper.resetSelectionMatrix();
      footControlHelper.restoreFootContactPoints();
   }

   private void setOnToesFreeMotionGains()
   {
      // TODO Pretty hackish there clean that up
      // We use the RigidBodySpatialAccelerationControlModule only for the orientation, the position control part is done in this class
      footControlHelper.setGainsToZero();
      footControlHelper.setOrientationGains(gains.getOrientationGains());
   }

   protected List<FramePoint2d> getEdgeContactPoints2d()
   {
      FrameVector direction = new FrameVector(contactableFoot.getFrameAfterParentJoint(), 1.0, 0.0, 0.0);

      List<FramePoint> contactPoints = DesiredFootstepCalculatorTools.computeMaximumPointsInDirection(contactableFoot.getContactPointsCopy(), direction, 2);

      List<FramePoint2d> contactPoints2d = new ArrayList<FramePoint2d>(contactPoints.size());
      for (FramePoint contactPoint : contactPoints)
      {
         contactPoint.changeFrame(contactableFoot.getSoleFrame());
         contactPoints2d.add(contactPoint.toFramePoint2d());
      }

      return contactPoints2d;
   }

   public void setPredictedToeOffDuration(double predictedToeOffDuration)
   {
      toeOffTrajectoryTime.set(predictedToeOffDuration);
   }
}
