package us.ihmc.commonWalkingControlModules.controlModules.foot;

import java.util.ArrayList;
import java.util.List;

import org.ejml.ops.CommonOps;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoContactPoint;
import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.YoPlaneContactState;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.RigidBodySpatialAccelerationControlModule;
import us.ihmc.commonWalkingControlModules.controlModules.foot.FootControlModule.ConstraintType;
import us.ihmc.commonWalkingControlModules.desiredFootStep.DesiredFootstepCalculatorTools;
import us.ihmc.commonWalkingControlModules.momentumBasedController.MomentumBasedController;
import us.ihmc.robotSide.RobotSide;
import us.ihmc.utilities.humanoidRobot.partNames.LegJointName;
import us.ihmc.utilities.math.geometry.FramePoint;
import us.ihmc.utilities.math.geometry.FramePoint2d;
import us.ihmc.utilities.math.geometry.FrameVector;
import us.ihmc.utilities.math.geometry.FrameVector2d;
import us.ihmc.utilities.screwTheory.OneDoFJoint;
import us.ihmc.utilities.screwTheory.SpatialMotionVector;
import us.ihmc.utilities.screwTheory.Twist;

import com.yobotics.simulationconstructionset.BooleanYoVariable;
import com.yobotics.simulationconstructionset.DoubleYoVariable;
import com.yobotics.simulationconstructionset.YoVariableRegistry;
import com.yobotics.simulationconstructionset.util.GainCalculator;
import com.yobotics.simulationconstructionset.util.math.frames.YoFramePoint;
import com.yobotics.simulationconstructionset.util.math.frames.YoFrameVector;
import com.yobotics.simulationconstructionset.util.trajectory.YoVariableDoubleProvider;

public class OnToesState extends AbstractFootControlState
{
   private static final int NUMBER_OF_CONTACTS_POINTS_TO_ROTATE_ABOUT = 2;
   private static final boolean USE_TOEOFF_FOOT_HOLD_POSITION = true;

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

   private double toeOffKdx;// = GainCalculator.computeDerivativeGain(toeOffKpx.getDoubleValue(), toeOffZeta.getDoubleValue());      
   private double toeOffKdy;// = GainCalculator.computeDerivativeGain(toeOffKpy.getDoubleValue(), toeOffZeta.getDoubleValue());      
   private double toeOffKdz;// = GainCalculator.computeDerivativeGain(toeOffKpz.getDoubleValue(), toeOffZeta.getDoubleValue());

   protected double toeOffKpx, toeOffKpy, toeOffKpz, toeOffKpRoll, toeOffKpPitch, toeOffKpYaw, toeOffZeta;

   private final int rootToFootJacobianId;

   private final YoPlaneContactState contactState = momentumBasedController.getContactState(contactableBody);
   private final List<YoContactPoint> contactPoints = contactState.getContactPoints();
   private final List<FramePoint> originalContactPointPositions;

   private final FramePoint2d singleToeContactPoint;

   double alphaShrinkFootSizeForToeOff = 0.0;
   double kneeToeOffGain = 0.0;
   double kneeToeOffQDesired = 0.7;
   double kneeToeOffDamp = 0.0;
   double controlledKneeToeOffQdd;
   
   private final OneDoFJoint kneeJoint;

   public OnToesState(WalkingControllerParameters walkingControllerParameters, YoFramePoint yoDesiredPosition, YoFrameVector yoDesiredLinearVelocity,
         YoFrameVector yoDesiredLinearAcceleration, RigidBodySpatialAccelerationControlModule accelerationControlModule,
         MomentumBasedController momentumBasedController, ContactablePlaneBody contactableBody, int jacobianId, DoubleYoVariable nullspaceMultiplier,
         BooleanYoVariable jacobianDeterminantInRange, BooleanYoVariable doSingularityEscape, RobotSide robotSide, YoVariableRegistry registry)
   {
      super(ConstraintType.TOES, yoDesiredPosition, yoDesiredLinearVelocity, yoDesiredLinearAcceleration, accelerationControlModule, momentumBasedController,
            contactableBody, jacobianId, nullspaceMultiplier, jacobianDeterminantInRange, doSingularityEscape, robotSide, registry);

      rootToFootJacobianId = momentumBasedController.getOrCreateGeometricJacobian(rootBody, jacobian.getEndEffector(), rootBody.getBodyFixedFrame());

      String namePrefix = contactableBody.getName();
      maximumToeOffAngleProvider = new YoVariableDoubleProvider(namePrefix + "MaximumToeOffAngle", registry);
      maximumToeOffAngleProvider.set(walkingControllerParameters.getMaximumToeOffAngle());

      //      if (edgeContactPoints.size() != NUMBER_OF_CONTACTS_POINTS_TO_ROTATE_ABOUT)
      //         throw new RuntimeException("Number of contacts not handled for OnEdgeState: " + edgeContactPoints.size());

      this.edgeContactPoints = getEdgeContactPoints2d();
      desiredEdgeContactPositions = new ArrayList<FramePoint>();
      for (int i = 0; i < 2; i++)
         desiredEdgeContactPositions.add(edgeContactPoints.get(i).toFramePoint());

      singleToeContactPoint = new FramePoint2d(edgeContactPoints.get(0).getReferenceFrame());
      singleToeContactPoint.interpolate(edgeContactPoints.get(0), edgeContactPoints.get(1), 0.5);

      originalContactPointPositions = new ArrayList<FramePoint>(contactPoints.size());
      copyOriginalContactPointPositions();
      
      kneeJoint = momentumBasedController.getFullRobotModel().getLegJoint(robotSide, LegJointName.KNEE);
   }

   public void doSpecificAction()
   {
      desiredOrientation.setToZero(contactableBody.getBodyFrame());
      desiredOrientation.changeFrame(worldFrame);
      desiredOrientation.getYawPitchRoll(tempYawPitchRoll);

      momentumBasedController.getTwistCalculator().packRelativeTwist(footTwist, rootBody, contactableBody.getRigidBody());
      footTwist.changeFrame(contactableBody.getBodyFrame());

      boolean blockToMaximumPitch = tempYawPitchRoll[1] > maximumToeOffAngleProvider.getValue();

      double footPitch = 0.0, footPitchd = 0.0, footPitchdd = 0.0;

      desiredPosition.setToZero(contactableBody.getBodyFrame());
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
      desiredAngularVelocity.setIncludingFrame(contactableBody.getBodyFrame(), 0.0, footPitchd, 0.0);
      desiredAngularVelocity.changeFrame(worldFrame);

      desiredLinearAcceleration.setToZero(worldFrame);
      desiredAngularAcceleration.setIncludingFrame(contactableBody.getBodyFrame(), 0.0, footPitchdd, 0.0);
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
         proportionalPart.scale(toeOffKpx, toeOffKpy, toeOffKpz);

         derivativePart.setToZero(rootBody.getBodyFixedFrame());
         derivativePart.sub(contactPointLinearVelocity);
         derivativePart.scale(toeOffKdx, toeOffKdy, toeOffKdz);

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
      toeOffKdx = GainCalculator.computeDerivativeGain(toeOffKpx, toeOffZeta);
      toeOffKdy = GainCalculator.computeDerivativeGain(toeOffKpy, toeOffZeta);
      toeOffKdz = GainCalculator.computeDerivativeGain(toeOffKpz, toeOffZeta);

      edgeToRotateAbout.set(edgeContactPoints.get(0), edgeContactPoints.get(1));
      for (int i = 0; i < 2; i++)
      {
         desiredEdgeContactPositions.get(i).setIncludingFrame(edgeContactPoints.get(i).getReferenceFrame(), edgeContactPoints.get(i).getX(),
               edgeContactPoints.get(i).getY(), 0.0);
         desiredEdgeContactPositions.get(i).changeFrame(rootBody.getBodyFixedFrame());
      }

      desiredOrientation.setToZero(contactableBody.getBodyFrame());
      desiredOrientation.changeFrame(worldFrame);
      desiredYawToHold = desiredOrientation.getYaw();
      desiredRollToHold = desiredOrientation.getRoll();

      setOnToesFreeMotionGains();
   }

   public void doTransitionOutOfAction()
   {
      // TODO: kind of a hack
      selectionMatrix.reshape(SpatialMotionVector.SIZE, SpatialMotionVector.SIZE);
      CommonOps.setIdentity(selectionMatrix);

      resetContactPointPositions();

      controlledKneeToeOffQdd = 0.0;
   }

   private void setOnToesFreeMotionGains()
   {
      double kPosition = 0.0;
      double dPosition = GainCalculator.computeDerivativeGain(0.0, toeOffZeta);
      double dxOrientation = GainCalculator.computeDerivativeGain(toeOffKpRoll, toeOffZeta);
      double dyOrientation = GainCalculator.computeDerivativeGain(toeOffKpPitch, toeOffZeta);
      double dzOrientation = GainCalculator.computeDerivativeGain(toeOffKpYaw, toeOffZeta);

      accelerationControlModule.setPositionProportionalGains(kPosition, kPosition, kPosition);
      accelerationControlModule.setPositionDerivativeGains(dPosition, dPosition, dPosition);
      accelerationControlModule.setOrientationProportionalGains(toeOffKpRoll, toeOffKpPitch, toeOffKpYaw);
      accelerationControlModule.setOrientationDerivativeGains(dxOrientation, dyOrientation, dzOrientation);
   }

   public void setToeOffGains(double toeOffZeta, double toeOffKpx, double toeOffKpy, double toeOffKpz, double toeOffKpRoll, double toeOffKpPitch,
         double toeOffKpYaw)
   {
      this.toeOffZeta = toeOffZeta;
      this.toeOffKpx = toeOffKpx;
      this.toeOffKpy = toeOffKpy;
      this.toeOffKpz = toeOffKpz;
      this.toeOffKpRoll = toeOffKpRoll;
      this.toeOffKpPitch = toeOffKpPitch;
      this.toeOffKpYaw = toeOffKpYaw;
   }

   protected List<FramePoint2d> getEdgeContactPoints2d()
   {
      FrameVector direction = new FrameVector(contactableBody.getBodyFrame(), 1.0, 0.0, 0.0);

      List<FramePoint> contactPoints = DesiredFootstepCalculatorTools.computeMaximumPointsInDirection(contactableBody.getContactPointsCopy(), direction,
            NUMBER_OF_CONTACTS_POINTS_TO_ROTATE_ABOUT);

      List<FramePoint2d> contactPoints2d = new ArrayList<FramePoint2d>(contactPoints.size());
      for (FramePoint contactPoint : contactPoints)
      {
         contactPoint.changeFrame(contactableBody.getPlaneFrame());
         contactPoints2d.add(contactPoint.toFramePoint2d());
      }

      return contactPoints2d;
   }
}
