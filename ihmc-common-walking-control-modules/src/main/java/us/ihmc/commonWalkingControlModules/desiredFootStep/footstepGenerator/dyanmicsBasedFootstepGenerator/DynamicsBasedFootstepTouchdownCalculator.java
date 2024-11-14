package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.dyanmicsBasedFootstepGenerator;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint2D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFramePoint3D;
import us.ihmc.yoVariables.euclid.referenceFrame.YoFrameVector2D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class DynamicsBasedFootstepTouchdownCalculator
{
   private final MovingReferenceFrame centerOfMassControlZUpFrame;

   private final FrameVector3DReadOnly centerOfMassVelocity;

   private final FrameVector3DReadOnly centroidalAngularMomentum;

//   private final FootSoleBasedGroundPlaneEstimator groundPlaneEstimator;

   private final YoBoolean isInitialStanceValid;

   // This is the stance foot at the beginning of the contact sequence. This is only used if the controller is using the TD;LO controller.
   private final YoFramePoint3D stanceFootPositionAtBeginningOfState;

   // This is the desired touchdown position using the capture point, assuming no stance width and walking in place.
   private final YoFramePoint2D desiredALIPTouchdownPositionWithoutDS;
   private final YoFramePoint2D desiredALIPTouchdownPositionWithDS;

   // This is the desired touchdown position using the capture point, assuming no stance width and walking in place.
   private final YoFrameVector2D predictedALIPVelocityAtTouchdownWithoutDS;
   private final YoFrameVector2D predictedALIPVelocityAtTouchdownWithDS;
   private final YoFrameVector2D predictedVelocityAtTouchdown;

   // This is the desired touchdown position to be used by the robot
   private final YoFramePoint3D desiredTouchdownPosition;
   private final FramePoint2D desiredTouchdownPosition2D;

   // This is the desired lateral offset from the touchdown location returned by the TD;LO and capture point laws to achieve stable walking at the desired
   // nominal width.
   private final YoDouble touchdownPositionOffsetForDesiredStanceWidth;
   // This is the desired lateral offset in the X and Y directions of the touchdown position to achieve walking at a desired speed.
   private final YoFrameVector2D touchdownPositionOffsetForDesiredSpeed;

   private final SideDependentList<MovingReferenceFrame> soleFrames;
   private final Vector2DReadOnly desiredVelocityProvider;

   // Temp value used for calculation
   private final FrameVector2D tempVector = new FrameVector2D();

   //
   private final FrameVector2D angularMomentum = new FrameVector2D();

   // Gains for Capture Point controllers
   private final YoDouble capturePointTimeConstantX;
   private final YoDouble capturePointTimeConstantY;
   private final YoDouble capturePointTimeConstantXWithoutDS;
   private final YoDouble capturePointTimeConstantYWithoutDS;

   // Poles for the Capture Point Controllers
   private final YoDouble capturePointPole;

   // Natural frequency of system
   // TODO make sure these are updating
   private final YoDouble omegaX;
   private final YoDouble omegaY;

   private final YoDouble desiredCoMHeight;
   private final YoDouble swingDuration;
   private final YoDouble doubleSupportFraction;
   private final YoDouble stanceWidth;

   private final YoDouble timeToReachGoal;
   private final RobotSide robotSide;

   private final double mass;
   private final double gravity;

   public DynamicsBasedFootstepTouchdownCalculator(RobotSide robotSide,
                                                   MovingReferenceFrame centerOfMassControlZUpFrame,
                                                   FrameVector3DReadOnly centerOfMassVelocity,
                                                   FrameVector3DReadOnly centroidalAngularMomentum,
                                                   FullHumanoidRobotModel robotModel,
                                                   DynamicsBasedFootstepParameters parameters,
                                                   double gravity,
                                                   Vector2DReadOnly desiredVelocityProvider,
                                                   YoRegistry parentRegistry)
   {
      this.robotSide = robotSide;
      this.centerOfMassControlZUpFrame = centerOfMassControlZUpFrame;
      this.centerOfMassVelocity = centerOfMassVelocity;
      this.centroidalAngularMomentum = centroidalAngularMomentum;
//      this.groundPlaneEstimator = groundPlaneEstimator;
      this.desiredVelocityProvider = desiredVelocityProvider;
      soleFrames = robotModel.getSoleFrames();
      mass = robotModel.getTotalMass();
      this.gravity = gravity;

      YoRegistry registry = new YoRegistry(robotSide.getCamelCaseName() + getClass().getSimpleName());

      String prefix = robotSide.getCamelCaseName() + "TouchdownCalculator";

      timeToReachGoal = new YoDouble(prefix + "TimeToReachGoal", registry);

      isInitialStanceValid = new YoBoolean(prefix + "IsInitialStancePositionValid", registry);
      stanceFootPositionAtBeginningOfState = new YoFramePoint3D(prefix + "StanceFootPositionAtBeginningOfState",
                                                                centerOfMassControlZUpFrame,
                                                                registry);

      // Predicted CoM velocity as computed by each different touchdown calculator
      predictedALIPVelocityAtTouchdownWithoutDS = new YoFrameVector2D(prefix + "PredictedALIPVelocityAtTouchdownWithoutDS", centerOfMassControlZUpFrame, registry);
      predictedALIPVelocityAtTouchdownWithDS = new YoFrameVector2D(prefix + "PredictedALIPVelocityAtTouchdownWithDS", centerOfMassControlZUpFrame, registry);

      // Desired touchdown position as computed by each different touchdown calculator
      desiredALIPTouchdownPositionWithoutDS = new YoFramePoint2D(prefix + "DesiredALIPTouchdownPositionWithoutDS", centerOfMassControlZUpFrame, registry);
      desiredALIPTouchdownPositionWithDS = new YoFramePoint2D(prefix + "DesiredALIPTouchdownPositionWithDS", centerOfMassControlZUpFrame, registry);

      // Final touchdown position and predicted velocity. These are populated by the touchdown calculator currently in use
      desiredTouchdownPosition = new YoFramePoint3D(prefix + "DesiredTouchdownPosition", centerOfMassControlZUpFrame, registry);
      desiredTouchdownPosition2D = new FramePoint2D(centerOfMassControlZUpFrame);
      predictedVelocityAtTouchdown = new YoFrameVector2D(prefix + "PredictedVelocityAtTouchdown", centerOfMassControlZUpFrame, registry);

      // Step position offsets for desired walking speed and stance width
      touchdownPositionOffsetForDesiredStanceWidth = new YoDouble(prefix + "TouchdownPositionOffsetForDesiredStanceWidth", registry);
      touchdownPositionOffsetForDesiredSpeed = new YoFrameVector2D(prefix + "TouchdownPositionOffsetForDesiredSpeed",
                                                                   centerOfMassControlZUpFrame,
                                                                   registry);

      capturePointTimeConstantX = new YoDouble(prefix + "CapturePointTimeConstantX", registry);
      capturePointTimeConstantY = new YoDouble(prefix + "CapturePointTimeConstantY", registry);
      capturePointTimeConstantXWithoutDS = new YoDouble(prefix + "CapturePointTimeConstantXWithoutDS", registry);
      capturePointTimeConstantYWithoutDS = new YoDouble(prefix + "CapturePointTimeConstantYWithoutDS", registry);

      omegaX = parameters.getOmega(robotSide);
      omegaY = parameters.getOmega(robotSide);

      capturePointPole = parameters.getPole(robotSide);

      desiredCoMHeight = parameters.getDesiredCoMHeight(robotSide);
      swingDuration = parameters.getSwingDuration(robotSide);
      doubleSupportFraction = parameters.getDoubleSupportFraction(robotSide);
      stanceWidth = parameters.getStanceWidth(robotSide);

      omegaX.addListener(v -> computeCapturePointTimeConstant());
      omegaY.addListener(v -> computeCapturePointTimeConstant());
      capturePointPole.addListener(v -> computeCapturePointTimeConstant());

      desiredCoMHeight.addListener(v -> computeCapturePointTimeConstant());
      swingDuration.addListener(v -> computeCapturePointTimeConstant());
      doubleSupportFraction.addListener(v -> computeCapturePointTimeConstant());

      computeCapturePointTimeConstant();

      parentRegistry.addChild(registry);
   }

   public void reset()
   {
      desiredALIPTouchdownPositionWithoutDS.setToNaN();
      desiredALIPTouchdownPositionWithDS.setToNaN();
      desiredTouchdownPosition.setToNaN();

      isInitialStanceValid.set(false);
   }

   public void initializeForContactChange(RobotSide supportSide)
   {
      isInitialStanceValid.set(true);
      stanceFootPositionAtBeginningOfState.setFromReferenceFrame(soleFrames.get(supportSide));
   }

   public void computeDesiredTouchdownPosition(RobotSide supportSide, double timeToReachGoal, FramePoint2DReadOnly pendulumBase, FramePoint2DReadOnly netPendulumBase, boolean isInDoubleSupport)
   {
      if (pendulumBase != null)
         pendulumBase.checkReferenceFrameMatch(centerOfMassControlZUpFrame);

      this.timeToReachGoal.set(timeToReachGoal);

      // compute all the touchdown positions and their offset values
      computeDesiredALIPTouchdownPositionWithDS(timeToReachGoal, pendulumBase, netPendulumBase, isInDoubleSupport);

      desiredTouchdownPosition2D.set(desiredALIPTouchdownPositionWithDS);
      predictedVelocityAtTouchdown.set(predictedALIPVelocityAtTouchdownWithDS);

      // compute offsets for desired walking speed and stance width
      computeDesiredTouchdownOffsetForVelocity(supportSide);
      computeDesiredTouchdownOffsetForStanceWidth(supportSide);

      // offset the touchdown position based on the desired velocity
      desiredTouchdownPosition2D.add(touchdownPositionOffsetForDesiredSpeed.getX(), touchdownPositionOffsetForDesiredSpeed.getY());

      // offset the touchdown position based on the stance width
      desiredTouchdownPosition2D.addY(touchdownPositionOffsetForDesiredStanceWidth.getValue());

      // add extra offsets for fixing drifting
//      desiredTouchdownPosition2D.addX(fastWalkingParameters.getFixedForwardOffset());
//      desiredTouchdownPosition2D.addY(supportSide.negateIfLeftSide(fastWalkingParameters.getFixedWidthOffset()));

      // determine touchdown z position from x position, y position, and ground plane
      desiredTouchdownPosition.setMatchingFrame(desiredTouchdownPosition2D, 0.0);
   }

   public FramePoint3DReadOnly getDesiredTouchdownPosition()
   {
      return desiredTouchdownPosition;
   }

   public FramePoint2DReadOnly getDesiredTouchdownPosition2D()
   {
      return desiredTouchdownPosition2D;
   }

   public FrameVector2DReadOnly getPredictedVelocityAtTouchdown()
   {
      return predictedVelocityAtTouchdown;
   }

   private void computeDesiredALIPTouchdownPositionWithDS(double timeToReachGoal, FramePoint2DReadOnly pendulumBase, FramePoint2DReadOnly netPendulumBase, boolean isInDoubleSupport)
   {

      // Get CoM velocity and change frame to CoM control frame
      tempVector.setIncludingFrame(centerOfMassVelocity);
      tempVector.changeFrameAndProjectToXYPlane(centerOfMassControlZUpFrame);

      // Get CoM angular momentum and change frame to CoM control frame
      angularMomentum.setIncludingFrame(centroidalAngularMomentum);
      angularMomentum.changeFrame(centerOfMassControlZUpFrame);

      // The capture point touchdown position is just the capture point. Which is the scaled velocity based on the time constant
      if (pendulumBase != null && netPendulumBase != null && Double.isFinite(timeToReachGoal))
      {
         double heightX = gravity / (omegaX.getDoubleValue() * omegaX.getDoubleValue());
         double heightY = gravity / (omegaY.getDoubleValue() * omegaY.getDoubleValue());
         double mhX = mass * heightX;
         double mhY = mass * heightY;
         double doubleSupportDuration = swingDuration.getDoubleValue() * doubleSupportFraction.getDoubleValue();
         double B2X;
         double B2Y;
         double C2X;
         double C2Y;

         // Prediction horizons to project/predict state at end of single and double support
         double singleSupportHorizon;
         double doubleSupportHorizon;
         if (isInDoubleSupport)
         {
            singleSupportHorizon = 0.0;
            doubleSupportHorizon = timeToReachGoal;
         }
         else
         {
            singleSupportHorizon = timeToReachGoal - doubleSupportDuration;
            doubleSupportHorizon = doubleSupportDuration;
         }

         double coshXSingleSupport = Math.cosh(omegaX.getDoubleValue() * singleSupportHorizon);
         double sinhXSingleSupport = Math.sinh(omegaX.getDoubleValue() * singleSupportHorizon);
         double coshYSingleSupport = Math.cosh(omegaY.getDoubleValue() * singleSupportHorizon);
         double sinhYSingleSupport = Math.sinh(omegaY.getDoubleValue() * singleSupportHorizon);

         double coshXDoubleSupport = Math.cosh(omegaX.getDoubleValue() * doubleSupportHorizon);
         double sinhXDoubleSupport = Math.sinh(omegaX.getDoubleValue() * doubleSupportHorizon);
         double coshYDoubleSupport = Math.cosh(omegaY.getDoubleValue() * doubleSupportHorizon);
         double sinhYDoubleSupport = Math.sinh(omegaY.getDoubleValue() * doubleSupportHorizon);

         // Current state variables
         double comPositionX = -pendulumBase.getX();
         double comPositionY = -pendulumBase.getY();
         double comVelocityX = tempVector.getX();
         double comVelocityY = tempVector.getY();
         double Ly = mhX *  comVelocityX + angularMomentum.getY();
         double Lx = -mhY *  comVelocityY + angularMomentum.getX();

         // State variables at end of single support
         double comXAtEndOfSingleSupport = comPositionX * coshXSingleSupport + Ly * sinhXSingleSupport / (omegaX.getDoubleValue() * mhX);
         double LyAtEndOfSingleSupport = comPositionX * mhX * omegaX.getDoubleValue() * sinhXSingleSupport + Ly * coshXSingleSupport;

         double comYAtEndOfSingleSupport = comPositionY * coshYSingleSupport - Lx * sinhYSingleSupport / (omegaY.getDoubleValue() * mhY);
         double LxAtEndOfSingleSupport = -comPositionY * mhY * omegaY.getDoubleValue() * sinhYSingleSupport + Lx * coshYSingleSupport;

         // Arbitrary constant resulting from double support dynamics
         if (doubleSupportDuration > 0)
         {
            B2X = mhX * (1-coshXDoubleSupport)/doubleSupportDuration;
            B2Y = -mhY * (1-coshYDoubleSupport)/doubleSupportDuration;
            C2X = -mhX * omegaX.getDoubleValue()*sinhXDoubleSupport;
            C2Y = mhY * omegaY.getDoubleValue()*sinhYDoubleSupport;
         }
         else
         {
            B2X = 0.0;
            B2Y = 0.0;
            C2X = 0.0;
            C2Y = 0.0;
         }

         // More arbitrary constants
         double DX = 1.0 / (1.0 - B2X * capturePointTimeConstantX.getDoubleValue() / mhX);
         double DY = 1.0 / (1.0 + B2Y * capturePointTimeConstantY.getDoubleValue() / mhY);
         double EX = C2X * netPendulumBase.getX() / mhX;
         double EY = -C2Y * netPendulumBase.getY() / mhY;

         double xDotAtEndOfDoubleSupport = DX * (comXAtEndOfSingleSupport * omegaX.getDoubleValue() * sinhXDoubleSupport + LyAtEndOfSingleSupport * coshXDoubleSupport / mhX + EX);
         double yDotAtEndOfDoubleSupport = DY * (comYAtEndOfSingleSupport * omegaY.getDoubleValue() * sinhYDoubleSupport - LxAtEndOfSingleSupport * coshYDoubleSupport / mhY + EY);

         predictedALIPVelocityAtTouchdownWithDS.setX(xDotAtEndOfDoubleSupport);
         predictedALIPVelocityAtTouchdownWithDS.setY(yDotAtEndOfDoubleSupport);
      }
      else
      {
         double heightX = gravity / (omegaX.getDoubleValue() * omegaX.getDoubleValue());
         double heightY = gravity / (omegaY.getDoubleValue() * omegaY.getDoubleValue());
         double mhX = mass * heightX;
         double mhY = mass * heightY;

         predictedALIPVelocityAtTouchdownWithDS.set(tempVector);
         predictedALIPVelocityAtTouchdownWithDS.addX(angularMomentum.getY() / mhX);
         predictedALIPVelocityAtTouchdownWithDS.addY(-angularMomentum.getX() / mhY);
      }

      desiredALIPTouchdownPositionWithDS.set(capturePointTimeConstantX.getDoubleValue() * predictedALIPVelocityAtTouchdownWithDS.getX(),
                                             capturePointTimeConstantY.getDoubleValue() * predictedALIPVelocityAtTouchdownWithDS.getY());
   }

   private void computeCapturePointTimeConstant()
   {
      double omegaX = this.omegaX.getDoubleValue();
      double omegaY = this.omegaY.getDoubleValue();
      double heightX = gravity / (omegaX * omegaX);
      double heightY = gravity / (omegaY * omegaY);
      double m = mass;
      double lambda = capturePointPole.getDoubleValue();
      double singleSupportDuration = swingDuration.getDoubleValue();
      double doubleSupportDuration = singleSupportDuration * doubleSupportFraction.getDoubleValue();

      double coshXDS = Math.cosh(omegaX * doubleSupportDuration);
      double coshYDS = Math.cosh(omegaY * doubleSupportDuration);
      double sinhXDS = Math.sinh(omegaX * doubleSupportDuration);
      double sinhYDS = Math.sinh(omegaY * doubleSupportDuration);
      double coshXSS = Math.cosh(omegaX * singleSupportDuration);
      double coshYSS = Math.cosh(omegaY * singleSupportDuration);
      double sinhXSS = Math.sinh(omegaX * singleSupportDuration);
      double sinhYSS = Math.sinh(omegaY * singleSupportDuration);

      double timeConstantXWithDS;
      double timeConstantYWithDS;
      double timeConstantXNoDS = (coshXSS - lambda) / (omegaX * sinhXSS);
      double timeConstantYNoDS = (coshYSS - lambda) / (omegaY * sinhYSS);
      capturePointTimeConstantXWithoutDS.set(timeConstantXNoDS);
      capturePointTimeConstantYWithoutDS.set(timeConstantYNoDS);

      if (doubleSupportDuration > 0)
      {
         timeConstantXWithDS = (doubleSupportDuration * (coshXDS * coshXSS - lambda + sinhXDS * sinhXSS)) /
                               (coshXDS * lambda - lambda + doubleSupportDuration * coshXDS * omegaX * sinhXSS + doubleSupportDuration * coshXSS * omegaX * sinhXDS);

         timeConstantYWithDS = (doubleSupportDuration * (coshYDS * coshYSS - lambda + sinhYDS * sinhYSS)) /
                               (coshYDS * lambda - lambda + doubleSupportDuration * coshYDS * omegaY * sinhYSS + doubleSupportDuration * coshYSS * omegaY * sinhYDS);

         capturePointTimeConstantX.set(timeConstantXWithDS);
         capturePointTimeConstantY.set(timeConstantYWithDS);
      }
      else
      {
         capturePointTimeConstantX.set(timeConstantXNoDS);
         capturePointTimeConstantY.set(timeConstantYNoDS);
      }
   }

   private void computeDesiredTouchdownOffsetForVelocity(RobotSide supportSide)
   {
      // this is the desired offset distance to make the robot stably walk at the desired velocity
      computeDesiredTouchdownOffsetForVelocity(swingDuration.getDoubleValue(),
                                               doubleSupportFraction.getDoubleValue() * swingDuration.getDoubleValue(),
                                               supportSide,
                                               1/capturePointTimeConstantX.getDoubleValue(),
                                               1/capturePointTimeConstantY.getDoubleValue(),
                                               desiredVelocityProvider,
                                               touchdownPositionOffsetForDesiredSpeed);
   }

   public static void computeDesiredTouchdownOffsetForVelocity(double swingDuration,
                                                               double doubleSupportDuration,
                                                               RobotSide supportSide,
                                                               double omegaX,
                                                               double omegaY,
                                                               Vector2DReadOnly desiredVelocity,
                                                               FixedFrameVector2DBasics touchdownPositionOffsetToPack)
   {
      // this is the desired offset distance to make the robot stably walk at the desired velocity

      double forwardOffset = computeForwardTouchdownOffsetForVelocity(swingDuration, doubleSupportDuration, omegaX, desiredVelocity.getX());

      double lateralOffset = computeLateralTouchdownOffsetForVelocity(swingDuration, doubleSupportDuration, supportSide, omegaY, desiredVelocity.getY());

      touchdownPositionOffsetToPack.set(forwardOffset, lateralOffset);
   }

   private static double computeForwardTouchdownOffsetForVelocity(double swingDuration, double doubleSupportDuration, double omega, double desiredVelocity)
   {
      double stepDuration = doubleSupportDuration + swingDuration;
      double exponential = Math.exp(omega * stepDuration);
      double forwardMultiplier = 1.0;
      if (doubleSupportDuration > 0.0)
      {
         forwardMultiplier += (Math.exp(omega * doubleSupportDuration) - 1.0) / (omega * doubleSupportDuration) - 1.0;
      }

      return -forwardMultiplier * desiredVelocity * stepDuration / (exponential - 1.0);
   }

   private static double computeLateralTouchdownOffsetForVelocity(double swingDuration,
                                                                  double doubleSupportDuration,
                                                                  RobotSide supportSide,
                                                                  double omega,
                                                                  double desiredVelocity)
   {
      double stepDuration = doubleSupportDuration + swingDuration;
      double exponential = Math.exp(omega * stepDuration);

      double lateralOffset = 0.0;
      if (desiredVelocity > 0.0 && supportSide == RobotSide.LEFT)
      {
         lateralOffset = -(desiredVelocity * 2.0 * swingDuration) / (exponential - 1.0);
      }
      else if (desiredVelocity < 0.0 && supportSide == RobotSide.RIGHT)
      {
         lateralOffset = -(desiredVelocity * 2.0 * swingDuration) / (exponential - 1.0);
      }

      return lateralOffset;
   }

   private void computeDesiredTouchdownOffsetForStanceWidth(RobotSide supportSide)
   {
      // this is the desired offset distance to make the robot stably walk in place.
      touchdownPositionOffsetForDesiredStanceWidth.set(computeDesiredTouchdownOffsetForStanceWidth(swingDuration.getDoubleValue(),
                                                                                                   doubleSupportFraction.getDoubleValue() * swingDuration.getDoubleValue(),
                                                                                                   stanceWidth.getDoubleValue(),
                                                                                                   supportSide,
                                                                                                   1/capturePointTimeConstantY.getDoubleValue()));
   }

   public static double computeDesiredTouchdownOffsetForStanceWidth(double swingDuration,
                                                                    double doubleSupportDuration,
                                                                    double stanceWidth,
                                                                    RobotSide supportSide,
                                                                    double omega)
   {
      double widthMultiplier = 1.0;
      if (doubleSupportDuration > 0.0)
      {
         widthMultiplier += (Math.exp(omega * doubleSupportDuration) - 1.0) / (omega * doubleSupportDuration) - 1.0;
      }
      // this is the desired offset distance to make the robot stably walk in place at our desired step width
      double stepWidthOffset = widthMultiplier * stanceWidth / (1.0 + Math.exp(omega * (swingDuration + doubleSupportDuration)));
      return supportSide.negateIfLeftSide(stepWidthOffset);
   }
}
