package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.quicksterFootstepProvider;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.robotics.robotSide.RobotSide;

public class ALIPCalculatorTools
{
   private final static double GRAVITY = -9.81;

   private final FramePose3D tempFutureCoMPose = new FramePose3D();
   private final FrameVector3D tempFutureCoMVelocity = new FrameVector3D();
   private final FrameVector3D tempFutureContactPointAngularMomentum = new FrameVector3D();
   private final PoseReferenceFrame tempFutureControlFrame = new PoseReferenceFrame("futureControlFrameALIPCalculator", ReferenceFrame.getWorldFrame());

   private final FramePose3D tempCurrentCoMPose = new FramePose3D();
   private final FrameVector3D tempCurrentContactPointAngularMomentum = new FrameVector3D();
   private final FramePoint3D tempCurrentStanceFootPosition = new FramePoint3D();

   /**
    * Using the ALIP reduced-order model, this method computes a predicted future state given an initial
    * state, and the time delta between the initial state and the future state.
    */
   public void computeFutureStateUsingALIP(FramePose3DReadOnly currentCoMPose,
                                           FrameVector3DReadOnly currentContactPointAngularMomentum,
                                           FramePoint3DReadOnly currentStanceFootPosition,
                                           FramePose3D futureCoMPoseToPack,
                                           FrameVector3D futureContactPointAngularMomentumToPack,
                                           ReferenceFrame controlFrame,
                                           double horizonDuration,
                                           double pendulumMass,
                                           double pendulumHeight,
                                           double desiredTurningVelocity,
                                           double updateDt)
   {
      tempCurrentCoMPose.setMatchingFrame(currentCoMPose);
      tempCurrentContactPointAngularMomentum.setMatchingFrame(currentContactPointAngularMomentum);
      tempCurrentStanceFootPosition.setMatchingFrame(currentStanceFootPosition);

      tempCurrentCoMPose.changeFrame(controlFrame);
      tempCurrentContactPointAngularMomentum.changeFrame(controlFrame);
      tempCurrentStanceFootPosition.changeFrame(controlFrame);

      tempCurrentCoMPose.setX(-tempCurrentStanceFootPosition.getX());
      tempCurrentCoMPose.setY(-tempCurrentStanceFootPosition.getY());
      tempCurrentCoMPose.setZ(-tempCurrentStanceFootPosition.getZ());

      double omega = computeNaturalFrequency(pendulumHeight);

      // Current position and angular momentum
      double x0 = tempCurrentCoMPose.getX();
      double Ly0 = tempCurrentContactPointAngularMomentum.getY();

      double y0 = tempCurrentCoMPose.getY();
      double Lx0 = tempCurrentContactPointAngularMomentum.getX();

      // Final position and angular momentum
      double xf = x0 * Math.cosh(omega * horizonDuration) + Ly0 * Math.sinh(omega * horizonDuration) / (pendulumMass * pendulumHeight * omega);
      double Lyf = x0 * pendulumMass * pendulumHeight * omega * Math.sinh(omega * horizonDuration) + Ly0 * Math.cosh(omega * horizonDuration);

      double yf = y0 * Math.cosh(omega * horizonDuration) - Lx0 * Math.sinh(omega * horizonDuration) / (pendulumMass * pendulumHeight * omega);
      double Lxf = -y0 * pendulumMass * pendulumHeight * omega * Math.sinh(omega * horizonDuration) + Lx0 * Math.cosh(omega * horizonDuration);

      ReferenceFrame originalPositionFrame = futureCoMPoseToPack.getReferenceFrame();
      ReferenceFrame originalMomentumFrame = futureContactPointAngularMomentumToPack.getReferenceFrame();

      futureCoMPoseToPack.setIncludingFrame(tempCurrentCoMPose);
      futureContactPointAngularMomentumToPack.changeFrame(controlFrame);

      futureCoMPoseToPack.setX(tempCurrentStanceFootPosition.getX());
      futureCoMPoseToPack.setY(tempCurrentStanceFootPosition.getY());
      futureCoMPoseToPack.setZ(tempCurrentCoMPose.getZ());

      int intervals = (int) Math.round(horizonDuration / updateDt);
      intervals = MathTools.clamp(intervals, 1, Integer.MAX_VALUE);
      horizonDuration = MathTools.clamp(horizonDuration, updateDt, Double.POSITIVE_INFINITY);

      for (int i = 0; i < intervals; i++)
      {
         futureCoMPoseToPack.appendYawRotation(desiredTurningVelocity * updateDt);
         futureCoMPoseToPack.getPosition().addX(xf * updateDt / horizonDuration);
         futureCoMPoseToPack.getPosition().addY(yf * updateDt / horizonDuration);
      }

      futureContactPointAngularMomentumToPack.setX(Lxf);
      futureContactPointAngularMomentumToPack.setY(Lyf);

      futureCoMPoseToPack.changeFrame(originalPositionFrame);
      futureContactPointAngularMomentumToPack.changeFrame(originalMomentumFrame);
   }

   /**
    * Using the ALIP reduced-order model and a desired steady-state CoM velocity, this method calculates the
    * footstep position needed to achieve the desired steady-state CoM velocity by the end of the following step
    */
   public void computeTouchdownPositionRegular(FramePose3DReadOnly currentCoMPose,
                                               FrameVector3DReadOnly currentContactPointAngularMomentum,
                                               FramePoint3DReadOnly currentStanceFootPosition,
                                               FramePoint2DBasics touchdownPositionToPack,
                                               RobotSide swingSide,
                                               ReferenceFrame controlFrame,
                                               double desiredVelocityX,
                                               double desiredVelocityY,
                                               double desiredTurningVelocity,
                                               double desiredStanceWidth,
                                               double timeRemainingInCurrentStep,
                                               double stepDuration,
                                               double pendulumMass,
                                               double pendulumHeight,
                                               boolean useFutureCoM,
                                               double updateDt)
   {
      double omega = computeNaturalFrequency(pendulumHeight);

      double LyDesired = pendulumMass * desiredVelocityX * pendulumHeight;
      double LxDesired = computeDesiredAngularMomentumForStanceWidth(swingSide, desiredStanceWidth, pendulumMass, pendulumHeight, stepDuration)
                         - pendulumMass * desiredVelocityY * pendulumHeight;

      computeFutureStateUsingALIP(currentCoMPose,
                                  currentContactPointAngularMomentum,
                                  currentStanceFootPosition,
                                  tempFutureCoMPose,
                                  tempFutureContactPointAngularMomentum,
                                  controlFrame,
                                  timeRemainingInCurrentStep,
                                  pendulumMass,
                                  pendulumHeight,
                                  desiredTurningVelocity,
                                  updateDt);

      double LyEndOfCurrentStep = tempFutureContactPointAngularMomentum.getY();
      double LxEndOfCurrentStep = tempFutureContactPointAngularMomentum.getX();

      double desiredFootstepPositionX =
            (LyDesired - Math.cosh(omega * stepDuration) * LyEndOfCurrentStep) / (pendulumMass * pendulumHeight * omega * Math.sinh(omega * stepDuration));
      double desiredFootstepPositionY =
            (Math.cosh(omega * stepDuration) * LxEndOfCurrentStep - LxDesired) / (pendulumMass * pendulumHeight * omega * Math.sinh(omega * stepDuration));

      ReferenceFrame controlFrameToUse;

      if (useFutureCoM)
         controlFrameToUse = tempFutureControlFrame;
      else
         controlFrameToUse = controlFrame;

      ReferenceFrame originalFrame = touchdownPositionToPack.getReferenceFrame();
      touchdownPositionToPack.setToZero(controlFrameToUse);
      touchdownPositionToPack.set(-desiredFootstepPositionX, -desiredFootstepPositionY);
      touchdownPositionToPack.changeFrameAndProjectToXYPlane(originalFrame);
   }

   /**
    * Using the ALIP reduced-order model, this method computes the predicted end-of-step state and uses the end-of-step CoM
    * velocity to calculate a desired touchdown position for steady state walking using the Raibert Heuristic footstep
    * control strategy. In this case the Raibert Heuristic gain is computed via pole placement and the desired closed loop
    * pole of our linear inverted pendulum dynamical system
    */
   public void computeTouchdownPositionUsingRaibertHeuristicAndPolePlacement(FramePose3DReadOnly currentCoMPose,
                                                                             FrameVector3DReadOnly currentContactPointAngularMomentum,
                                                                             FramePoint3DReadOnly currentStanceFootPosition,
                                                                             FramePoint2DBasics touchdownPositionToPack,
                                                                             RobotSide swingSide,
                                                                             ReferenceFrame controlFrame,
                                                                             double desiredVelocityX,
                                                                             double desiredVelocityY,
                                                                             double desiredTurningVelocity,
                                                                             double desiredStanceWidth,
                                                                             double timeRemainingInCurrentStep,
                                                                             double stepDuration,
                                                                             double doubleSupportDuration,
                                                                             double pendulumMass,
                                                                             double pendulumHeight,
                                                                             double pole,
                                                                             boolean useFutureCoM,
                                                                             double updateDt)
   {
      double omega = computeNaturalFrequency(pendulumHeight);

      computeFutureStateUsingALIP(currentCoMPose,
                                  currentContactPointAngularMomentum,
                                  currentStanceFootPosition,
                                  tempFutureCoMPose,
                                  tempFutureContactPointAngularMomentum,
                                  controlFrame,
                                  timeRemainingInCurrentStep,
                                  pendulumMass,
                                  pendulumHeight,
                                  desiredTurningVelocity,
                                  updateDt);

      tempFutureCoMPose.changeFrame(tempFutureControlFrame.getParent());
      tempFutureControlFrame.setPoseAndUpdate(tempFutureCoMPose);

      tempFutureCoMVelocity.changeFrame(tempFutureContactPointAngularMomentum.getReferenceFrame());
      tempFutureCoMVelocity.setX(tempFutureContactPointAngularMomentum.getY() / (pendulumMass * pendulumHeight));
      tempFutureCoMVelocity.setY(-tempFutureContactPointAngularMomentum.getX() / (pendulumMass * pendulumHeight));

      ReferenceFrame controlFrameToUse;

      if (useFutureCoM)
         controlFrameToUse = tempFutureControlFrame;
      else
         controlFrameToUse = controlFrame;

      tempFutureCoMVelocity.changeFrame(controlFrameToUse);
      double timeConstant = calculateTimeConstantUsingPolePlacement(pole, stepDuration, omega);
      double swingDuration = stepDuration - doubleSupportDuration;

      computeTouchdownPositionUsingRaibertHeuristic(timeConstant, tempFutureCoMVelocity, controlFrameToUse, touchdownPositionToPack);

      ReferenceFrame originalFrame = touchdownPositionToPack.getReferenceFrame();
      touchdownPositionToPack.changeFrameAndProjectToXYPlane(controlFrameToUse);
      touchdownPositionToPack.addX(computeForwardTouchdownOffsetForVelocity(swingDuration, doubleSupportDuration, 1 / timeConstant, desiredVelocityX));
      touchdownPositionToPack.addY(computeLateralTouchdownOffsetForVelocity(swingDuration,
                                                                            doubleSupportDuration,
                                                                            swingSide.getOppositeSide(),
                                                                            1 / timeConstant,
                                                                            desiredVelocityY));
      touchdownPositionToPack.addY(computeDesiredTouchdownOffsetForStanceWidth(swingDuration,
                                                                               doubleSupportDuration,
                                                                               desiredStanceWidth,
                                                                               swingSide.getOppositeSide(),
                                                                               1 / timeConstant));
      touchdownPositionToPack.changeFrameAndProjectToXYPlane(originalFrame);
   }

   /**
    * Given a predicted end-of-step CoM velocity, this method computes a desired touchdown position to achieve
    * steady state walking using the Raibert Heuristic footstep control strategy. In this case the Raibert
    * Heuristic gain is computed via pole placement and the desired closed loop pole of our linear inverted
    * pendulum dynamical system
    */
   public static void computeTouchdownPositionUsingRaibertHeuristicAndPolePlacement(FrameVector3DReadOnly velocity,
                                                                                    ReferenceFrame controlFrame,
                                                                                    FramePoint2DBasics touchdownPositionToPack,
                                                                                    double pole,
                                                                                    double stepDuration,
                                                                                    double omega)
   {
      computeTouchdownPositionUsingRaibertHeuristic(calculateTimeConstantUsingPolePlacement(pole, stepDuration, omega),
                                                    velocity,
                                                    controlFrame,
                                                    touchdownPositionToPack);
   }

   /**
    * Computes a desired touchdown position to achieve steady state walking using the Raibert Heuristic footstep
    * control strategy
    */
   public static void computeTouchdownPositionUsingRaibertHeuristic(double timeConstant,
                                                                    FrameVector3DReadOnly velocity,
                                                                    ReferenceFrame controlFrame,
                                                                    FramePoint2DBasics touchdownPositionToPack)
   {
      ReferenceFrame originalFrame = touchdownPositionToPack.getReferenceFrame();
      touchdownPositionToPack.setToZero(controlFrame);
      touchdownPositionToPack.set(velocity.getX(), velocity.getY());
      touchdownPositionToPack.scale(timeConstant);
      touchdownPositionToPack.changeFrameAndProjectToXYPlane(originalFrame);
   }

   /**
    * Computes time constant, aka Raibert Heuristic gain, using pole placement in order to dictate the desired
    * closed loop dynamic performance of the pendulum system
    */
   public static double calculateTimeConstantUsingPolePlacement(double pole, double stepDuration, double omega)
   {
      return (Math.cosh(omega * stepDuration) - pole) / (omega * Math.sinh(omega * stepDuration));
   }

   /**
    * Computes touchdown position offset in the forward/backwards (x) direction necessary to achieve a
    * desired walking speed along that direction. This is used in conjunction with the Raibert Heuristic
    * method of touchdown position control
    */
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

   /**
    * Computes touchdown position offset in the lateral (y) direction necessary to achieve a desired
    * walking speed along that direction. This is used in conjunction with the Raibert Heuristic
    * method of touchdown position control
    */
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

   /**
    * Computes touchdown position offset necessary to achieve a desired stance width while walking.
    * This is used in conjunction with the Raibert Heuristic method of touchdown position control
    */
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

   /**
    * Computes desired angular momentum reference required to achieve a desired stance width while walking.
    * This is used in conjunction with the Gong method of touchdown position control
    */
   public static double computeDesiredAngularMomentumForStanceWidth(RobotSide swingSide,
                                                                    double desiredStanceWidth,
                                                                    double pendulumMass,
                                                                    double pendulumHeight,
                                                                    double stepDuration)
   {
      double sideSignMultiplier = swingSide == RobotSide.LEFT ? 1.0 : -1.0;
      double omega = computeNaturalFrequency(pendulumHeight);
      return sideSignMultiplier * 0.5 * pendulumMass * pendulumHeight * desiredStanceWidth * (omega * Math.sinh(omega * stepDuration)) / (1 + Math.cosh(
            omega * stepDuration));
   }

   /**
    * Computes natural frequency of Linear Inverted Pendulum
    */
   public static double computeNaturalFrequency(double pendulumHeight)
   {
      return Math.sqrt(Math.abs(GRAVITY / pendulumHeight));
   }
}
