package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.quicksterFootstepProvider;

import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.robotics.robotSide.RobotSide;

public class ALIPCalculatorTools
{
   private final static double GRAVITY = -9.81;

   private final static FramePose3D tempFutureCoMPose = new FramePose3D();
   private final static FrameVector3D tempFutureCoMVelocity = new FrameVector3D();
   private final static FrameVector3D tempFutureContactPointAngularMomentum = new FrameVector3D();
   private final static PoseReferenceFrame tempFutureControlFrame = new PoseReferenceFrame("futureControlFrameALIPCalculator", ReferenceFrame.getWorldFrame());

   private final static FramePose3D tempCurrentCoMPose = new FramePose3D();
   private final static FrameVector3D tempCurrentContactPointAngularMomentum = new FrameVector3D();
   private final static FramePoint3D tempCurrentStanceFootPosition = new FramePoint3D();

   public static void computeFutureStateUsingALIP(FramePose3DReadOnly currentCoMPose,
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

      double omega = calculateOmega(pendulumHeight);

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

      for (double i = 0; i < intervals ; i ++)
      {
         futureCoMPoseToPack.appendYawRotation(desiredTurningVelocity * updateDt);
         futureCoMPoseToPack.getPosition().addX(xf / intervals);
         futureCoMPoseToPack.getPosition().addY(yf / intervals);


      }

      futureContactPointAngularMomentumToPack.setX(Lxf);
      futureContactPointAngularMomentumToPack.setY(Lyf);


      futureCoMPoseToPack.changeFrame(originalPositionFrame);
      futureContactPointAngularMomentumToPack.changeFrame(originalMomentumFrame);


   }

//   public static void computeTouchdownPositionRegular(FramePoint3DReadOnly currentPosition,
//                                                      FrameVector3DReadOnly currentVelocity,
//                                                      FrameVector3DReadOnly currentCentroidalAngularMomentum,
//                                                      RobotSide swingSide,
//                                                      double desiredVelocityX,
//                                                      double desiredVelocityY,
//                                                      double desiredStanceWidth,
//                                                      double timeRemainingInCurrentStep,
//                                                      double stepDuration,
//                                                      double pendulumMass,
//                                                      double pendulumHeight,
//                                                      FramePoint2DBasics touchdownPositionToPack,
//                                                      ReferenceFrame stanceFootFrame,
//                                                      ReferenceFrame swingFootFrame,
//                                                      ReferenceFrame controlFrame)
//   {
//      double omega = calculateOmega(pendulumHeight);
//
//      double LyDesired = pendulumMass * desiredVelocityX * pendulumHeight;
//      double LxDesired = computeDesiredAngularMomentumForStanceWidth(swingSide, desiredStanceWidth, pendulumMass, pendulumHeight, stepDuration) -pendulumMass * desiredVelocityY * pendulumHeight;
//
//      //////
//      // Get CoM velocity and change frame to CoM control frame
//      tempVelocity.setIncludingFrame(currentVelocity);
//      tempVelocity.changeFrame(stanceFootFrame);
//
//      // Get CoM angular momentum and change frame to CoM control frame
//      tempCentroidalAngularMomentum.setIncludingFrame(currentCentroidalAngularMomentum);
//      tempCentroidalAngularMomentum.changeFrame(stanceFootFrame);
//      /////
//
//      tempCurrentContactPointAngularMomentum.setToZero(stanceFootFrame);
//      tempCurrentContactPointAngularMomentum.setY(pendulumMass * pendulumHeight * tempVelocity.getX() + tempCentroidalAngularMomentum.getY());
//      tempCurrentContactPointAngularMomentum.setX(-pendulumMass * pendulumHeight * tempVelocity.getY() + tempCentroidalAngularMomentum.getX());
//      computeFutureStateUsingALIP(currentPosition, tempCurrentContactPointAngularMomentum, tempFuturePosition, tempFutureContactPointAngularMomentum, timeRemainingInCurrentStep, pendulumMass, pendulumHeight, stanceFootFrame);
//
//      double LyEndOfCurrentStep = tempFutureContactPointAngularMomentum.getY();
//      double LxEndOfCurrentStep = tempFutureContactPointAngularMomentum.getX();
//
//      double desiredFootstepPositionX = (LyDesired - Math.cosh(omega * stepDuration) * LyEndOfCurrentStep) / (pendulumMass * pendulumHeight * omega * Math.sinh(omega * stepDuration));
//      double desiredFootstepPositionY = (Math.cosh(omega * stepDuration) * LxEndOfCurrentStep - LxDesired) / (pendulumMass * pendulumHeight * omega * Math.sinh(omega * stepDuration));
//
//      ReferenceFrame originalFrame = touchdownPositionToPack.getReferenceFrame();
//      touchdownPositionToPack.setToZero(controlFrame);
//      touchdownPositionToPack.set(-desiredFootstepPositionX, -desiredFootstepPositionY);
//      touchdownPositionToPack.changeFrameAndProjectToXYPlane(originalFrame);
//   }

   public static void computeTouchdownPositionUsingRaibertHeuristicAndPolePlacement(FramePose3DReadOnly currentCoMPose,
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
      double omega = calculateOmega(pendulumHeight);

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
      computeTouchdownPositionUsingRaibertHeuristicAndPolePlacement(tempFutureCoMVelocity, controlFrameToUse, touchdownPositionToPack, pole, stepDuration, omega);

      double swingDuration = stepDuration - doubleSupportDuration;

      ReferenceFrame originalFrame = touchdownPositionToPack.getReferenceFrame();
      touchdownPositionToPack.changeFrameAndProjectToXYPlane(controlFrameToUse);
      touchdownPositionToPack.addX(computeForwardTouchdownOffsetForVelocity(swingDuration, doubleSupportDuration, omega, desiredVelocityX));
      touchdownPositionToPack.addY(computeLateralTouchdownOffsetForVelocity(swingDuration, doubleSupportDuration, swingSide.getOppositeSide(), omega, desiredVelocityY));
      touchdownPositionToPack.addY(computeDesiredTouchdownOffsetForStanceWidth(swingDuration, doubleSupportDuration, desiredStanceWidth, swingSide.getOppositeSide(), omega));
      touchdownPositionToPack.changeFrameAndProjectToXYPlane(originalFrame);
   }

   public static void computeTouchdownPositionUsingRaibertHeuristicAndPolePlacement(FrameVector3DReadOnly velocity, ReferenceFrame controlFrame, FramePoint2DBasics touchdownPositionToPack, double pole, double stepDuration, double omega)
   {
      computeTouchdownPositionUsingRaibertHeuristic(calculateTimeConstantUsingPolePlacement(pole, stepDuration, omega), velocity, controlFrame, touchdownPositionToPack);
   }

   public static void computeTouchdownPositionUsingRaibertHeuristic(double timeConstant, FrameVector3DReadOnly velocity, ReferenceFrame controlFrame, FramePoint2DBasics touchdownPositionToPack)
   {
      ReferenceFrame originalFrame = touchdownPositionToPack.getReferenceFrame();
      touchdownPositionToPack.setToZero(controlFrame);
      touchdownPositionToPack.set(velocity.getX(), velocity.getY());
      touchdownPositionToPack.scale(timeConstant);
      touchdownPositionToPack.changeFrameAndProjectToXYPlane(originalFrame);
   }

   public static double calculateTimeConstantUsingPolePlacement(double pole, double stepDuration, double omega)
   {
      return (Math.cosh(omega * stepDuration) - pole) / (omega * Math.sinh(omega * stepDuration));
   }

   public static double computeDesiredAngularMomentumForStanceWidth(RobotSide swingSide, double desiredStanceWidth, double pendulumMass, double pendulumHeight, double stepDuration)
   {
      double sideSignMultiplier = swingSide == RobotSide.LEFT ? 1.0  : -1.0;
      double omega = calculateOmega(pendulumHeight);
      return sideSignMultiplier * 0.5 * pendulumMass * pendulumHeight * desiredStanceWidth * (omega * Math.sinh(omega * stepDuration)) / (1 + Math.cosh(omega * stepDuration));
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

   public static double calculateOmega(double height)
   {
      return Math.sqrt(Math.abs(GRAVITY / height));
   }
}
