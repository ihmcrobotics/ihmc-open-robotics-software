package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.quicksterFootstepProvider;

import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;
import us.ihmc.euclid.tuple2D.interfaces.Vector2DReadOnly;
import us.ihmc.robotics.robotSide.RobotSide;

public class ALIPCalculatorTools
{
   private final static double GRAVITY = -9.81;

   private final static FramePoint3D tempPosition = new FramePoint3D();
   private final static FrameVector3D tempAngularMomentum = new FrameVector3D();
   private final static FrameVector3D tempAngularMomentum2 = new FrameVector3D();
   private final static FrameVector3D tempVelocity = new FrameVector3D();
   private final static FrameVector3D tempCentroidalAngularMomentum = new FrameVector3D();

   public static void computeFutureStateUsingALIP(FramePoint3DReadOnly currentPosition,
                                                  FrameVector3DReadOnly currentAngularMomentum,
                                                  FramePoint3D futurePositionToPack,
                                                  FrameVector3D futureAngularMomentumToPack,
                                                  double deltaT,
                                                  double pendulumMass,
                                                  double pendulumHeight,
                                                  ReferenceFrame stanceFootFrame)
   {
      // Use future position/angular momentum as placeholder variables to make sure current position/angular momentum are in correct frame
      futurePositionToPack.setMatchingFrame(currentPosition);
      futurePositionToPack.changeFrame(stanceFootFrame);

      futureAngularMomentumToPack.setMatchingFrame(currentAngularMomentum);
      futureAngularMomentumToPack.changeFrame(stanceFootFrame);

      double omega = calculateOmega(pendulumHeight);

      // Current position and angular momentum
      double x0 = futurePositionToPack.getX();
      double Ly0 = futureAngularMomentumToPack.getY();

      double y0 = futurePositionToPack.getY();
      double Lx0 = futureAngularMomentumToPack.getX();

      // Final position and angular momentum
      double xf = x0 * Math.cosh(omega * deltaT) + Ly0 * Math.sinh(omega * deltaT) / (pendulumMass * pendulumHeight * omega);
      double Lyf = x0 * pendulumMass * pendulumHeight * omega * Math.sinh(omega * deltaT) + Ly0 * Math.cosh(omega * deltaT);

      double yf = y0 * Math.cosh(omega * deltaT) - Lx0 * Math.sinh(omega * deltaT) / (pendulumMass * pendulumHeight * omega);
      double Lxf = -y0 * pendulumMass * pendulumHeight * omega * Math.sinh(omega * deltaT) + Lx0 * Math.cosh(omega * deltaT);

      futurePositionToPack.setX(xf);
      futurePositionToPack.setY(yf);

      futureAngularMomentumToPack.setX(Lxf);
      futureAngularMomentumToPack.setY(Lyf);
   }

   public static void computeTouchdownPositionRegular(FramePoint3DReadOnly currentPosition,
                                                      FrameVector3DReadOnly currentVelocity,
                                                      FrameVector3DReadOnly currentCentroidalAngularMomentum,
                                                      RobotSide swingSide,
                                                      double desiredVelocityX,
                                                      double desiredVelocityY,
                                                      double desiredStanceWidth,
                                                      double timeRemainingInCurrentStep,
                                                      double stepDuration,
                                                      double pendulumMass,
                                                      double pendulumHeight,
                                                      FramePoint2DBasics touchdownPositionToPack,
                                                      ReferenceFrame stanceFootFrame,
                                                      ReferenceFrame swingFootFrame,
                                                      ReferenceFrame controlFrame)
   {
      double omega = calculateOmega(pendulumHeight);

      double LyDesired = pendulumMass * desiredVelocityX * pendulumHeight;
      double LxDesired = computeDesiredAngularMomentumForStanceWidth(swingSide, desiredStanceWidth, pendulumMass, pendulumHeight, stepDuration) -pendulumMass * desiredVelocityY * pendulumHeight;

      //////
      // Get CoM velocity and change frame to CoM control frame
      tempVelocity.setIncludingFrame(currentVelocity);
      tempVelocity.changeFrame(stanceFootFrame);

      // Get CoM angular momentum and change frame to CoM control frame
      tempCentroidalAngularMomentum.setIncludingFrame(currentCentroidalAngularMomentum);
      tempCentroidalAngularMomentum.changeFrame(stanceFootFrame);
      /////

      tempAngularMomentum2.setToZero(stanceFootFrame);
      tempAngularMomentum2.setY(pendulumMass * pendulumHeight * tempVelocity.getX() + tempCentroidalAngularMomentum.getY());
      tempAngularMomentum2.setX(-pendulumMass * pendulumHeight * tempVelocity.getY() + tempCentroidalAngularMomentum.getX());
      computeFutureStateUsingALIP(currentPosition, tempAngularMomentum2, tempPosition, tempAngularMomentum, timeRemainingInCurrentStep, pendulumMass, pendulumHeight, stanceFootFrame);

      double LyEndOfCurrentStep = tempAngularMomentum.getY();
      double LxEndOfCurrentStep = tempAngularMomentum.getX();

      double desiredFootstepPositionX = (LyDesired - Math.cosh(omega * stepDuration) * LyEndOfCurrentStep) / (pendulumMass * pendulumHeight * omega * Math.sinh(omega * stepDuration));
      double desiredFootstepPositionY = (Math.cosh(omega * stepDuration) * LxEndOfCurrentStep - LxDesired) / (pendulumMass * pendulumHeight * omega * Math.sinh(omega * stepDuration));

      ReferenceFrame originalFrame = touchdownPositionToPack.getReferenceFrame();
      touchdownPositionToPack.setToZero(controlFrame);
      touchdownPositionToPack.set(-desiredFootstepPositionX, -desiredFootstepPositionY);
      touchdownPositionToPack.changeFrameAndProjectToXYPlane(originalFrame);
   }

   public static void computeTouchdownPositionUsingRaibertHeuristicAndPolePlacement(FramePoint3DReadOnly currentPosition,
                                                      FrameVector3DReadOnly currentVelocity,
                                                      FrameVector3DReadOnly currentCentroidalAngularMomentum,
                                                      RobotSide swingSide,
                                                      double desiredVelocityX,
                                                      double desiredVelocityY,
                                                      double desiredStanceWidth,
                                                      double timeRemainingInCurrentStep,
                                                      double stepDuration,
                                                      double doubleSupportDuration,
                                                      double pendulumMass,
                                                      double pendulumHeight,
                                                      double pole,
                                                      FramePoint2DBasics touchdownPositionToPack,
                                                      ReferenceFrame stanceFootFrame,
                                                      ReferenceFrame swingFootFrame,
                                                      ReferenceFrame controlFrame)
   {
      double omega = calculateOmega(pendulumHeight);

      //////
      // Get CoM velocity and change frame to CoM control frame
      tempVelocity.setIncludingFrame(currentVelocity);
      tempVelocity.changeFrame(stanceFootFrame);

      // Get CoM angular momentum and change frame to CoM control frame
      tempCentroidalAngularMomentum.setIncludingFrame(currentCentroidalAngularMomentum);
      tempCentroidalAngularMomentum.changeFrame(stanceFootFrame);
      /////

      tempAngularMomentum2.setToZero(stanceFootFrame);
      tempAngularMomentum2.setY(pendulumMass * pendulumHeight * tempVelocity.getX() + tempCentroidalAngularMomentum.getY());
      tempAngularMomentum2.setX(-pendulumMass * pendulumHeight * tempVelocity.getY() + tempCentroidalAngularMomentum.getX());
      computeFutureStateUsingALIP(currentPosition, tempAngularMomentum2, tempPosition, tempAngularMomentum, timeRemainingInCurrentStep, pendulumMass, pendulumHeight, stanceFootFrame);

      tempVelocity.setX(tempAngularMomentum.getY() / (pendulumMass * pendulumHeight));
      tempVelocity.setY(-tempAngularMomentum.getX() / (pendulumMass * pendulumHeight));

      computeTouchdownPositionUsingRaibertHeuristicAndPolePlacement(tempVelocity, touchdownPositionToPack, pole, stepDuration, omega, controlFrame);

      double swingDuration = stepDuration - doubleSupportDuration;

      ReferenceFrame originalFrame = touchdownPositionToPack.getReferenceFrame();
      touchdownPositionToPack.changeFrameAndProjectToXYPlane(controlFrame);
      touchdownPositionToPack.addX(computeForwardTouchdownOffsetForVelocity(swingDuration, doubleSupportDuration, omega, desiredVelocityX));
      touchdownPositionToPack.addY(computeLateralTouchdownOffsetForVelocity(swingDuration, doubleSupportDuration, swingSide.getOppositeSide(), omega, desiredVelocityY));
      touchdownPositionToPack.addY(computeDesiredTouchdownOffsetForStanceWidth(swingDuration, doubleSupportDuration, desiredStanceWidth, swingSide.getOppositeSide(), omega));
      touchdownPositionToPack.changeFrameAndProjectToXYPlane(originalFrame);
   }

   public static void computeTouchdownPositionUsingRaibertHeuristicAndPolePlacement(FrameVector3DReadOnly velocity, FramePoint2DBasics touchdownPositionToPack, double pole, double stepDuration, double omega, ReferenceFrame controlFrame)
   {
      computeTouchdownPositionUsingRaibertHeuristic(calculateTimeConstantUsingPolePlacement(pole, stepDuration, omega), velocity, touchdownPositionToPack, controlFrame);
   }

   public static void computeTouchdownPositionUsingRaibertHeuristic(double timeConstant, FrameVector3DReadOnly velocity, FramePoint2DBasics touchdownPositionToPack, ReferenceFrame controlFrame)
   {
      ReferenceFrame originalFrame = touchdownPositionToPack.getReferenceFrame();
      touchdownPositionToPack.changeFrameAndProjectToXYPlane(controlFrame);
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
