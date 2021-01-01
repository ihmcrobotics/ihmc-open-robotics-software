package us.ihmc.commonWalkingControlModules.modelPredictiveController;

import org.ejml.data.DMatrix;
import us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning.ContactStateProvider;
import us.ihmc.commons.MathTools;
import us.ihmc.commons.lists.RecyclingArrayList;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;

import java.util.List;

import static us.ihmc.commonWalkingControlModules.modelPredictiveController.MPCQPInputCalculator.sufficientlyLargeValue;
import static us.ihmc.commonWalkingControlModules.modelPredictiveController.MPCQPInputCalculator.sufficientlyLongTime;

public class CoMMPCTools
{
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

   public static void constructDesiredCoMPosition(FixedFramePoint3DBasics comPositionToPack,
                                                  FramePoint3DReadOnly firstCoefficient,
                                                  FramePoint3DReadOnly secondCoefficient,
                                                  FramePoint3DReadOnly thirdCoefficient,
                                                  FramePoint3DReadOnly fourthCoefficient,
                                                  FramePoint3DReadOnly fifthCoefficient,
                                                  FramePoint3DReadOnly sixthCoefficient,
                                                  double timeInPhase,
                                                  double omega)
   {
      timeInPhase = Math.min(timeInPhase, sufficientlyLongTime);

      double exponential = Math.min(Math.exp(omega * timeInPhase), sufficientlyLargeValue);
      double t2 = timeInPhase * timeInPhase;
      double t3 = timeInPhase * t2;

      comPositionToPack.checkReferenceFrameMatch(worldFrame);
      comPositionToPack.setToZero();
      comPositionToPack.scaleAdd(exponential, firstCoefficient, comPositionToPack);
      comPositionToPack.scaleAdd(1.0 / exponential, secondCoefficient, comPositionToPack);
      comPositionToPack.scaleAdd(t3, thirdCoefficient, comPositionToPack);
      comPositionToPack.scaleAdd(t2, fourthCoefficient, comPositionToPack);
      comPositionToPack.scaleAdd(timeInPhase, fifthCoefficient, comPositionToPack);
      comPositionToPack.add(sixthCoefficient);
   }

   public static void constructDesiredCoMVelocity(FixedFrameVector3DBasics comVelocityToPack,
                                                  FramePoint3DReadOnly firstCoefficient,
                                                  FramePoint3DReadOnly secondCoefficient,
                                                  FramePoint3DReadOnly thirdCoefficient,
                                                  FramePoint3DReadOnly fourthCoefficient,
                                                  FramePoint3DReadOnly fifthCoefficient,
                                                  FramePoint3DReadOnly sixthCoefficient,
                                                  double timeInPhase,
                                                  double omega)
   {
      timeInPhase = Math.min(timeInPhase, sufficientlyLongTime);

      double exponential = Math.min(Math.exp(omega * timeInPhase), sufficientlyLargeValue);

      comVelocityToPack.checkReferenceFrameMatch(worldFrame);
      comVelocityToPack.setToZero();
      comVelocityToPack.scaleAdd(omega * exponential, firstCoefficient, comVelocityToPack);
      comVelocityToPack.scaleAdd(-omega / exponential, secondCoefficient, comVelocityToPack);
      comVelocityToPack.scaleAdd(3.0 * timeInPhase * timeInPhase, thirdCoefficient, comVelocityToPack);
      comVelocityToPack.scaleAdd(2.0 * timeInPhase, fourthCoefficient, comVelocityToPack);
      comVelocityToPack.add(fifthCoefficient);
   }

   public static void constructDesiredCoMAcceleration(FixedFrameVector3DBasics comAccelerationToPack,
                                                      FramePoint3DReadOnly firstCoefficient,
                                                      FramePoint3DReadOnly secondCoefficient,
                                                      FramePoint3DReadOnly thirdCoefficient,
                                                      FramePoint3DReadOnly fourthCoefficient,
                                                      FramePoint3DReadOnly fifthCoefficient,
                                                      FramePoint3DReadOnly sixthCoefficient,
                                                      double timeInPhase,
                                                      double omega)
   {
      timeInPhase = Math.min(timeInPhase, sufficientlyLongTime);
      double exponential = Math.min(Math.exp(omega * timeInPhase), sufficientlyLargeValue);
      double omega2 = omega * omega;

      comAccelerationToPack.checkReferenceFrameMatch(worldFrame);
      comAccelerationToPack.setToZero();
      comAccelerationToPack.scaleAdd(omega2 * exponential, firstCoefficient, comAccelerationToPack);
      comAccelerationToPack.scaleAdd(omega2 / exponential, secondCoefficient, comAccelerationToPack);
      comAccelerationToPack.scaleAdd(6.0 * timeInPhase, thirdCoefficient, comAccelerationToPack);
      comAccelerationToPack.scaleAdd(2.0, fourthCoefficient, comAccelerationToPack);
   }

   public static void constructDesiredDCMPosition(FixedFramePoint3DBasics dcmPositionToPack,
                                                  FramePoint3DReadOnly firstCoefficient,
                                                  FramePoint3DReadOnly secondCoefficient,
                                                  FramePoint3DReadOnly thirdCoefficient,
                                                  FramePoint3DReadOnly fourthCoefficient,
                                                  FramePoint3DReadOnly fifthCoefficient,
                                                  FramePoint3DReadOnly sixthCoefficient,
                                                  double timeInPhase,
                                                  double omega)
   {
      timeInPhase = Math.min(timeInPhase, sufficientlyLongTime);
      double exponential = Math.min(Math.exp(omega * timeInPhase), sufficientlyLargeValue);

      double t2 = timeInPhase * timeInPhase;
      double t3 = timeInPhase * t2;

      dcmPositionToPack.checkReferenceFrameMatch(worldFrame);
      dcmPositionToPack.setToZero();
      dcmPositionToPack.scaleAdd(2.0 * exponential, firstCoefficient, dcmPositionToPack);
      dcmPositionToPack.scaleAdd(t3 + 3.0 * t2 / omega, thirdCoefficient, dcmPositionToPack);
      dcmPositionToPack.scaleAdd(t2 + 2.0 * timeInPhase / omega, fourthCoefficient, dcmPositionToPack);
      dcmPositionToPack.scaleAdd(timeInPhase + 1.0 / omega, fifthCoefficient, dcmPositionToPack);
      dcmPositionToPack.add(sixthCoefficient);
   }

   public static void constructDesiredVRPPosition(FixedFramePoint3DBasics vrpPositionToPack,
                                                  FramePoint3DReadOnly firstCoefficient,
                                                  FramePoint3DReadOnly secondCoefficient,
                                                  FramePoint3DReadOnly thirdCoefficient,
                                                  FramePoint3DReadOnly fourthCoefficient,
                                                  FramePoint3DReadOnly fifthCoefficient,
                                                  FramePoint3DReadOnly sixthCoefficient,
                                                  double timeInPhase,
                                                  double omega)
   {
      timeInPhase = Math.min(timeInPhase, sufficientlyLongTime);

      double t2 = timeInPhase * timeInPhase;
      double t3 = timeInPhase * t2;

      double omega2 = omega * omega;

      vrpPositionToPack.checkReferenceFrameMatch(worldFrame);
      vrpPositionToPack.setToZero();
      vrpPositionToPack.scaleAdd(t3 - 6.0 * timeInPhase / omega2, thirdCoefficient, vrpPositionToPack);
      vrpPositionToPack.scaleAdd(t2 - 2.0 / omega2, fourthCoefficient, vrpPositionToPack);
      vrpPositionToPack.scaleAdd(timeInPhase, fifthCoefficient, vrpPositionToPack);
      vrpPositionToPack.add(sixthCoefficient);
   }

   public static void constructDesiredVRPVelocity(FixedFrameVector3DBasics vrpVelocityToPack,
                                                  FramePoint3DReadOnly firstCoefficient,
                                                  FramePoint3DReadOnly secondCoefficient,
                                                  FramePoint3DReadOnly thirdCoefficient,
                                                  FramePoint3DReadOnly fourthCoefficient,
                                                  FramePoint3DReadOnly fifthCoefficient,
                                                  FramePoint3DReadOnly sixthCoefficient,
                                                  double timeInPhase,
                                                  double omega)
   {
      timeInPhase = Math.min(timeInPhase, sufficientlyLongTime);

      double t2 = timeInPhase * timeInPhase;

      double omega2 = omega * omega;

      vrpVelocityToPack.checkReferenceFrameMatch(worldFrame);
      vrpVelocityToPack.setToZero();
      vrpVelocityToPack.scaleAdd(3.0 * t2 - 6.0 / omega2, thirdCoefficient, vrpVelocityToPack);
      vrpVelocityToPack.scaleAdd(2.0 * timeInPhase, fourthCoefficient, vrpVelocityToPack);
      vrpVelocityToPack.add(fifthCoefficient);
   }

   public static void constructedDesiredBodyOrientation(FixedFrameOrientation3DBasics orientationToPack,
                                                        FramePoint3DReadOnly firstCoefficient,
                                                        FramePoint3DReadOnly secondCoefficient,
                                                        FramePoint3DReadOnly thirdCoefficient,
                                                        FramePoint3DReadOnly fourthCoefficient,
                                                        FramePoint3DReadOnly fifthCoefficient,
                                                        FramePoint3DReadOnly sixthCoefficient,
                                                        double omega,
                                                        double timeInPhase)
   {
      timeInPhase = Math.min(timeInPhase, sufficientlyLongTime);

      double exponential = Math.exp(omega * timeInPhase);
      double t2 = timeInPhase * timeInPhase;
      double t3 = timeInPhase * t2;

      orientationToPack.checkReferenceFrameMatch(worldFrame);
      orientationToPack.setToZero();

      double roll, pitch, yaw;
      if (MPCIndexHandler.includeExponentialInOrientation)
      {
         roll = exponential * firstCoefficient.getX() + secondCoefficient.getX() / exponential + t3 * thirdCoefficient.getX() + t2 * fourthCoefficient.getX()
                + timeInPhase * fifthCoefficient.getX() + sixthCoefficient.getX();
         pitch = exponential * firstCoefficient.getY() + secondCoefficient.getY() / exponential + t3 * thirdCoefficient.getY() + t2 * fourthCoefficient.getY()
                 + timeInPhase * fifthCoefficient.getY() + sixthCoefficient.getY();
         yaw = exponential * firstCoefficient.getZ() + secondCoefficient.getZ() / exponential + t3 * thirdCoefficient.getZ() + t2 * fourthCoefficient.getZ()
               + timeInPhase * fifthCoefficient.getZ() + sixthCoefficient.getZ();
      }
      else
      {
         roll = t3 * firstCoefficient.getX() + t2 * secondCoefficient.getX() + timeInPhase * thirdCoefficient.getX() + fourthCoefficient.getX();
         pitch = t3 * firstCoefficient.getY() + t2 * secondCoefficient.getY() + timeInPhase * thirdCoefficient.getY() + fourthCoefficient.getY();
         yaw = t3 * firstCoefficient.getZ() + t2 * secondCoefficient.getZ() + timeInPhase * thirdCoefficient.getZ() + fourthCoefficient.getZ();
      }

      orientationToPack.setYawPitchRoll(yaw, pitch, roll);
   }

   public static void constructedDesiredBodyAngularVelocity(FixedFrameVector3DBasics angularRateToPack,
                                                            FramePoint3DReadOnly firstCoefficient,
                                                            FramePoint3DReadOnly secondCoefficient,
                                                            FramePoint3DReadOnly thirdCoefficient,
                                                            FramePoint3DReadOnly fourthCoefficient,
                                                            FramePoint3DReadOnly fifthCoefficient,
                                                            FramePoint3DReadOnly sixthCoefficient,
                                                            double omega,
                                                            double timeInPhase)
   {
      timeInPhase = Math.min(timeInPhase, sufficientlyLongTime);

      double exponential = Math.exp(omega * timeInPhase);
      double c0 = 3.0 * timeInPhase * timeInPhase;
      double c1 = 2.0 * timeInPhase;

      angularRateToPack.checkReferenceFrameMatch(worldFrame);
      angularRateToPack.setToZero();

      if (MPCIndexHandler.includeExponentialInOrientation)
      {
         angularRateToPack.scaleAdd(omega * exponential, firstCoefficient);
         angularRateToPack.scaleAdd(-omega / exponential, secondCoefficient);
         angularRateToPack.scaleAdd(c0, thirdCoefficient);
         angularRateToPack.scaleAdd(c1, fourthCoefficient);
         angularRateToPack.add(fifthCoefficient);
      }
      else
      {
         angularRateToPack.scaleAdd(c0, firstCoefficient);
         angularRateToPack.scaleAdd(c1, secondCoefficient);
         angularRateToPack.add(thirdCoefficient);
      }
   }
}
