package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.interfaces.*;

/**
 * Note: CMP stands for Centroidal Momentum Pivot
 *
 */
public class CapturePointTools
{
   private static final double EPSILON = 1.0e-15;

   /**
    * Computes the Instantaneous Capture Point position by
    * <p>
    *    x<sup>ICP</sup> = x<sup>CoM</sup> + 1&frasl;&omega;  x&#775;<sup>CoM</sup>
    * </p>
    *
    * @param centerOfMassPosition
    * @param centerOfMassVelocity
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param icpToPack
    */
   public static void computeCapturePointPosition(FramePoint3DReadOnly centerOfMassPosition, FrameVector3DReadOnly centerOfMassVelocity, double omega0,
                                                  FixedFramePoint3DBasics icpToPack)
   {
      icpToPack.scaleAdd(1.0 / omega0, centerOfMassVelocity, centerOfMassPosition);
   }

   /**
    * Computes the Instantaneous Capture Point position by
    * <p>
    *    x<sup>ICP</sup> = x<sup>CoM</sup> + 1&frasl;&omega;  x&#775;<sup>CoM</sup>
    * </p>
    *
    * @param centerOfMassPosition
    * @param centerOfMassVelocity
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param icpToPack
    */
   public static void computeCapturePointPosition(FramePoint2DReadOnly centerOfMassPosition, FrameVector2DReadOnly centerOfMassVelocity, double omega0,
                                                  FixedFramePoint2DBasics icpToPack)
   {
      icpToPack.scaleAdd(1.0 / omega0, centerOfMassVelocity, centerOfMassPosition);
   }

   /**
    * Computes the Instantaneous Capture Point velocity by
    * <p>
    *    x&#775;<sup>ICP</sup> = x&#775;<sup>CoM</sup> + 1&frasl;&omega;  x&#776;<sup>CoM</sup>
    * </p>
    *
    * @param centerOfMassVelocity
    * @param centerOfMassAcceleration
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param icpVelocityToPack
    */
   public static void computeCapturePointVelocity(FrameVector3DReadOnly centerOfMassVelocity, FrameVector3DReadOnly centerOfMassAcceleration,
                                                  double omega0, FixedFrameVector3DBasics icpVelocityToPack)
   {
      icpVelocityToPack.scaleAdd(1.0 / omega0, centerOfMassAcceleration, centerOfMassVelocity);
   }

   /**
    * Computes the Instantaneous Capture Point velocity by
    * <p>
    *    x&#775;<sup>ICP</sup> = x&#775;<sup>CoM</sup> + 1&frasl;&omega;  x&#776;<sup>CoM</sup>
    * </p>
    *
    * @param centerOfMassVelocity
    * @param centerOfMassAcceleration
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param icpVelocityToPack
    */
   public static void computeCapturePointVelocity(FrameVector2DReadOnly centerOfMassVelocity, FrameVector2DReadOnly centerOfMassAcceleration,
                                                  double omega0, FixedFrameVector2DBasics icpVelocityToPack)
   {
      icpVelocityToPack.scaleAdd(1.0 / omega0, centerOfMassAcceleration, centerOfMassVelocity);
   }

   /**
    * Computes the Instantaneous Capture Point velocity by
    * <p>
    *    x&#775;<sup>ICP</sup> = &omega; (x<sup>ICP</sup> - x<sup>CMP</sup>)
    * </p>
    *
    * @param capturePointPosition
    * @param cmpPosition
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param icpVelocityToPack
    */
   public static void computeCapturePointVelocity(FramePoint3DReadOnly capturePointPosition, FramePoint3DReadOnly cmpPosition,
                                                  double omega0, FixedFrameVector3DBasics icpVelocityToPack)
   {
      icpVelocityToPack.sub(capturePointPosition, cmpPosition);
      icpVelocityToPack.scale(omega0);
   }


   /**
    * FIXME this method is probably wrong.
    * Compute the capture point acceleration given the desired capture point velocity
    *
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param capturePointVelocity
    * @param icpAccelerationAccelerationToPack
    */
   public static void computeCapturePointAcceleration(double omega0, FrameVector3DReadOnly capturePointVelocity,
                                                      FixedFrameVector3DBasics icpAccelerationAccelerationToPack)
   {
      icpAccelerationAccelerationToPack.setMatchingFrame(capturePointVelocity);
      icpAccelerationAccelerationToPack.scale(omega0);
   }

   /**
    * Computes the centroidal momentum pivot by
    * <p>
    *    x<sup>CMP</sup> = x<sup>ICP</sup> - 1&frasl;&omega;  x&#775;<sup>ICP</sup>
    * </p>
    *
    * @param capturePointPosition
    * @param capturePointVelocity
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param cmpPositionToPack
    */
   public static void computeCentroidalMomentumPivot(FramePoint2DReadOnly capturePointPosition, FrameVector2DReadOnly capturePointVelocity, double omega0,
                                                     FixedFramePoint2DBasics cmpPositionToPack)
   {
      cmpPositionToPack.scaleAdd(-1.0 / omega0, capturePointVelocity, capturePointPosition);
   }

   /**
    * Computes the centroidal momentum pivot by
    * <p>
    *    x<sup>CMP</sup> = x<sup>ICP</sup> - 1&frasl;&omega;  x&#775;<sup>ICP</sup>
    * </p>
    *
    * @param capturePointPosition
    * @param capturePointVelocity
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param cmpPositionToPack
    */
   public static void computeCentroidalMomentumPivot(FramePoint3DReadOnly capturePointPosition, FrameVector3DReadOnly capturePointVelocity, double omega0,
                                                     FixedFramePoint3DBasics cmpPositionToPack)
   {
      cmpPositionToPack.scaleAdd(-1.0 / omega0, capturePointVelocity, capturePointPosition);
   }

   /**
    * Computes the centroidal momentum pivot velocity by, \dot{CMP}_{d} = \dot{ICP}_{d} -
    * \ddot{ICP}_{d}/omega0
    *
    * @param capturePointVelocity
    * @param capturePointAcceleration
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param cmpVelocityToPack
    */
   public static void computeCentroidalMomentumPivotVelocity(FrameVector3DReadOnly capturePointVelocity, FrameVector3DReadOnly capturePointAcceleration,
                                                             double omega0, FixedFrameVector3DBasics cmpVelocityToPack)
   {
      cmpVelocityToPack.scaleAdd(-1.0 / omega0, capturePointAcceleration, capturePointVelocity);
   }

   /**
    * Compute the desired capture point position at a given time. x<sup>ICP<sub>des</sub></sup> =
    * (e<sup>&omega;0 t</sup>) x<sup>ICP<sub>0</sub></sup> + (1-e<sup>&omega;0
    * t</sup>)x<sup>CMP<sub>0</sub></sup>
    *
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param time
    * @param initialDesiredCapturePoint
    * @param initialDesiredCMP
    * @param desiredCapturePointToPack
    */
   public static void computeDesiredCapturePointPosition(double omega0, double time, FramePoint3DReadOnly initialDesiredCapturePoint,
                                                         FramePoint3DReadOnly initialDesiredCMP, FixedFramePoint3DBasics desiredCapturePointToPack)
   {
      if (initialDesiredCapturePoint.distance(initialDesiredCMP) > EPSILON)
         desiredCapturePointToPack.interpolate(initialDesiredCMP, initialDesiredCapturePoint, Math.exp(omega0 * time));
      else
         desiredCapturePointToPack.set(initialDesiredCapturePoint);
   }

   /**
    * Compute the desired capture point position at a given time.
    * <p>
    *    x<sup>ICP<sub>des</sub></sup> =
    * (e<sup>&omega;0 t</sup>) x<sup>ICP<sub>0</sub></sup> + (1-e<sup>&omega;0
    * t</sup>)x<sup>CMP<sub>0</sub></sup>
    * </p>
    *
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           robot.
    * @param time forward time to integrate. t in the above equation.
    * @param initialDesiredCapturePoint capture point position at t = 0. x<sup>ICP<sub>0</sub></sup> in the above equation
    * @param initialDesiredCMP CMP position at t = 0. x<sup>CMP<sub>0</sub></sup> in the above equation
    * @param desiredCapturePointToPack Desired Capture Point position at time t given the initial ICP and CMP states. Ix<sup>ICP<sub>des</sub></sup> in the above equation.
    */
   public static void computeDesiredCapturePointPosition(double omega0, double time, FramePoint2DReadOnly initialDesiredCapturePoint,
                                                         FramePoint2DReadOnly initialDesiredCMP, FixedFramePoint2DBasics desiredCapturePointToPack)
   {
      if (initialDesiredCapturePoint.distance(initialDesiredCMP) > EPSILON)
         desiredCapturePointToPack.interpolate(initialDesiredCMP, initialDesiredCapturePoint, Math.exp(omega0 * time));
      else
         desiredCapturePointToPack.set(initialDesiredCapturePoint);
   }

   /**
    * Compute the desired capture point velocity at a given time.
    * <p>
    *    ICPv<sub>d</sub> = &omega; e<sup>&omega; t</sup> (ICP<sub>0</sub> - p<sub>0</sub>)
    * </p>
    *
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           robot.
    * @param time forward time to integrate. t in the above equation.
    * @param initialDesiredCapturePoint capture point position at t = 0. ICP<sub>0</sub> in the above equation.
    * @param initialDesiredCMP CMP position at t = 0. p<sub>0</sub> in the above equation
    * @param desiredCapturePointVelocityToPack Desired Capture Point velocity at time t given the initial ICP and CMP states. ICPv<sub>d</sub> in the above equation.
    */
   public static void computeDesiredCapturePointVelocity(double omega0, double time, FramePoint3DReadOnly initialDesiredCapturePoint,
                                                         FramePoint3DReadOnly initialDesiredCMP, FixedFrameVector3DBasics desiredCapturePointVelocityToPack)
   {
      if (initialDesiredCapturePoint.distance(initialDesiredCMP) > EPSILON)
      {
         desiredCapturePointVelocityToPack.sub(initialDesiredCapturePoint, initialDesiredCMP);
         desiredCapturePointVelocityToPack.scale(omega0 * Math.exp(omega0 * time));
      }
      else
         desiredCapturePointVelocityToPack.setToZero();
   }

   /**
    * Compute the desired capture point velocity at a given time. ICPv_d = w^2 * e^{w*t} * ICP0 - p0
    * * w^2 * e^{w*t}
    *
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param time
    * @param initialDesiredCapturePoint
    * @param initialDesiredCMP
    * @param desiredCapturePointAccelerationToPack
    */
   public static void computeDesiredCapturePointAcceleration(double omega0, double time, FramePoint3DReadOnly initialDesiredCapturePoint,
                                                             FramePoint3DReadOnly initialDesiredCMP,
                                                             FixedFrameVector3DBasics desiredCapturePointAccelerationToPack)
   {
      if (initialDesiredCapturePoint.distance(initialDesiredCMP) > EPSILON)
      {
         desiredCapturePointAccelerationToPack.sub(initialDesiredCapturePoint, initialDesiredCMP);
         desiredCapturePointAccelerationToPack.scale(omega0 * omega0 * Math.exp(omega0 * time));
      }
      else
         desiredCapturePointAccelerationToPack.setToZero();
   }

   /**
    * Compute the distance along the capture point guide line from the current capture point
    * position to the desired capture point position.
    *
    * @param currentCapturePointPosition
    * @param desiredCapturePointPosition
    * @param desiredCapturePointVelocity
    * @return
    */
   public static double computeDistanceToCapturePointFreezeLineIn2d(FramePoint2DReadOnly currentCapturePointPosition,
                                                                    FramePoint2DReadOnly desiredCapturePointPosition,
                                                                    FrameVector2DReadOnly desiredCapturePointVelocity)
   {
      currentCapturePointPosition.checkReferenceFrameMatch(desiredCapturePointPosition);
      desiredCapturePointVelocity.checkReferenceFrameMatch(desiredCapturePointPosition);

      double desiredCapturePointVelocityMagnitude = Math
            .sqrt(MathTools.square(desiredCapturePointVelocity.getX()) + MathTools.square(desiredCapturePointVelocity.getY()));

      if (desiredCapturePointVelocityMagnitude == 0.0)
      {
         return Double.NaN;
      }
      else
      {
         double normalizedCapturePointVelocityX = desiredCapturePointVelocity.getX() / desiredCapturePointVelocityMagnitude;
         double normalizedCapturePointVelocityY = desiredCapturePointVelocity.getY() / desiredCapturePointVelocityMagnitude;

         double capturePointErrorX = currentCapturePointPosition.getX() - desiredCapturePointPosition.getX();
         double capturePointErrorY = currentCapturePointPosition.getY() - desiredCapturePointPosition.getY();

         return -(normalizedCapturePointVelocityX * capturePointErrorX + normalizedCapturePointVelocityY * capturePointErrorY);
      }
   }

   public static void computeCenterOfMassVelocity(FramePoint2DReadOnly comPosition, FramePoint2DReadOnly dcmPosition, double omega0,
                                                  FixedFrameVector2DBasics comVelocityToPack)
   {
      comVelocityToPack.sub(dcmPosition, comPosition);
      comVelocityToPack.scale(omega0);
   }

   public static void computeCenterOfMassVelocity(FramePoint3DReadOnly comPosition, FramePoint3DReadOnly dcmPosition, double omega0,
                                                  FixedFrameVector3DBasics comVelocityToPack)
   {
      comVelocityToPack.sub(dcmPosition, comPosition);
      comVelocityToPack.scale(omega0);
   }

   public static void computeCenterOfMassAcceleration(FrameVector3DReadOnly comVelocity, FrameVector3DReadOnly dcmVelocity, double omega0,
                                                      FixedFrameVector3DBasics comAccelerationToPack)
   {
      comAccelerationToPack.sub(dcmVelocity, comVelocity);
      comAccelerationToPack.scale(omega0);
   }
}
