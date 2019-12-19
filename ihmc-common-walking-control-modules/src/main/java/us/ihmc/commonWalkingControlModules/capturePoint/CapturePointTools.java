package us.ihmc.commonWalkingControlModules.capturePoint;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.FrameLine2D;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.interfaces.*;

/**
 * Note: CMP stands for Centroidal Momentum Pivot
 *
 */
public class CapturePointTools
{
   private static final double EPSILON = 1.0e-15;

   /**
    * Computes the desired Instantaneous Capture Point position by
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
   public static void computeDesiredCapturePointPosition(FramePoint3DReadOnly centerOfMassPosition, FrameVector3DReadOnly centerOfMassVelocity,
                                                         double omega0, FixedFramePoint3DBasics icpToPack)
   {
      icpToPack.scaleAdd(1.0 / omega0, centerOfMassVelocity, centerOfMassPosition);
   }

   /**
    * Computes the Instantaneous Capture Point position by
    * <p>
    *    x&#775;<sup>ICP</sup> = x&#775;<sup>CoM</sup> + 1&frasl;&omega;  x&#776;<sup>CoM</sup>
    * </p>
    *
    * @param centerOfMassAcceleration
    * @param centerOfMassAcceleration
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param icpVelocityToPack
    */
   public static void computeDesiredCapturePointVelocity(FrameVector3DReadOnly centerOfMassVelocity, FrameVector3DReadOnly centerOfMassAcceleration,
                                                         double omega0, FixedFrameVector3DBasics icpVelocityToPack)
   {
      icpVelocityToPack.scaleAdd(1.0 / omega0, centerOfMassAcceleration, centerOfMassVelocity);
   }

   /**
    * Compute the desired capture point acceleration given the desired capture point velocity
    *
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param desiredCapturePointVelocity
    * @param desiredCapturePointAccelerationToPack
    */
   public static void computeDesiredCapturePointAcceleration(double omega0, FrameVector3DReadOnly desiredCapturePointVelocity,
                                                             FixedFrameVector3DBasics desiredCapturePointAccelerationToPack)
   {
      desiredCapturePointAccelerationToPack.setMatchingFrame(desiredCapturePointVelocity);
      desiredCapturePointAccelerationToPack.scale(omega0);
   }

   /**
    * Computes the desired centroidal momentum pivot by
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
   public static void computeCentroidalMomentumPivot(FramePoint2DReadOnly capturePointPosition, FrameVector2DReadOnly capturePointVelocity,
                                                     double omega0, FixedFramePoint2DBasics cmpPositionToPack)
   {
      cmpPositionToPack.scaleAdd(-1.0 / omega0, capturePointVelocity, capturePointPosition);
   }

   /**
    * Computes the desired centroidal momentum pivot by
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
   public static void computeCentroidalMomentumPivot(FramePoint3DReadOnly capturePointPosition, FrameVector3DReadOnly capturePointVelocity,
                                                     double omega0, FixedFramePoint3DBasics cmpPositionToPack)
   {
      cmpPositionToPack.scaleAdd(-1.0 / omega0, capturePointVelocity, capturePointPosition);
   }

   /**
    * Computes the desired centroidal momentum pivot velocity by, \dot{CMP}_{d} = \dot{ICP}_{d} -
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
    * Compute the capture point position at a given time. x<sup>ICP<sub>des</sub></sup> =
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
    * ICPv_d = w * e^{w*t} * ICP0 - p0 * w * e^{w*t}
    * <p>
    *    ICPv_d = &omega; * e<sup>&omega; t</sup> * ICP<sub>d</sub> - p<sub>0</sub> * &omega; * e<sup>&omega; t</sup>
    * </p>
    *
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param time
    * @param initialDesiredCapturePoint
    * @param initialDesiredCMP
    * @param desiredCapturePointVelocityToPack
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
                                                             FramePoint3DReadOnly initialDesiredCMP, FixedFrameVector3DBasics desiredCapturePointAccelerationToPack)
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

      double desiredCapturePointVelocityMagnitude = Math.sqrt(MathTools.square(desiredCapturePointVelocity.getX())
            + MathTools.square(desiredCapturePointVelocity.getY()));

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
}
