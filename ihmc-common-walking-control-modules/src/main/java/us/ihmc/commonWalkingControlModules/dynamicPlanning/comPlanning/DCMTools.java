package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import us.ihmc.commons.MathTools;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

/**
 * This is a tools class used to help compute a desired DCM position using either a constant or linearly moving VRP waypoint. This is useful
 * for validating the outputs from {@link CoMTrajectoryPlanner}.
 */
public class DCMTools
{
   private static final double EPSILON = 1.0e-15;

   /**
    * Compute the desired DCM position at a given time. x<sup>DCM<sub>des</sub></sup> =
    * (e<sup>&omega;0 t</sup>) x<sup>DCM<sub>0</sub></sup> + (1-e<sup>&omega;0
    * t</sup>)x<sup>VRP<sub>0</sub></sup>
    *
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param time
    * @param initialDCM
    * @param initialVRP
    * @param desiredDCMToPack
    */
   public static void computeDesiredDCMPosition(double omega0, double time, FramePoint3DReadOnly initialDCM, FramePoint3DReadOnly initialVRP,
                                                FixedFramePoint3DBasics desiredDCMToPack)
   {
      if (initialDCM.distance(initialVRP) > EPSILON)
         desiredDCMToPack.interpolate(initialVRP, initialDCM, Math.exp(omega0 * time));
      else
         desiredDCMToPack.set(initialDCM);
   }

   /**
    * Computes the desired DCM position by
    * <p>
    *    x<sup>DCM</sup> = x<sup>CoM</sup> + 1&frasl;&omega;  x&#775;<sup>CoM</sup>
    * </p>
    *
    * @param desiredCenterOfMassPosition
    * @param desiredCenterOfMassVelocity
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param desiredDCMToPack
    */
   public static void computeDesiredDCMPosition(FramePoint3DReadOnly desiredCenterOfMassPosition, FrameVector3DReadOnly desiredCenterOfMassVelocity,
                                                         double omega0, FixedFramePoint3DBasics desiredDCMToPack)
   {
      desiredDCMToPack.scaleAdd(1.0 / omega0, desiredCenterOfMassVelocity, desiredCenterOfMassPosition);
   }

   /**
    * Computes the desired DCM velocity by
    * <p>
    *    x&#775;<sup>DCM</sup> = x&#775;<sup>CoM</sup> + 1&frasl;&omega;  x&#776;<sup>CoM</sup>
    * </p>
    *
    * @param desiredCenterOfMassAcceleration
    * @param desiredCenterOfMassAcceleration
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param desiredDCMVelocityToPack
    */
   public static void computeDesiredDCMVelocity(FrameVector3DReadOnly desiredCenterOfMassVelocity, FrameVector3DReadOnly desiredCenterOfMassAcceleration,
                                                double omega0, FixedFrameVector3DBasics desiredDCMVelocityToPack)
   {
      desiredDCMVelocityToPack.scaleAdd(1.0 / omega0, desiredCenterOfMassAcceleration, desiredCenterOfMassVelocity);
   }

   /**
    * Computes the desired VRP by
    * <p>
    *    x<sup>VRP</sup> = x<sup>DCM</sup> - 1&frasl;&omega;  x&#775;<sup>DCM</sup>
    * </p>
    *
    * @param desiredDCMPosition
    * @param desiredDCMVelocity
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param desiredVRPToPack
    */
   public static void computeDesiredVRPPosition(FramePoint3DReadOnly desiredDCMPosition, FrameVector3DReadOnly desiredDCMVelocity, double omega0,
                                                FixedFramePoint3DBasics desiredVRPToPack)
   {
      desiredVRPToPack.scaleAdd(-1.0 / omega0, desiredDCMVelocity, desiredDCMPosition);
   }

   public static void computeDesiredVRPPositionFromCoM(FramePoint3DReadOnly desiredCoMPosition, FrameVector3DReadOnly desiredCoMAcceleration, double omega0,
                                                FixedFramePoint3DBasics desiredVRPToPack)
   {
      desiredVRPToPack.scaleAdd(-1.0 / MathTools.square(omega0), desiredCoMAcceleration, desiredCoMPosition);
   }
}
