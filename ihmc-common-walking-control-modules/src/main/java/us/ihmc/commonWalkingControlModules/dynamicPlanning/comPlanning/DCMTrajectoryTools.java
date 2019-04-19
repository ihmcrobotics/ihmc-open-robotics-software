package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;

/**
 * This is a tools class used to help compute a desired DCM position using either a constant or linearly moving VRP waypoint. This is useful
 * for validating the outputs from {@link CoMTrajectoryPlanner}.
 */
public class DCMTrajectoryTools
{
   private static final double EPSILON = 1.0e-15;

   /**
    * Compute the desired capture point position at a given time.
    * <p>
    *    &xi;(t) = &xi;<sub>0</sub> + (1 - e<sup>&omega;0 t</sup>) x<sup>VRP<sub>0</sub></sup>
    * </p>
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param time time from now to compute. t in the above equation.
    * @param initialDCMPosition initial DCM position. &xi;<sub>0</sub> in the above equation
    * @param constantVRPPosition constant location of the DCM. x<sup>VRP<sub>0</sub></sup> in the above equation.
    * @param desiredDCMPosition DCM position at time t. &xi;(t) in the above equation.
    */
   public static void computeDCMUsingConstantVRP(double omega0, double time, FramePoint3DReadOnly initialDCMPosition, FramePoint3DReadOnly constantVRPPosition,
                                                 FixedFramePoint3DBasics desiredDCMPosition)
   {
      CapturePointTools.computeDesiredCapturePointPosition(omega0, time, initialDCMPosition, constantVRPPosition, desiredDCMPosition);
   }

   public static void computeDCMUsingLinearVRP(double omega0, double time, double timeAtFinalVRP, FramePoint3DReadOnly initialDCMPosition,
                                               FramePoint3DReadOnly initialVRPPosition, FramePoint3DReadOnly finalVRPPosition,
                                               FixedFramePoint3DBasics desiredDCMPosition)
   {
      if (initialDCMPosition == desiredDCMPosition)
         throw new IllegalArgumentException("The initial DCM object must be different from the desired DCM object.");

      if (initialVRPPosition.distance(finalVRPPosition) > EPSILON)
      {
         double exponential = Math.exp(omega0 * time);
         double beta = 1.0 / (omega0 * timeAtFinalVRP);
         double phaseThrough = time / timeAtFinalVRP;
         desiredDCMPosition.interpolate(initialVRPPosition, finalVRPPosition, beta);
         desiredDCMPosition.interpolate(initialDCMPosition, exponential);
         desiredDCMPosition.scaleAdd(-phaseThrough, initialVRPPosition, desiredDCMPosition);
         desiredDCMPosition.scaleAdd(phaseThrough, finalVRPPosition, desiredDCMPosition);
      }
      else
      {
         computeDCMUsingConstantVRP(omega0, time, initialDCMPosition, initialVRPPosition, desiredDCMPosition);
      }
   }

}
