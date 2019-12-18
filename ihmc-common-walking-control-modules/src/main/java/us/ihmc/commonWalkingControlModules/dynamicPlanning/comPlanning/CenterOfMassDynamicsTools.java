package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;

public class CenterOfMassDynamicsTools
{
   private static final double EPSILON = 1.0e-15;

   /**
    * Compute the desired DCM position at a given time, assuming a constant VRP.
    * &xi; (t<sub>1</sub>) = e<sup>&omega; (t<sub>1</sub> - t<sub>0</sub>)</sup>
    * &xi; (t<sub>0</sub>) + (1 - e<sup>&omega; (t<sub>1</sub> - t<sub>0</sub>)</sup>) x<sub>vrp</sub>(t<sub>0</sub>)
    *
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param time desired time relative to the start of this time function. t<sub>1</sub> - t<sub>0</sub> in the above equation.
    * @param initialDCM DCM at the start of this time function. &xi; (t<sub>0</sub>) in the above equation.
    * @param initialVRP VRP at the start of this time function x<sub>vrp</sub>(t<sub>0</sub>) in the above equation.
    * @param desiredDCMToPack DCM at the time {@param time} relative to the start of this time function. &xi; (t<sub>1</sub>) in the above equation.
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
    * Compute the desired DCM position at a given time, assuming a constant VRP, and assuming that a positive time is greater than the time at t = 0
    * &xi; (t) = e<sup>&omega; t</sup> &xi; (0) + (1 - e<sup>&omega; (t)</sup>) x<sub>vrp</sub>(0) + (t / T + 1 / &omega; T (1 - e<sup>&omega; t</sup>)) (x<sub>vrp</sub>(T) - x<sub>vrp</sub>(0))
    *
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param time desired time relative to the start of this time function. t in the above equation.
    * @param duration duration of the time function. T in the above equation.
    * @param initialDCM DCM at the start of this time function. &xi; (0) in the above equation.
    * @param initialVRP VRP at the start of this time function x<sub>vrp</sub>(0) in the above equation.
    * @param finalVRP VRP at the end of this time function x<sub>vrp</sub>(T) in the above equation.
    * @param desiredDCMToPack DCM at the time {@param time} relative to the start of this time function. &xi; (t<sub>1</sub>) in the above equation.
    */
   public static void computeDesiredDCMPositionForwardTime(double omega0, double time, double duration, FramePoint3DReadOnly initialDCM,
                                                           FramePoint3DReadOnly initialVRP, FramePoint3DReadOnly finalVRP,
                                                           FixedFramePoint3DBasics desiredDCMToPack)
   {
      double exponential = Math.exp(omega0 * time);
      desiredDCMToPack.interpolate(initialVRP, initialDCM, exponential);

      double linearFactor = (time / duration + 1.0 / (omega0 * duration) * (1.0 - exponential));
      desiredDCMToPack.scaleAdd(linearFactor, finalVRP, desiredDCMToPack);
      desiredDCMToPack.scaleAdd(-linearFactor, initialVRP, desiredDCMToPack);
   }

   public static void computeDesiredDCMPositionBackwardTime(double omega0, double time, double duration, FramePoint3DReadOnly finalDCM,
                                                            FramePoint3DReadOnly initialVRP, FramePoint3DReadOnly finalVRP, FixedFramePoint3DBasics desiredDCMToPack)
   {
      double exponential = Math.exp(-omega0 * time);
      desiredDCMToPack.interpolate(initialVRP, finalDCM, exponential);

      double linearFactor = (-time / duration * exponential + 1.0 / (omega0 * duration) * (1.0 - exponential));
      desiredDCMToPack.scaleAdd(linearFactor, finalVRP, desiredDCMToPack);
      desiredDCMToPack.scaleAdd(-linearFactor, initialVRP, desiredDCMToPack);
   }

}
