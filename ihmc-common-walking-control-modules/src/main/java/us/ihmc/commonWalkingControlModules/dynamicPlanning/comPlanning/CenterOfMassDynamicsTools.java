package us.ihmc.commonWalkingControlModules.dynamicPlanning.comPlanning;

import us.ihmc.commonWalkingControlModules.capturePoint.CapturePointTools;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFramePoint3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FixedFrameVector3DBasics;
import us.ihmc.euclid.referenceFrame.interfaces.FramePoint3DReadOnly;
import us.ihmc.euclid.referenceFrame.interfaces.FrameVector3DReadOnly;

public class CenterOfMassDynamicsTools
{
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
      CapturePointTools.computeDesiredCapturePointPosition(omega0, time, initialDCM, initialVRP, desiredDCMToPack);
   }

   /**
    * Compute the desired CoM position at a given time, assuming a constant VRP.
    * <p>
    * x (t) = 0.5 (e<sup>&omega; t</sup> - e<sup>-&omega; t</sup>) &xi;<sub>0</sub> +
    * + e<sup>-&omega; t</sup> x<sub>0</sub> + (1 - 0.5 (e<sup>&omega; t</sup> + e<sup>-&omega; t</sup>)) x<sub>vrp</sub>(t<sub>0</sub>)
    * </p>
    *
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param time desired time relative to the start of this time function. t<sub>1</sub> - t<sub>0</sub> in the above equation.
    * @param initialCoM CoM at the start of this time function. x<sub>0</sub> in the above equation.
    * @param initialDCM DCM at the start of this time function. &xi;<sub>0</sub> in the above equation.
    * @param initialVRP VRP at the start of this time function x<sub>vrp</sub>(t<sub>0</sub>) in the above equation.
    * @param desiredCoMToPack CoM at the time {@param time} relative to the start of this time function. x(t) in the above equation.
    */
   public static void computeDesiredCoMPositionForwardTime(double omega0, double time, FramePoint3DReadOnly initialCoM, FramePoint3DReadOnly initialDCM,
                                                           FramePoint3DReadOnly initialVRP, FixedFramePoint3DBasics desiredCoMToPack)
   {
      double exponential = Math.exp(omega0 * time);
      double negativeExponential = 1.0 / exponential;

      desiredCoMToPack.set(initialCoM);
      desiredCoMToPack.scale(negativeExponential);

      desiredCoMToPack.scaleAdd(0.5 * (exponential - negativeExponential), initialDCM, desiredCoMToPack);
      desiredCoMToPack.scaleAdd(1.0 - 0.5 * (exponential + negativeExponential), initialVRP, desiredCoMToPack);
   }

   /**
    * Compute the initial CoM position given the final conditions at time t, assuming a constant VRP.
    * <p>
    * x<sub>0</sub> = e<sup>&omega; t</sup> x(t) + 0.5 (e<sup>-&omega; t</sup> - e<sup>&omega; t</sup>) &xi; (t) + (1 - 0.5 (e<sup>&omega; t</sup> + e<sup>-&omega; t</sup>))x<sub>vrp</sub>
    * </p>
    *
    * @param omega0 the natural frequency &omega; =
    *           &radic;<span style="text-decoration:overline;">&nbsp; g / z0&nbsp;</span> of the
    *           biped.
    * @param time desired time of the end conditions. t in the above equation.
    * @param finalCoM CoM at the end of this time function. x(t) in the above equation.
    * @param finalDCM DCM at the end of this time function. &xi;(t) in the above equation.
    * @param initialVRP constant VRP in this time function. x<sub>vrp</sub> in the above equation.
    * @param desiredCoMToPack initial CoM  of this time function. x<sub>0</sub> in the above equation.
    */
   public static void computeDesiredCoMPositionBackwardTime(double omega0, double time, FramePoint3DReadOnly finalCoM, FramePoint3DReadOnly finalDCM,
                                                           FramePoint3DReadOnly initialVRP, FixedFramePoint3DBasics desiredCoMToPack)
   {
      double exponential = Math.exp(omega0 * time);
      double negativeExponential = 1.0 / exponential;

      desiredCoMToPack.set(finalCoM);
      desiredCoMToPack.scale(exponential);

      desiredCoMToPack.scaleAdd(0.5 * (negativeExponential - exponential), finalDCM, desiredCoMToPack);
      desiredCoMToPack.scaleAdd(1.0 - 0.5 * (exponential + negativeExponential), initialVRP, desiredCoMToPack);
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
      double sigmaT = computeLinearSigma(omega0, time, duration);
      double sigma0 = computeLinearSigma(omega0, 0.0, duration);
      double exponential = Math.exp(omega0 * time);

      double alpha = 1.0 - sigmaT - exponential * (1.0 - sigma0);
      double beta = sigmaT - exponential * sigma0;

      desiredDCMToPack.set(initialVRP);
      desiredDCMToPack.scale(alpha);
      desiredDCMToPack.scaleAdd(beta, finalVRP, desiredDCMToPack);
      desiredDCMToPack.scaleAdd(exponential, initialDCM, desiredDCMToPack);
   }

   public static void computeDesiredDCMPositionBackwardTime(double omega0, double time, double duration, FramePoint3DReadOnly finalDCM,
                                                            FramePoint3DReadOnly initialVRP, FramePoint3DReadOnly finalVRP, FixedFramePoint3DBasics desiredDCMToPack)
   {
      /*
      double sigmaT = computeLinearSigma(omega0, time, duration);
      double sigmaF = computeLinearSigma(omega0, duration, duration);
      double exponential = Math.exp(omega0 * (time - duration));

      double alpha = 1.0 - sigmaT - exponential * (1.0 - sigmaF);
      double beta = sigmaT - exponential * sigmaF;

      desiredDCMToPack.set(initialVRP);
      desiredDCMToPack.scale(alpha);
      desiredDCMToPack.scaleAdd(beta, finalVRP, desiredDCMToPack);
      desiredDCMToPack.scaleAdd(exponential, finalDCM, desiredDCMToPack);
      */

      double exponential = Math.exp(-omega0 * time);
      desiredDCMToPack.interpolate(initialVRP, finalDCM, exponential);

      double linearFactor = (-time / duration * exponential + 1.0 / (omega0 * duration) * (1.0 - exponential));
      desiredDCMToPack.scaleAdd(linearFactor, finalVRP, desiredDCMToPack);
      desiredDCMToPack.scaleAdd(-linearFactor, initialVRP, desiredDCMToPack);
   }

   public static void computeDesiredCoMPositionForwardTime(double omega0, double time, double duration, FramePoint3DReadOnly initialCoM,
                                                           FramePoint3DReadOnly initialDCM, FramePoint3DReadOnly initialVRP, FramePoint3DReadOnly finalVRP,
                                                           FixedFramePoint3DBasics desiredCoMToPack)
   {
      double wt = omega0 * time;
      double negativeExponential = Math.exp(-wt);
      double sinh = Math.sinh(wt);
      double cosh = Math.cosh(wt);


      desiredCoMToPack.set(initialCoM);
      desiredCoMToPack.scale(negativeExponential);

      desiredCoMToPack.scaleAdd(sinh, initialDCM, desiredCoMToPack);
      desiredCoMToPack.scaleAdd(1.0 / (omega0 * duration) * sinh - cosh - time / duration + 1.0, initialVRP, desiredCoMToPack);
      desiredCoMToPack.scaleAdd(time / duration - 1.0 / (omega0 * duration) * sinh, finalVRP, desiredCoMToPack);
   }

   public static void computeDesiredCoMPositionBackwardTime(double omega0, double time, double duration, FramePoint3DReadOnly finalCoM,
                                                            FramePoint3DReadOnly finalDCM, FramePoint3DReadOnly initialVRP, FramePoint3DReadOnly finalVRP,
                                                            FixedFramePoint3DBasics desiredCoMToPack)
   {
      double wt = omega0 * time;
      double exponential = Math.exp(wt);
      double sinh = Math.sinh(wt);

      desiredCoMToPack.set(finalCoM);
      desiredCoMToPack.scale(exponential);

      desiredCoMToPack.scaleAdd(-sinh, finalDCM, desiredCoMToPack);


      double initialCoefficient = exponential * (1.0 - time / duration - exponential) + sinh * (2.0 / (omega0 * duration) * exponential - 1.0 / (omega0 * duration) - time / duration + 1.0);
      double finalCoefficient = time / duration * exponential + sinh * (1.0 / (omega0 * duration) + time / duration - 2.0 / (omega0 * duration) * exponential);
      desiredCoMToPack.scaleAdd(initialCoefficient, initialVRP, desiredCoMToPack);
      desiredCoMToPack.scaleAdd(finalCoefficient, finalVRP, desiredCoMToPack);
   }

   private static double computeLinearSigma(double omega0, double time, double duration)
   {
      return time / duration + 1.0 / (omega0 * duration);
   }

   private static double computeCubicSigma(double omega0, double time, double duration)
   {
      double t2 = time * time;
      double T2 = duration * duration;
      double T3 = duration * T2;
      double w2 = omega0 * omega0;
      double w3 = omega0 * w2;

      return t2 / T2 * (3 - 2 * time / duration) + 6.0 / omega0 * (time / T2) * (1.0 - time / duration) + 6.0 / (w2 * T2) * (1.0  - 2.0 * time / duration)
            - 12.0 / (w3 * T3);
   }

}
