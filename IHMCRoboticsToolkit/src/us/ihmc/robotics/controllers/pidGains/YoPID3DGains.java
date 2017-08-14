package us.ihmc.robotics.controllers.pidGains;

import us.ihmc.yoVariables.variable.YoDouble;

/**
 * An extension of the {@link PID3DGains} interface that provides additional access
 * to YoVariables in the implementation.
 */
public interface YoPID3DGains extends PID3DGains
{
   /**
    * Returns the maximum allowed controller output as a YoVariable.
    *
    * @return the maximum output.
    */
   public abstract YoDouble getYoMaximumFeedback();

   /**
    * Returns the maximum allowed controller output rate as a YoVariable.
    *
    * @return the maximum output rate.
    */
   public abstract YoDouble getYoMaximumFeedbackRate();

   /**
    * Returns the maximum error in the derivative input to the controller that
    * should be considered as a YoVariable.
    *
    * @return the maximum derivative error.
    */
   public abstract YoDouble getYoMaximumDerivativeError();

   /**
    * Returns the maximum error in the proportional input to the controller that
    * should be considered as a YoVariable.
    *
    * @return the maximum proportional error.
    */
   public abstract YoDouble getYoMaximumProportionalError();
}
