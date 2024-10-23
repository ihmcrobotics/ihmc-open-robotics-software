package us.ihmc.wholeBodyControlCore.pidGains;

import us.ihmc.yoVariables.providers.DoubleProvider;

public interface PDGainsReadOnly
{
   /**
    * Get the proportional gain for a PD (PID) controller.
    * 
    * @return the proportional gain
    */
   double getKp();

   /**
    * Get the derivative gain for a PD (PID) controller.
    * 
    * @return the derivative gain
    */
   double getKd();

   /**
    * Get the maximum feedback that the controller can use. This is equivalent to limiting the maximum
    * output for a PD or PID controller. Not all PD controller implementations use this parameter.
    * <p>
    * To disable feedback limiting set to {@link Double#POSITIVE_INFINITY}
    * </p>
    * 
    * @return the maximum feedback
    */
   double getMaximumFeedback();

   /**
    * Get the maximum rate of feedback that the controller can return. Not all PD controller
    * implementations use this parameter.
    * <p>
    * To disable feedback rate limiting set to {@link Double#POSITIVE_INFINITY}
    * </p>
    * 
    * @return the maximum feedback rate
    */
   double getMaximumFeedbackRate();

   /**
    * Get a provider of the maximum feedback that the controller can use. This is equivalent to limiting the maximum
    * output for a PD or PID controller. Not all PD controller implementations use this parameter.
    *
    * @return the maximum feedback provider
    */
   DoubleProvider getMaximumFeedbackProvider();

   /**
    * Get a provider of the maximum rate of feedback that the controller can return. Not all PD controller
    * implementations use this parameter.
    *
    * @return the maximum feedback rate provider
    */
   DoubleProvider getMaximumFeedbackRateProvider();

   /**
    * Get a position deadband for the controller. If the magnitude of the position error is below this
    * value it will be set to zero. Not all PD controller implementations use this parameter.
    * <p>
    * To disable the deadband set to {@code 0.0}
    * </p>
    * 
    * @return the position deadband
    */
   double getPositionDeadband();

   default boolean equals(PDGainsReadOnly other)
   {
      if (other == null)
      {
         return false;
      }
      else if (other == this)
      {
         return true;
      }
      else
      {
         if (getKp() != other.getKp())
            return false;
         if (getKd() != other.getKd())
            return false;
         if (getMaximumFeedback() != other.getMaximumFeedback())
            return false;
         if (getMaximumFeedbackRate() != other.getMaximumFeedbackRate())
            return false;
         if (getPositionDeadband() != other.getPositionDeadband())
            return false;

         return true;
      }
   }
}