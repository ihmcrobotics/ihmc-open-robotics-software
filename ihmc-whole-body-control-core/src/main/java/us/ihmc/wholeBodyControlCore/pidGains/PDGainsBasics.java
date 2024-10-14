package us.ihmc.wholeBodyControlCore.pidGains;

import us.ihmc.euclid.interfaces.Settable;

public interface PDGainsBasics extends PDGainsReadOnly, Settable<PDGainsReadOnly>
{
   void setKp(double kp);

   void setKd(double kd);

   void setMaximumFeedback(double maximumFeedback);

   void setMaximumFeedbackRate(double maximumFeedbackRate);

   void setPositionDeadband(double positionDeadband);

   default void set(double kp, double kd, double maxFeedback, double maxFeedbackRate)
   {
      set(kp, kd, maxFeedback, maxFeedbackRate, 0.0);
   }

   @Override
   default void set(PDGainsReadOnly other)
   {
      set(other.getKp(), other.getKd(), other.getMaximumFeedback(), other.getMaximumFeedbackRate(), other.getPositionDeadband());
   }

   default void set(double kp, double kd, double maxFeedback, double maxFeedbackRate, double positionDeadband)
   {
      setKp(kp);
      setKd(kd);
      setMaximumFeedback(maxFeedback);
      setMaximumFeedbackRate(maxFeedbackRate);
      setPositionDeadband(positionDeadband);
   }
}