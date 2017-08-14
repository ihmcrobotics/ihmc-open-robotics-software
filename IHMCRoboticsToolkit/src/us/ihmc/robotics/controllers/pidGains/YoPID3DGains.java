package us.ihmc.robotics.controllers.pidGains;

import us.ihmc.yoVariables.variable.YoDouble;

public interface YoPID3DGains extends PID3DGains
{
   public abstract YoDouble getYoMaximumFeedback();

   public abstract YoDouble getYoMaximumFeedbackRate();

   public abstract YoDouble getYoMaximumDerivativeError();

   public abstract YoDouble getYoMaximumProportionalError();
}
