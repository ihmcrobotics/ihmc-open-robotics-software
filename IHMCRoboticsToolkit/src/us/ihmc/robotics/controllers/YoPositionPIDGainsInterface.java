package us.ihmc.robotics.controllers;

import us.ihmc.robotics.controllers.pidGains.PID3DGains;
import us.ihmc.yoVariables.variable.YoDouble;

public interface YoPositionPIDGainsInterface extends PID3DGains
{
   public abstract YoDouble getYoMaximumFeedback();

   public abstract YoDouble getYoMaximumFeedbackRate();

   public abstract YoDouble getYoMaximumDerivativeError();

   public abstract YoDouble getYoMaximumProportionalError();
}
