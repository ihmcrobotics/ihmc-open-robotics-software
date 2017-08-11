package us.ihmc.robotics.controllers;

import us.ihmc.robotics.controllers.pidGains.OrientationPIDGainsInterface;
import us.ihmc.yoVariables.variable.YoDouble;

public interface YoOrientationPIDGainsInterface extends OrientationPIDGainsInterface
{
   public abstract YoDouble getYoMaximumFeedback();

   public abstract YoDouble getYoMaximumFeedbackRate();

   public abstract YoDouble getYoMaximumDerivativeError();

   public abstract YoDouble getYoMaximumProportionalError();
}
