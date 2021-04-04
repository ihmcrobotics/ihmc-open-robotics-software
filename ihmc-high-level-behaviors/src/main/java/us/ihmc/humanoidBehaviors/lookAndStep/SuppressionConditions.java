package us.ihmc.humanoidBehaviors.lookAndStep;

import us.ihmc.tools.string.StringTools;

import java.util.function.Supplier;

public class SuppressionConditions
{
   public static SuppressionCondition neckPitchWithCorrection(Supplier<Double> currentNeckPitch,
                                                              Supplier<Double> desiredNeckPitch,
                                                              Supplier<Double> epsilon,
                                                              Runnable correctiveAction)
   {
      return new SuppressionCondition(() -> StringTools.format3D("Neck at wrong angle: {} != {} +/- {}",
                                                                 currentNeckPitch.get(),
                                                                 desiredNeckPitch.get(),
                                                                 epsilon.get())
                                                       .get(),
                                      () -> Math.abs(currentNeckPitch.get() - desiredNeckPitch.get()) > epsilon.get(),
                                      correctiveAction);
   }

}
