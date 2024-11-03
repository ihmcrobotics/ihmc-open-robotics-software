package us.ihmc.behaviors.lookAndStep;

import us.ihmc.tools.string.StringTools;

import java.util.function.Supplier;

/**
 * Predefined suppression conditions.
 */
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
