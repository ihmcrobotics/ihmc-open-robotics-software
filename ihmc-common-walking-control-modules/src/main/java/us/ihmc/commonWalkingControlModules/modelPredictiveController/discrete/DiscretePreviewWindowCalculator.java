package us.ihmc.commonWalkingControlModules.modelPredictiveController.discrete;

import us.ihmc.commonWalkingControlModules.modelPredictiveController.ContactPlaneProvider;
import us.ihmc.commonWalkingControlModules.modelPredictiveController.PreviewWindowCalculator;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.List;

public class DiscretePreviewWindowCalculator extends PreviewWindowCalculator
{
   private final YoDouble orientationPreviewWindowDuration = new YoDouble("orientationPreviewWindowDuration", registry);
   private final YoDouble maximumOrientationPreviewWindowDuration = new YoDouble("maximumOrientationPreviewWindowDuration", registry);

   public DiscretePreviewWindowCalculator(YoRegistry parentRegistry)
   {
      super(parentRegistry);
   }

   @Override
   public void compute(List<ContactPlaneProvider> fullContactSequence, double timeAtStartOfWindow)
   {
      super.compute(fullContactSequence, timeAtStartOfWindow);

      orientationPreviewWindowDuration.set(Math.min(maximumOrientationPreviewWindowDuration.getDoubleValue(), getPreviewWindowDuration()));
   }

   public double getOrientationPreviewWindowDuration()
   {
      return orientationPreviewWindowDuration.getDoubleValue();
   }
}
