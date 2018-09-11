package us.ihmc.commonWalkingControlModules.controlModules.leapOfFaith;

import us.ihmc.commonWalkingControlModules.configurations.LeapOfFaithParameters;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class FootLeapOfFaithModule
{
   private static final String yoNamePrefix = "leapOfFaith";

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());


   private final YoDouble swingDuration;

   private final DoubleProvider fractionOfSwing;
   private final BooleanProvider scaleFootWeight;

   private final DoubleProvider verticalFootWeightScaleFactor;
   private final DoubleProvider horizontalFootWeightScaleFactor;

   private final YoDouble verticalFootWeightScaleFraction = new YoDouble(yoNamePrefix + "VerticalFootWeightScaleFraction", registry);
   private final YoDouble horizontalFootWeightScaleFraction = new YoDouble(yoNamePrefix + "HorizontalFootWeightScaleFraction", registry);

   public FootLeapOfFaithModule(YoDouble swingDuration, LeapOfFaithParameters parameters, YoVariableRegistry parentRegistry)
   {
      this.swingDuration = swingDuration;

      if (parameters == null)
      {
         scaleFootWeight = new BooleanParameter(yoNamePrefix + "ScaleFootWeight", registry, false);
         fractionOfSwing = new DoubleParameter(yoNamePrefix + "FractionOfSwingToScaleFootWeight", registry, 1.0);

         verticalFootWeightScaleFactor = new DoubleParameter(yoNamePrefix + "VerticalFootWeightScaleFactor", registry, 1.0);
         horizontalFootWeightScaleFactor = new DoubleParameter(yoNamePrefix + "HorizontalFootWeightScaleFactor", registry, 1.0);
      }
      else
      {
         scaleFootWeight = new BooleanParameter(yoNamePrefix + "ScaleFootWeight", registry, parameters.scaleFootWeight());
         fractionOfSwing = new DoubleParameter(yoNamePrefix + "FractionOfSwingToScaleFootWeight", registry, parameters.getFractionOfSwingToScaleFootWeight());

         verticalFootWeightScaleFactor = new DoubleParameter(yoNamePrefix + "VerticalFootWeightScaleFactor", registry, parameters.getVerticalFootWeightScaleFactor());
         horizontalFootWeightScaleFactor = new DoubleParameter(yoNamePrefix + "HorizontalFootWeightScaleFactor", registry, parameters.getHorizontalFootWeightScaleFactor());
      }

      parentRegistry.addChild(registry);
   }

   public void compute(double currentTime)
   {
      horizontalFootWeightScaleFraction.set(1.0);
      verticalFootWeightScaleFraction.set(1.0);

      double timeToScaleWith = Math.max(currentTime - fractionOfSwing.getValue() * swingDuration.getDoubleValue(), 0.0);

      if (timeToScaleWith == 0.0)
         return;

      if (scaleFootWeight.getValue())
      {
         double horizontalFootWeightScaleFraction = Math.max(1.0, 1.0 + timeToScaleWith * horizontalFootWeightScaleFactor.getValue());
         double verticalFootWeightScaleFraction = Math.max(1.0, 1.0 + timeToScaleWith * verticalFootWeightScaleFactor.getValue());

         this.horizontalFootWeightScaleFraction.set(horizontalFootWeightScaleFraction);
         this.verticalFootWeightScaleFraction.set(verticalFootWeightScaleFraction);
      }
   }

   public void scaleFootWeight(Vector3DReadOnly unscaledLinearWeight, Vector3DBasics scaledLinearWeight)
   {
      scaledLinearWeight.set(unscaledLinearWeight);

      if (!scaleFootWeight.getValue())
         return;

      scaledLinearWeight.scale(horizontalFootWeightScaleFraction.getDoubleValue());

      double verticalWeight = unscaledLinearWeight.getZ() * verticalFootWeightScaleFraction.getDoubleValue();
      scaledLinearWeight.setZ(verticalWeight);
   }
}
