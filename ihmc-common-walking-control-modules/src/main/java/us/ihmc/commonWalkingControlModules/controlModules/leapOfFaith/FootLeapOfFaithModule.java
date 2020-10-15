package us.ihmc.commonWalkingControlModules.controlModules.leapOfFaith;

import us.ihmc.commonWalkingControlModules.configurations.LeapOfFaithParameters;
import us.ihmc.commons.MathTools;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DReadOnly;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.parameters.DoubleParameter;
import us.ihmc.yoVariables.providers.BooleanProvider;
import us.ihmc.yoVariables.providers.DoubleProvider;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;

public class FootLeapOfFaithModule
{
   private static final String yoNamePrefix = "leapOfFaith";

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   
   private final YoDouble swingDuration;

   private final DoubleProvider fractionOfSwing;
   private final BooleanProvider scaleFootWeight;

   private final DoubleProvider verticalFootWeightScaleFactor;
   private final DoubleProvider horizontalFootWeightScaleFactor;

   private final YoDouble verticalFootWeightScaleFraction = new YoDouble(yoNamePrefix + "VerticalFootWeightScaleFraction", registry);
   private final YoDouble horizontalFootWeightScaleFraction = new YoDouble(yoNamePrefix + "HorizontalFootWeightScaleFraction", registry);
   private final DoubleProvider minimumHorizontalWeight;

   public FootLeapOfFaithModule(YoDouble swingDuration, LeapOfFaithParameters parameters, YoRegistry parentRegistry)
   {
      this.swingDuration = swingDuration;

      scaleFootWeight = new BooleanParameter(yoNamePrefix + "ScaleFootWeight", registry, parameters.scaleFootWeight());
      fractionOfSwing = new DoubleParameter(yoNamePrefix + "FractionOfSwingToScaleFootWeight", registry, parameters.getFractionOfSwingToScaleFootWeight());

      horizontalFootWeightScaleFactor = new DoubleParameter(yoNamePrefix + "HorizontalFootWeightScaleFactor", registry, parameters.getHorizontalFootWeightScaleFactor());
      verticalFootWeightScaleFactor = new DoubleParameter(yoNamePrefix + "VerticalFootWeightScaleFactor", registry, parameters.getVerticalFootWeightScaleFactor());
      minimumHorizontalWeight = new DoubleParameter(yoNamePrefix + "MinimumHorizontalFootWeight", registry, parameters.getMinimumHorizontalFootWeight());

      parentRegistry.addChild(registry);
   }

   public void compute(double currentTime)
   {
      horizontalFootWeightScaleFraction.set(1.0);
      verticalFootWeightScaleFraction.set(1.0);

      double exceededTime = Math.max(currentTime - fractionOfSwing.getValue() * swingDuration.getDoubleValue(), 0.0);

      if (exceededTime <= 0.0)
         return;

      if (scaleFootWeight.getValue())
      {
         double horizontalFootWeightScaleFraction = MathTools.clamp(1.0 - horizontalFootWeightScaleFactor.getValue() * exceededTime, 0.0, 1.0);
         double verticalFootWeightScaleFraction = Math.max(1.0, 1.0 + exceededTime * verticalFootWeightScaleFactor.getValue());

         this.horizontalFootWeightScaleFraction.set(horizontalFootWeightScaleFraction);
         this.verticalFootWeightScaleFraction.set(verticalFootWeightScaleFraction);
      }
   }

   public void scaleFootWeight(Vector3DReadOnly unscaledLinearWeight, Vector3DBasics scaledLinearWeightToPack)
   {
      scaledLinearWeightToPack.set(unscaledLinearWeight);

      if (!scaleFootWeight.getValue())
         return;

      scaledLinearWeightToPack.scale(horizontalFootWeightScaleFraction.getDoubleValue());

      scaledLinearWeightToPack.setX(Math.max(minimumHorizontalWeight.getValue(), scaledLinearWeightToPack.getX()));
      scaledLinearWeightToPack.setY(Math.max(minimumHorizontalWeight.getValue(), scaledLinearWeightToPack.getY()));

      scaledLinearWeightToPack.setZ(unscaledLinearWeight.getZ() * verticalFootWeightScaleFraction.getDoubleValue());
   }
}
