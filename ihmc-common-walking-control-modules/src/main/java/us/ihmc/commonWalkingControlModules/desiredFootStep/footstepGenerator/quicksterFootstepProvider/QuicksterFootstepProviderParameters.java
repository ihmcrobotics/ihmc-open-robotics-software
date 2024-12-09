package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator.quicksterFootstepProvider;

import us.ihmc.commons.MathTools;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import java.util.ArrayList;
import java.util.List;

public class QuicksterFootstepProviderParameters
{
   // Static variables
   private static final double SWING_DURATION = 0.55;
   private static final double DOUBLE_SUPPORT_FRACTION = 0.05;
   private static final double STANCE_WIDTH = 0.2;
   private static final double SWING_HEIGHT = 0.09;
   private static final double COM_HEIGHT = 0.9;
   private static final double POLE = 0.0;
   private static final double MAX_ACCELERATION_PER_STEP = 0.075;
   private static final double MAX_DECELERATION_PER_STEP = -0.2;

   // YoVariables
   private final YoDouble swingDuration;
   private final YoDouble doubleSupportFraction;
   private final YoDouble stanceWidth;
   private final YoDouble swingHeight;
   private final YoDouble comHeight;
   private final YoDouble pole;
   private final YoDouble maxAccelerationPerStep;
   private final YoDouble maxDecelerationPerStep;
   private final YoDouble omega;

   // Stageable YoVariables
   private final SideDependentList<StageableYoDouble> swingDurationCurrentStep = new SideDependentList<>();
   private final SideDependentList<StageableYoDouble> doubleSupportFractionCurrentStep = new SideDependentList<>();
   private final SideDependentList<StageableYoDouble> stanceWidthCurrentStep = new SideDependentList<>();
   private final SideDependentList<StageableYoDouble> swingHeightCurrentStep = new SideDependentList<>();
   private final SideDependentList<StageableYoDouble> comHeightCurrentStep = new SideDependentList<>();
   private final SideDependentList<StageableYoDouble> poleCurrentStep = new SideDependentList<>();
   private final SideDependentList<StageableYoDouble> maxAccelerationPerStepCurrentStep = new SideDependentList<>();
   private final SideDependentList<StageableYoDouble> maxDecelerationPerStepCurrentStep = new SideDependentList<>();
   private final SideDependentList<StageableYoDouble> omegaCurrentStep = new SideDependentList<>();

   private final SideDependentList<List<StageableYoDouble>> stageableYoDoubles = new SideDependentList<>();

   public QuicksterFootstepProviderParameters(double gravityZ, YoRegistry parentRegistry)
   {
      YoRegistry registry = new YoRegistry(getClass().getSimpleName());
      String suffix2 = "QFP";

      swingDuration = new YoDouble("swingDuration" + suffix2, registry);
      doubleSupportFraction = new YoDouble("doubleSupportFraction" + suffix2, registry);
      stanceWidth = new YoDouble("stanceWidth" + suffix2, registry);
      swingHeight = new YoDouble("desiredSwingHeight" + suffix2, registry);
      comHeight = new YoDouble("desiredComHeight" + suffix2, registry);
      pole = new YoDouble("pole" + suffix2, registry);
      maxAccelerationPerStep = new YoDouble("maxAccelerationPerStep" + suffix2, registry);
      maxDecelerationPerStep = new YoDouble("maxDecelerationPerStep" + suffix2, registry);
      omega = new YoDouble("omega" + suffix2, registry);

      swingDuration.set(SWING_DURATION);
      doubleSupportFraction.set(DOUBLE_SUPPORT_FRACTION);
      stanceWidth.set(STANCE_WIDTH);
      swingHeight.set(SWING_HEIGHT);
      comHeight.set(COM_HEIGHT);
      pole.set(POLE);
      maxAccelerationPerStep.set(MAX_ACCELERATION_PER_STEP);
      maxDecelerationPerStep.set(MAX_DECELERATION_PER_STEP);
      omega.set(Math.sqrt(Math.abs(gravityZ / comHeight.getDoubleValue())));

      for (RobotSide robotSide : RobotSide.values)
      {
         String suffix = robotSide.getPascalCaseName() + "CurrentStep";
         stageableYoDoubles.put(robotSide, new ArrayList<>());

         swingDurationCurrentStep.put(robotSide, createStageableYoDouble(robotSide, "swingDuration", suffix + suffix2, swingDuration, registry));
         doubleSupportFractionCurrentStep.put(robotSide, createStageableYoDouble(robotSide, "doubleSupportFraction", suffix + suffix2, doubleSupportFraction, registry));
         stanceWidthCurrentStep.put(robotSide, createStageableYoDouble(robotSide, "stanceWidthCurrentStep", suffix + suffix2, stanceWidth, registry));
         swingHeightCurrentStep.put(robotSide, createStageableYoDouble(robotSide, "desiredSwingHeight", suffix + suffix2, swingHeight, registry));
         comHeightCurrentStep.put(robotSide, createStageableYoDouble(robotSide, "desiredComHeight", suffix + suffix2, comHeight, registry));
         poleCurrentStep.put(robotSide, createStageableYoDouble(robotSide, "pole", suffix + suffix2, pole, registry));
         maxAccelerationPerStepCurrentStep.put(robotSide, createStageableYoDouble(robotSide, "maxAccelerationPerStep", suffix + suffix2, maxAccelerationPerStep, registry));
         maxDecelerationPerStepCurrentStep.put(robotSide, createStageableYoDouble(robotSide, "maxDecelerationPerStep", suffix + suffix2, maxDecelerationPerStep, registry));
         omegaCurrentStep.put(robotSide,  createStageableYoDouble(robotSide, "omega", suffix + suffix2, omega, registry));

      }

      comHeight.addListener(change -> omega.set(Math.sqrt(Math.abs(gravityZ / comHeight.getDoubleValue()))));

      parentRegistry.addChild(registry);
   }

   private StageableYoDouble createStageableYoDouble(RobotSide robotSide, String prefix, String suffix, YoDouble valueToWatch, YoRegistry registry)
   {
      StageableYoDouble stageableYoDouble = new StageableYoDouble(prefix, suffix, valueToWatch, registry);
      stageableYoDoubles.get(robotSide).add(stageableYoDouble);

      return stageableYoDouble;
   }

   public void setParametersForUpcomingSwing(RobotSide swingSide)
   {
      for (int i = 0; i < stageableYoDoubles.get(swingSide).size(); i++)
         stageableYoDoubles.get(swingSide).get(i).loadFromStage();
   }

   public void setParametersForUpcomingSwing(RobotSide swingSide, double swingHeight, double swingDuration, double doubleSupportFraction)
   {
      for (int i = 0; i < stageableYoDoubles.get(swingSide).size(); i++)
         stageableYoDoubles.get(swingSide).get(i).loadFromStage();

      if (!Double.isNaN(swingHeight) && swingHeight >= 0.02)
         this.swingHeightCurrentStep.get(swingSide).set(swingHeight);
      if (!Double.isNaN(swingDuration) && MathTools.intervalContains(swingDuration, 0.2, 1.0))
         this.swingDurationCurrentStep.get(swingSide).set(swingDuration);
      if (!Double.isNaN(doubleSupportFraction) && MathTools.intervalContains(doubleSupportFraction, 0.0, 0.5))
         this.doubleSupportFractionCurrentStep.get(swingSide).set(doubleSupportFraction);
   }

   public YoDouble getSwingDuration(RobotSide robotSide)
   {
      return swingDurationCurrentStep.get(robotSide);
   }

   public YoDouble getDoubleSupportFraction(RobotSide robotSide)
   {
      return doubleSupportFractionCurrentStep.get(robotSide);
   }

   public YoDouble getStanceWidth(RobotSide robotSide)
   {
      return stanceWidthCurrentStep.get(robotSide);
   }

   public YoDouble getSwingHeight(RobotSide robotSide)
   {
      return swingHeightCurrentStep.get(robotSide);
   }

   public YoDouble getDesiredCoMHeight(RobotSide robotSide)
   {
      return comHeightCurrentStep.get(robotSide);
   }

   public YoDouble getPole(RobotSide robotSide)
   {
      return poleCurrentStep.get(robotSide);
   }

   public YoDouble getMaxAccelerationPerStepCurrentStep(RobotSide robotSide)
   {
      return maxAccelerationPerStepCurrentStep.get(robotSide);
   }

   public YoDouble getMaxDecelerationPerStepCurrentStep(RobotSide robotSide)
   {
      return maxDecelerationPerStepCurrentStep.get(robotSide);
   }

   public YoDouble getOmega(RobotSide robotSide)
   {
      return omegaCurrentStep.get(robotSide);
   }

   private static class StageableYoDouble extends YoDouble
   {
      private double stagedValue;
      private final YoDouble valueToWatch;

      public StageableYoDouble(String prefix, String suffix, YoDouble valueToWatch, YoRegistry registry)
      {
         super(prefix + suffix, registry);
         this.valueToWatch = valueToWatch;
         stagedValue = valueToWatch.getDoubleValue();
         loadFromStage();
         valueToWatch.addListener((v) -> stageValue());
      }

      private void stageValue()
      {
         if (stagedValue != valueToWatch.getDoubleValue())
            stagedValue = valueToWatch.getDoubleValue();
      }

      public void loadFromStage()
      {
         if (getDoubleValue() != stagedValue)
            set(stagedValue);
      }
   }
}