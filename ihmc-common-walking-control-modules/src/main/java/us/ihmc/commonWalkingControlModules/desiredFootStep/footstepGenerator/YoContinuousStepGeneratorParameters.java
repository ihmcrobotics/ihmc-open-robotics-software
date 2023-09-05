package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator;

import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoInteger;

public class YoContinuousStepGeneratorParameters implements ContinuousStepGeneratorParametersBasics
{
   private final YoInteger numberOfFootstepsToPlan;
   private final YoInteger numberOfFixedFootsteps;
   private final YoDouble swingHeight;
   private final YoDouble swingDuration, transferDuration;
   private final YoDouble maxStepLength;
   private final YoDouble inPlaceWidth, minStepWidth, maxStepWidth;
   private final YoDouble turnMaxAngleInward, turnMaxAngleOutward;
   private final YoBoolean stepsAreAdjustable;
   private final YoBoolean shiftUpcomingStepsWithTouchdown;
   private final YoInteger ticksToUpdateTheEnvironment;

   public YoContinuousStepGeneratorParameters(String nameSuffix, YoRegistry registry)
   {
      numberOfFootstepsToPlan = new YoInteger("numberOfFootstepsToPlan" + nameSuffix, registry);
      numberOfFixedFootsteps = new YoInteger("numberOfFixedFootsteps" + nameSuffix, registry);
      swingHeight = new YoDouble("swingHeight" + nameSuffix, registry);
      swingDuration = new YoDouble("swingTime" + nameSuffix, registry);
      transferDuration = new YoDouble("transferTime" + nameSuffix, registry);
      inPlaceWidth = new YoDouble("inPlaceWidth" + nameSuffix, registry);
      minStepWidth = new YoDouble("minStepWidth" + nameSuffix, registry);
      maxStepWidth = new YoDouble("maxStepWidth" + nameSuffix, registry);
      maxStepLength = new YoDouble("maxStepLength" + nameSuffix, registry);
      turnMaxAngleOutward = new YoDouble("maxAngleTurnOutwards" + nameSuffix, registry);
      turnMaxAngleInward = new YoDouble("maxAngleTurnInwards" + nameSuffix, registry);
      stepsAreAdjustable = new YoBoolean("stepsAreAdjustable" + nameSuffix, registry);
      shiftUpcomingStepsWithTouchdown = new YoBoolean("shiftUpcomingStepsWithTouchdown" + nameSuffix, registry);
      ticksToUpdateTheEnvironment = new YoInteger("ticksToUpdateTheEnvironment" + nameSuffix, registry);
   }

   @Override
   public void setStepsAreAdjustable(boolean stepsAreAdjustable)
   {
      this.stepsAreAdjustable.set(stepsAreAdjustable);
   }

   @Override
   public void setShiftUpcomingStepsWithTouchdown(boolean shiftUpcomingStepsWithTouchdown)
   {
      this.shiftUpcomingStepsWithTouchdown.set(shiftUpcomingStepsWithTouchdown);
   }

   @Override
   public void setNumberOfFootstepsToPlan(int numberOfFootstepsToPlan)
   {
      this.numberOfFootstepsToPlan.set(numberOfFootstepsToPlan);
   }

   @Override
   public void setNumberOfFixedFootsteps(int numberOfFixedFootsteps)
   {
      this.numberOfFixedFootsteps.set(numberOfFixedFootsteps);
   }

   @Override
   public void setTicksToUpdateTheEnvironment(int ticksToUpdateTheEnvironment)
   {
      this.ticksToUpdateTheEnvironment.set(ticksToUpdateTheEnvironment);
   }

   @Override
   public void setSwingHeight(double swingHeight)
   {
      this.swingHeight.set(swingHeight);
   }

   @Override
   public void setSwingDuration(double swingDuration)
   {
      this.swingDuration.set(swingDuration);
   }

   @Override
   public void setTransferDuration(double transferDuration)
   {
      this.transferDuration.set(transferDuration);
   }

   @Override
   public void setMaxStepLength(double maxStepLength)
   {
      this.maxStepLength.set(maxStepLength);
   }

   @Override
   public void setDefaultStepWidth(double defaultStepWidth)
   {
      this.inPlaceWidth.set(defaultStepWidth);
   }

   @Override
   public void setMinStepWidth(double minStepWidth)
   {
      this.minStepWidth.set(minStepWidth);
   }

   @Override
   public void setMaxStepWidth(double maxStepWidth)
   {
      this.maxStepWidth.set(maxStepWidth);
   }

   @Override
   public void setTurnMaxAngleInward(double turnMaxAngleInward)
   {
      this.turnMaxAngleInward.set(turnMaxAngleInward);
   }

   @Override
   public void setTurnMaxAngleOutward(double turnMaxAngleOutward)
   {
      this.turnMaxAngleOutward.set(turnMaxAngleOutward);
   }

   @Override
   public int getNumberOfFootstepsToPlan()
   {
      return numberOfFootstepsToPlan.getValue();
   }

   @Override
   public int getNumberOfFixedFootsteps()
   {
      return numberOfFixedFootsteps.getValue();
   }

   @Override
   public int getTicksToUpdateTheEnvironment()
   {
      return ticksToUpdateTheEnvironment.getIntegerValue();
   }

   @Override
   public double getSwingHeight()
   {
      return swingHeight.getValue();
   }

   @Override
   public double getSwingDuration()
   {
      return swingDuration.getValue();
   }

   @Override
   public double getTransferDuration()
   {
      return transferDuration.getValue();
   }

   @Override
   public double getMaxStepLength()
   {
      return maxStepLength.getValue();
   }

   @Override
   public double getDefaultStepWidth()
   {
      return inPlaceWidth.getValue();
   }

   @Override
   public double getMinStepWidth()
   {
      return minStepWidth.getValue();
   }

   @Override
   public double getMaxStepWidth()
   {
      return maxStepWidth.getValue();
   }

   @Override
   public double getTurnMaxAngleInward()
   {
      return turnMaxAngleInward.getValue();
   }

   @Override
   public double getTurnMaxAngleOutward()
   {
      return turnMaxAngleOutward.getValue();
   }

   @Override
   public boolean getStepsAreAdjustable()
   {
      return stepsAreAdjustable.getBooleanValue();
   }

   @Override
   public boolean getShiftUpcomingStepsWithTouchdown()
   {
      return shiftUpcomingStepsWithTouchdown.getBooleanValue();
   }

   @Override
   public String toString()
   {
      return "number of fixed footsteps: " + numberOfFixedFootsteps + ", swing height: " + swingHeight + ", swing duration: " + swingDuration
            + ", transfer duration: " + transferDuration + ", max step length: " + maxStepLength + ", default step width: " + inPlaceWidth
            + ", min step width: " + minStepWidth + ", max step width: " + maxStepWidth + ", turn max angle inward: " + turnMaxAngleInward
            + ", turn max angle outward: " + turnMaxAngleOutward;
   }
}
