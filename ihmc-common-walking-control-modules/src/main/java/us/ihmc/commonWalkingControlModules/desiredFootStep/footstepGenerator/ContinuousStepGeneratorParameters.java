package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator;

import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;

public class ContinuousStepGeneratorParameters implements ContinuousStepGeneratorParametersBasics
{
   private int numberOfFootstepsToPlan = DEFAULT_NUMBER_OF_FOOTSTEPS_TO_PLAN;
   private int numberOfFixedFootsteps = DEFAULT_NUMBER_OF_FIXED_FOOTSTEPS;
   private int ticksToUpdateTheEnvironment = DEFAULT_TICKS_TO_UPDATE_ENVIRONMENT;
   private double swingHeight;
   private double swingDuration, transferDuration;
   private double maxStepLengthForwards;
   private double maxStepLengthBackwards;
   private double defaultStepWidth, minStepWidth, maxStepWidth;
   private double turnMaxAngleInward, turnMaxAngleOutward;
   private boolean stepsAreAdjustable = DEFAULT_STEPS_ARE_ADJUSTABLE;
   private boolean requestSnapToHeightMap = DEFAULT_REQUEST_SNAP_TO_HEIGHTMAP;
   private boolean accountForGroundDrift = DEFAULT_ACCOUNT_FOR_GROUND_DRIFT;
   private boolean shiftUpcomingStepsWithTouchdown = DEFAULT_SHIFT_UPCOMING_STEPS_WITH_TOUCHDOWN;

   public ContinuousStepGeneratorParameters()
   {
   }

   public ContinuousStepGeneratorParameters(WalkingControllerParameters walkingControllerParameters)
   {
      set(walkingControllerParameters);
   }

   public ContinuousStepGeneratorParameters(ContinuousStepGeneratorParametersBasics other)
   {
      set(other);
   }

   @Override
   public void setStepsAreAdjustable(boolean stepsAreAdjustable)
   {
      this.stepsAreAdjustable = stepsAreAdjustable;
   }

   @Override
   public void setShiftUpcomingStepsWithTouchdown(boolean shiftUpcomingStepsWithTouchdown)
   {
      this.shiftUpcomingStepsWithTouchdown = shiftUpcomingStepsWithTouchdown;
   }

   @Override
   public void setNumberOfFootstepsToPlan(int numberOfFootstepsToPlan)
   {
      this.numberOfFootstepsToPlan = numberOfFootstepsToPlan;
   }

   @Override
   public void setNumberOfFixedFootsteps(int numberOfFixedFootsteps)
   {
      this.numberOfFixedFootsteps = numberOfFixedFootsteps;
   }

   @Override
   public void setTicksToUpdateTheEnvironment(int ticksToUpdateTheEnvironment)
   {
      this.ticksToUpdateTheEnvironment = ticksToUpdateTheEnvironment;
   }

   @Override
   public void setSwingHeight(double swingHeight)
   {
      this.swingHeight = swingHeight;
   }

   @Override
   public void setSwingDuration(double swingDuration)
   {
      this.swingDuration = swingDuration;
   }

   @Override
   public void setTransferDuration(double transferDuration)
   {
      this.transferDuration = transferDuration;
   }

   @Override
   public void setMaxStepLengthForwards(double maxStepLengthForwards)
   {
      this.maxStepLengthForwards = maxStepLengthForwards;
   }

   @Override
   public void setMaxStepLengthBackwards(double maxStepLengthBackwards)
   {
      this.maxStepLengthBackwards = maxStepLengthBackwards;
   }

   @Override
   public void setDefaultStepWidth(double defaultStepWidth)
   {
      this.defaultStepWidth = defaultStepWidth;
   }

   @Override
   public void setMinStepWidth(double minStepWidth)
   {
      this.minStepWidth = minStepWidth;
   }

   @Override
   public void setMaxStepWidth(double maxStepWidth)
   {
      this.maxStepWidth = maxStepWidth;
   }

   @Override
   public void setTurnMaxAngleInward(double turnMaxAngleInward)
   {
      this.turnMaxAngleInward = turnMaxAngleInward;
   }

   @Override
   public void setTurnMaxAngleOutward(double turnMaxAngleOutward)
   {
      this.turnMaxAngleOutward = turnMaxAngleOutward;
   }

   @Override
   public void setRequestSnapToHeightmap(boolean requestSnapToHeightMap)
   {
      this.requestSnapToHeightMap = requestSnapToHeightMap;
   }

   @Override
   public void setAccountForGroundDrift(boolean accountForGroundDrift)
   {
      this.accountForGroundDrift = accountForGroundDrift;
   }

   @Override
   public int getNumberOfFootstepsToPlan()
   {
      return numberOfFootstepsToPlan;
   }

   @Override
   public int getNumberOfFixedFootsteps()
   {
      return numberOfFixedFootsteps;
   }

   @Override
   public int getTicksToUpdateTheEnvironment()
   {
      return ticksToUpdateTheEnvironment;
   }

   @Override
   public double getSwingHeight()
   {
      return swingHeight;
   }

   @Override
   public double getSwingDuration()
   {
      return swingDuration;
   }

   @Override
   public double getTransferDuration()
   {
      return transferDuration;
   }

   @Override
   public double getMaxStepLengthForwards()
   {
      return maxStepLengthForwards;
   }

   @Override
   public double getMaxStepLengthBackwards()
   {
      return maxStepLengthBackwards;
   }

   @Override
   public double getDefaultStepWidth()
   {
      return defaultStepWidth;
   }

   @Override
   public double getMinStepWidth()
   {
      return minStepWidth;
   }

   @Override
   public double getMaxStepWidth()
   {
      return maxStepWidth;
   }

   @Override
   public double getTurnMaxAngleInward()
   {
      return turnMaxAngleInward;
   }

   @Override
   public double getTurnMaxAngleOutward()
   {
      return turnMaxAngleOutward;
   }

   @Override
   public boolean getStepsAreAdjustable()
   {
      return stepsAreAdjustable;
   }

   @Override
   public boolean getRequestSnapToHeightmap()
   {
      return requestSnapToHeightMap;
   }

   @Override
   public boolean getAccountForGroundDrift()
   {
      return accountForGroundDrift;
   }

   @Override
   public boolean getShiftUpcomingStepsWithTouchdown()
   {
      return shiftUpcomingStepsWithTouchdown;
   }

   @Override
   public String toString()
   {
      return "number of footsteps to plan: " + numberOfFootstepsToPlan + ", number of fixed footsteps: " + numberOfFixedFootsteps
             + "ticks to update the environment: " + ticksToUpdateTheEnvironment + ", swing height: " + swingHeight + ", swing duration: " + swingDuration
             + ", transfer duration: " + transferDuration + ", max step length (forwards): " + maxStepLengthForwards + ", max step length (backwards): "
             + maxStepLengthBackwards + ", default step width: " + defaultStepWidth + ", min step width: " + minStepWidth + ", max step width: " + maxStepWidth
             + ", turn max angle inward: " + turnMaxAngleInward + ", turn max angle outward: " + turnMaxAngleOutward;
   }
}
