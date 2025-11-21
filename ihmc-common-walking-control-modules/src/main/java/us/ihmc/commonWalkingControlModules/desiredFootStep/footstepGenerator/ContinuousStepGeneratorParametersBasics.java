package us.ihmc.commonWalkingControlModules.desiredFootStep.footstepGenerator;

import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;

public interface ContinuousStepGeneratorParametersBasics
{
   public static final int DEFAULT_NUMBER_OF_FOOTSTEPS_TO_PLAN = 4;
   public static final int DEFAULT_TICKS_TO_UPDATE_ENVIRONMENT = Integer.MAX_VALUE;
   public static final int DEFAULT_NUMBER_OF_FIXED_FOOTSTEPS = 0;
   public static final boolean DEFAULT_STEPS_ARE_ADJUSTABLE = true;
   public static final boolean DEFAULT_SHIFT_UPCOMING_STEPS_WITH_TOUCHDOWN = true;
   public static final boolean DEFAULT_REQUEST_SNAP_TO_HEIGHTMAP = false;
   public static final boolean DEFAULT_ACCOUNT_FOR_GROUND_DRIFT = false;

   default void clear()
   {
      setNumberOfFootstepsToPlan(DEFAULT_NUMBER_OF_FOOTSTEPS_TO_PLAN);
      setNumberOfFixedFootsteps(DEFAULT_NUMBER_OF_FIXED_FOOTSTEPS);
      setTicksToUpdateTheEnvironment(DEFAULT_TICKS_TO_UPDATE_ENVIRONMENT);
      setSwingHeight(0.0);
      setMinStepWidth(0.0);
      setMaxStepWidth(Double.POSITIVE_INFINITY);
      setMaxStepLengthForwards(Double.POSITIVE_INFINITY);
      setMaxStepLengthBackwards(Double.POSITIVE_INFINITY);
      setTurnMaxAngleOutward(Math.PI / 2.0);
      setTurnMaxAngleInward(-Math.PI / 2.0);
      setStepsAreAdjustable(DEFAULT_STEPS_ARE_ADJUSTABLE);
      setShiftUpcomingStepsWithTouchdown(DEFAULT_SHIFT_UPCOMING_STEPS_WITH_TOUCHDOWN);
      setRequestSnapToHeightmap(DEFAULT_REQUEST_SNAP_TO_HEIGHTMAP);
      setAccountForGroundDrift(DEFAULT_ACCOUNT_FOR_GROUND_DRIFT);
   }

   default void set(ContinuousStepGeneratorParametersBasics other)
   {
      setNumberOfFootstepsToPlan(other.getNumberOfFootstepsToPlan());
      setNumberOfFixedFootsteps(other.getNumberOfFixedFootsteps());
      setTicksToUpdateTheEnvironment(other.getTicksToUpdateTheEnvironment());
      setSwingHeight(other.getSwingHeight());
      setSwingDuration(other.getSwingDuration());
      setTransferDuration(other.getTransferDuration());
      setMaxStepLengthForwards(other.getMaxStepLengthForwards());
      setMaxStepLengthBackwards(other.getMaxStepLengthBackwards());
      setDefaultStepWidth(other.getDefaultStepWidth());
      setMinStepWidth(other.getMinStepWidth());
      setMaxStepWidth(other.getMaxStepWidth());
      setTurnMaxAngleInward(other.getTurnMaxAngleInward());
      setTurnMaxAngleOutward(other.getTurnMaxAngleOutward());
      setStepsAreAdjustable(other.getStepsAreAdjustable());
      setShiftUpcomingStepsWithTouchdown(other.getShiftUpcomingStepsWithTouchdown());
      setRequestSnapToHeightmap(other.getRequestSnapToHeightmap());
      setAccountForGroundDrift(other.getAccountForGroundDrift());
   }

   default void set(WalkingControllerParameters walkingControllerParameters)
   {
      setNumberOfFootstepsToPlan(DEFAULT_NUMBER_OF_FOOTSTEPS_TO_PLAN);
      setNumberOfFixedFootsteps(DEFAULT_NUMBER_OF_FIXED_FOOTSTEPS);
      setTicksToUpdateTheEnvironment(DEFAULT_TICKS_TO_UPDATE_ENVIRONMENT);
      setStepsAreAdjustable(DEFAULT_STEPS_ARE_ADJUSTABLE);
      setShiftUpcomingStepsWithTouchdown(DEFAULT_SHIFT_UPCOMING_STEPS_WITH_TOUCHDOWN);
      setRequestSnapToHeightmap(DEFAULT_REQUEST_SNAP_TO_HEIGHTMAP);
      setAccountForGroundDrift(DEFAULT_ACCOUNT_FOR_GROUND_DRIFT);
      setSwingDuration(walkingControllerParameters.getDefaultSwingTime());
      setTransferDuration(walkingControllerParameters.getDefaultTransferTime());

      SteppingParameters steppingParameters = walkingControllerParameters.getSteppingParametersForStepGeneration();
      setSwingHeight(walkingControllerParameters.getSwingTrajectoryParameters().getDefaultSwingHeight());
      setMaxStepLengthForwards(steppingParameters.getMaxStepLength());
      setMaxStepLengthBackwards(steppingParameters.getMaxBackwardStepLength());
      setDefaultStepWidth(steppingParameters.getInPlaceWidth());
      setMinStepWidth(steppingParameters.getMinStepWidth());
      setMaxStepWidth(steppingParameters.getMaxStepWidth());
      setTurnMaxAngleInward(steppingParameters.getMaxAngleTurnInwards());
      setTurnMaxAngleOutward(steppingParameters.getMaxAngleTurnOutwards());
   }

   void setStepsAreAdjustable(boolean stepsAreAdjustable);

   void setShiftUpcomingStepsWithTouchdown(boolean shiftUpcomingStepsWithTouchdown);

   void setNumberOfFootstepsToPlan(int numberOfFootstepsToPlan);

   void setNumberOfFixedFootsteps(int numberOfFixedFootsteps);

   void setTicksToUpdateTheEnvironment(int ticksToUpdateTheEnvironment);

   void setSwingHeight(double swingHeight);

   void setSwingDuration(double swingDuration);

   void setTransferDuration(double transferDuration);

   void setMaxStepLengthForwards(double maxStepLengthForwards);

   void setMaxStepLengthBackwards(double maxStepLengthBackwards);

   void setDefaultStepWidth(double defaultStepWidth);

   void setMinStepWidth(double minStepWidth);

   void setMaxStepWidth(double maxStepWidth);

   void setTurnMaxAngleInward(double turnMaxAngleInward);

   void setTurnMaxAngleOutward(double turnMaxAngleOutward);

   void setRequestSnapToHeightmap(boolean requestSnapToHeightmap);

   void setAccountForGroundDrift(boolean accountForGroundDrift);

   int getNumberOfFootstepsToPlan();

   int getNumberOfFixedFootsteps();

   int getTicksToUpdateTheEnvironment();

   double getSwingHeight();

   double getSwingDuration();

   double getTransferDuration();

   double getMaxStepLengthForwards();

   double getMaxStepLengthBackwards();

   double getDefaultStepWidth();

   double getMinStepWidth();

   double getMaxStepWidth();

   double getTurnMaxAngleInward();

   double getTurnMaxAngleOutward();

   boolean getStepsAreAdjustable();

   boolean getRequestSnapToHeightmap();

   boolean getAccountForGroundDrift();

   boolean getShiftUpcomingStepsWithTouchdown();

   default String getString()
   {
      return "number of footsteps to plan: " + getNumberOfFootstepsToPlan() + ", number of fixed footsteps: " + getNumberOfFixedFootsteps()
             + ", ticks to update the environment: " + getTicksToUpdateTheEnvironment() + ", swing height: " + getSwingHeight() + ", swing duration: "
             + getSwingDuration() + ", transfer duration: " + getTransferDuration() + ", max step length (forwards): " + getMaxStepLengthForwards()
             + ", max step length (backwards): " + getMaxStepLengthBackwards() + ", default step width: " + getDefaultStepWidth() + ", min step width: "
             + getMinStepWidth() + ", max step width: " + getMaxStepWidth() + ", turn max angle inward: " + getTurnMaxAngleInward()
             + ", turn max angle outward: " + getTurnMaxAngleOutward();
   }
}
