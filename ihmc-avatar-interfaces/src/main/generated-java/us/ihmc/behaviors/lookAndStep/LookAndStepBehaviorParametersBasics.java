package us.ihmc.behaviors.lookAndStep;

import us.ihmc.tools.property.StoredPropertySetBasics;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface LookAndStepBehaviorParametersBasics extends LookAndStepBehaviorParametersReadOnly, StoredPropertySetBasics
{
   /**
    * If true, when look and step is initiated, it is given some support regions under
    * it's feet, assuming it is currently supported. This helps the robot take its
    * first steps when the sensor can't see under the feet very well.
    */
   default void setUseInitialSupportRegions(boolean useInitialSupportRegions)
   {
      set(LookAndStepBehaviorParameters.useInitialSupportRegions, useInitialSupportRegions);
   }

   /**
    * If true, the footstep planning algorithm will assume flat ground and walk
    * regardless of sensor input, staying on the same plane and simply generating
    * footsteps on that plane. Only use if the robot has plenty of space and the
    * ground is very flat along the path you're taking.
    */
   default void setAssumeFlatGround(boolean assumeFlatGround)
   {
      set(LookAndStepBehaviorParameters.assumeFlatGround, assumeFlatGround);
   }

   /**
    * If true, there will be virtual (not sensor based) flat regions introduced when
    * the sensor generated planar regions seem locally flat. This parameter is
    * included to help the robot turn in place and take tight corners when the sensor
    * field of view doesn't include the feet, but also because the feet are going to
    * always prevent clean flat regions beneath you. This is an option because it can
    * be a little risky depending on the environment.
    */
   default void setDetectFlatGround(boolean detectFlatGround)
   {
      set(LookAndStepBehaviorParameters.detectFlatGround, detectFlatGround);
   }

   /**
    * How strict to be about the detect flat ground parameter.
    */
   default void setDetectFlatGroundZTolerance(double detectFlatGroundZTolerance)
   {
      set(LookAndStepBehaviorParameters.detectFlatGroundZTolerance, detectFlatGroundZTolerance);
   }

   /**
    * How strict to be about the detect flat ground parameter.
    */
   default void setDetectFlatGroundOrientationTolerance(double detectFlatGroundOrientationTolerance)
   {
      set(LookAndStepBehaviorParameters.detectFlatGroundOrientationTolerance, detectFlatGroundOrientationTolerance);
   }

   /**
    * How strict to be about the detect flat ground parameter.
    */
   default void setDetectFlatGroundMinRegionAreaToConsider(double detectFlatGroundMinRegionAreaToConsider)
   {
      set(LookAndStepBehaviorParameters.detectFlatGroundMinRegionAreaToConsider, detectFlatGroundMinRegionAreaToConsider);
   }

   /**
    * How strict to be about the detect flat ground parameter.
    */
   default void setDetectFlatGroundMinRadius(double detectFlatGroundMinRadius)
   {
      set(LookAndStepBehaviorParameters.detectFlatGroundMinRadius, detectFlatGroundMinRadius);
   }

   /**
    * The max size of the generated circle when assuming flat ground. A big circle can
    * help the robot keep moving without pauses by allowing 2-3 step ahead plans.
    */
   default void setAssumedFlatGroundCircleRadius(double assumedFlatGroundCircleRadius)
   {
      set(LookAndStepBehaviorParameters.assumedFlatGroundCircleRadius, assumedFlatGroundCircleRadius);
   }

   /**
    * When true, the robot takes an extra step to square up it's feet once it's
    * reached the goal.
    */
   default void setSquareUpAtTheEnd(boolean squareUpAtTheEnd)
   {
      set(LookAndStepBehaviorParameters.squareUpAtTheEnd, squareUpAtTheEnd);
   }

   /**
    * This is a scalar of the foot support polygons, used in the "Use initial support
    * regions" setting. Should be greater than 1. For example a value of 3 will give
    * the robot two big foot shaped regions, where it's feet are, that are scaled up
    * 3x, to assist with initial steps when the sensor can't see that area or the feet
    * are blocking it.
    */
   default void setSupportRegionScaleFactor(double supportRegionScaleFactor)
   {
      set(LookAndStepBehaviorParameters.supportRegionScaleFactor, supportRegionScaleFactor);
   }

   /**
    * When set to 1, look and step will simply be using the latest set of planar
    * regions available. When set to n = 2+, the past n sets of planar regions from
    * successive scans of the environment will be merged together using the
    * PlanarRegionSLAM algorithm and the resulting map will be used for footstep
    * planning. A value of 0 means ignoring all planar regions from the sensor.
    */
   default void setPlanarRegionsHistorySize(int planarRegionsHistorySize)
   {
      set(LookAndStepBehaviorParameters.planarRegionsHistorySize, planarRegionsHistorySize);
   }

   /**
    * The realtime walking controller has a footstep queue that it processes. Users
    * can override or queue additional footsteps. The parameter decides how many
    * footsteps to send to the controller at once. Currently it always overrides all
    * the footsteps every plan and this parameters determines the maximum that it will
    * send each time. Adding more steps can help the walking controller plan ahead for
    * balance. Sometimes the footstepl planner plans less than this amount. Needs to
    * be at least 1. 3 is usually a reasonable number.
    */
   default void setMaxStepsToSendToController(int maxStepsToSendToController)
   {
      set(LookAndStepBehaviorParameters.maxStepsToSendToController, maxStepsToSendToController);
   }

   /**
    * When true, basically circumvents any body path planning and sets a path straight
    * to the goal.
    */
   default void setFlatGroundBodyPathPlan(boolean flatGroundBodyPathPlan)
   {
      set(LookAndStepBehaviorParameters.flatGroundBodyPathPlan, flatGroundBodyPathPlan);
   }

   /**
    * (Exprimental) Use height map based body path planning algorithm.
    */
   default void setHeightMapBodyPathPlan(boolean heightMapBodyPathPlan)
   {
      set(LookAndStepBehaviorParameters.heightMapBodyPathPlan, heightMapBodyPathPlan);
   }

   /**
    * Swing planner enum ordinal. See SwingPlannerType.
    */
   default void setSwingPlannerType(int swingPlannerType)
   {
      set(LookAndStepBehaviorParameters.swingPlannerType, swingPlannerType);
   }

   /**
    * Prevent the robot taking unecessary steps in place.
    */
   default void setMinimumStepTranslation(double minimumStepTranslation)
   {
      set(LookAndStepBehaviorParameters.minimumStepTranslation, minimumStepTranslation);
   }

   /**
    * Prevent the robot taking unecessary steps in place.
    */
   default void setMinimumStepOrientation(double minimumStepOrientation)
   {
      set(LookAndStepBehaviorParameters.minimumStepOrientation, minimumStepOrientation);
   }

   /**
    * When the sensor for body path planning is mounted on the head and there's a neck
    * pitch, make sure it's at that pitch for body path planning.
    */
   default void setNeckPitchForBodyPath(double neckPitchForBodyPath)
   {
      set(LookAndStepBehaviorParameters.neckPitchForBodyPath, neckPitchForBodyPath);
   }

   /**
    * Tolerance for neck pitch so it's not stuck going up and down trying to correct
    * itself. It doesn't need to be that accurate.
    */
   default void setNeckPitchTolerance(double neckPitchTolerance)
   {
      set(LookAndStepBehaviorParameters.neckPitchTolerance, neckPitchTolerance);
   }

   /**
    * Decides the point during swing in which we start planning the next step. We want
    * to do this later for reactivity, but sooner to give the planner enough time to
    * complete before touchdown.
    */
   default void setPercentSwingToWait(double percentSwingToWait)
   {
      set(LookAndStepBehaviorParameters.percentSwingToWait, percentSwingToWait);
   }

   /**
    * Step swing duration.
    */
   default void setSwingDuration(double swingDuration)
   {
      set(LookAndStepBehaviorParameters.swingDuration, swingDuration);
   }

   /**
    * Double support transfer duration.
    */
   default void setTransferDuration(double transferDuration)
   {
      set(LookAndStepBehaviorParameters.transferDuration, transferDuration);
   }

   /**
    * The amount of time a reset of the whole behavior takes before becoming active
    * again. You don't want to do this too fast because for safety, you want to wait
    * for things to settle down.
    */
   default void setResetDuration(double resetDuration)
   {
      set(LookAndStepBehaviorParameters.resetDuration, resetDuration);
   }

   /**
    * How close to the goal suffices to conclude the behavior.
    */
   default void setGoalSatisfactionRadius(double goalSatisfactionRadius)
   {
      set(LookAndStepBehaviorParameters.goalSatisfactionRadius, goalSatisfactionRadius);
   }

   /**
    * Facing the same yaw as the goal tolerance for what suffices to conclude the
    * behavior.
    */
   default void setGoalSatisfactionOrientationDelta(double goalSatisfactionOrientationDelta)
   {
      set(LookAndStepBehaviorParameters.goalSatisfactionOrientationDelta, goalSatisfactionOrientationDelta);
   }

   /**
    * Only try to plan steps out this far on each footstep plan. Useful if your sensor
    * is long range and you don't want to waste time planning far out.
    */
   default void setPlanHorizon(double planHorizon)
   {
      set(LookAndStepBehaviorParameters.planHorizon, planHorizon);
   }

   /**
    * Let the planner go for longer in tricky situations (to the planner).
    */
   default void setFootstepPlannerTimeoutWhileStopped(double footstepPlannerTimeoutWhileStopped)
   {
      set(LookAndStepBehaviorParameters.footstepPlannerTimeoutWhileStopped, footstepPlannerTimeoutWhileStopped);
   }

   /**
    * Expiration so we don't use data that's old because we haven't received new data
    * in a while.
    */
   default void setPlanarRegionsExpiration(double planarRegionsExpiration)
   {
      set(LookAndStepBehaviorParameters.planarRegionsExpiration, planarRegionsExpiration);
   }

   /**
    * Expiration so we don't use data that's old because we haven't received new data
    * in a while.
    */
   default void setHeightMapExpiration(double heightMapExpiration)
   {
      set(LookAndStepBehaviorParameters.heightMapExpiration, heightMapExpiration);
   }

   /**
    * We want to wait a little after footstep planning fails so we can get new sensor
    * data, not free spin, let things settle down, not generate too many failure logs,
    * etc.
    */
   default void setWaitTimeAfterPlanFailed(double waitTimeAfterPlanFailed)
   {
      set(LookAndStepBehaviorParameters.waitTimeAfterPlanFailed, waitTimeAfterPlanFailed);
   }

   /**
    * We allow the footstep planner to terminate early if it plans this many steps.
    * Take care that this number is as high or higher than "Max steps to send to
    * controller".
    */
   default void setNumberOfStepsToTryToPlan(int numberOfStepsToTryToPlan)
   {
      set(LookAndStepBehaviorParameters.numberOfStepsToTryToPlan, numberOfStepsToTryToPlan);
   }

   /**
    * Expiration so we don't use data that's old because we haven't received new data
    * in a while.
    */
   default void setRobotConfigurationDataExpiration(double robotConfigurationDataExpiration)
   {
      set(LookAndStepBehaviorParameters.robotConfigurationDataExpiration, robotConfigurationDataExpiration);
   }

   /**
    * How many steps as tracked by the walking status tracker are allowed to be
    * incomplete in order to plan again. This is usually 1 for the currently swinging
    * step. Not sure if it makes sense to set this higher. Setting this to 0 would
    * force the robot to pause every step.
    */
   default void setAcceptableIncompleteFootsteps(int acceptableIncompleteFootsteps)
   {
      set(LookAndStepBehaviorParameters.acceptableIncompleteFootsteps, acceptableIncompleteFootsteps);
   }

   /**
    * When "Stop for impassibilities" is set to true, how far away from the obstacle
    * to stop and wait.
    */
   default void setHorizonFromDebrisToStop(double horizonFromDebrisToStop)
   {
      set(LookAndStepBehaviorParameters.horizonFromDebrisToStop, horizonFromDebrisToStop);
   }

   /**
    * If true, look and step will accept obstacle bounding boxes and stop short of
    * them if they are in the way, reporting to the operator that it's reached and
    * impassibility.
    */
   default void setStopForImpassibilities(boolean stopForImpassibilities)
   {
      set(LookAndStepBehaviorParameters.stopForImpassibilities, stopForImpassibilities);
   }
}
