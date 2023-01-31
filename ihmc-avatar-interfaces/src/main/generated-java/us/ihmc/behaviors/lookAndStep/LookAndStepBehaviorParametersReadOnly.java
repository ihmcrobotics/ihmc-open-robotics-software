package us.ihmc.behaviors.lookAndStep;

import us.ihmc.tools.property.StoredPropertySetReadOnly;

import static us.ihmc.behaviors.lookAndStep.LookAndStepBehaviorParameters.*;

/**
 * This class was auto generated. Do not edit by hand. Edit the cooresponding JSON file
 * and run the main in super to regenerate.
 */
public interface LookAndStepBehaviorParametersReadOnly extends StoredPropertySetReadOnly
{
   /**
    * If true, when look and step is initiated, it is given some support regions under
    * it's feet, assuming it is currently supported. This helps the robot take its
    * first steps when the sensor can't see under the feet very well.
    */
   default boolean getUseInitialSupportRegions()
   {
      return get(useInitialSupportRegions);
   }

   /**
    * If true, the footstep planning algorithm will assume flat ground and walk
    * regardless of sensor input, staying on the same plane and simply generating
    * footsteps on that plane. Only use if the robot has plenty of space and the
    * ground is very flat along the path you're taking.
    */
   default boolean getAssumeFlatGround()
   {
      return get(assumeFlatGround);
   }

   /**
    * If true, there will be virtual (not sensor based) flat regions introduced when
    * the sensor generated planar regions seem locally flat. This parameter is
    * included to help the robot turn in place and take tight corners when the sensor
    * field of view doesn't include the feet, but also because the feet are going to
    * always prevent clean flat regions beneath you. This is an option because it can
    * be a little risky depending on the environment.
    */
   default boolean getDetectFlatGround()
   {
      return get(detectFlatGround);
   }

   /**
    * How strict to be about the detect flat ground parameter.
    */
   default double getDetectFlatGroundZTolerance()
   {
      return get(detectFlatGroundZTolerance);
   }

   /**
    * How strict to be about the detect flat ground parameter.
    */
   default double getDetectFlatGroundOrientationTolerance()
   {
      return get(detectFlatGroundOrientationTolerance);
   }

   /**
    * How strict to be about the detect flat ground parameter.
    */
   default double getDetectFlatGroundMinRegionAreaToConsider()
   {
      return get(detectFlatGroundMinRegionAreaToConsider);
   }

   /**
    * How strict to be about the detect flat ground parameter.
    */
   default double getDetectFlatGroundMinRadius()
   {
      return get(detectFlatGroundMinRadius);
   }

   /**
    * The max size of the generated circle when assuming flat ground. A big circle can
    * help the robot keep moving without pauses by allowing 2-3 step ahead plans.
    */
   default double getAssumedFlatGroundCircleRadius()
   {
      return get(assumedFlatGroundCircleRadius);
   }

   /**
    * When true, the robot takes an extra step to square up it's feet once it's
    * reached the goal.
    */
   default boolean getSquareUpAtTheEnd()
   {
      return get(squareUpAtTheEnd);
   }

   /**
    * This is a scalar of the foot support polygons, used in the "Use initial support
    * regions" setting. Should be greater than 1. For example a value of 3 will give
    * the robot two big foot shaped regions, where it's feet are, that are scaled up
    * 3x, to assist with initial steps when the sensor can't see that area or the feet
    * are blocking it.
    */
   default double getSupportRegionScaleFactor()
   {
      return get(supportRegionScaleFactor);
   }

   /**
    * When set to 1, look and step will simply be using the latest set of planar
    * regions available. When set to n = 2+, the past n sets of planar regions from
    * successive scans of the environment will be merged together using the
    * PlanarRegionSLAM algorithm and the resulting map will be used for footstep
    * planning. A value of 0 means ignoring all planar regions from the sensor.
    */
   default int getPlanarRegionsHistorySize()
   {
      return get(planarRegionsHistorySize);
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
   default int getMaxStepsToSendToController()
   {
      return get(maxStepsToSendToController);
   }

   /**
    * When true, basically circumvents any body path planning and sets a path straight
    * to the goal.
    */
   default boolean getFlatGroundBodyPathPlan()
   {
      return get(flatGroundBodyPathPlan);
   }

   /**
    * (Exprimental) Use height map based body path planning algorithm.
    */
   default boolean getHeightMapBodyPathPlan()
   {
      return get(heightMapBodyPathPlan);
   }

   /**
    * Swing planner enum ordinal. See SwingPlannerType.
    */
   default int getSwingPlannerType()
   {
      return get(swingPlannerType);
   }

   /**
    * Prevent the robot taking unecessary steps in place.
    */
   default double getMinimumStepTranslation()
   {
      return get(minimumStepTranslation);
   }

   /**
    * Prevent the robot taking unecessary steps in place.
    */
   default double getMinimumStepOrientation()
   {
      return get(minimumStepOrientation);
   }

   /**
    * When the sensor for body path planning is mounted on the head and there's a neck
    * pitch, make sure it's at that pitch for body path planning.
    */
   default double getNeckPitchForBodyPath()
   {
      return get(neckPitchForBodyPath);
   }

   /**
    * Tolerance for neck pitch so it's not stuck going up and down trying to correct
    * itself. It doesn't need to be that accurate.
    */
   default double getNeckPitchTolerance()
   {
      return get(neckPitchTolerance);
   }

   /**
    * Decides the point during swing in which we start planning the next step. We want
    * to do this later for reactivity, but sooner to give the planner enough time to
    * complete before touchdown.
    */
   default double getPercentSwingToWait()
   {
      return get(percentSwingToWait);
   }

   /**
    * Step swing duration.
    */
   default double getSwingDuration()
   {
      return get(swingDuration);
   }

   /**
    * Double support transfer duration.
    */
   default double getTransferDuration()
   {
      return get(transferDuration);
   }

   /**
    * The amount of time a reset of the whole behavior takes before becoming active
    * again. You don't want to do this too fast because for safety, you want to wait
    * for things to settle down.
    */
   default double getResetDuration()
   {
      return get(resetDuration);
   }

   /**
    * How close to the goal suffices to conclude the behavior.
    */
   default double getGoalSatisfactionRadius()
   {
      return get(goalSatisfactionRadius);
   }

   /**
    * Facing the same yaw as the goal tolerance for what suffices to conclude the
    * behavior.
    */
   default double getGoalSatisfactionOrientationDelta()
   {
      return get(goalSatisfactionOrientationDelta);
   }

   /**
    * Only try to plan steps out this far on each footstep plan. Useful if your sensor
    * is long range and you don't want to waste time planning far out.
    */
   default double getPlanHorizon()
   {
      return get(planHorizon);
   }

   /**
    * Let the planner go for longer in tricky situations (to the planner).
    */
   default double getFootstepPlannerTimeoutWhileStopped()
   {
      return get(footstepPlannerTimeoutWhileStopped);
   }

   /**
    * Expiration so we don't use data that's old because we haven't received new data
    * in a while.
    */
   default double getPlanarRegionsExpiration()
   {
      return get(planarRegionsExpiration);
   }

   /**
    * Expiration so we don't use data that's old because we haven't received new data
    * in a while.
    */
   default double getHeightMapExpiration()
   {
      return get(heightMapExpiration);
   }

   /**
    * We want to wait a little after footstep planning fails so we can get new sensor
    * data, not free spin, let things settle down, not generate too many failure logs,
    * etc.
    */
   default double getWaitTimeAfterPlanFailed()
   {
      return get(waitTimeAfterPlanFailed);
   }

   /**
    * We allow the footstep planner to terminate early if it plans this many steps.
    * Take care that this number is as high or higher than "Max steps to send to
    * controller".
    */
   default int getNumberOfStepsToTryToPlan()
   {
      return get(numberOfStepsToTryToPlan);
   }

   /**
    * Expiration so we don't use data that's old because we haven't received new data
    * in a while.
    */
   default double getRobotConfigurationDataExpiration()
   {
      return get(robotConfigurationDataExpiration);
   }

   /**
    * How many steps as tracked by the walking status tracker are allowed to be
    * incomplete in order to plan again. This is usually 1 for the currently swinging
    * step. Not sure if it makes sense to set this higher. Setting this to 0 would
    * force the robot to pause every step.
    */
   default int getAcceptableIncompleteFootsteps()
   {
      return get(acceptableIncompleteFootsteps);
   }

   /**
    * When "Stop for impassibilities" is set to true, how far away from the obstacle
    * to stop and wait.
    */
   default double getHorizonFromDebrisToStop()
   {
      return get(horizonFromDebrisToStop);
   }

   /**
    * If true, look and step will accept obstacle bounding boxes and stop short of
    * them if they are in the way, reporting to the operator that it's reached and
    * impassibility.
    */
   default boolean getStopForImpassibilities()
   {
      return get(stopForImpassibilities);
   }
}
