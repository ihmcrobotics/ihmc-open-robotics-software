package us.ihmc.footstepPlanning;

import us.ihmc.tools.property.*;

public abstract class LocomotionParameters extends StoredPropertySet
{
   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final DoubleStoredPropertyKey swingTime = keys.addDoubleKey("Swing time", 0.8);
   public static final DoubleStoredPropertyKey transferTime = keys.addDoubleKey("Transfer time", 0.4);
   public static final BooleanStoredPropertyKey assumeFlatGround = keys.addBooleanKey("Assume flat ground", true);
   public static final BooleanStoredPropertyKey performAStarSearch = keys.addBooleanKey("Perform A star search", true);
   public static final BooleanStoredPropertyKey areFootstepsAdjustable = keys.addBooleanKey("Are footsteps adjustable", false);
   public static final BooleanStoredPropertyKey planSwingTrajectories = keys.addBooleanKey("Plan swing trajectories", false);
   public static final BooleanStoredPropertyKey planWidthBodyPath = keys.addBooleanKey("Plan with body path", false);
   public static final BooleanStoredPropertyKey replanSwingTrajectoriesOnChange = keys.addBooleanKey("Replan swing trajectories on change", false);
   public static final IntegerStoredPropertyKey initialStanceSide = keys.addIntegerKey("Initial stance side", 0);
   public static final DoubleStoredPropertyKey footstepPlannerTimeout = keys.addDoubleKey("Footstep planner timeout", 12.0);
   public static final DoubleStoredPropertyKey idealGoalFootstepWidth = keys.addDoubleKey("Ideal goal footstep width", 0.22);

   public LocomotionParameters(Class<?> robotSubclassForLoading)
   {
      super(keys, robotSubclassForLoading);
   }

   public double getSwingTime()
   {
      return get(swingTime);
   }

   public double getTransferTime()
   {
      return get(transferTime);
   }

   public boolean getAssumeFlatGround()
   {
      return get(assumeFlatGround);
   }

   public void setAssumeFlatGround(boolean assumeFlatGroundFlag)
   {
      set(assumeFlatGround, assumeFlatGroundFlag);
   }

   public boolean getAreFootstepsAdjustable()
   {
      return get(areFootstepsAdjustable);
   }

   public boolean getPlanSwingTrajectories()
   {
      return get(planSwingTrajectories);
   }

   public boolean getReplanSwingTrajectoryOnChange()
   {
      return get(replanSwingTrajectoriesOnChange);
   }

   public boolean getPlanWithBodyPath()
   {
      return get(planWidthBodyPath);
   }

   public boolean getPerformAStarSearch()
   {
      return get(performAStarSearch);
   }

   public int getInitialStanceSide()
   {
      return get(initialStanceSide);
   }

   public double getFootstepPlannerTimeout()
   {
      return get(footstepPlannerTimeout);
   }

   public double getIdealGoalFootstepWidth()
   {
      return get(idealGoalFootstepWidth);
   }
}