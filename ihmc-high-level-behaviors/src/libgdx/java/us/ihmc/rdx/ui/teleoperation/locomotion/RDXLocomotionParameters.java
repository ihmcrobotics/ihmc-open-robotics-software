package us.ihmc.rdx.ui.teleoperation.locomotion;

import org.apache.commons.lang3.StringUtils;
import us.ihmc.tools.property.*;

public class RDXLocomotionParameters extends StoredPropertySet
{
   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final DoubleStoredPropertyKey swingTime = keys.addDoubleKey("Swing time");
   public static final DoubleStoredPropertyKey transferTime = keys.addDoubleKey("Transfer time");
   public static final BooleanStoredPropertyKey assumeFlatGround = keys.addBooleanKey("Assume flat ground");
   public static final BooleanStoredPropertyKey areFootstepsAdjustable = keys.addBooleanKey("Are footsteps adjustable");
   public static final BooleanStoredPropertyKey planSwingTrajectories = keys.addBooleanKey("Plan swing trajectories");
   public static final BooleanStoredPropertyKey planWidthBodyPath = keys.addBooleanKey("Plan with body path");
   public static final BooleanStoredPropertyKey replanSwingTrajectoriesOnChange = keys.addBooleanKey("Replan swing trajectories on change");
   public static final IntegerStoredPropertyKey initialStanceSide = keys.addIntegerKey("Initial stance side");
   public static final DoubleStoredPropertyKey footstepPlannerTimeout = keys.addDoubleKey("Footstep planner timeout");
   public static final DoubleStoredPropertyKey idealGoalFootstepWidth = keys.addDoubleKey("Ideal goal footstep width");

   public RDXLocomotionParameters(String robotName)
   {
      super(keys, RDXLocomotionParameters.class, StringUtils.capitalize(robotName));
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