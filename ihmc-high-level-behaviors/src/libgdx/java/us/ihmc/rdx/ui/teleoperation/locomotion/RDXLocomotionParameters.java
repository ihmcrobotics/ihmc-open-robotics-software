package us.ihmc.rdx.ui.teleoperation.locomotion;

import org.apache.commons.lang3.StringUtils;
import us.ihmc.tools.property.*;

public class RDXLocomotionParameters extends StoredPropertySet
{
   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final BooleanStoredPropertyKey areFootstepsAdjustable = keys.addBooleanKey("Are footsteps adjustable");
   public static final DoubleStoredPropertyKey swingTime = keys.addDoubleKey("Swing time");
   public static final DoubleStoredPropertyKey transferTime = keys.addDoubleKey("Transfer time");
   public static final DoubleStoredPropertyKey straightStepLength = keys.addDoubleKey("Straight step length");
   public static final DoubleStoredPropertyKey straightStepWidth = keys.addDoubleKey("Straight step width");
   public static final DoubleStoredPropertyKey reverseStepLength = keys.addDoubleKey("Reverse step length");
   public static final DoubleStoredPropertyKey reverseStepWidth = keys.addDoubleKey("Reverse step width");
   public static final DoubleStoredPropertyKey shuffleStepLength = keys.addDoubleKey("Shuffle step length");
   public static final DoubleStoredPropertyKey shuffleStepWidth = keys.addDoubleKey("Shuffle step width");
   public static final DoubleStoredPropertyKey turningStepWidth = keys.addDoubleKey("Turning step width");
   public static final DoubleStoredPropertyKey footstepLengthMultiplier = keys.addDoubleKey("Footstep length multiplier");
   public static final BooleanStoredPropertyKey planWidthBodyPath = keys.addBooleanKey("Plan with body path");
   public static final BooleanStoredPropertyKey planSwingTrajectories = keys.addBooleanKey("Plan swing trajectories");
   public static final BooleanStoredPropertyKey replanSwingTrajectoriesOnChange = keys.addBooleanKey("Replan swing trajectories on change");
   public static final BooleanStoredPropertyKey assumeFlatGround = keys.addBooleanKey("Assume flat ground");
   public static final IntegerStoredPropertyKey initialStanceSide = keys.addIntegerKey("Initial stance side");

   public RDXLocomotionParameters(String robotName)
   {
      super(keys, RDXLocomotionParameters.class, StringUtils.capitalize(robotName));
   }

   public boolean getAreFootstepsAdjustable()
   {
      return get(areFootstepsAdjustable);
   }

   public double getSwingTime()
   {
      return get(swingTime);
   }

   public double getTransferTime()
   {
      return get(transferTime);
   }

   public double getStraightStepLength()
   {
      return get(straightStepLength);
   }

   public double getStraightStepWidth()
   {
      return get(straightStepWidth);
   }

   public double getReverseStepLength()
   {
      return get(reverseStepLength);
   }

   public double getReverseStepWidth()
   {
      return get(reverseStepWidth);
   }

   public double getShuffleStepLength()
   {
      return get(shuffleStepLength);
   }

   public double getShuffleStepWidth()
   {
      return get(shuffleStepWidth);
   }

   public double getTurningStepWidth()
   {
      return get(turningStepWidth);
   }

   public boolean getPlanWithBodyPath()
   {
      return get(planWidthBodyPath);
   }

   public boolean getPlanSwingTrajectories()
   {
      return get(planSwingTrajectories);
   }

   public boolean getReplanSwingTrajectoryOnChange()
   {
      return get(replanSwingTrajectoriesOnChange);
   }

   public double getFootstepLengthMultiplier()
   {
      return get(footstepLengthMultiplier);
   }

   public boolean getAssumeFlatGround()
   {
      return get(assumeFlatGround);
   }

   public int getInitialStanceSide()
   {
      return get(initialStanceSide);
   }
}