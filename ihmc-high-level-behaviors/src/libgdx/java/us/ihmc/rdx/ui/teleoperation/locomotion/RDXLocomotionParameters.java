package us.ihmc.rdx.ui.teleoperation.locomotion;

import org.apache.commons.lang3.StringUtils;
import us.ihmc.tools.property.BooleanStoredPropertyKey;
import us.ihmc.tools.property.DoubleStoredPropertyKey;
import us.ihmc.tools.property.StoredPropertyKeyList;
import us.ihmc.tools.property.StoredPropertySet;

public class RDXLocomotionParameters extends StoredPropertySet
{
   public static final StoredPropertyKeyList keys = new StoredPropertyKeyList();

   public static final DoubleStoredPropertyKey swingTime = keys.addDoubleKey("Swing time");
   public static final DoubleStoredPropertyKey transferTime = keys.addDoubleKey("Transfer time");
   public static final BooleanStoredPropertyKey assumeFlatGround = keys.addBooleanKey("Assume flat ground");
   public static final BooleanStoredPropertyKey areFootstepsAdjustable = keys.addBooleanKey("Are footsteps adjustable");
   public static final BooleanStoredPropertyKey planSwingTrajectories = keys.addBooleanKey("Plan swing trajectories");
   public static final BooleanStoredPropertyKey replanSwingTrajectoriesOnChange = keys.addBooleanKey("Replan swing trajectories on change");
   public static final BooleanStoredPropertyKey planWidthBodyPath = keys.addBooleanKey("Plan with body path");

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
}