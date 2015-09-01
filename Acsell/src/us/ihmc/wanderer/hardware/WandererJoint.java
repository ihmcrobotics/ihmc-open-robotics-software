package us.ihmc.wanderer.hardware;

import us.ihmc.acsell.hardware.AcsellJoint;
import us.ihmc.acsell.hardware.configuration.StrainGaugeInformation;

public enum WandererJoint implements AcsellJoint
{
   // BeltRatio = 1.3333;

   LEFT_HIP_X("l_leg_mhx", 10, true, false, WandererActuator.LEFT_HIP_X),
   LEFT_HIP_Z("l_leg_uhz", 5, true, false, WandererActuator.LEFT_HIP_Z),
   LEFT_HIP_Y("l_leg_lhy", 10, true, false, WandererActuator.LEFT_HIP_Y),
   LEFT_KNEE_Y("l_leg_kny", 8, true, true, WandererActuator.LEFT_KNEE),
   LEFT_ANKLE_Y("l_leg_uay", 6, true, true, WandererActuator.LEFT_ANKLE_RIGHT, WandererActuator.LEFT_ANKLE_LEFT),
   LEFT_ANKLE_X("l_leg_lax", LEFT_ANKLE_Y.getRatio(), true, true, LEFT_ANKLE_Y.getActuators()),

   RIGHT_HIP_X("r_leg_mhx", 10, true, false, WandererActuator.RIGHT_HIP_X),
   RIGHT_HIP_Z("r_leg_uhz", 5, true, false, WandererActuator.RIGHT_HIP_Z),
   RIGHT_HIP_Y("r_leg_lhy", 10, true, false, WandererActuator.RIGHT_HIP_Y),
   RIGHT_KNEE_Y("r_leg_kny", 8, true, true, WandererActuator.RIGHT_KNEE),
   RIGHT_ANKLE_Y("r_leg_uay", 6, true, true, WandererActuator.RIGHT_ANKLE_RIGHT, WandererActuator.RIGHT_ANKLE_LEFT),
   RIGHT_ANKLE_X("r_leg_lax", RIGHT_ANKLE_Y.getRatio(), true, true, RIGHT_ANKLE_Y.getActuators()),

   TORSO_Z("back_ubz", 120, false, false, WandererActuator.TORSO_Z),
   TORSO_Y("back_mby", 120, false, false, WandererActuator.TORSO_Y),
   TORSO_X("back_lbx", 120, true, false, WandererActuator.TORSO_X);

   public static final WandererJoint[] values = values();
   private final boolean hasOutputEncoder;
   private final String sdfName;
   private final double ratio;
   private final WandererActuator[] actuators;
   private final boolean hasNonlinearTransmission;

   private WandererJoint(String sdfName, double ratio, boolean hasOutputEncoder, boolean hasNonlinearTransmission, WandererActuator... actuators)
   {
      this.sdfName = sdfName;
      this.ratio = ratio;
      this.actuators = actuators;
      this.hasOutputEncoder = hasOutputEncoder;
      this.hasNonlinearTransmission = hasNonlinearTransmission;
   }

   public String getSdfName()
   {
      return sdfName;
   }

   public double getRatio()
   {
      return ratio;
   }

   public WandererActuator[] getActuators()
   {
      return actuators;
   }

   public boolean isLinear()
   {
      return actuators.length == 1;
   }
   
   public boolean hasNonlinearTransmission()
   {
      return hasNonlinearTransmission; 
   }

   public boolean hasOutputEncoder()
   {
      return hasOutputEncoder;
   }

   public StrainGaugeInformation getStrainGaugeInformation()
   {
      return null;
   }

}