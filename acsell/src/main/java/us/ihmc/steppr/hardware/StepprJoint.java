package us.ihmc.steppr.hardware;

import us.ihmc.acsell.hardware.AcsellJoint;
import us.ihmc.acsell.hardware.configuration.StrainGaugeInformation;



public enum StepprJoint implements AcsellJoint
{
	// BeltRatio = 1.3333;
   LEFT_HIP_X("l_leg_mhx", 10, true, new StrainGaugeInformation(StepprActuator.LEFT_HIP_Z, 1, -73.33, 2.5), StepprActuator.LEFT_HIP_X),
   LEFT_HIP_Z("l_leg_uhz", 5, true,  null,  StepprActuator.LEFT_HIP_Z),
   LEFT_HIP_Y("l_leg_lhy", 10, true, new StrainGaugeInformation(StepprActuator.LEFT_KNEE, 1, -111.2, 2.5), StepprActuator.LEFT_HIP_Y),
   LEFT_KNEE_Y("l_leg_kny", 6 * 1.3333333333, true, new StrainGaugeInformation(StepprActuator.LEFT_KNEE, 0, 79.65*1.3333, 2.5), StepprActuator.LEFT_KNEE, StepprActuator.LEFT_ANKLE_LEFT),
   LEFT_ANKLE_Y("l_leg_uay", 6, true, new StrainGaugeInformation(StepprActuator.LEFT_ANKLE_RIGHT, 2, -211*.12/6, 2.1), StepprActuator.LEFT_ANKLE_RIGHT, StepprActuator.LEFT_ANKLE_LEFT),
   LEFT_ANKLE_X("l_leg_lax", LEFT_ANKLE_Y.getRatio(), true, new StrainGaugeInformation(StepprActuator.LEFT_ANKLE_RIGHT, 1, -426.8*.12/6, 3.45), LEFT_ANKLE_Y.getActuators()),

   RIGHT_HIP_X("r_leg_mhx", 10, true, new StrainGaugeInformation(StepprActuator.RIGHT_HIP_Z, 1, -64.42, 2.5), StepprActuator.RIGHT_HIP_X),
   RIGHT_HIP_Z("r_leg_uhz", 5, true, null, StepprActuator.RIGHT_HIP_Z),
   RIGHT_HIP_Y("r_leg_lhy", 10, true, new StrainGaugeInformation(StepprActuator.RIGHT_KNEE, 1, -112.45, 2.5), StepprActuator.RIGHT_HIP_Y),
   RIGHT_KNEE_Y("r_leg_kny", 6 * 1.3333333333, true, new StrainGaugeInformation(StepprActuator.RIGHT_KNEE, 0, -78.5*1.3333, 2.5), StepprActuator.RIGHT_KNEE, StepprActuator.RIGHT_ANKLE_RIGHT),
   RIGHT_ANKLE_Y("r_leg_uay", 6, true, new StrainGaugeInformation(StepprActuator.RIGHT_ANKLE_RIGHT, 2, -219.4*.12/6, 3.53), StepprActuator.RIGHT_ANKLE_RIGHT, StepprActuator.RIGHT_ANKLE_LEFT),
   RIGHT_ANKLE_X("r_leg_lax", RIGHT_ANKLE_Y.getRatio(), true, new StrainGaugeInformation(StepprActuator.RIGHT_ANKLE_RIGHT, 1, -217.4*.12/6, 4.03), RIGHT_ANKLE_Y.getActuators()),

   
   TORSO_Z("back_ubz", 120, false, null, StepprActuator.TORSO_Z),
   TORSO_Y("back_mby", 120, false, null, StepprActuator.TORSO_Y),
   TORSO_X("back_lbx", 120, false, null, StepprActuator.TORSO_X);
   
   
   public static final StepprJoint[] values = values();
   
   private boolean hasOutputEncoder;
   private String sdfName;
   private double ratio;   
   private StepprActuator[] actuators;
   private StrainGaugeInformation strainGaugeInformation;

   
   private StepprJoint(String sdfName, double ratio, boolean hasOutputEncoder, StrainGaugeInformation strainGaugeInformation, StepprActuator... actuators)
   {
      this.sdfName = sdfName;
      this.ratio = ratio;
      this.actuators = actuators;
      this.hasOutputEncoder = hasOutputEncoder;
      this.strainGaugeInformation = strainGaugeInformation;
     
   }
   
   public String getSdfName()
   {
      return sdfName;
   }
   
   public double getRatio()
   {	   
      return ratio;
   }
   
   public StepprActuator[] getActuators()
   {
      return actuators;
   }
   
   public boolean isLinear()
   {
      return actuators.length == 1;
   }
   
   public boolean hasOutputEncoder()
   {
      return hasOutputEncoder;
   }
   
   public StrainGaugeInformation getStrainGaugeInformation()
   {
      return strainGaugeInformation;
   }
   

}
