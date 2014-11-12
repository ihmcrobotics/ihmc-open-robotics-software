package us.ihmc.steppr.hardware;

public enum StepprJoint
{
   LEFT_HIP_X("l_leg_mhx", 10, StepprActuator.LEFT_HIP_X),
   LEFT_HIP_Z("l_leg_uhz", 5, StepprActuator.LEFT_HIP_Z),
   LEFT_HIP_Y("l_leg_lhy", 10, StepprActuator.LEFT_HIP_Y),
   LEFT_KNEE_Y("l_leg_kny", 6 * 1.3333333333, StepprActuator.LEFT_KNEE, StepprActuator.LEFT_ANKLE_LEFT),
   LEFT_ANKLE_Y("l_leg_uay", 6, StepprActuator.LEFT_ANKLE_RIGHT, StepprActuator.LEFT_ANKLE_LEFT),
   LEFT_ANKLE_X("l_leg_lax", LEFT_ANKLE_Y.getRatio(), LEFT_ANKLE_Y.getActuators()),

   RIGHT_HIP_X("r_leg_mhx", 10, StepprActuator.RIGHT_HIP_X),
   RIGHT_HIP_Z("r_leg_uhz", 5, StepprActuator.RIGHT_HIP_Z),
   RIGHT_HIP_Y("r_leg_lhy", 10, StepprActuator.RIGHT_HIP_Y),
   RIGHT_KNEE_Y("r_leg_kny", 6 * 1.3333333333, StepprActuator.RIGHT_KNEE, StepprActuator.RIGHT_ANKLE_RIGHT),
   RIGHT_ANKLE_Y("r_leg_uay", 6, StepprActuator.RIGHT_ANKLE_RIGHT, StepprActuator.RIGHT_ANKLE_LEFT),
   RIGHT_ANKLE_X("r_leg_lax", RIGHT_ANKLE_Y.getRatio(), RIGHT_ANKLE_Y.getActuators()),

   
   TORSO_Z("back_ubz", 120, StepprActuator.TORSO_Z),
   TORSO_Y("back_mby", 120, StepprActuator.TORSO_Y),
   TORSO_X("back_lbx", 120, StepprActuator.TORSO_X);
   
   
   public static final StepprJoint[] values = values();
   
   private String sdfName;
   private double ratio;
   private StepprActuator[] actuators;
   
   private StepprJoint(String sdfName, double ratio, StepprActuator... actuators)
   {
      this.sdfName = sdfName;
      this.ratio = ratio;
      this.actuators = actuators;
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
}
