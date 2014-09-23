package us.ihmc.steppr.hardware;

public enum StepprActuator
{
   LEFT_ANKLE_RIGHT("leftAnkleRightActuator", 1.152),
   LEFT_ANKLE_LEFT("leftAnkleLeftActuator", 1.152),
   LEFT_KNEE("leftKneeActuator", 2.286),
   LEFT_HIP_Y("leftHipYActuator", 0.893),
   LEFT_HIP_Z("leftHipZActuator", 0.702),
   LEFT_HIP_X("leftHipXActuator", 2.286),
   
   RIGHT_ANKLE_RIGHT("rightAnkleRightActuator", 1.152),
   RIGHT_ANKLE_LEFT("rightAnkleLeftActuator", 1.152),
   RIGHT_KNEE("rightKneeActuator", 2.286),
   RIGHT_HIP_Y("rightHipYActuator", 0.893),
   RIGHT_HIP_Z("rightHipZActuator", 0.702),
   RIGHT_HIP_X("rightHipXActuator", 2.286),
   
   TORSO_X("torsoXActuator", 0.398),
   TORSO_Y("torsoYActuator", 0.45),
   TORSO_Z("torsoZActuator", 0.316)
   ;
   public static final StepprActuator[] values = values();
   
   private final String name;
   private final double kt;
   private StepprActuator(String name, double kt)
   {
      this.name = name;
      this.kt = kt;
   }
   
   public String getName()
   {
      return name;
   }
   
   public double getKt()
   {
      return kt;
   }
}
