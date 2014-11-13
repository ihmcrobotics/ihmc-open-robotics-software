package us.ihmc.steppr.hardware;

public enum StepprActuator
{
   LEFT_ANKLE_RIGHT("leftAnkleRightActuator", 1.152, 0, 1),
   LEFT_ANKLE_LEFT("leftAnkleLeftActuator", 1.152, 0, 2),
   LEFT_KNEE("leftKneeActuator", 2.286, 0, 3),
   LEFT_HIP_Y("leftHipYActuator", 0.893, 0, 4),
   LEFT_HIP_Z("leftHipZActuator", 0.702, 0, 5),
   LEFT_HIP_X("leftHipXActuator", 2.286, 0, 6),
   
   RIGHT_ANKLE_RIGHT("rightAnkleRightActuator", 1.152, 1, 1),
   RIGHT_ANKLE_LEFT("rightAnkleLeftActuator", 1.152, 1, 2),
   RIGHT_KNEE("rightKneeActuator", 2.286, 1, 3),
   RIGHT_HIP_Y("rightHipYActuator", 0.893, 1, 4),
   RIGHT_HIP_Z("rightHipZActuator", 0.702, 1, 5),
   RIGHT_HIP_X("rightHipXActuator", 2.286, 1, 6),
   
   TORSO_X("torsoXActuator", 0.398, 2, 1),
   TORSO_Y("torsoYActuator", 0.45, 2, 2),
   TORSO_Z("torsoZActuator", 0.316, 2, 3)
   ;
   public static final StepprActuator[] values = values();
   
   private final String name;
   private final double ktSinesoidal;
   private final int bus;
   private final int index;
   
   private StepprActuator(String name, double ktPeak, int bus, int index)
   {
      this.name = name;
      this.ktSinesoidal = ktPeak * Math.sqrt(3.0) / 2.0;
      this.bus = bus;
      this.index = index;
   }
   
   public String getName()
   {
      return name;
   }
   
   public double getKt()
   {
      return ktSinesoidal;
   }

   public int getBus()
   {
      return bus;
   }

   public int getIndex()
   {
      return index;
   }
   
   
}
