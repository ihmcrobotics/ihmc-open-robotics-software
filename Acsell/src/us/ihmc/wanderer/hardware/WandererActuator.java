package us.ihmc.wanderer.hardware;

import us.ihmc.acsell.hardware.AcsellActuator;

//X,Y,Z,K,L,R
public enum WandererActuator implements AcsellActuator
{
   RIGHT_HIP_X("rightHipXActuator",             1.551e-3, 2.286, 0.749, 11.0, 0, 0, -2.0*Math.PI/16384.0, +2.0*Math.PI/16384.0, 16017.0),
   RIGHT_HIP_Y("rightHipYActuator",             2.142e-3, 0.612, 0.991, 50.0, 0, 1, -2.0*Math.PI/16384.0, -2.0*Math.PI/16384.0, 12982.4),
   RIGHT_HIP_Z("rightHipZActuator",             3.55e-4,  0.702, 0.299, 18.1, 0, 2, +2.0*Math.PI/16384.0, -2.0*Math.PI/16384.0, 10894.0),
   RIGHT_KNEE("rightKneeActuator",              2.142e-3, 0.612, 0.991, 50.0, 0, 3, -2.0*Math.PI/16384.0, -2.0*Math.PI/16384.0, 12987.0),
   RIGHT_ANKLE_LEFT("rightAnkleLeftActuator",   1.335e-3, 1.152, 0.587, 26.0, 0, 4, +2.0*Math.PI/16384.0, -2.0*Math.PI/16384.0, 496),//484
   RIGHT_ANKLE_RIGHT("rightAnkleRightActuator", 1.335e-3, 1.152, 0.587, 26.0, 0, 5, +2.0*Math.PI/16384.0, -2.0*Math.PI/16384.0, 6471.5),

   LEFT_HIP_X("leftHipXActuator",             1.551e-3, 2.286, 0.749, 11.0, 1, 0, -2.0*Math.PI/16384.0, +2.0*Math.PI/16384.0, 6850.0),
   LEFT_HIP_Y("leftHipYActuator",             2.142e-3, 0.612, 0.991, 50.0, 1, 1, +2.0*Math.PI/16384.0, +2.0*Math.PI/16384.0, 11218.6),
   LEFT_HIP_Z("leftHipZActuator",             3.55e-4,  0.702, 0.299, 18.1, 1, 2, +2.0*Math.PI/16384.0, -2.0*Math.PI/16384.0, 14826.0),
   LEFT_KNEE("leftKneeActuator",              2.142e-3, 0.612, 0.991, 50.0, 1, 3, +2.0*Math.PI/16384.0, +2.0*Math.PI/16384.0, 15603.0),
   LEFT_ANKLE_LEFT("leftAnkleLeftActuator",   1.335e-3, 1.152, 0.587, 26.0, 1, 4, -2.0*Math.PI/16384.0, +2.0*Math.PI/16384.0, 16033.0),//15975.0
   LEFT_ANKLE_RIGHT("leftAnkleRightActuator", 1.335e-3, 1.152, 0.587, 26.0, 1, 5, -2.0*Math.PI/16384.0, -2.0*Math.PI/16384.0, 2390.1),

   TORSO_Z("torsoZActuator", 0.069e-3, 0.316, .104, 10.8, 2, 3, +2.0*Math.PI/16384.0, 0*2.0*Math.PI/16384.0, 0),
   TORSO_Y("torsoYActuator", 0.197e-3, 0.450, .192, 18.3, 2, 2, +2.0*Math.PI/16384.0, +2.0*Math.PI/16384.0, 9130.0),
   TORSO_X("torsoXActuator", 0.069e-3, 0.398, .104,  6.8, 2, 1, -2.0*Math.PI/16384.0, -2.0*Math.PI/16384.0, 130.0);
   

   public static final WandererActuator[] values = values();

   public static final double VELOCITY_SCALE = 1000000.0;
   
   private final String name;
   private final double motorInertia;
   private final double ktSinesoidal;
   private final double Km;
   private final double currentLimit;
   private final int bus;
   private final int index;
   public static final double motorScalingConstantFromDiagnostics = 1.0;
   private final double motorEncoderScale;
   private final double jointEncoderScale;
   private final double jointEncoderOffset;

   private WandererActuator(String name, double motorInertial, double ktPeak, double km, double currentLimit, int bus, int index, double motorEncoderScale, double jointEncoderScale, double jointEncoderOffset)
   {
      this.name = name;
      this.motorInertia = motorInertial;
      this.ktSinesoidal = (ktPeak * Math.sqrt(3.0) / 2.0) / motorScalingConstantFromDiagnostics;
      this.Km = km;
      this.currentLimit = currentLimit;
      this.bus = bus;
      this.index = index;
      this.motorEncoderScale = motorEncoderScale;
      this.jointEncoderScale = jointEncoderScale;
      this.jointEncoderOffset = jointEncoderOffset;
   }

   public String getName()
   {
      return name;
   }

   public double getKt()
   {
      return ktSinesoidal;
   }

   public double getKm()
   {
      return Km;
   }

   public int getBus()
   {
      return bus;
   }

   public int getIndex()
   {
      return index;
   }

   public int getSensedCurrentToTorqueDirection()
   {
      return 1;
   }

   public double getMotorInertia()
   {
      return motorInertia;
   }
   
   public double getCurrentLimit()
   {
      return currentLimit;
   }

   @Override
   public double getJointEncoderScale()
   {
      return jointEncoderScale;
   }

   @Override
   public double getMotorEncoderScale()
   {
      return motorEncoderScale;
   }
   
   @Override
   public double getDampingSign()
   {
      return Math.signum(motorEncoderScale);
   }   

   @Override
   public double getJointEncoderOffset()
   {
      return jointEncoderOffset;
   }

}
