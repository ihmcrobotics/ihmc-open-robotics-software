package us.ihmc.wanderer.hardware.controllers;

import us.ihmc.wanderer.hardware.WandererActuator;
import us.ihmc.wanderer.hardware.WandererJoint;

public enum WandererStandPrepSetpoints
{
   
   TORSO_Z(      0, 11820,    0, 455.04,  1, WandererJoint.TORSO_Z),
   TORSO_Y(   0.05, 16800,    0,    648,  1, WandererJoint.TORSO_Y),
   TORSO_X(      0, 14890,    0, 573.12,  1, WandererJoint.TORSO_X),
   HIP_X(     0.03,   500,  250,    100, -1, WandererJoint.LEFT_HIP_X, WandererJoint.RIGHT_HIP_X),
   HIP_Z(        0,   125, 62.5,     10, -1, WandererJoint.LEFT_HIP_Z, WandererJoint.RIGHT_HIP_Z), // damping must < 20
   HIP_Y(    -0.24,   500,  250,   89.3,  1, WandererJoint.LEFT_HIP_Y, WandererJoint.RIGHT_HIP_Y),
   KNEE(      0.45,   320,  160,     58,  1, WandererJoint.LEFT_KNEE_Y, WandererJoint.RIGHT_KNEE_Y), //v2 knee motors (same as hip y)
   ANKLE_Y( -0.234,   180,   90,     15,  1, WandererJoint.LEFT_ANKLE_Y, WandererJoint.RIGHT_ANKLE_Y),
   ANKLE_X(      0,    10,    5,   41.5, -1, WandererJoint.LEFT_ANKLE_X, WandererJoint.RIGHT_ANKLE_X);

   public static final WandererStandPrepSetpoints[] values = values();

   public static final double VELOCITY_SCALE = 100000.0;
   private final double q;
   private final double kp;
   private final double kd;
   private final double damping;
   private final double reflectRight;
   private final WandererJoint[] joints;
   public double motorScalingConstantFromDiagnostics = WandererActuator.motorScalingConstantFromDiagnostics;

   private WandererStandPrepSetpoints(double q, double kp, double kd, double damping, double reflectRight, WandererJoint... joints)
   {
      
      this.q = q;
      this.kp = kp / motorScalingConstantFromDiagnostics;
      this.kd = kd / motorScalingConstantFromDiagnostics * WandererStandPrepSetpoints.VELOCITY_SCALE;
      this.damping = damping / motorScalingConstantFromDiagnostics;
      this.reflectRight = reflectRight;
      this.joints = joints;
   }

   public double getQ()
   {
      return q;
   }

   public double getKp()
   {
      return kp;
   }

   public double getKd()
   {
      return kd;
   }

   public double getDamping()
   {
      return damping;
   }

   public double getReflectRight()
   {
      return reflectRight;
   }

   public WandererJoint[] getJoints()
   {
      return joints;
   }

   public String getName()
   {
      return name().toLowerCase();
   }

}
