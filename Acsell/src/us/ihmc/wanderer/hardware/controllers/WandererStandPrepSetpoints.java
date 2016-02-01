package us.ihmc.wanderer.hardware.controllers;

import us.ihmc.wanderer.hardware.WandererActuator;
import us.ihmc.wanderer.hardware.WandererJoint;

public enum WandererStandPrepSetpoints
{
   
   TORSO_Z(      0,  6800,    0, 432*.316,  1, WandererJoint.TORSO_Z),
   TORSO_Y(   0.05,  9000,    0, 648*.450,  1, WandererJoint.TORSO_Y),
   TORSO_X(      0,  4500,    0, 576*.398,  1, WandererJoint.TORSO_X),
   HIP_X(     0.05,   500, 25.0,    22.86, -1, WandererJoint.LEFT_HIP_X, WandererJoint.RIGHT_HIP_X),
   HIP_Z(        0,   125, 6.25,   0.1755, -1, WandererJoint.LEFT_HIP_Z, WandererJoint.RIGHT_HIP_Z), // damping must < 20
   HIP_Y(    -0.234,   500, 25.0,   0.6120,  1, WandererJoint.LEFT_HIP_Y, WandererJoint.RIGHT_HIP_Y),
   KNEE(      0.4419,   800, 6.0,   1.2,  1, WandererJoint.LEFT_KNEE_Y, WandererJoint.RIGHT_KNEE_Y), //v2 knee motors (same as hip y)
   ANKLE_Y( -0.234,   600,  9.0,   5.8061,  1, WandererJoint.LEFT_ANKLE_Y, WandererJoint.RIGHT_ANKLE_Y),
   ANKLE_X(      0,    10,  5.0,   5.8061, -1, WandererJoint.LEFT_ANKLE_X, WandererJoint.RIGHT_ANKLE_X);

   public static final WandererStandPrepSetpoints[] values = values();

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
      this.kd = kd / motorScalingConstantFromDiagnostics;
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
