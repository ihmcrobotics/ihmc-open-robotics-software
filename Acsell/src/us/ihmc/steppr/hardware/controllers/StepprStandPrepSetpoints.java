package us.ihmc.steppr.hardware.controllers;

import us.ihmc.steppr.hardware.StepprActuator;
import us.ihmc.steppr.hardware.StepprJoint;

public enum StepprStandPrepSetpoints
{
   TORSO_Z(0, 11820, 0, 455.04, 1, StepprJoint.TORSO_Z),
   TORSO_Y(0.05, 16800, 0, 648, 1, StepprJoint.TORSO_Y),
   TORSO_X(0, 14890, 0, 573.12, 1, StepprJoint.TORSO_X),
   HIP_X(0.03, 2600, 0, 100, -1, StepprJoint.LEFT_HIP_X, StepprJoint.RIGHT_HIP_X),
   HIP_Z(0, 390, 0, 10, -1, StepprJoint.LEFT_HIP_Z, StepprJoint.RIGHT_HIP_Z), // damping must < 20
   HIP_Y(-0.24, 1546, 0, 89.3, 1, StepprJoint.LEFT_HIP_Y, StepprJoint.RIGHT_HIP_Y),
//   KNEE(0.45, 866, 0, 75, 1, StepprJoint.LEFT_KNEE_Y, StepprJoint.RIGHT_KNEE_Y), //original knee motors
   KNEE(0.45, 989, 0, 58, 1, StepprJoint.LEFT_KNEE_Y, StepprJoint.RIGHT_KNEE_Y), //v2 knee motors (same as hip y)
   ANKLE_X(0, 10, 0, 41.5, -1, StepprJoint.LEFT_ANKLE_X, StepprJoint.RIGHT_ANKLE_X),
   ANKLE_Y(-0.234, 1472, 0, 15, 1, StepprJoint.LEFT_ANKLE_Y, StepprJoint.RIGHT_ANKLE_Y);
//   TORSO_Z(0, 11820, 0, 455.04, 1, StepprJoint.TORSO_Z),
//   TORSO_Y(0, 16800, 0, 648, 1, StepprJoint.TORSO_Y),
//   TORSO_X(0, 14890, 0, 573.12, 1, StepprJoint.TORSO_X),
//   HIP_X(0.0, 2600, 0, 100, -1, StepprJoint.LEFT_HIP_X, StepprJoint.RIGHT_HIP_X),
//   HIP_Z(0, 390, 0, 4, -1, StepprJoint.LEFT_HIP_Z, StepprJoint.RIGHT_HIP_Z),
//   HIP_Y(0, 1546, 0, 89.3, 1, StepprJoint.LEFT_HIP_Y, StepprJoint.RIGHT_HIP_Y),
//   KNEE(0, 866, 0, 75, 1, StepprJoint.LEFT_KNEE_Y, StepprJoint.RIGHT_KNEE_Y),
//   ANKLE_X(0, 500, 0, 41.5, -1, StepprJoint.LEFT_ANKLE_X, StepprJoint.RIGHT_ANKLE_X),
//   ANKLE_Y(0, 1472, 0, 41.5, 1, StepprJoint.LEFT_ANKLE_Y, StepprJoint.RIGHT_ANKLE_Y);

   public static final StepprStandPrepSetpoints[] values = values();

   private final double q;
   private final double kp;
   private final double kd;
   private final double damping;
   private final double reflectRight;
   private final StepprJoint[] joints;
   public double motorScalingConstantFromDiagnostics = StepprActuator.values[0].motorScalingConstantFromDiagnostics;

   private StepprStandPrepSetpoints(double q, double kp, double kd, double damping, double reflectRight, StepprJoint... joints)
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

   public StepprJoint[] getJoints()
   {
      return joints;
   }

   public String getName()
   {
      return name().toLowerCase();
   }

}
