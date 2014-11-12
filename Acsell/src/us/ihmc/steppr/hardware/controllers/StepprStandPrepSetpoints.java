package us.ihmc.steppr.hardware.controllers;

import us.ihmc.steppr.hardware.StepprJoint;

public enum StepprStandPrepSetpoints
{
   TORSO_Z(0, 13650, 0, 455.04, 1, StepprJoint.TORSO_Z),
   TORSO_Y(0, 19440, 0, 648, 1, StepprJoint.TORSO_Y),
   TORSO_X(0, 17193, 0, 573.12, 1, StepprJoint.TORSO_X),
   HIP_X(0.12, 4500, 0, 100, -1, StepprJoint.LEFT_HIP_X, StepprJoint.RIGHT_HIP_X),
   HIP_Z(0, 758.16, 0, 4, -1, StepprJoint.LEFT_HIP_Z, StepprJoint.RIGHT_HIP_Z),
   HIP_Y(-0.24, 1786, 0, 89.3, 1, StepprJoint.LEFT_HIP_Y, StepprJoint.RIGHT_HIP_Y),
   KNEE(0.45, 1000, 0, 75, 1, StepprJoint.LEFT_KNEE_Y, StepprJoint.RIGHT_KNEE_Y),
   ANKLE_X(0, 10, 0, 41.5, -1, StepprJoint.LEFT_ANKLE_X, StepprJoint.RIGHT_ANKLE_X),
   ANKLE_Y(-0.23, 1700, 0, 41.5, 1, StepprJoint.LEFT_ANKLE_Y, StepprJoint.RIGHT_ANKLE_Y);

   public static final StepprStandPrepSetpoints[] values = values();

   private final double q;
   private final double kp;
   private final double kd;
   private final double damping;
   private final double reflectRight;
   private final StepprJoint[] joints;

   private StepprStandPrepSetpoints(double q, double kp, double kd, double damping, double reflectRight, StepprJoint... joints)
   {
      this.q = q;
      this.kp = kp;
      this.kd = kd;
      this.damping = damping;
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
