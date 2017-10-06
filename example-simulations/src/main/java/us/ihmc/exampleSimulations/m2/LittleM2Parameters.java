package us.ihmc.exampleSimulations.m2;

public class LittleM2Parameters extends M2Parameters
{
   public LittleM2Parameters()
   {
      BODY_CYLINDER_HEIGHT.value = 0.0381;
      BODY_R.value = 0.203;
      BODY_ELLIPSE_HEIGHT.value = 0.381;
      THIGH_R.value = 0.051;
      SHIN_R.value = 0.051;
      HIP_TO_THIGH_OFF.value = 0.0064;
      THIGH_LENGTH.value = 0.432;
      SHIN_LENGTH.value = 0.432;
      ANKLE_JOINT_OFF.value = 0.0254;
      FOOT_HEIGHT.value = 0.051;
      FOOT_WIDTH.value = 0.0889;
      FOOT_FORWARD.value = 0.1524;
      FOOT_BACK.value = 0.051;
      FOOT_LENGTH.value = FOOT_FORWARD.value + FOOT_BACK.value;
      HIP_SPACING.value = 0.184;
      HIP_JOINT_OFF.value = 0.0254;
      HIP_OFFSET_Y.value = HIP_SPACING.value / 2.0;

      BODY_CG_Z.value = 0.20;

      BODY_MASS.value = 12.1011;    // (FULL_MASS * UPPER_TO_LOWER_RATIO) * 0.225;
      WAIST_MASS.value = 0.325575;    // (FULL_MASS / UPPER_TO_LOWER_RATIO) * 0.2;
      THIGH_MASS.value = 2.735745;    // (FULL_MASS / UPPER_TO_LOWER_RATIO) * 0.0625;
      SHIN_MASS.value = 2.69484;    // (FULL_MASS / UPPER_TO_LOWER_RATIO) * 0.0625;
      RETINACULUM_MASS.value = 0.250041;    // (FULL_MASS * UPPER_TO_LOWER_RATIO) * 0.225;
      FOOT_MASS.value = 0.414988;    // (FULL_MASS / UPPER_TO_LOWER_RATIO) * 0.025;

      BODY_Ixx.value = 0.019 * BODY_MASS.value;
      BODY_Iyy.value = 0.019 * BODY_MASS.value;
      BODY_Izz.value = 0.017 * BODY_MASS.value;
      WAIST_Ixx.value = 0.000529262;
      WAIST_Iyy.value = 0.000529262;
      WAIST_Izz.value = 0.000529262;
      THIGH_Ixx.value = 0.0443252;
      THIGH_Iyy.value = 0.0443252;
      THIGH_Izz.value = 0.00355784;
      SHIN_Ixx.value = 0.054163;
      SHIN_Iyy.value = 0.054163;
      SHIN_Izz.value = 0.003457;
      RETINACULUM_Ixx.value = 0.000260143;
      RETINACULUM_Iyy.value = 0.000260143;
      RETINACULUM_Izz.value = 0.000260143;
      FOOT_Ixx.value = 0.00036326;
      FOOT_Iyy.value = 0.00152067;
      FOOT_Izz.value = 0.00170404;

      BODY_COM_X.value = 0.0;
      BODY_COM_Y.value = 0.0;
      BODY_COM_Z.value = 0.159854;
      WAIST_COM_X.value = 0.0;
      WAIST_COM_Y.value = 0.0;
      WAIST_COM_Z.value = 0.0;
      THIGH_COM_X.value = 0.0;
      L_THIGH_COM_Y.value = 0.006400;
      R_THIGH_COM_Y.value = -0.006400;
      THIGH_COM_Z.value = -0.21600;
      SHIN_COM_X.value = 0.0;
      SHIN_COM_Y.value = 0.0;
      SHIN_COM_Z.value = -0.181082;
      RETINACULUM_COM_X.value = 0.0;
      RETINACULUM_COM_Y.value = 0.0;
      RETINACULUM_COM_Z.value = 0.0;
      FOOT_COM_X.value = 0.050700;
      FOOT_COM_Y.value = 0.0;
      FOOT_COM_Z.value = -0.025500;
   }

}
