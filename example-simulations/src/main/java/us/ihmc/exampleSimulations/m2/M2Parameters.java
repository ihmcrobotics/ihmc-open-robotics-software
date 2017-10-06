package us.ihmc.exampleSimulations.m2;


public class M2Parameters
{
   // Some Conversion Factors:
   public double POUNDS = 1.0 / 2.205;
   public double INCHES = 0.0254;

   public Parameter BODY_CYLINDER_HEIGHT = new Parameter(), BODY_R = new Parameter(), BODY_ELLIPSE_HEIGHT = new Parameter(), THIGH_R = new Parameter(),
                    SHIN_R = new Parameter(), HIP_TO_THIGH_OFF = new Parameter(), THIGH_LENGTH = new Parameter(), SHIN_LENGTH = new Parameter(),
                    ANKLE_JOINT_OFF = new Parameter(), FOOT_HEIGHT = new Parameter(), FOOT_WIDTH = new Parameter(), FOOT_FORWARD = new Parameter(),
                    FOOT_BACK = new Parameter(), FOOT_LENGTH = new Parameter(), HIP_SPACING = new Parameter(), HIP_JOINT_OFF = new Parameter(),
                    HIP_OFFSET_Y = new Parameter(), BODY_CG_Z = new Parameter(), BODY_MASS = new Parameter(), WAIST_MASS = new Parameter(),
                    THIGH_MASS = new Parameter(), SHIN_MASS = new Parameter(), RETINACULUM_MASS = new Parameter(), FOOT_MASS = new Parameter(),
                    BODY_Ixx = new Parameter(), BODY_Iyy = new Parameter(), BODY_Izz = new Parameter(), WAIST_Ixx = new Parameter(),
                    WAIST_Iyy = new Parameter(), WAIST_Izz = new Parameter(), THIGH_Ixx = new Parameter(), THIGH_Iyy = new Parameter(),
                    THIGH_Izz = new Parameter(), SHIN_Ixx = new Parameter(), SHIN_Iyy = new Parameter(), SHIN_Izz = new Parameter(),
                    RETINACULUM_Ixx = new Parameter(), RETINACULUM_Iyy = new Parameter(), RETINACULUM_Izz = new Parameter(), FOOT_Ixx = new Parameter(),
                    FOOT_Iyy = new Parameter(), FOOT_Izz = new Parameter(), BODY_COM_X = new Parameter(), BODY_COM_Y = new Parameter(),
                    BODY_COM_Z = new Parameter(), WAIST_COM_X = new Parameter(), WAIST_COM_Y = new Parameter(), WAIST_COM_Z = new Parameter(),
                    THIGH_COM_X = new Parameter(), L_THIGH_COM_Y = new Parameter(), R_THIGH_COM_Y = new Parameter(), THIGH_COM_Z = new Parameter(),
                    SHIN_COM_X = new Parameter(), SHIN_COM_Y = new Parameter(), SHIN_COM_Z = new Parameter(), RETINACULUM_COM_X = new Parameter(),
                    RETINACULUM_COM_Y = new Parameter(), RETINACULUM_COM_Z = new Parameter(), FOOT_COM_X = new Parameter(), FOOT_COM_Y = new Parameter(),
                                           FOOT_COM_Z = new Parameter()
   ;


   public Parameter[] allParameters = new Parameter[]
   {
      BODY_CYLINDER_HEIGHT, BODY_R, BODY_ELLIPSE_HEIGHT, THIGH_R, SHIN_R, HIP_TO_THIGH_OFF, THIGH_LENGTH, SHIN_LENGTH, ANKLE_JOINT_OFF, FOOT_HEIGHT, FOOT_WIDTH,
      FOOT_FORWARD, FOOT_BACK, FOOT_LENGTH, HIP_SPACING, HIP_JOINT_OFF, HIP_OFFSET_Y, BODY_CG_Z, BODY_MASS, WAIST_MASS, THIGH_MASS, SHIN_MASS, RETINACULUM_MASS,
      FOOT_MASS, BODY_Ixx, BODY_Iyy, BODY_Izz, WAIST_Ixx, WAIST_Iyy, WAIST_Izz, THIGH_Ixx, THIGH_Iyy, THIGH_Izz, SHIN_Ixx, SHIN_Iyy, SHIN_Izz, RETINACULUM_Ixx,
      RETINACULUM_Iyy, RETINACULUM_Izz, FOOT_Ixx, FOOT_Iyy, FOOT_Izz, BODY_COM_X, BODY_COM_Y, BODY_COM_Z, WAIST_COM_X, WAIST_COM_Y, WAIST_COM_Z, THIGH_COM_X,
      L_THIGH_COM_Y, R_THIGH_COM_Y, THIGH_COM_Z, SHIN_COM_X, SHIN_COM_Y, SHIN_COM_Z, RETINACULUM_COM_X, RETINACULUM_COM_Y, RETINACULUM_COM_Z, FOOT_COM_X,
      FOOT_COM_Y, FOOT_COM_Z
   };


   public static M2Parameters mergeParameters(M2Parameters params1, M2Parameters params2, double mergePercent)
   {
      M2Parameters ret = new M2Parameters();

      for (int i = 0; i < params1.allParameters.length; i++)
      {
         ret.allParameters[i].value = (1.0 - mergePercent) * params1.allParameters[i].value + mergePercent * params2.allParameters[i].value;
      }

      return ret;

   }
}
