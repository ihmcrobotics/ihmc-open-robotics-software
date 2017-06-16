package us.ihmc.exampleSimulations.agileHexapod;

import us.ihmc.yoVariables.variable.YoDouble;

public class InverseFunction
{
   private static YoDouble DET = new YoDouble("DET", null);


   /*
    * InverseA()
    * takes the generalized force on the body...
    * ... and calculates the Minimum Force Set
    * input: generalized forces(6v.)
    * & joints angles(18v.)
    * & pointers to the minimum forces(6v.)
    * output: minimum force set values(6v.)
    */

   public static void InverseA(double pos_x1, double pos_x2, double pos_x3, double pos_y1, double pos_y2, double pos_y3, double fX, double fY, double fZ,
                               double tX, double tY, double tZ, double h11, double h21, double k1, double h12, double h22, double k2, double h13, double h23,
                               double k3, YoDouble fx1, YoDouble fz1, YoDouble fx2, YoDouble fy2, YoDouble fz2, YoDouble fz3)

   {
      double s42, s44, s45, s46, s51, s52;
      double s53, s55, s56, s61, s63, s64;

      double t11, t12, t16;
      double t21, t22, t23, t24, t25, t26;
      double t31, t32, t36;
      double t42;
      double t51, t52, t53, t54, t55, t56;
      double t61, t62, t63, t64, t65, t66;

      double cd;

      s42 = (pos_y1 + AgileHexapodRobot.LEN * Math.cos(h21) * (Math.sin(h11 + k1) + Math.sin(h11)));
      s44 = (-2.0 * AgileHexapodRobot.LEN * Math.sin(h11) * Math.sin(k1) - AgileHexapodRobot.LEN * Math.sin(h12) * Math.sin(k2)
             - AgileHexapodRobot.LEN * Math.sin(h13) * Math.sin(k3) + AgileHexapodRobot.LEN * Math.cos(h12) * (Math.cos(k2) + 1)
             + AgileHexapodRobot.LEN * Math.cos(h13) * (Math.cos(k3) + 1) + 2.0 * AgileHexapodRobot.LEN * Math.cos(h11) * (Math.cos(k1) + 1));
      s45 = (pos_y2 + AgileHexapodRobot.LEN * Math.cos(h22) * (Math.sin(h12 + k2) + Math.sin(h12)));
      s46 = (pos_y3 + AgileHexapodRobot.LEN * Math.cos(h23) * (Math.sin(h13 + k3) + Math.sin(h13)));
      s51 = (-AgileHexapodRobot.LEN * Math.cos(h11 + k1) - AgileHexapodRobot.LEN * Math.cos(h11));
      s52 = (-pos_x1 + AgileHexapodRobot.LEN * Math.sin(h21) * (Math.sin(h11 + k1) + Math.sin(h11)));
      s53 = (AgileHexapodRobot.LEN * Math.cos(h12) * (-Math.cos(k2) - 1) + AgileHexapodRobot.LEN * Math.cos(h13) * (-Math.cos(k3) - 1)
             + AgileHexapodRobot.LEN * Math.sin(h12) * Math.sin(k2) + AgileHexapodRobot.LEN * Math.sin(h13) * Math.sin(k3));
      s55 = (-pos_x2 + AgileHexapodRobot.LEN * Math.sin(h22) * (Math.sin(h12) * (Math.cos(k2) + 1) + Math.cos(h12) * Math.sin(k2)));
      s56 = (-pos_x3 + AgileHexapodRobot.LEN * Math.sin(h23) * (Math.sin(h13 + k3) + Math.sin(h13)));
      s61 = (-pos_y1 - AgileHexapodRobot.LEN * Math.cos(h21) * (Math.sin(h11 + k1) + Math.sin(h11)));
      s63 = (-pos_y2 - pos_y3 - AgileHexapodRobot.LEN * Math.cos(h22) * (Math.sin(h12 + k2) + Math.sin(h12))
             - AgileHexapodRobot.LEN * Math.cos(h23) * (Math.sin(h13 + k3) + Math.sin(h13)));
      s64 = (2.0 * pos_x1 + pos_x2 + pos_x3 - 2.0 * AgileHexapodRobot.LEN * Math.sin(h21) * (Math.sin(h11 + k1) + Math.sin(h11))
             - AgileHexapodRobot.LEN * Math.sin(h22) * (Math.sin(h12 + k2) + Math.sin(h12))
             - AgileHexapodRobot.LEN * Math.sin(h23) * (Math.sin(h13 + k3) + Math.sin(h13)));

      DET.set((4.0 * s63 - 8.0 * s61) * (s42 * (s55 - s56) + s45 * (-s52 + s56) + s46 * (s52 - s55)));

      if ((DET.getDoubleValue() > -0.001) && (DET.getDoubleValue() < 0.0))
         DET.set(-0.001);
      if ((DET.getDoubleValue() < 0.001) && (DET.getDoubleValue() > 0.0))
         DET.set(0.001);

      cd = 4.0 * s63 - 8.0 * s61;

      t11 = (4.0 * s63 * DET.getDoubleValue()) / (cd);
      t12 = (2.0 * s64 * DET.getDoubleValue()) / (cd);
      t16 = (-8.0 * DET.getDoubleValue()) / (cd);

      t21 = 4.0 * (s53 * s61 - s51 * s63) * (s46 - s45);
      t22 = s51 * s64 * (-2.0 * s46 + 2.0 * s45) + s53 * s64 * (-s45 + s46) + s55 * s44 * (-s63 + 2.0 * s61) + s56 * s44 * (-2.0 * s61 + s63);
      t23 = cd * (s45 * s56 - s46 * s55);
      t24 = cd * (s55 - s56);
      t25 = cd * (-s45 + s46);
      t26 = (4.0 * s53 - 8.0 * s51) * (s45 - s46);

      t31 = (-4.0 * s61 * DET.getDoubleValue()) / (cd);
      t32 = (-s64 * DET.getDoubleValue()) / (cd);
      t36 = (4.0 * DET.getDoubleValue()) / (cd);

      t42 = DET.getDoubleValue() / 4.0;

      t51 = 4.0 * (s46 - s42) * (s51 * s63 - s61 * s53);
      t52 = (2.0 * s51 - s53) * (-s42 * s64 + s46 * s64) + (2.0 * s61 - s63) * (s56 * s44 - s52 * s44);
      t53 = cd * (-s42 * s56 + s46 * s52);
      t54 = cd * (s56 - s52);
      t55 = cd * (-s46 + s42);
      t56 = (4.0 * s53 - 8.0 * s51) * (s46 - s42);

      t61 = 4.0 * (s63 * s51 - s61 * s53) * (s42 - s45);
      t62 = (2.0 * s51 - s53) * (-s45 * s64 + s42 * s64) + (2.0 * s61 - s63) * (s52 * s44 - s55 * s44);
      t63 = cd * (s42 * s55 - s45 * s52);
      t64 = cd * (s52 - s55);
      t65 = cd * (s45 - s42);
      t66 = (8.0 * s51 - 4.0 * s53) * (s45 - s42);


      fx1.set((1.0 / DET.getDoubleValue()) * (t11 * fX + t12 * fY + t16 * tZ));
      fz1.set((1.0 / DET.getDoubleValue()) * (t21 * fX + t22 * fY + t23 * fZ + t24 * tX + t25 * tY + t26 * tZ));
      fx2.set((1.0 / DET.getDoubleValue()) * (t31 * fX + t32 * fY + t36 * tZ));
      fy2.set((1.0 / DET.getDoubleValue()) * (t42 * fY));
      fz2.set((1.0 / DET.getDoubleValue()) * (t51 * fX + t52 * fY + t53 * fZ + t54 * tX + t55 * tY + t56 * tZ));
      fz3.set((1.0 / DET.getDoubleValue()) * (t61 * fX + t62 * fY + t63 * fZ + t64 * tX + t65 * tY + t66 * tZ));


      /* test it */

      /*
       *   ls.sminv_err_1 = (*fx1) + 2.0* (*fx2) - fX;
       *   ls.sminv_err_2 = 4.0 * (*fy2) - fY;
       *   ls.sminv_err_3 = (*fz1) + (*fz2) + (*fz3) - fZ;
       *
       *   ls.sminv_err_4 = s42*(*fz1) + s44*(*fy2) + s45*(*fz2) + s46*(*fz3) - tX;
       *   ls.sminv_err_5 = s51*(*fx1) + s52*(*fz1) + s53*(*fx2) + s55*(*fz2) + s56*(*fz3) - tY;
       *   ls.sminv_err_6 = s61*(*fx1) + s63*(*fx2) + s64*(*fy2) - tZ;
       */

   }


}
