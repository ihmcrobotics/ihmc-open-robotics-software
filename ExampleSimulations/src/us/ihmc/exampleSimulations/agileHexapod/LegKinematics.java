package us.ihmc.exampleSimulations.agileHexapod;

import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;

/* ** leg_kinematics.c                                   * * */
/* ** Kinematics for one Bug Leg                         * **/
/* ** Controls the position of the foot wrt to the body  * **/
/* ** this code includes calculation of:                 * **/
/* **      kinematics                                    * **/
/* **      jacobian                                      * **/
/* **      force                                         * **/
/* **      torque                                        * **/


public class LegKinematics
{
   /** Kinematics */

/* finding x,y,z of foot as a function of hip(x&z) and knee angles */

   public static void leg_kin(double hip1, double hip2, double knee, double pos_x, double pos_y, DoubleYoVariable foot_x, DoubleYoVariable foot_y, DoubleYoVariable foot_z)
   {
      foot_x.set(pos_x - Math.sin(hip1) * Math.sin(hip2) * AgileHexapodRobot.LEN - Math.cos(hip1) * Math.sin(hip2) * Math.sin(knee) * AgileHexapodRobot.LEN
                   - Math.sin(hip1) * Math.sin(hip2) * Math.cos(knee) * AgileHexapodRobot.LEN);

      foot_y.set(pos_y + Math.sin(hip1) * Math.cos(hip2) * AgileHexapodRobot.LEN + Math.cos(hip1) * Math.cos(hip2) * Math.sin(knee) * AgileHexapodRobot.LEN
                   + Math.sin(hip1) * Math.cos(hip2) * Math.cos(knee) * AgileHexapodRobot.LEN);

      foot_z.set(-Math.cos(hip1) * Math.cos(knee) * AgileHexapodRobot.LEN + Math.sin(hip1) * Math.sin(knee) * AgileHexapodRobot.LEN
                   - Math.cos(hip1) * AgileHexapodRobot.LEN);

   }



   /** Jacobian Tranformation */

/* finding x,y,z velocities given joint angles and angular speeds */

   public static void jacobian(double hip1, double hip2, double knee, double dhip1, double dhip2, double dknee, DoubleYoVariable vel_x, DoubleYoVariable vel_y,
                               DoubleYoVariable vel_z)

   {
      double J11, J12, J13;
      double J21, J22, J23;
      double J31, J32, J33;


      J11 = -Math.cos(hip1) * Math.sin(hip2) * AgileHexapodRobot.LEN - Math.cos(hip1) * Math.sin(hip2) * Math.cos(knee) * AgileHexapodRobot.LEN
            + Math.sin(hip1) * Math.sin(hip2) * Math.sin(knee) * AgileHexapodRobot.LEN;
      J12 = -Math.sin(hip1) * Math.cos(hip2) * AgileHexapodRobot.LEN - Math.sin(hip1) * Math.cos(hip2) * Math.cos(knee) * AgileHexapodRobot.LEN
            - Math.cos(hip1) * Math.cos(hip2) * Math.sin(knee) * AgileHexapodRobot.LEN;
      J13 = Math.sin(hip1) * Math.sin(hip2) * Math.sin(knee) * AgileHexapodRobot.LEN - Math.cos(hip1) * Math.sin(hip2) * Math.cos(knee) * AgileHexapodRobot.LEN;

      J21 = Math.cos(hip1) * Math.cos(hip2) * AgileHexapodRobot.LEN + Math.cos(hip1) * Math.cos(hip2) * Math.cos(knee) * AgileHexapodRobot.LEN
            - Math.sin(hip1) * Math.cos(hip2) * Math.sin(knee) * AgileHexapodRobot.LEN;
      J22 = -Math.sin(hip1) * Math.sin(hip2) * AgileHexapodRobot.LEN - Math.sin(hip1) * Math.sin(hip2) * Math.cos(knee) * AgileHexapodRobot.LEN
            - Math.cos(hip1) * Math.sin(hip2) * Math.sin(knee) * AgileHexapodRobot.LEN;
      J23 = -Math.sin(hip1) * Math.cos(hip2) * Math.sin(knee) * AgileHexapodRobot.LEN
            + Math.cos(hip1) * Math.cos(hip2) * Math.cos(knee) * AgileHexapodRobot.LEN;

      J31 = Math.sin(hip1) * Math.cos(knee) * AgileHexapodRobot.LEN + Math.cos(hip1) * Math.sin(knee) * AgileHexapodRobot.LEN
            + Math.sin(hip1) * AgileHexapodRobot.LEN;
      J32 = 0;
      J33 = Math.cos(hip1) * Math.sin(knee) * AgileHexapodRobot.LEN + Math.sin(hip1) * Math.cos(knee) * AgileHexapodRobot.LEN;



      vel_x.set(J11 * dhip1 + J12 * dhip2 + J13 * dknee);
      vel_y.set(J21 * dhip1 + J22 * dhip2 + J23 * dknee);
      vel_z.set(J31 * dhip1 + J32 * dhip2 + J33 * dknee);


   }


   public static void virtual_f(double x_d, double y_d, double z_d, double dz_d, double x, double y, double z, double dx, double dy, double dz, double K_leg,
                                double B_leg, double Kz_leg, double Bz_leg, DoubleYoVariable Fx, DoubleYoVariable Fy, DoubleYoVariable Fz)


   {
//    ls.K_leg & ls.B_leg are spring and damping constants set in simulation
//    ls.Kz_leg & ls.Bz_leg are spring and damping constants set in bug_walk.c
//    the motion of x and y are dictated uMath.sing position control
//    the motion of z is dictated uMath.sing position or velocity control

      Fx.set(K_leg * (x_d - x) - B_leg * (dx));
      Fy.set(K_leg * (y_d - y) - B_leg * (dy));
      Fz.set(Kz_leg * (z_d - z) + Bz_leg * (dz_d - dz));

   }


   public static void torque(double hip1, double hip2, double knee, double Fx, double Fy, double Fz, DoubleYoVariable Torq_h1, DoubleYoVariable Torq_h2, DoubleYoVariable Torq_k)

   {
      double JT11, JT12, JT13;
      double JT21, JT22, JT23;
      double JT31, JT32, JT33;

      JT11 = -Math.cos(hip1) * Math.sin(hip2) * AgileHexapodRobot.LEN - Math.cos(hip1) * Math.sin(hip2) * Math.cos(knee) * AgileHexapodRobot.LEN
             + Math.sin(hip1) * Math.sin(hip2) * Math.sin(knee) * AgileHexapodRobot.LEN;
      JT12 = Math.cos(hip1) * Math.cos(hip2) * AgileHexapodRobot.LEN + Math.cos(hip1) * Math.cos(hip2) * Math.cos(knee) * AgileHexapodRobot.LEN
             - Math.sin(hip1) * Math.cos(hip2) * Math.sin(knee) * AgileHexapodRobot.LEN;
      JT13 = Math.sin(hip1) * Math.cos(knee) * AgileHexapodRobot.LEN + Math.cos(hip1) * Math.sin(knee) * AgileHexapodRobot.LEN
             + Math.sin(hip1) * AgileHexapodRobot.LEN;

      JT21 = -Math.sin(hip1) * Math.cos(hip2) * AgileHexapodRobot.LEN - Math.sin(hip1) * Math.cos(hip2) * Math.cos(knee) * AgileHexapodRobot.LEN
             - Math.cos(hip1) * Math.cos(hip2) * Math.sin(knee) * AgileHexapodRobot.LEN;
      JT22 = -Math.sin(hip1) * Math.sin(hip2) * AgileHexapodRobot.LEN - Math.sin(hip1) * Math.sin(hip2) * Math.cos(knee) * AgileHexapodRobot.LEN
             - Math.cos(hip1) * Math.sin(hip2) * Math.sin(knee) * AgileHexapodRobot.LEN;
      JT23 = 0;

      JT31 = Math.sin(hip1) * Math.sin(hip2) * Math.sin(knee) * AgileHexapodRobot.LEN
             - Math.cos(hip1) * Math.sin(hip2) * Math.cos(knee) * AgileHexapodRobot.LEN;
      JT32 = -Math.sin(hip1) * Math.cos(hip2) * Math.sin(knee) * AgileHexapodRobot.LEN
             + Math.cos(hip1) * Math.cos(hip2) * Math.cos(knee) * AgileHexapodRobot.LEN;
      JT33 = Math.cos(hip1) * Math.sin(knee) * AgileHexapodRobot.LEN + Math.sin(hip1) * Math.cos(knee) * AgileHexapodRobot.LEN;


      Torq_h1.set(JT11 * Fx + JT12 * Fy + JT13 * Fz);
      Torq_h2.set(JT21 * Fx + JT22 * Fy + JT23 * Fz);
      Torq_k.set(JT31 * Fx + JT32 * Fy + JT33 * Fz);

   }
}
