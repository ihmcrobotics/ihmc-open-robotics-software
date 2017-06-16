package us.ihmc.exampleSimulations.agileHexapod;

import us.ihmc.yoVariables.variable.YoDouble;

public class VirtualBodyForces
{

   // private YoVariable q_yaw, q_roll, q_pitch;
   private YoDouble q_hip1_x, q_hip1_z, q_knee1;
   private YoDouble q_hip2_x, q_hip2_z, q_knee2;
   private YoDouble q_hip3_x, q_hip3_z, q_knee3;
   private YoDouble q_hip4_x, q_hip4_z, q_knee4;
   private YoDouble q_hip5_x, q_hip5_z, q_knee5;
   private YoDouble q_hip6_x, q_hip6_z, q_knee6;


   private YoDouble q_yaw, q_roll, q_pitch;

   private YoDouble fx1, fy1, fz1, tx1, ty1, tz1;
   private YoDouble fx2, fy2, fz2, tx2, ty2, tz2;
   private YoDouble fx3, fy3, fz3, tx3, ty3, tz3;
   private YoDouble fx4, fy4, fz4, tx4, ty4, tz4;
   private YoDouble fx5, fy5, fz5, tx5, ty5, tz5;
   private YoDouble fx6, fy6, fz6, tx6, ty6, tz6;

   private YoDouble ff_hip1_x, ff_hip1_z, ff_knee1;
   private YoDouble ff_hip2_x, ff_hip2_z, ff_knee2;
   private YoDouble ff_hip3_x, ff_hip3_z, ff_knee3;
   private YoDouble ff_hip4_x, ff_hip4_z, ff_knee4;
   private YoDouble ff_hip5_x, ff_hip5_z, ff_knee5;
   private YoDouble ff_hip6_x, ff_hip6_z, ff_knee6;

   public VirtualBodyForces(AgileHexapodRobot rob, AgileHexapodController con)
   {
      // q_yaw = con.getVar("q_yaw"); q_roll = con.getVar("q_roll"); q_pitch = con.getVar("q_pitch");

      q_hip1_x = (YoDouble)rob.getVariable("q_hip1_x");
      q_hip1_z = (YoDouble)rob.getVariable("q_hip1_z");
      q_knee1 = (YoDouble)rob.getVariable("q_knee1");
      q_hip2_x = (YoDouble)rob.getVariable("q_hip2_x");
      q_hip2_z = (YoDouble)rob.getVariable("q_hip2_z");
      q_knee2 = (YoDouble)rob.getVariable("q_knee2");
      q_hip3_x = (YoDouble)rob.getVariable("q_hip3_x");
      q_hip3_z = (YoDouble)rob.getVariable("q_hip3_z");
      q_knee3 = (YoDouble)rob.getVariable("q_knee3");
      q_hip4_x = (YoDouble)rob.getVariable("q_hip4_x");
      q_hip4_z = (YoDouble)rob.getVariable("q_hip4_z");
      q_knee4 = (YoDouble)rob.getVariable("q_knee4");
      q_hip5_x = (YoDouble)rob.getVariable("q_hip5_x");
      q_hip5_z = (YoDouble)rob.getVariable("q_hip5_z");
      q_knee5 = (YoDouble)rob.getVariable("q_knee5");
      q_hip6_x = (YoDouble)rob.getVariable("q_hip6_x");
      q_hip6_z = (YoDouble)rob.getVariable("q_hip6_z");
      q_knee6 = (YoDouble)rob.getVariable("q_knee6");

      q_yaw = con.q_yaw;
      q_roll = con.q_roll;
      q_pitch = con.q_pitch;

      fx1 = con.fx1;
      fy1 = con.fy1;
      fz1 = con.fz1;
      tx1 = con.tx1;
      ty1 = con.ty1;
      tz1 = con.tz1;
      fx2 = con.fx2;
      fy2 = con.fy2;
      fz2 = con.fz2;
      tx2 = con.tx2;
      ty2 = con.ty2;
      tz2 = con.tz2;
      fx3 = con.fx3;
      fy3 = con.fy3;
      fz3 = con.fz3;
      tx3 = con.tx3;
      ty3 = con.ty3;
      tz3 = con.tz3;
      fx4 = con.fx4;
      fy4 = con.fy4;
      fz4 = con.fz4;
      tx4 = con.tx4;
      ty4 = con.ty4;
      tz4 = con.tz4;
      fx5 = con.fx5;
      fy5 = con.fy5;
      fz5 = con.fz5;
      tx5 = con.tx5;
      ty5 = con.ty5;
      tz5 = con.tz5;
      fx6 = con.fx6;
      fy6 = con.fy6;
      fz6 = con.fz6;
      tx6 = con.tx6;
      ty6 = con.ty6;
      tz6 = con.tz6;

      ff_hip1_x = con.ff_hip1_x;
      ff_hip1_z = con.ff_hip1_z;
      ff_knee1 = con.ff_knee1;
      ff_hip2_x = con.ff_hip2_x;
      ff_hip2_z = con.ff_hip2_z;
      ff_knee2 = con.ff_knee2;
      ff_hip3_x = con.ff_hip3_x;
      ff_hip3_z = con.ff_hip3_z;
      ff_knee3 = con.ff_knee3;
      ff_hip4_x = con.ff_hip4_x;
      ff_hip4_z = con.ff_hip4_z;
      ff_knee4 = con.ff_knee4;
      ff_hip5_x = con.ff_hip5_x;
      ff_hip5_z = con.ff_hip5_z;
      ff_knee5 = con.ff_knee5;
      ff_hip6_x = con.ff_hip6_x;
      ff_hip6_z = con.ff_hip6_z;
      ff_knee6 = con.ff_knee6;


   }


   /*
    * transform_forces_to_body()
    *    Transforms the virtual forces, which are specified in world
    * coordinates, into body coordinates so that virtual_forces()
    * does the right thing.
    *
    * Uses the Transpose of ZXY(yaw,roll,pitch) Euler Angles but only on
    * Cartesian Forces (Not Torques).
    *
    */

   public void transform_forces_to_body(YoDouble fX, YoDouble fY, YoDouble fZ, YoDouble tX, YoDouble tY, YoDouble tZ)
   {
      double f_x, f_y, f_z, t_x, t_y, t_z;

      double alpha, beta, gamma;
      double ca, cb, cg, sa, sb, sg;

      alpha = q_yaw.getDoubleValue();
      beta = q_roll.getDoubleValue();
      gamma = q_pitch.getDoubleValue();

      ca = Math.cos(alpha);
      cb = Math.cos(beta);
      cg = Math.cos(gamma);
      sa = Math.sin(alpha);
      sb = Math.sin(beta);
      sg = Math.sin(gamma);

      f_z = (cb * cg) * (fZ.getDoubleValue()) + (sa * sb * cg + ca * sg) * (fX.getDoubleValue()) + (-ca * sb * cg + sa * sg) * (fY.getDoubleValue());
      f_x = (-cb * sg) * (fZ.getDoubleValue()) + (-sa * sb * sg + ca * cg) * (fX.getDoubleValue()) + (ca * sb * sg + sa * cg) * (fY.getDoubleValue());
      f_y = (sb) * (fZ.getDoubleValue()) + (-sa * cb) * (fX.getDoubleValue()) + (ca * cb) * (fY.getDoubleValue());

      t_z = (cb * cg) * (tZ.getDoubleValue()) + (sg) * (tX.getDoubleValue()) + (0) * (tY.getDoubleValue());
      t_x = (-cb * sg) * (tZ.getDoubleValue()) + (cg) * (tX.getDoubleValue()) + (0) * (tY.getDoubleValue());
      t_y = (sb) * (tZ.getDoubleValue()) + 0 * (tX.getDoubleValue()) + (1) * (tY.getDoubleValue());

      fX.set(f_x);
      fY.set(f_y);
      fZ.set(f_z);

      tX.set(t_x);
      tY.set(t_y);
      tZ.set(t_z);



   }


   /*
    * VirtualForces_a()
    * calculates the torques at each joint of Set a (leg 1,2,&3)...
    * ...given a generalized force acting between the body and the ground
    * this functions call the following function in order:
    * ..........InverseA()
    * ..........RedundantForces()
    * ..........ConstrainedForces()
    * ..........Jacobian()
    */

   public void virtualForces_a(double fXa, double fYa, double fZa, double tXa, double tYa, double tZa)
   {
      InverseFunction.InverseA(AgileHexapodRobot.PX1, AgileHexapodRobot.PX2, AgileHexapodRobot.PX3, AgileHexapodRobot.PY1, AgileHexapodRobot.PY2,
                               AgileHexapodRobot.PY3, fXa, fYa, fZa, tXa, tYa, tZa, q_hip1_x.getDoubleValue(), q_hip1_z.getDoubleValue(), q_knee1.getDoubleValue(), q_hip2_x.getDoubleValue(), q_hip2_z.getDoubleValue(),
                               q_knee2.getDoubleValue(), q_hip3_x.getDoubleValue(), q_hip3_z.getDoubleValue(), q_knee3.getDoubleValue(), fx1, fz1, fx2, fy2, fz2, fz3);


      RedundantForces(fx1.getDoubleValue(), fz1.getDoubleValue(), fx2.getDoubleValue(), fy2.getDoubleValue(), fz2.getDoubleValue(), fz3.getDoubleValue(), fx3, fy3, fy1);

      ConstrainedForces(fx1.getDoubleValue(), fy1.getDoubleValue(), fz1.getDoubleValue(), AgileHexapodRobot.PX1, AgileHexapodRobot.PY1, q_hip1_x.getDoubleValue(), q_hip1_z.getDoubleValue(), q_knee1.getDoubleValue(), tx1, ty1, tz1);

      ConstrainedForces(fx2.getDoubleValue(), fy2.getDoubleValue(), fz2.getDoubleValue(), AgileHexapodRobot.PX2, AgileHexapodRobot.PY2, q_hip2_x.getDoubleValue(), q_hip2_z.getDoubleValue(), q_knee2.getDoubleValue(), tx2, ty2, tz2);

      ConstrainedForces(fx3.getDoubleValue(), fy3.getDoubleValue(), fz3.getDoubleValue(), AgileHexapodRobot.PX3, AgileHexapodRobot.PY3, q_hip3_x.getDoubleValue(), q_hip3_z.getDoubleValue(), q_knee3.getDoubleValue(), tx3, ty3, tz3);


      Jacobian(fx1.getDoubleValue(), fy1.getDoubleValue(), fz1.getDoubleValue(), tx1.getDoubleValue(), ty1.getDoubleValue(), tz1.getDoubleValue(), q_hip1_x.getDoubleValue(), q_hip1_z.getDoubleValue(), q_knee1.getDoubleValue(), AgileHexapodRobot.PX1, AgileHexapodRobot.PY1,
               ff_hip1_x, ff_hip1_z, ff_knee1);

      Jacobian(fx2.getDoubleValue(), fy2.getDoubleValue(), fz2.getDoubleValue(), tx2.getDoubleValue(), ty2.getDoubleValue(), tz2.getDoubleValue(), q_hip2_x.getDoubleValue(), q_hip2_z.getDoubleValue(), q_knee2.getDoubleValue(), AgileHexapodRobot.PX2, AgileHexapodRobot.PY2,
               ff_hip2_x, ff_hip2_z, ff_knee2);

      Jacobian(fx3.getDoubleValue(), fy3.getDoubleValue(), fz3.getDoubleValue(), tx3.getDoubleValue(), ty3.getDoubleValue(), tz3.getDoubleValue(), q_hip3_x.getDoubleValue(), q_hip3_z.getDoubleValue(), q_knee3.getDoubleValue(), AgileHexapodRobot.PX3, AgileHexapodRobot.PY3,
               ff_hip3_x, ff_hip3_z, ff_knee3);


   }


   /*
    * VirtualForces_b()
    * calculates the torques at each joint of Set b (legs 4,5,&6)...
    * ...given a generalized force acting between the body and the ground
    * this functions call the following function in order:
    * ..........InverseA()
    * ..........RedundantForces()
    * ..........ConstrainedForces()
    * ..........Jacobian()
    */


   public void virtualForces_b(double fXb, double fYb, double fZb, double tXb, double tYb, double tZb)
   {
      InverseFunction.InverseA(AgileHexapodRobot.PX4, AgileHexapodRobot.PX5, AgileHexapodRobot.PX6, AgileHexapodRobot.PY4, AgileHexapodRobot.PY5,
                               AgileHexapodRobot.PY6, fXb, fYb, fZb, tXb, tYb, tZb, q_hip4_x.getDoubleValue(), q_hip4_z.getDoubleValue(), q_knee4.getDoubleValue(), q_hip5_x.getDoubleValue(), q_hip5_z.getDoubleValue(),
                               q_knee5.getDoubleValue(), q_hip6_x.getDoubleValue(), q_hip6_z.getDoubleValue(), q_knee6.getDoubleValue(), fx4, fz4, fx5, fy5, fz5, fz6);


      RedundantForces(fx4.getDoubleValue(), fz4.getDoubleValue(), fx5.getDoubleValue(), fy5.getDoubleValue(), fz5.getDoubleValue(), fz6.getDoubleValue(), fx6, fy6, fy4);

      ConstrainedForces(fx4.getDoubleValue(), fy4.getDoubleValue(), fz4.getDoubleValue(), AgileHexapodRobot.PX4, AgileHexapodRobot.PY4, q_hip4_x.getDoubleValue(), q_hip4_z.getDoubleValue(), q_knee4.getDoubleValue(), tx4, ty4, tz4);

      ConstrainedForces(fx5.getDoubleValue(), fy5.getDoubleValue(), fz5.getDoubleValue(), AgileHexapodRobot.PX5, AgileHexapodRobot.PY5, q_hip5_x.getDoubleValue(), q_hip5_z.getDoubleValue(), q_knee5.getDoubleValue(), tx5, ty5, tz5);

      ConstrainedForces(fx6.getDoubleValue(), fy6.getDoubleValue(), fz6.getDoubleValue(), AgileHexapodRobot.PX6, AgileHexapodRobot.PY6, q_hip6_x.getDoubleValue(), q_hip6_z.getDoubleValue(), q_knee6.getDoubleValue(), tx6, ty6, tz6);


      Jacobian(fx4.getDoubleValue(), fy4.getDoubleValue(), fz4.getDoubleValue(), tx4.getDoubleValue(), ty4.getDoubleValue(), tz4.getDoubleValue(), q_hip4_x.getDoubleValue(), q_hip4_z.getDoubleValue(), q_knee4.getDoubleValue(), AgileHexapodRobot.PX4, AgileHexapodRobot.PY4,
               ff_hip4_x, ff_hip4_z, ff_knee4);

      Jacobian(fx5.getDoubleValue(), fy5.getDoubleValue(), fz5.getDoubleValue(), tx5.getDoubleValue(), ty5.getDoubleValue(), tz5.getDoubleValue(), q_hip5_x.getDoubleValue(), q_hip5_z.getDoubleValue(), q_knee5.getDoubleValue(), AgileHexapodRobot.PX5, AgileHexapodRobot.PY5,
               ff_hip5_x, ff_hip5_z, ff_knee5);

      Jacobian(fx6.getDoubleValue(), fy6.getDoubleValue(), fz6.getDoubleValue(), tx6.getDoubleValue(), ty6.getDoubleValue(), tz6.getDoubleValue(), q_hip6_x.getDoubleValue(), q_hip6_z.getDoubleValue(), q_knee6.getDoubleValue(), AgileHexapodRobot.PX6, AgileHexapodRobot.PY6,
               ff_hip6_x, ff_hip6_z, ff_knee6);

   }


   /*
    * Jacobian()
    * maps virtual forces to joint torques
    * takes the 6 vitual forces of one leg(6v.)
    * & the 3 joint angles of the leg(3v.)
    * & the x,y position of the hip wrt the {B}(2v.)
    * & also takes the address of the joint torques(3v.)
    * 'returns' modified joint torques
    */

   public void Jacobian(double force_x, double force_y, double force_z, double torq_x, double torq_y, double torq_z, double theta_ha, double theta_hb,
                        double theta_k, double pos_x, double pos_y, YoDouble torq_ha, YoDouble torq_hb, YoDouble torq_k)
   {
      torq_ha.set((-pos_x * Math.sin(theta_hb) + pos_y * Math.cos(theta_hb)) * force_z - Math.cos(theta_hb) * torq_x - Math.sin(theta_hb) * torq_y);
      torq_hb.set(-pos_y * force_x + pos_x * force_y - torq_z);
      torq_k.set(-AgileHexapodRobot.LEN * Math.sin(theta_hb) * Math.cos(theta_ha) * force_x
                   + AgileHexapodRobot.LEN * Math.cos(theta_hb) * Math.cos(theta_ha) * force_y
                   + (AgileHexapodRobot.LEN * Math.sin(theta_ha) - pos_x * Math.sin(theta_hb) + pos_y * Math.cos(theta_hb)) * force_z
                   - Math.cos(theta_hb) * torq_x - Math.sin(theta_hb) * torq_y);

   }



   /*
    * RedundantForces()
    * calculates the Redundant Force Set given the Minimum Force Set
    * input: MFS(6v.) & pointers to the Redundant Force values(3v.)
    * ouput: modified values for the RFS(3v.)
    */

   public void RedundantForces(double fx1, double fz1, double fx2, double fy2, double fz2, double fz3, YoDouble fx3, YoDouble fy3, YoDouble fy1)

   {
      fx3.set(fx2);
      fy3.set(fy2);
      fy1.set(2.0 * fy2);

   }


/*    ***********************************************************************
      ConstrainedForces()
      calculates the Constrained Force set given the MFS & the RFS
      (calculates the torques for each leg given the forces)
      input: fxi,fyi,fzi for one leg(3v.)
      & x,y hip position of the ith leg(2v.)
      & three angle joints of the ith leg(3v.)
      & pointers to the values of the leg torques(3v.)
      output: modified values of the leg torques(3v.)
   *********************************************************************** */

   public void ConstrainedForces(double force_x, double force_y, double force_z, double pos_x, double pos_y, double hip_x, double hip_z, double knee,
                                 YoDouble torq_x, YoDouble torq_y, YoDouble torq_z)
   {
      double JB_12, JB_13, JB_21, JB_23, JB_31, JB_32;

      JB_12 = AgileHexapodRobot.LEN * Math.cos(knee + hip_x) + AgileHexapodRobot.LEN * Math.cos(hip_x);
      JB_13 = pos_y + AgileHexapodRobot.LEN * Math.cos(hip_z) * (Math.sin(knee + hip_x) + Math.sin(hip_x));
      JB_21 = -AgileHexapodRobot.LEN * Math.cos(knee + hip_x) - AgileHexapodRobot.LEN * Math.cos(hip_x);
      JB_23 = AgileHexapodRobot.LEN * Math.sin(hip_z) * (Math.sin(knee + hip_x) + Math.sin(hip_x)) - pos_x;
      JB_31 = -pos_y - AgileHexapodRobot.LEN * Math.cos(hip_z) * (Math.sin(knee + hip_x) + Math.sin(hip_x));
      JB_32 = -AgileHexapodRobot.LEN * Math.sin(hip_z) * (Math.sin(knee + hip_x) + Math.sin(hip_x)) + pos_x;

      torq_x.set(JB_12 * force_y + JB_13 * force_z);
      torq_y.set(JB_21 * force_x + JB_23 * force_z);
      torq_z.set(JB_31 * force_x + JB_32 * force_y);


   }




}
