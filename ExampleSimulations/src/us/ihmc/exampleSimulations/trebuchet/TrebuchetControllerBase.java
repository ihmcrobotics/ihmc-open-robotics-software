package us.ihmc.exampleSimulations.trebuchet;

import java.util.ArrayList;

import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;


public class TrebuchetControllerBase
{
   protected TrebuchetRobot rob;

   // These are the variables that are automatically created when the robot is created:
   DoubleYoVariable t;
   DoubleYoVariable q_ball_x, q_ball_y, q_ball_z, qd_ball_x, qd_ball_y, qd_ball_z, qdd_ball_x, qdd_ball_y, qdd_ball_z, q_ball_qs;
   DoubleYoVariable q_ball_qx, q_ball_qy, q_ball_qz, qd_ball_wx, qd_ball_wy, qd_ball_wz, qdd_ball_wx, qdd_ball_wy, qdd_ball_wz;
   DoubleYoVariable ef_ball_x, ef_ball_y, ef_ball_z, ef_ball_dx, ef_ball_dy, ef_ball_dz, ef_ball_fx, ef_ball_fy, ef_ball_fz, ef_ball_px;
   DoubleYoVariable ef_ball_py, ef_ball_pz;
   DoubleYoVariable q_x, qd_x, qdd_x, tau_x;
   DoubleYoVariable q_pivot, qd_pivot, qdd_pivot, tau_pivot;
   DoubleYoVariable ef_pole_x, ef_pole_y, ef_pole_z, ef_pole_dx, ef_pole_dy, ef_pole_dz, ef_pole_fx, ef_pole_fy, ef_pole_fz, ef_pole_px;
   DoubleYoVariable ef_pole_py, ef_pole_pz;

   // User defined control variables will be placed in this ArrayList when they are registered:
   ArrayList<DoubleYoVariable> controlVars;

   public TrebuchetControllerBase(TrebuchetRobot rob)
   {
      this.rob = rob;

      // Get the variables that are stored with the robot:

      t = (DoubleYoVariable)rob.getVariable("t");

      q_ball_x = (DoubleYoVariable)rob.getVariable("q_ball_x");
      q_ball_y = (DoubleYoVariable)rob.getVariable("q_ball_y");
      q_ball_z = (DoubleYoVariable)rob.getVariable("q_ball_z");
      qd_ball_x = (DoubleYoVariable)rob.getVariable("qd_ball_x");
      qd_ball_y = (DoubleYoVariable)rob.getVariable("qd_ball_y");
      qd_ball_z = (DoubleYoVariable)rob.getVariable("qd_ball_z");
      qdd_ball_x = (DoubleYoVariable)rob.getVariable("qdd_ball_x");
      qdd_ball_y = (DoubleYoVariable)rob.getVariable("qdd_ball_y");
      qdd_ball_z = (DoubleYoVariable)rob.getVariable("qdd_ball_z");
      q_ball_qs = (DoubleYoVariable)rob.getVariable("q_ball_qs");
      q_ball_qx = (DoubleYoVariable)rob.getVariable("q_ball_qx");
      q_ball_qy = (DoubleYoVariable)rob.getVariable("q_ball_qy");
      q_ball_qz = (DoubleYoVariable)rob.getVariable("q_ball_qz");
      qd_ball_wx = (DoubleYoVariable)rob.getVariable("qd_ball_wx");
      qd_ball_wy = (DoubleYoVariable)rob.getVariable("qd_ball_wy");
      qd_ball_wz = (DoubleYoVariable)rob.getVariable("qd_ball_wz");
      qdd_ball_wx = (DoubleYoVariable)rob.getVariable("qdd_ball_wx");
      qdd_ball_wy = (DoubleYoVariable)rob.getVariable("qdd_ball_wy");
      qdd_ball_wz = (DoubleYoVariable)rob.getVariable("qdd_ball_wz");
      ef_ball_x = (DoubleYoVariable)rob.getVariable("ef_ball_x");
      ef_ball_y = (DoubleYoVariable)rob.getVariable("ef_ball_y");
      ef_ball_z = (DoubleYoVariable)rob.getVariable("ef_ball_z");
      ef_ball_dx = (DoubleYoVariable)rob.getVariable("ef_ball_dx");
      ef_ball_dy = (DoubleYoVariable)rob.getVariable("ef_ball_dy");
      ef_ball_dz = (DoubleYoVariable)rob.getVariable("ef_ball_dz");
      ef_ball_fx = (DoubleYoVariable)rob.getVariable("ef_ball_fx");
      ef_ball_fy = (DoubleYoVariable)rob.getVariable("ef_ball_fy");
      ef_ball_fz = (DoubleYoVariable)rob.getVariable("ef_ball_fz");
      ef_ball_px = (DoubleYoVariable)rob.getVariable("ef_ball_px");
      ef_ball_py = (DoubleYoVariable)rob.getVariable("ef_ball_py");
      ef_ball_pz = (DoubleYoVariable)rob.getVariable("ef_ball_pz");
      q_x = (DoubleYoVariable)rob.getVariable("q_x");
      qd_x = (DoubleYoVariable)rob.getVariable("qd_x");
      qdd_x = (DoubleYoVariable)rob.getVariable("qdd_x");
      tau_x = (DoubleYoVariable)rob.getVariable("tau_x");
      q_pivot = (DoubleYoVariable)rob.getVariable("q_pivot");
      qd_pivot = (DoubleYoVariable)rob.getVariable("qd_pivot");
      qdd_pivot = (DoubleYoVariable)rob.getVariable("qdd_pivot");
      tau_pivot = (DoubleYoVariable)rob.getVariable("tau_pivot");
      ef_pole_x = (DoubleYoVariable)rob.getVariable("ef_pole_x");
      ef_pole_y = (DoubleYoVariable)rob.getVariable("ef_pole_y");
      ef_pole_z = (DoubleYoVariable)rob.getVariable("ef_pole_z");
      ef_pole_dx = (DoubleYoVariable)rob.getVariable("ef_pole_dx");
      ef_pole_dy = (DoubleYoVariable)rob.getVariable("ef_pole_dy");
      ef_pole_dz = (DoubleYoVariable)rob.getVariable("ef_pole_dz");
      ef_pole_fx = (DoubleYoVariable)rob.getVariable("ef_pole_fx");
      ef_pole_fy = (DoubleYoVariable)rob.getVariable("ef_pole_fy");
      ef_pole_fz = (DoubleYoVariable)rob.getVariable("ef_pole_fz");
      ef_pole_px = (DoubleYoVariable)rob.getVariable("ef_pole_px");
      ef_pole_py = (DoubleYoVariable)rob.getVariable("ef_pole_py");
      ef_pole_pz = (DoubleYoVariable)rob.getVariable("ef_pole_pz");

   }
}
