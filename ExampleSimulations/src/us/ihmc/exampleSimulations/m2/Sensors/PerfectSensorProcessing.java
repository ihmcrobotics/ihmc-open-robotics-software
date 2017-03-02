package us.ihmc.exampleSimulations.m2.Sensors;


import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.exampleSimulations.m2.ContactPointName;
import us.ihmc.exampleSimulations.m2.JointName;
import us.ihmc.exampleSimulations.m2.M2Robot;
import us.ihmc.exampleSimulations.m2.M2Simulation;
import us.ihmc.exampleSimulations.m2.RobotAxis;
import us.ihmc.exampleSimulations.m2.RobotOrientation;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.robotSide.RobotSide;

/**
 * <p>
 * Title: PerfectSensorProcessing
 * </p>
 *
 * <p>
 * Description: Perfect, noiseless sensor inputs. this class takes the sensor
 * values from the simulated robot and saves them into processed sensors.
 * </p>
 *
 * <p>
 * Copyright: Copyright (c) 2010
 * </p>
 *
 * <p>
 * Company: IHMC
 * </p>
 *
 * @version 2.0
 */

public class PerfectSensorProcessing
{
   protected M2Robot robot;

   // These are the variables that are automatically created when the robot is
   // created:
   private BooleanYoVariable gc_right_toe_in_slip, gc_right_toe_out_slip, gc_right_heel_in_slip, gc_right_heel_out_slip, gc_left_toe_in_slip, gc_left_toe_out_slip;
   private BooleanYoVariable gc_left_heel_in_slip, gc_left_heel_out_slip;
   private DoubleYoVariable t;

   private DoubleYoVariable q_yaw;
   private DoubleYoVariable q_pitch;
   private DoubleYoVariable q_roll;

   private DoubleYoVariable q_x, q_y, q_z, qd_x, qd_y, qd_z, qdd_x, qdd_y, qdd_z, q_qs;
   private DoubleYoVariable q_qx, q_qy, q_qz, qd_wx, qd_wy, qd_wz, qdd_wx, qdd_wy, qdd_wz;
   private DoubleYoVariable q_right_hip_yaw, qd_right_hip_yaw, qdd_right_hip_yaw, tau_right_hip_yaw;
   private DoubleYoVariable q_right_hip_roll, qd_right_hip_roll, qdd_right_hip_roll, tau_right_hip_roll;
   private DoubleYoVariable q_right_hip_pitch, qd_right_hip_pitch, qdd_right_hip_pitch, tau_right_hip_pitch;
   private DoubleYoVariable q_right_knee, qd_right_knee, qdd_right_knee, tau_right_knee, tau_lim_right_knee;
   private DoubleYoVariable q_right_ankle_roll, qd_right_ankle_roll, qdd_right_ankle_roll, tau_right_ankle_roll;
   private DoubleYoVariable q_right_ankle_pitch, qd_right_ankle_pitch, qdd_right_ankle_pitch, tau_right_ankle_pitch;
   private DoubleYoVariable gc_right_toe_in_x, gc_right_toe_in_y, gc_right_toe_in_z, gc_right_toe_in_dx, gc_right_toe_in_dy, gc_right_toe_in_dz, gc_right_toe_in_fx,
                      gc_right_toe_in_fy, gc_right_toe_in_fz, gc_right_toe_in_px;
   private DoubleYoVariable gc_right_toe_in_py, gc_right_toe_in_pz, gc_right_toe_in_tdx, gc_right_toe_in_tdy, gc_right_toe_in_tdz, gc_right_toe_in_fs;
   private IntegerYoVariable gc_right_toe_in_coll;
   private DoubleYoVariable gc_right_toe_out_x, gc_right_toe_out_y, gc_right_toe_out_z, gc_right_toe_out_dx, gc_right_toe_out_dy, gc_right_toe_out_dz,
                      gc_right_toe_out_fx, gc_right_toe_out_fy, gc_right_toe_out_fz, gc_right_toe_out_px;
   private DoubleYoVariable gc_right_toe_out_py, gc_right_toe_out_pz, gc_right_toe_out_tdx, gc_right_toe_out_tdy, gc_right_toe_out_tdz, gc_right_toe_out_fs;
   private IntegerYoVariable gc_right_toe_out_coll;
   private DoubleYoVariable gc_right_heel_in_x, gc_right_heel_in_y, gc_right_heel_in_z, gc_right_heel_in_dx, gc_right_heel_in_dy, gc_right_heel_in_dz,
                      gc_right_heel_in_fx, gc_right_heel_in_fy, gc_right_heel_in_fz, gc_right_heel_in_px;
   private DoubleYoVariable gc_right_heel_in_py, gc_right_heel_in_pz, gc_right_heel_in_tdx, gc_right_heel_in_tdy, gc_right_heel_in_tdz, gc_right_heel_in_fs;
   private IntegerYoVariable gc_right_heel_in_coll;
   private DoubleYoVariable gc_right_heel_out_x, gc_right_heel_out_y, gc_right_heel_out_z, gc_right_heel_out_dx, gc_right_heel_out_dy, gc_right_heel_out_dz,
                      gc_right_heel_out_fx, gc_right_heel_out_fy, gc_right_heel_out_fz, gc_right_heel_out_px;
   private DoubleYoVariable gc_right_heel_out_py, gc_right_heel_out_pz, gc_right_heel_out_tdx, gc_right_heel_out_tdy, gc_right_heel_out_tdz, gc_right_heel_out_fs;
   private IntegerYoVariable gc_right_heel_out_coll;
   private DoubleYoVariable q_left_hip_yaw, qd_left_hip_yaw, qdd_left_hip_yaw, tau_left_hip_yaw;
   private DoubleYoVariable q_left_hip_roll, qd_left_hip_roll, qdd_left_hip_roll, tau_left_hip_roll;
   private DoubleYoVariable q_left_hip_pitch, qd_left_hip_pitch, qdd_left_hip_pitch, tau_left_hip_pitch;
   private DoubleYoVariable q_left_knee, qd_left_knee, qdd_left_knee, tau_left_knee, tau_lim_left_knee;
   private DoubleYoVariable q_left_ankle_roll, qd_left_ankle_roll, qdd_left_ankle_roll, tau_left_ankle_roll;
   private DoubleYoVariable q_left_ankle_pitch, qd_left_ankle_pitch, qdd_left_ankle_pitch, tau_left_ankle_pitch;
   private DoubleYoVariable gc_left_toe_in_x, gc_left_toe_in_y, gc_left_toe_in_z, gc_left_toe_in_dx, gc_left_toe_in_dy, gc_left_toe_in_dz, gc_left_toe_in_fx,
                      gc_left_toe_in_fy, gc_left_toe_in_fz, gc_left_toe_in_px;
   private DoubleYoVariable gc_left_toe_in_py, gc_left_toe_in_pz, gc_left_toe_in_tdx, gc_left_toe_in_tdy, gc_left_toe_in_tdz, gc_left_toe_in_fs;
   private IntegerYoVariable gc_left_toe_in_coll;
   private DoubleYoVariable gc_left_toe_out_x, gc_left_toe_out_y, gc_left_toe_out_z, gc_left_toe_out_dx, gc_left_toe_out_dy, gc_left_toe_out_dz, gc_left_toe_out_fx,
                      gc_left_toe_out_fy, gc_left_toe_out_fz, gc_left_toe_out_px;
   private DoubleYoVariable gc_left_toe_out_py, gc_left_toe_out_pz, gc_left_toe_out_tdx, gc_left_toe_out_tdy, gc_left_toe_out_tdz, gc_left_toe_out_fs;
   private IntegerYoVariable gc_left_toe_out_coll;
   private DoubleYoVariable gc_left_heel_in_x, gc_left_heel_in_y, gc_left_heel_in_z, gc_left_heel_in_dx, gc_left_heel_in_dy, gc_left_heel_in_dz, gc_left_heel_in_fx,
                      gc_left_heel_in_fy, gc_left_heel_in_fz, gc_left_heel_in_px;
   private DoubleYoVariable gc_left_heel_in_py, gc_left_heel_in_pz, gc_left_heel_in_tdx, gc_left_heel_in_tdy, gc_left_heel_in_tdz, gc_left_heel_in_fs;
   private IntegerYoVariable gc_left_heel_in_coll;
   private DoubleYoVariable gc_left_heel_out_x, gc_left_heel_out_y, gc_left_heel_out_z, gc_left_heel_out_dx, gc_left_heel_out_dy, gc_left_heel_out_dz,
                      gc_left_heel_out_fx, gc_left_heel_out_fy, gc_left_heel_out_fz, gc_left_heel_out_px;
   private DoubleYoVariable gc_left_heel_out_py, gc_left_heel_out_pz, gc_left_heel_out_tdx, gc_left_heel_out_tdy, gc_left_heel_out_tdz, gc_left_heel_out_fs;
   private IntegerYoVariable gc_left_heel_out_coll;

   private DoubleYoVariable[] robotBodyPosition;
   private DoubleYoVariable[] robotBodyVelocity;
   private DoubleYoVariable[] robotBodyAcceleration;

   private DoubleYoVariable[][] limbJointPositions;
   private DoubleYoVariable[][] limbJointVelocities;
   private DoubleYoVariable[][] limbJointAccelerations;
   private DoubleYoVariable[] robotYawPitchAndRoll;
   private DoubleYoVariable[] robotYawPitchAndRollVelocity;
   private DoubleYoVariable[] robotYawPitchAndRollAcceleration;

   private DoubleYoVariable[][] groundContactFramePointsPositions;
   private DoubleYoVariable[][] groundContactFramePointsForces;
   private DoubleYoVariable[][] groundContactFramePointsFootSwitch;

   ProcessedSensors processedSensors;

   public PerfectSensorProcessing(M2Robot robot, ProcessedSensors processedSensors)
   {
      this.processedSensors = processedSensors;
      this.robot = robot;
      initialize();

      setInitialValues();

      if (M2Simulation.USE_HEAVY_M2)
      {
         setInitialValuesForHeavyM2();
      }
   }

   // Get the variables that are stored with the robot:
   public void initialize()
   {
      t = (DoubleYoVariable)robot.getVariable("t");

      q_yaw = (DoubleYoVariable)robot.getVariable("q_yaw");
      q_pitch = (DoubleYoVariable)robot.getVariable("q_pitch");
      q_roll = (DoubleYoVariable)robot.getVariable("q_roll");

      q_x = (DoubleYoVariable)robot.getVariable("q_x");
      q_y = (DoubleYoVariable)robot.getVariable("q_y");
      q_z = (DoubleYoVariable)robot.getVariable("q_z");
      qd_x = (DoubleYoVariable)robot.getVariable("qd_x");
      qd_y = (DoubleYoVariable)robot.getVariable("qd_y");
      qd_z = (DoubleYoVariable)robot.getVariable("qd_z");
      qdd_x = (DoubleYoVariable)robot.getVariable("qdd_x");
      qdd_y = (DoubleYoVariable)robot.getVariable("qdd_y");
      qdd_z = (DoubleYoVariable)robot.getVariable("qdd_z");
      q_qs = (DoubleYoVariable)robot.getVariable("q_qs");
      q_qx = (DoubleYoVariable)robot.getVariable("q_qx");
      q_qy = (DoubleYoVariable)robot.getVariable("q_qy");
      q_qz = (DoubleYoVariable)robot.getVariable("q_qz");
      qd_wx = (DoubleYoVariable)robot.getVariable("qd_wx");
      qd_wy = (DoubleYoVariable)robot.getVariable("qd_wy");
      qd_wz = (DoubleYoVariable)robot.getVariable("qd_wz");
      qdd_wx = (DoubleYoVariable)robot.getVariable("qdd_wx");
      qdd_wy = (DoubleYoVariable)robot.getVariable("qdd_wy");
      qdd_wz = (DoubleYoVariable)robot.getVariable("qdd_wz");
      q_right_hip_yaw = (DoubleYoVariable)robot.getVariable("q_right_hip_yaw");
      qd_right_hip_yaw = (DoubleYoVariable)robot.getVariable("qd_right_hip_yaw");
      qdd_right_hip_yaw = (DoubleYoVariable)robot.getVariable("qdd_right_hip_yaw");
      tau_right_hip_yaw = (DoubleYoVariable)robot.getVariable("tau_right_hip_yaw");
      q_right_hip_roll = (DoubleYoVariable)robot.getVariable("q_right_hip_roll");
      qd_right_hip_roll = (DoubleYoVariable)robot.getVariable("qd_right_hip_roll");
      qdd_right_hip_roll = (DoubleYoVariable)robot.getVariable("qdd_right_hip_roll");
      tau_right_hip_roll = (DoubleYoVariable)robot.getVariable("tau_right_hip_roll");
      q_right_hip_pitch = (DoubleYoVariable)robot.getVariable("q_right_hip_pitch");
      qd_right_hip_pitch = (DoubleYoVariable)robot.getVariable("qd_right_hip_pitch");
      qdd_right_hip_pitch = (DoubleYoVariable)robot.getVariable("qdd_right_hip_pitch");
      tau_right_hip_pitch = (DoubleYoVariable)robot.getVariable("tau_right_hip_pitch");
      q_right_knee = (DoubleYoVariable)robot.getVariable("q_right_knee");
      qd_right_knee = (DoubleYoVariable)robot.getVariable("qd_right_knee");
      qdd_right_knee = (DoubleYoVariable)robot.getVariable("qdd_right_knee");
      tau_right_knee = (DoubleYoVariable)robot.getVariable("tau_right_knee");
      tau_lim_right_knee = (DoubleYoVariable)robot.getVariable("tau_lim_right_knee");
      q_right_ankle_roll = (DoubleYoVariable)robot.getVariable("q_right_ankle_roll");
      qd_right_ankle_roll = (DoubleYoVariable)robot.getVariable("qd_right_ankle_roll");
      qdd_right_ankle_roll = (DoubleYoVariable)robot.getVariable("qdd_right_ankle_roll");
      tau_right_ankle_roll = (DoubleYoVariable)robot.getVariable("tau_right_ankle_roll");
      q_right_ankle_pitch = (DoubleYoVariable)robot.getVariable("q_right_ankle_pitch");
      qd_right_ankle_pitch = (DoubleYoVariable)robot.getVariable("qd_right_ankle_pitch");
      qdd_right_ankle_pitch = (DoubleYoVariable)robot.getVariable("qdd_right_ankle_pitch");
      tau_right_ankle_pitch = (DoubleYoVariable)robot.getVariable("tau_right_ankle_pitch");
      gc_right_toe_in_x = (DoubleYoVariable)robot.getVariable("gc_right_toe_in_x");
      gc_right_toe_in_y = (DoubleYoVariable)robot.getVariable("gc_right_toe_in_y");
      gc_right_toe_in_z = (DoubleYoVariable)robot.getVariable("gc_right_toe_in_z");
      gc_right_toe_in_dx = (DoubleYoVariable)robot.getVariable("gc_right_toe_in_dx");
      gc_right_toe_in_dy = (DoubleYoVariable)robot.getVariable("gc_right_toe_in_dy");
      gc_right_toe_in_dz = (DoubleYoVariable)robot.getVariable("gc_right_toe_in_dz");
      gc_right_toe_in_fx = (DoubleYoVariable)robot.getVariable("gc_right_toe_in_fx");
      gc_right_toe_in_fy = (DoubleYoVariable)robot.getVariable("gc_right_toe_in_fy");
      gc_right_toe_in_fz = (DoubleYoVariable)robot.getVariable("gc_right_toe_in_fz");
      gc_right_toe_in_px = (DoubleYoVariable)robot.getVariable("gc_right_toe_in_px");
      gc_right_toe_in_py = (DoubleYoVariable)robot.getVariable("gc_right_toe_in_py");
      gc_right_toe_in_pz = (DoubleYoVariable)robot.getVariable("gc_right_toe_in_pz");
      gc_right_toe_in_tdx = (DoubleYoVariable)robot.getVariable("gc_right_toe_in_tdx");
      gc_right_toe_in_tdy = (DoubleYoVariable)robot.getVariable("gc_right_toe_in_tdy");
      gc_right_toe_in_tdz = (DoubleYoVariable)robot.getVariable("gc_right_toe_in_tdz");
      gc_right_toe_in_fs = (DoubleYoVariable)robot.getVariable("gc_right_toe_in_fs");
      gc_right_toe_in_slip = (BooleanYoVariable)robot.getVariable("gc_right_toe_in_slip");
      gc_right_toe_in_coll = (IntegerYoVariable)robot.getVariable("gc_right_toe_in_coll");
      gc_right_toe_out_x = (DoubleYoVariable)robot.getVariable("gc_right_toe_out_x");
      gc_right_toe_out_y = (DoubleYoVariable)robot.getVariable("gc_right_toe_out_y");
      gc_right_toe_out_z = (DoubleYoVariable)robot.getVariable("gc_right_toe_out_z");
      gc_right_toe_out_dx = (DoubleYoVariable)robot.getVariable("gc_right_toe_out_dx");
      gc_right_toe_out_dy = (DoubleYoVariable)robot.getVariable("gc_right_toe_out_dy");
      gc_right_toe_out_dz = (DoubleYoVariable)robot.getVariable("gc_right_toe_out_dz");
      gc_right_toe_out_fx = (DoubleYoVariable)robot.getVariable("gc_right_toe_out_fx");
      gc_right_toe_out_fy = (DoubleYoVariable)robot.getVariable("gc_right_toe_out_fy");
      gc_right_toe_out_fz = (DoubleYoVariable)robot.getVariable("gc_right_toe_out_fz");
      gc_right_toe_out_px = (DoubleYoVariable)robot.getVariable("gc_right_toe_out_px");
      gc_right_toe_out_py = (DoubleYoVariable)robot.getVariable("gc_right_toe_out_py");
      gc_right_toe_out_pz = (DoubleYoVariable)robot.getVariable("gc_right_toe_out_pz");
      gc_right_toe_out_tdx = (DoubleYoVariable)robot.getVariable("gc_right_toe_out_tdx");
      gc_right_toe_out_tdy = (DoubleYoVariable)robot.getVariable("gc_right_toe_out_tdy");
      gc_right_toe_out_tdz = (DoubleYoVariable)robot.getVariable("gc_right_toe_out_tdz");
      gc_right_toe_out_fs = (DoubleYoVariable)robot.getVariable("gc_right_toe_out_fs");
      gc_right_toe_out_slip = (BooleanYoVariable)robot.getVariable("gc_right_toe_out_slip");
      gc_right_toe_out_coll = (IntegerYoVariable)robot.getVariable("gc_right_toe_out_coll");
      gc_right_heel_in_x = (DoubleYoVariable)robot.getVariable("gc_right_heel_in_x");
      gc_right_heel_in_y = (DoubleYoVariable)robot.getVariable("gc_right_heel_in_y");
      gc_right_heel_in_z = (DoubleYoVariable)robot.getVariable("gc_right_heel_in_z");
      gc_right_heel_in_dx = (DoubleYoVariable)robot.getVariable("gc_right_heel_in_dx");
      gc_right_heel_in_dy = (DoubleYoVariable)robot.getVariable("gc_right_heel_in_dy");
      gc_right_heel_in_dz = (DoubleYoVariable)robot.getVariable("gc_right_heel_in_dz");
      gc_right_heel_in_fx = (DoubleYoVariable)robot.getVariable("gc_right_heel_in_fx");
      gc_right_heel_in_fy = (DoubleYoVariable)robot.getVariable("gc_right_heel_in_fy");
      gc_right_heel_in_fz = (DoubleYoVariable)robot.getVariable("gc_right_heel_in_fz");
      gc_right_heel_in_px = (DoubleYoVariable)robot.getVariable("gc_right_heel_in_px");
      gc_right_heel_in_py = (DoubleYoVariable)robot.getVariable("gc_right_heel_in_py");
      gc_right_heel_in_pz = (DoubleYoVariable)robot.getVariable("gc_right_heel_in_pz");
      gc_right_heel_in_tdx = (DoubleYoVariable)robot.getVariable("gc_right_heel_in_tdx");
      gc_right_heel_in_tdy = (DoubleYoVariable)robot.getVariable("gc_right_heel_in_tdy");
      gc_right_heel_in_tdz = (DoubleYoVariable)robot.getVariable("gc_right_heel_in_tdz");
      gc_right_heel_in_fs = (DoubleYoVariable)robot.getVariable("gc_right_heel_in_fs");
      gc_right_heel_in_slip = (BooleanYoVariable)robot.getVariable("gc_right_heel_in_slip");
      gc_right_heel_in_coll = (IntegerYoVariable)robot.getVariable("gc_right_heel_in_coll");
      gc_right_heel_out_x = (DoubleYoVariable)robot.getVariable("gc_right_heel_out_x");
      gc_right_heel_out_y = (DoubleYoVariable)robot.getVariable("gc_right_heel_out_y");
      gc_right_heel_out_z = (DoubleYoVariable)robot.getVariable("gc_right_heel_out_z");
      gc_right_heel_out_dx = (DoubleYoVariable)robot.getVariable("gc_right_heel_out_dx");
      gc_right_heel_out_dy = (DoubleYoVariable)robot.getVariable("gc_right_heel_out_dy");
      gc_right_heel_out_dz = (DoubleYoVariable)robot.getVariable("gc_right_heel_out_dz");
      gc_right_heel_out_fx = (DoubleYoVariable)robot.getVariable("gc_right_heel_out_fx");
      gc_right_heel_out_fy = (DoubleYoVariable)robot.getVariable("gc_right_heel_out_fy");
      gc_right_heel_out_fz = (DoubleYoVariable)robot.getVariable("gc_right_heel_out_fz");
      gc_right_heel_out_px = (DoubleYoVariable)robot.getVariable("gc_right_heel_out_px");
      gc_right_heel_out_py = (DoubleYoVariable)robot.getVariable("gc_right_heel_out_py");
      gc_right_heel_out_pz = (DoubleYoVariable)robot.getVariable("gc_right_heel_out_pz");
      gc_right_heel_out_tdx = (DoubleYoVariable)robot.getVariable("gc_right_heel_out_tdx");
      gc_right_heel_out_tdy = (DoubleYoVariable)robot.getVariable("gc_right_heel_out_tdy");
      gc_right_heel_out_tdz = (DoubleYoVariable)robot.getVariable("gc_right_heel_out_tdz");
      gc_right_heel_out_fs = (DoubleYoVariable)robot.getVariable("gc_right_heel_out_fs");
      gc_right_heel_out_slip = (BooleanYoVariable)robot.getVariable("gc_right_heel_out_slip");
      gc_right_heel_out_coll = (IntegerYoVariable)robot.getVariable("gc_right_heel_out_coll");
      q_left_hip_yaw = (DoubleYoVariable)robot.getVariable("q_left_hip_yaw");
      qd_left_hip_yaw = (DoubleYoVariable)robot.getVariable("qd_left_hip_yaw");
      qdd_left_hip_yaw = (DoubleYoVariable)robot.getVariable("qdd_left_hip_yaw");
      tau_left_hip_yaw = (DoubleYoVariable)robot.getVariable("tau_left_hip_yaw");
      q_left_hip_roll = (DoubleYoVariable)robot.getVariable("q_left_hip_roll");
      qd_left_hip_roll = (DoubleYoVariable)robot.getVariable("qd_left_hip_roll");
      qdd_left_hip_roll = (DoubleYoVariable)robot.getVariable("qdd_left_hip_roll");
      tau_left_hip_roll = (DoubleYoVariable)robot.getVariable("tau_left_hip_roll");
      q_left_hip_pitch = (DoubleYoVariable)robot.getVariable("q_left_hip_pitch");
      qd_left_hip_pitch = (DoubleYoVariable)robot.getVariable("qd_left_hip_pitch");
      qdd_left_hip_pitch = (DoubleYoVariable)robot.getVariable("qdd_left_hip_pitch");
      tau_left_hip_pitch = (DoubleYoVariable)robot.getVariable("tau_left_hip_pitch");
      q_left_knee = (DoubleYoVariable)robot.getVariable("q_left_knee");
      qd_left_knee = (DoubleYoVariable)robot.getVariable("qd_left_knee");
      qdd_left_knee = (DoubleYoVariable)robot.getVariable("qdd_left_knee");
      tau_left_knee = (DoubleYoVariable)robot.getVariable("tau_left_knee");
      tau_lim_left_knee = (DoubleYoVariable)robot.getVariable("tau_lim_left_knee");
      q_left_ankle_roll = (DoubleYoVariable)robot.getVariable("q_left_ankle_roll");
      qd_left_ankle_roll = (DoubleYoVariable)robot.getVariable("qd_left_ankle_roll");
      qdd_left_ankle_roll = (DoubleYoVariable)robot.getVariable("qdd_left_ankle_roll");
      tau_left_ankle_roll = (DoubleYoVariable)robot.getVariable("tau_left_ankle_roll");
      q_left_ankle_pitch = (DoubleYoVariable)robot.getVariable("q_left_ankle_pitch");
      qd_left_ankle_pitch = (DoubleYoVariable)robot.getVariable("qd_left_ankle_pitch");
      qdd_left_ankle_pitch = (DoubleYoVariable)robot.getVariable("qdd_left_ankle_pitch");
      tau_left_ankle_pitch = (DoubleYoVariable)robot.getVariable("tau_left_ankle_pitch");
      gc_left_toe_in_x = (DoubleYoVariable)robot.getVariable("gc_left_toe_in_x");
      gc_left_toe_in_y = (DoubleYoVariable)robot.getVariable("gc_left_toe_in_y");
      gc_left_toe_in_z = (DoubleYoVariable)robot.getVariable("gc_left_toe_in_z");
      gc_left_toe_in_dx = (DoubleYoVariable)robot.getVariable("gc_left_toe_in_dx");
      gc_left_toe_in_dy = (DoubleYoVariable)robot.getVariable("gc_left_toe_in_dy");
      gc_left_toe_in_dz = (DoubleYoVariable)robot.getVariable("gc_left_toe_in_dz");
      gc_left_toe_in_fx = (DoubleYoVariable)robot.getVariable("gc_left_toe_in_fx");
      gc_left_toe_in_fy = (DoubleYoVariable)robot.getVariable("gc_left_toe_in_fy");
      gc_left_toe_in_fz = (DoubleYoVariable)robot.getVariable("gc_left_toe_in_fz");
      gc_left_toe_in_px = (DoubleYoVariable)robot.getVariable("gc_left_toe_in_px");
      gc_left_toe_in_py = (DoubleYoVariable)robot.getVariable("gc_left_toe_in_py");
      gc_left_toe_in_pz = (DoubleYoVariable)robot.getVariable("gc_left_toe_in_pz");
      gc_left_toe_in_tdx = (DoubleYoVariable)robot.getVariable("gc_left_toe_in_tdx");
      gc_left_toe_in_tdy = (DoubleYoVariable)robot.getVariable("gc_left_toe_in_tdy");
      gc_left_toe_in_tdz = (DoubleYoVariable)robot.getVariable("gc_left_toe_in_tdz");
      gc_left_toe_in_fs = (DoubleYoVariable)robot.getVariable("gc_left_toe_in_fs");
      gc_left_toe_in_slip = (BooleanYoVariable)robot.getVariable("gc_left_toe_in_slip");
      gc_left_toe_in_coll = (IntegerYoVariable)robot.getVariable("gc_left_toe_in_coll");
      gc_left_toe_out_x = (DoubleYoVariable)robot.getVariable("gc_left_toe_out_x");
      gc_left_toe_out_y = (DoubleYoVariable)robot.getVariable("gc_left_toe_out_y");
      gc_left_toe_out_z = (DoubleYoVariable)robot.getVariable("gc_left_toe_out_z");
      gc_left_toe_out_dx = (DoubleYoVariable)robot.getVariable("gc_left_toe_out_dx");
      gc_left_toe_out_dy = (DoubleYoVariable)robot.getVariable("gc_left_toe_out_dy");
      gc_left_toe_out_dz = (DoubleYoVariable)robot.getVariable("gc_left_toe_out_dz");
      gc_left_toe_out_fx = (DoubleYoVariable)robot.getVariable("gc_left_toe_out_fx");
      gc_left_toe_out_fy = (DoubleYoVariable)robot.getVariable("gc_left_toe_out_fy");
      gc_left_toe_out_fz = (DoubleYoVariable)robot.getVariable("gc_left_toe_out_fz");
      gc_left_toe_out_px = (DoubleYoVariable)robot.getVariable("gc_left_toe_out_px");
      gc_left_toe_out_py = (DoubleYoVariable)robot.getVariable("gc_left_toe_out_py");
      gc_left_toe_out_pz = (DoubleYoVariable)robot.getVariable("gc_left_toe_out_pz");
      gc_left_toe_out_tdx = (DoubleYoVariable)robot.getVariable("gc_left_toe_out_tdx");
      gc_left_toe_out_tdy = (DoubleYoVariable)robot.getVariable("gc_left_toe_out_tdy");
      gc_left_toe_out_tdz = (DoubleYoVariable)robot.getVariable("gc_left_toe_out_tdz");
      gc_left_toe_out_fs = (DoubleYoVariable)robot.getVariable("gc_left_toe_out_fs");
      gc_left_toe_out_slip = (BooleanYoVariable)robot.getVariable("gc_left_toe_out_slip");
      gc_left_toe_out_coll = (IntegerYoVariable)robot.getVariable("gc_left_toe_out_coll");
      gc_left_heel_in_x = (DoubleYoVariable)robot.getVariable("gc_left_heel_in_x");
      gc_left_heel_in_y = (DoubleYoVariable)robot.getVariable("gc_left_heel_in_y");
      gc_left_heel_in_z = (DoubleYoVariable)robot.getVariable("gc_left_heel_in_z");
      gc_left_heel_in_dx = (DoubleYoVariable)robot.getVariable("gc_left_heel_in_dx");
      gc_left_heel_in_dy = (DoubleYoVariable)robot.getVariable("gc_left_heel_in_dy");
      gc_left_heel_in_dz = (DoubleYoVariable)robot.getVariable("gc_left_heel_in_dz");
      gc_left_heel_in_fx = (DoubleYoVariable)robot.getVariable("gc_left_heel_in_fx");
      gc_left_heel_in_fy = (DoubleYoVariable)robot.getVariable("gc_left_heel_in_fy");
      gc_left_heel_in_fz = (DoubleYoVariable)robot.getVariable("gc_left_heel_in_fz");
      gc_left_heel_in_px = (DoubleYoVariable)robot.getVariable("gc_left_heel_in_px");
      gc_left_heel_in_py = (DoubleYoVariable)robot.getVariable("gc_left_heel_in_py");
      gc_left_heel_in_pz = (DoubleYoVariable)robot.getVariable("gc_left_heel_in_pz");
      gc_left_heel_in_tdx = (DoubleYoVariable)robot.getVariable("gc_left_heel_in_tdx");
      gc_left_heel_in_tdy = (DoubleYoVariable)robot.getVariable("gc_left_heel_in_tdy");
      gc_left_heel_in_tdz = (DoubleYoVariable)robot.getVariable("gc_left_heel_in_tdz");
      gc_left_heel_in_fs = (DoubleYoVariable)robot.getVariable("gc_left_heel_in_fs");
      gc_left_heel_in_slip = (BooleanYoVariable)robot.getVariable("gc_left_heel_in_slip");
      gc_left_heel_in_coll = (IntegerYoVariable)robot.getVariable("gc_left_heel_in_coll");
      gc_left_heel_out_x = (DoubleYoVariable)robot.getVariable("gc_left_heel_out_x");
      gc_left_heel_out_y = (DoubleYoVariable)robot.getVariable("gc_left_heel_out_y");
      gc_left_heel_out_z = (DoubleYoVariable)robot.getVariable("gc_left_heel_out_z");
      gc_left_heel_out_dx = (DoubleYoVariable)robot.getVariable("gc_left_heel_out_dx");
      gc_left_heel_out_dy = (DoubleYoVariable)robot.getVariable("gc_left_heel_out_dy");
      gc_left_heel_out_dz = (DoubleYoVariable)robot.getVariable("gc_left_heel_out_dz");
      gc_left_heel_out_fx = (DoubleYoVariable)robot.getVariable("gc_left_heel_out_fx");
      gc_left_heel_out_fy = (DoubleYoVariable)robot.getVariable("gc_left_heel_out_fy");
      gc_left_heel_out_fz = (DoubleYoVariable)robot.getVariable("gc_left_heel_out_fz");
      gc_left_heel_out_px = (DoubleYoVariable)robot.getVariable("gc_left_heel_out_px");
      gc_left_heel_out_py = (DoubleYoVariable)robot.getVariable("gc_left_heel_out_py");
      gc_left_heel_out_pz = (DoubleYoVariable)robot.getVariable("gc_left_heel_out_pz");
      gc_left_heel_out_tdx = (DoubleYoVariable)robot.getVariable("gc_left_heel_out_tdx");
      gc_left_heel_out_tdy = (DoubleYoVariable)robot.getVariable("gc_left_heel_out_tdy");
      gc_left_heel_out_tdz = (DoubleYoVariable)robot.getVariable("gc_left_heel_out_tdz");
      gc_left_heel_out_fs = (DoubleYoVariable)robot.getVariable("gc_left_heel_out_fs");
      gc_left_heel_out_slip = (BooleanYoVariable)robot.getVariable("gc_left_heel_out_slip");
      gc_left_heel_out_coll = (IntegerYoVariable)robot.getVariable("gc_left_heel_out_coll");

      limbJointPositions = new DoubleYoVariable[][]
      {
         {
            q_left_hip_yaw, q_left_hip_roll, q_left_hip_pitch, q_left_knee, q_left_ankle_pitch, q_left_ankle_roll
         },
         {
            q_right_hip_yaw, q_right_hip_roll, q_right_hip_pitch, q_right_knee, q_right_ankle_pitch, q_right_ankle_roll
         }
      };
      limbJointVelocities = new DoubleYoVariable[][]
      {
         {
            qd_left_hip_yaw, qd_left_hip_roll, qd_left_hip_pitch, qd_left_knee, qd_left_ankle_pitch, qd_left_ankle_roll
         },
         {
            qd_right_hip_yaw, qd_right_hip_roll, qd_right_hip_pitch, qd_right_knee, qd_right_ankle_pitch, qd_right_ankle_roll
         }
      };
      limbJointAccelerations = new DoubleYoVariable[][]
      {
         {
            qdd_left_hip_yaw, qdd_left_hip_roll, qdd_left_hip_pitch, qdd_left_knee, qdd_left_ankle_pitch, qdd_left_ankle_roll
         },
         {
            qdd_right_hip_yaw, qdd_right_hip_roll, qdd_right_hip_pitch, qdd_right_knee, qdd_right_ankle_pitch, qdd_right_ankle_roll
         }
      };

      robotBodyPosition = new DoubleYoVariable[] {q_x, q_y, q_z};
      robotBodyVelocity = new DoubleYoVariable[] {qd_x, qd_y, qd_z};
      robotBodyAcceleration = new DoubleYoVariable[] {qdd_x, qdd_y, qdd_z};

      robotYawPitchAndRoll = new DoubleYoVariable[] {q_yaw, q_pitch, q_roll};

      robotYawPitchAndRollVelocity = new DoubleYoVariable[] {qd_wz, qd_wy, qd_wx};
      robotYawPitchAndRollAcceleration = new DoubleYoVariable[] {qdd_wz, qdd_wy, qdd_wx};

      groundContactFramePointsPositions = new DoubleYoVariable[][]
      {
         {
            gc_left_toe_in_x, gc_left_toe_in_y, gc_left_toe_in_z, gc_left_toe_out_x, gc_left_toe_out_y, gc_left_toe_out_z, gc_left_heel_in_x, gc_left_heel_in_y,
            gc_left_heel_in_z, gc_left_heel_out_x, gc_left_heel_out_y, gc_left_heel_out_z
         },
         {
            gc_right_toe_in_x, gc_right_toe_in_y, gc_right_toe_in_z, gc_right_toe_out_x, gc_right_toe_out_y, gc_right_toe_out_z, gc_right_heel_in_x,
            gc_right_heel_in_y, gc_right_heel_in_z, gc_right_heel_out_x, gc_right_heel_out_y, gc_right_heel_out_z,
         }
      };

      groundContactFramePointsForces = new DoubleYoVariable[][]
      {
         {
            gc_left_toe_in_fx, gc_left_toe_in_fy, gc_left_toe_in_fz, gc_left_toe_out_fx, gc_left_toe_out_fy, gc_left_toe_out_fz, gc_left_heel_in_fx,
            gc_left_heel_in_fy, gc_left_heel_in_fz, gc_left_heel_out_fx, gc_left_heel_out_fy, gc_left_heel_out_fz
         },
         {
            gc_right_toe_in_fx, gc_right_toe_in_fy, gc_right_toe_in_fz, gc_right_toe_out_fx, gc_right_toe_out_fy, gc_right_toe_out_fz, gc_right_heel_in_fx,
            gc_right_heel_in_fy, gc_right_heel_in_fz, gc_right_heel_out_fx, gc_right_heel_out_fy, gc_right_heel_out_fz,
         }
      };

      groundContactFramePointsFootSwitch = new DoubleYoVariable[][]
      {
         {gc_left_toe_in_fs, gc_left_toe_out_fs, gc_left_heel_in_fs, gc_left_heel_out_fs},
         {gc_right_toe_in_fs, gc_right_toe_out_fs, gc_right_heel_in_fs, gc_right_heel_out_fs}
      };

   }

   public void update()
   {
      processedSensors.time.set(robot.getTime());

      robot.updateYawPitchRoll();

      Point3D[][] footPositions = new Point3D[groundContactFramePointsPositions.length][groundContactFramePointsPositions[0].length / 3];
      for (int j = 0; j < footPositions.length; j++)
      {
         for (int i = 0; i < footPositions[0].length; i++)
         {
            footPositions[j][i] = new Point3D();
            footPositions[j][i].setX(groundContactFramePointsPositions[j][i * 3 + 0].getDoubleValue());
            footPositions[j][i].setY(groundContactFramePointsPositions[j][i * 3 + 1].getDoubleValue());
            footPositions[j][i].setZ(groundContactFramePointsPositions[j][i * 3 + 2].getDoubleValue());
         }
      }

      processedSensors.setGroundContactPointsPositions(footPositions);

      Point3D[][] footForces = new Point3D[groundContactFramePointsForces.length][groundContactFramePointsForces[0].length / 3];
      for (int j = 0; j < footForces.length; j++)
      {
         for (int i = 0; i < footForces[0].length; i++)
         {
            footForces[j][i] = new Point3D();
            footForces[j][i].setX(groundContactFramePointsForces[j][i * 3 + 0].getDoubleValue());
            footForces[j][i].setY(groundContactFramePointsForces[j][i * 3 + 1].getDoubleValue());
            footForces[j][i].setZ(groundContactFramePointsForces[j][i * 3 + 2].getDoubleValue());
         }
      }

      processedSensors.setGroundContactPointsForces(footForces);

      double[] leftFootSwitch = new double[groundContactFramePointsFootSwitch[RobotSide.LEFT.ordinal()].length];

      for (ContactPointName pointName : ContactPointName.values())
      {
         leftFootSwitch[pointName.ordinal()] = groundContactFramePointsFootSwitch[RobotSide.LEFT.ordinal()][pointName.ordinal()].getDoubleValue();
      }

      processedSensors.setGroundContactPointFootSwitch(RobotSide.LEFT, leftFootSwitch);

      double[] rightFootSwitch = new double[groundContactFramePointsFootSwitch[RobotSide.RIGHT.ordinal()].length];

      for (ContactPointName pointName : ContactPointName.values())
      {
         rightFootSwitch[pointName.ordinal()] = groundContactFramePointsFootSwitch[RobotSide.RIGHT.ordinal()][pointName.ordinal()].getDoubleValue();
      }

      processedSensors.setGroundContactPointFootSwitch(RobotSide.RIGHT, rightFootSwitch);

      // DO THIS FOR ALL -----
      double[] bodyOrientation = new double[RobotOrientation.values().length];
      for (RobotOrientation robotOrientation : RobotOrientation.values())
      {
         bodyOrientation[robotOrientation.ordinal()] = robotYawPitchAndRoll[robotOrientation.ordinal()].getDoubleValue();
      }

      processedSensors.setRobotYawPitchAndRoll(bodyOrientation);

      double[] bodyOrientationVelocity = new double[RobotOrientation.values().length];
      for (RobotOrientation robotOrientation : RobotOrientation.values())
      {
         bodyOrientationVelocity[robotOrientation.ordinal()] = robotYawPitchAndRollVelocity[robotOrientation.ordinal()].getDoubleValue();
      }

      processedSensors.setRobotYawPitchAndRollVelocity(bodyOrientationVelocity);

      double[] bodyOrientationAcceleration = new double[RobotOrientation.values().length];
      for (RobotOrientation robotOrientation : RobotOrientation.values())
      {
         bodyOrientationAcceleration[robotOrientation.ordinal()] = robotYawPitchAndRollAcceleration[robotOrientation.ordinal()].getDoubleValue();
      }

      processedSensors.setRobotYawPitchAndRollAcceleration(bodyOrientationAcceleration);

      double[] bodyPosition = new double[RobotAxis.values().length];
      for (RobotAxis robotAxis : RobotAxis.values())
      {
         bodyPosition[robotAxis.ordinal()] = robotBodyPosition[robotAxis.ordinal()].getDoubleValue();
      }

      processedSensors.setRobotBodyPositionInWorldcoordinates(bodyPosition);

      double[] bodyVelocity = new double[RobotAxis.values().length];
      for (RobotAxis robotAxis : RobotAxis.values())
      {
         bodyVelocity[robotAxis.ordinal()] = robotBodyVelocity[robotAxis.ordinal()].getDoubleValue();
      }

      processedSensors.setRobotBodyVelocityInWorldCoordiantes(bodyVelocity);

      double[] bodyAcceleration = new double[RobotAxis.values().length];
      for (RobotAxis robotAxis : RobotAxis.values())
      {
         bodyAcceleration[robotAxis.ordinal()] = robotBodyAcceleration[robotAxis.ordinal()].getDoubleValue();
      }

      processedSensors.setRobotBodyAccelerationInWorldcoordinates(bodyAcceleration);

      double[] leftSide = new double[limbJointPositions[RobotSide.LEFT.ordinal()].length];
      for (JointName jointName : JointName.values())
      {
         leftSide[jointName.ordinal()] = limbJointPositions[RobotSide.LEFT.ordinal()][jointName.ordinal()].getDoubleValue();
      }

      processedSensors.setLimbJointAngles(RobotSide.LEFT, leftSide);

      double[] rightSide = new double[limbJointPositions[RobotSide.RIGHT.ordinal()].length];
      for (JointName jointName : JointName.values())
      {
         rightSide[jointName.ordinal()] = limbJointPositions[RobotSide.RIGHT.ordinal()][jointName.ordinal()].getDoubleValue();
      }

      processedSensors.setLimbJointAngles(RobotSide.RIGHT, rightSide);

      double[] leftSideVelocities = new double[limbJointVelocities[RobotSide.LEFT.ordinal()].length];
      for (JointName jointName : JointName.values())
      {
         leftSideVelocities[jointName.ordinal()] = limbJointVelocities[RobotSide.LEFT.ordinal()][jointName.ordinal()].getDoubleValue();
      }

      processedSensors.setLimbJointVelocities(RobotSide.LEFT, leftSideVelocities);

      double[] rightSideVelocities = new double[limbJointVelocities[RobotSide.RIGHT.ordinal()].length];
      for (JointName jointName : JointName.values())
      {
         rightSideVelocities[jointName.ordinal()] = limbJointVelocities[RobotSide.RIGHT.ordinal()][jointName.ordinal()].getDoubleValue();
      }

      processedSensors.setLimbJointVelocities(RobotSide.RIGHT, rightSideVelocities);

      double[] leftSideAccelerations = new double[limbJointAccelerations[RobotSide.LEFT.ordinal()].length];
      for (JointName jointName : JointName.values())
      {
         leftSideAccelerations[jointName.ordinal()] = limbJointAccelerations[RobotSide.LEFT.ordinal()][jointName.ordinal()].getDoubleValue();
      }

      processedSensors.setLimbJointAccelerations(RobotSide.LEFT, leftSideAccelerations);

      double[] rightSideAccelerations = new double[limbJointAccelerations[RobotSide.RIGHT.ordinal()].length];
      for (JointName jointName : JointName.values())
      {
         rightSideAccelerations[jointName.ordinal()] = limbJointAccelerations[RobotSide.RIGHT.ordinal()][jointName.ordinal()].getDoubleValue();
      }

      processedSensors.setLimbJointAccelerations(RobotSide.RIGHT, rightSideAccelerations);
   }

   public void setInitialValues()
   {
      q_right_hip_yaw.set(-0.0828842);
      qd_right_hip_yaw.set(0.512097);
      tau_right_hip_yaw.set(-1.4211);

      q_right_hip_roll.set(0.00797754);
      qd_right_hip_roll.set(-0.134597);
      tau_right_hip_roll.set(2.90733);

      q_right_hip_pitch.set(0.323176);
      qd_right_hip_pitch.set(-3.74621);
      tau_right_hip_pitch.set(-10);

      q_right_knee.set(0.318247);
      qd_right_knee.set(6.8272);
      tau_right_knee.set(-1.70324);

      q_right_ankle_pitch.set(0.356919);
      qd_right_ankle_pitch.set(-5.93372);
      tau_right_ankle_pitch.set(-1.16255);

      q_right_ankle_roll.set(-0.0467784);
      qd_right_ankle_roll.set(0.0686819);
      tau_right_ankle_roll.set(0.109022);

      q_left_hip_yaw.set(-0.00831082);
      qd_left_hip_yaw.set(0.521853);
      tau_left_hip_yaw.set(-1.31891);

      q_left_hip_roll.set(-0.0354033);
      qd_left_hip_roll.set(-0.158869);
      tau_left_hip_roll.set(15.6981);

      q_left_hip_pitch.set(-0.194488);
      qd_left_hip_pitch.set(0.545706);
      tau_left_hip_pitch.set(19.0055);

      q_left_knee.set(-0.000996309);
      qd_left_knee.set(-0.0172213);
      tau_left_knee.set(0.0273088);

      q_left_ankle_pitch.set(0.164786);
      qd_left_ankle_pitch.set(-0.874391);
      tau_left_ankle_pitch.set(0);

      q_left_ankle_roll.set(0.0498968);
      qd_left_ankle_roll.set(-0.108174);
      tau_left_ankle_roll.set(-2.80329);

      q_x.set(-0.5);    // q_x.val = 0;
      qd_x.set(0.713894);

      q_y.set(-13.5);    // q_y.val = 0.183737;
      qd_y.set(0.156301);

      q_z.set(0.947832);
      qd_z.set(0.111067);

      q_yaw.set(0.0311369);
      qd_wz.set(-0.565967);    // qd_yaw;
      q_pitch.set(0.0184769);
      qd_wy.set(0.357002);    // qd_pitch
      q_roll.set(-0.0293983);
      qd_wx.set(0.0402278);    // qd_roll

      robot.setYawPitchRoll(q_yaw.getDoubleValue(), q_pitch.getDoubleValue(), q_roll.getDoubleValue());

   }

   private void setInitialValuesForHeavyM2()
   {
      qd_y.set(0.05);    // 0.0;

   }

}
