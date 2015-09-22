package us.ihmc.exampleSimulations.springflamingo;

import java.util.ArrayList;

import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.dataStructures.variable.BooleanYoVariable;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.simulationconstructionset.CameraMount;
import us.ihmc.simulationconstructionset.FloatingPlanarJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.IMUMount;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.JointWrenchSensor;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;

/**
 *
 * <p>Title: SpringFlamingoRobot</p>
 *
 * <p>Description: Simulation Model of SpringFlamingo.</p>
 *
 * <p>Copyright: Copyright (c) 2001</p>
 *
 * <p>Company: Yobotics, Inc.</p>
 *
 * @author not attributable
 * @version 1.0
 */
public class SpringFlamingoRobot extends Robot
{
   private static final long serialVersionUID = -2722107799882903451L;
   
   private static final boolean SHOW_MASS_PROPERTIES_GRAPHICS = false;
   private static final boolean SHOW_CARTOON_GRAPHICS = true;

   private static final double UPPER_LEG_MASS = 0.4598;
   private static final double
      UPPER_LEG_Ixx = 0.01256, UPPER_LEG_Iyy = 0.01256, UPPER_LEG_Izz = 0.00013;

   private static final double LOWER_LEG_MASS = 0.306;
   private static final double
      LOWER_LEG_Ixx = 0.00952, LOWER_LEG_Iyy = 0.00952, LOWER_LEG_Izz = 5.67e-5;

   private static final double FOOT_MASS = 0.3466;

   // private static final double FOOT_Ixx = 0.0015, FOOT_Iyy = 0.0015, FOOT_Izz = 7.15e-5;
   private static final double
      FOOT_Ixx = 7.15e-5, FOOT_Iyy = 0.0015, FOOT_Izz = 0.0015;

   public static final double BODY_Z = .385;
   public static final double BODY_Y = .12;
   public static final double BODY_X = .05;
   public static final double BODY_CG_Z = 0.20;
   public static final double BODY_Z_LEGPLOT = 0.21;
   public static final double BODY_Y_LEGPLOT = 0.363;
   public static final double BODY_X_LEGPLOT = 0.45;
   public static final double UPPER_LINK_LENGTH = 0.42;
   public static final double UPPER_LEG_ZMAX = 0.0;
   public static final double UPPER_LEG_ZMIN = -0.42;
   public static final double UPPER_LEG_Y = 0.02175;
   public static final double UPPER_LEG_X = 0.02175;
   public static final double LOWER_LINK_LENGTH = 0.42;
   public static final double LOWER_LEG_ZMAX = 0.0;
   public static final double LOWER_LEG_ZMIN = -0.42;
   public static final double LOWER_LEG_Y = 0.02175;
   public static final double LOWER_LEG_X = 0.02175;
   public static final double FOOT_ZMIN = -0.04;
   public static final double FOOT_ZMAX = -0.01;
   public static final double FOOT_Y = 0.04;
   public static final double FOOT_X = 0.23;
   public static final double FOOT_H = (0.04);
   public static final double FOOT_OFFSET_PERCENT = 0.25;
   public static final double FOOT_FORWARD = (FOOT_X * FOOT_OFFSET_PERCENT);
   public static final double FOOT_BEHIND = FOOT_X - FOOT_FORWARD;
   public static final double HIP_OFFSET_Y = 0.12;

   public final DoubleYoVariable t, q_x, q_z, q_pitch, qd_x, qd_z, qd_pitch, qdd_x, qdd_z, qdd_pitch;
   public final DoubleYoVariable q_rh, qd_rh, qdd_rh, tau_rh, q_rk, qd_rk, qdd_rk, tau_rk, tau_joint_limit_rk, q_ra, qd_ra, qdd_ra, tau_ra;
   public final DoubleYoVariable q_lh, qd_lh, qdd_lh, tau_lh, q_lk, qd_lk, qdd_lk, tau_lk, tau_joint_limit_lk, q_la, qd_la, qdd_la, tau_la;

   public final DoubleYoVariable gc_rheel_x, gc_rheel_y, gc_rheel_z, gc_rheel_dx, gc_rheel_dy, gc_rheel_dz, gc_rheel_fx, gc_rheel_fy, gc_rheel_fz;
   public final DoubleYoVariable gc_rheel_px, gc_rheel_py, gc_rheel_pz, gc_rheel_tdx, gc_rheel_tdy, gc_rheel_tdz, gc_rheel_fs;
   public final BooleanYoVariable gc_rheel_slip;

   public final DoubleYoVariable gc_rtoe_x, gc_rtoe_y, gc_rtoe_z, gc_rtoe_dx, gc_rtoe_dy, gc_rtoe_dz, gc_rtoe_fx, gc_rtoe_fy, gc_rtoe_fz;
   public final DoubleYoVariable gc_rtoe_px, gc_rtoe_py, gc_rtoe_pz, gc_rtoe_tdx, gc_rtoe_tdy, gc_rtoe_tdz, gc_rtoe_fs;
   public final BooleanYoVariable gc_rtoe_slip;

   public final DoubleYoVariable gc_lheel_x, gc_lheel_y, gc_lheel_z, gc_lheel_dx, gc_lheel_dy, gc_lheel_dz, gc_lheel_fx, gc_lheel_fy, gc_lheel_fz;
   public final DoubleYoVariable gc_lheel_px, gc_lheel_py, gc_lheel_pz, gc_lheel_tdx, gc_lheel_tdy, gc_lheel_tdz, gc_lheel_fs; 
   public final BooleanYoVariable gc_lheel_slip;

   public final DoubleYoVariable gc_ltoe_x, gc_ltoe_y, gc_ltoe_z, gc_ltoe_dx, gc_ltoe_dy, gc_ltoe_dz, gc_ltoe_fx, gc_ltoe_fy, gc_ltoe_fz;
   public final DoubleYoVariable gc_ltoe_px, gc_ltoe_py, gc_ltoe_pz, gc_ltoe_tdx, gc_ltoe_tdy, gc_ltoe_tdz, gc_ltoe_fs;
   public final BooleanYoVariable gc_ltoe_slip;
   
   private final Joint plane, rightHip, rightKnee, rightAnkle, leftHip, leftKnee, leftAnkle;

   private final ArrayList<GroundContactPoint> gcPoints = new ArrayList<GroundContactPoint>(4);

   public SpringFlamingoRobot(String name)
   {
      super(name);
      this.setGravity(0.0, 0.0, -9.81);

      plane = new FloatingPlanarJoint("plane", this, FloatingPlanarJoint.XZ);
      Link body = body();
      plane.setLink(body);
      this.addRootJoint(plane);

      RigidBodyTransform camRotation = new RigidBodyTransform();
      camRotation.rotZ(Math.PI);

      CameraMount robotCam = new CameraMount("robot cam mount", camRotation, this);
      plane.addCameraMount(robotCam);
      
      RigidBodyTransform imuTransform = new RigidBodyTransform();
      imuTransform.setTranslation(new Vector3d(0.0, 0.0, 0.2));
      IMUMount imuMount = new IMUMount("FlamingoIMU", imuTransform, this);
      plane.addIMUMount(imuMount);
      
      /** ************************ Right limb ********************************** */

      rightHip = new PinJoint("rh", new Vector3d(0.0, -HIP_OFFSET_Y, 0.0), this, Axis.Y);    // right hip joint
      Link r_upper_leg = upper_leg("r_upper_leg");
      rightHip.setLink(r_upper_leg);
      plane.addJoint(rightHip);

      JointWrenchSensor rightHipWrenchSensor = new JointWrenchSensor("rightHipWrenchSensor",  new Vector3d(), this);
      rightHip.addJointWrenchSensor(rightHipWrenchSensor);
      
      rightKnee = new PinJoint("rk", new Vector3d(0.0, 0.0, -UPPER_LINK_LENGTH), this, Axis.Y);    // right knee joint
      Link r_lower_leg = lower_leg("r_lower_leg");
      rightKnee.setLink(r_lower_leg);
      rightHip.addJoint(rightKnee);
      ((PinJoint) rightKnee).setLimitStops(-Math.PI, 0.0, 1000.0, 40.0);

      JointWrenchSensor rightKneeWrenchSensor = new JointWrenchSensor("rightKneeWrenchSensor",  new Vector3d(), this);
      rightKnee.addJointWrenchSensor(rightKneeWrenchSensor);
      
      rightAnkle = new PinJoint("ra", new Vector3d(0.0, 0.0, -LOWER_LINK_LENGTH), this, Axis.Y);    // right ankle joint
      Link r_foot = foot("r_foot");
      rightAnkle.setLink(r_foot);
      rightKnee.addJoint(rightAnkle);
      
      GroundContactPoint gc_rheel = new GroundContactPoint("gc_rheel", new Vector3d(FOOT_OFFSET_PERCENT * FOOT_X, 0.0, FOOT_ZMIN), this);
      GroundContactPoint gc_rtoe = new GroundContactPoint("gc_rtoe", new Vector3d(-(1.0 - FOOT_OFFSET_PERCENT) * FOOT_X, 0.0, FOOT_ZMIN), this);

      gcPoints.add(gc_rheel);
      gcPoints.add(gc_rtoe);

      rightAnkle.addGroundContactPoint(gc_rheel);
      rightAnkle.addGroundContactPoint(gc_rtoe);
      
      JointWrenchSensor rightAnkleWrenchSensor = new JointWrenchSensor("rightAnkleWrenchSensor",  new Vector3d(), this);
      rightAnkle.addJointWrenchSensor(rightAnkleWrenchSensor);

      /** ************************ Left limb ********************************** */

      leftHip = new PinJoint("lh", new Vector3d(0.0, HIP_OFFSET_Y, 0.0), this, Axis.Y);    // left hip joint
      Link l_upper_leg = upper_leg("l_upper_leg");
      leftHip.setLink(l_upper_leg);
      plane.addJoint(leftHip);

      JointWrenchSensor leftHipWrenchSensor = new JointWrenchSensor("leftHipWrenchSensor",  new Vector3d(), this);
      leftHip.addJointWrenchSensor(leftHipWrenchSensor);
      
      leftKnee = new PinJoint("lk", new Vector3d(0.0, 0.0, -UPPER_LINK_LENGTH), this, Axis.Y);    // left knee joint
      Link l_lower_leg = lower_leg("l_lower_leg");
      leftKnee.setLink(l_lower_leg);
      leftHip.addJoint(leftKnee);
      ((PinJoint) leftKnee).setLimitStops(-Math.PI, 0.0, 1000.0, 40.0);

      JointWrenchSensor leftKneeWrenchSensor = new JointWrenchSensor("leftKneeWrenchSensor", new Vector3d(), this);
      leftKnee.addJointWrenchSensor(leftKneeWrenchSensor);
      
      leftAnkle = new PinJoint("la", new Vector3d(0.0, 0.0, -LOWER_LINK_LENGTH), this, Axis.Y);    // left ankle joint
      Link l_foot = foot("l_foot");
      leftAnkle.setLink(l_foot);
      leftKnee.addJoint(leftAnkle);

      GroundContactPoint gc_lheel = new GroundContactPoint("gc_lheel", new Vector3d(FOOT_OFFSET_PERCENT * FOOT_X, 0.0, FOOT_ZMIN), this);
      GroundContactPoint gc_ltoe = new GroundContactPoint("gc_ltoe", new Vector3d(-(1.0 - FOOT_OFFSET_PERCENT) * FOOT_X, 0.0, FOOT_ZMIN), this);

      gcPoints.add(gc_lheel);
      gcPoints.add(gc_ltoe);
      
      leftAnkle.addGroundContactPoint(gc_lheel);
      leftAnkle.addGroundContactPoint(gc_ltoe);

      JointWrenchSensor leftAnkleWrenchSensor = new JointWrenchSensor("leftAnkleWrenchSensor",  new Vector3d(), this);
      leftAnkle.addJointWrenchSensor(leftAnkleWrenchSensor);
      
      // Extract YoVariables:

      t = (DoubleYoVariable)getVariable("t");
      q_x = (DoubleYoVariable)getVariable("q_x");
      q_z = (DoubleYoVariable)getVariable("q_z");
      q_pitch = (DoubleYoVariable)getVariable("q_pitch");
      qd_x = (DoubleYoVariable)getVariable("qd_x");
      qd_z = (DoubleYoVariable)getVariable("qd_z");
      qd_pitch = (DoubleYoVariable)getVariable("qd_pitch");
      qdd_x = (DoubleYoVariable)getVariable("qdd_x");
      qdd_z = (DoubleYoVariable)getVariable("qdd_z");
      qdd_pitch = (DoubleYoVariable)getVariable("qdd_pitch");
      q_rh = (DoubleYoVariable)getVariable("q_rh");
      qd_rh = (DoubleYoVariable)getVariable("qd_rh");
      qdd_rh = (DoubleYoVariable)getVariable("qdd_rh");
      tau_rh = (DoubleYoVariable)getVariable("tau_rh");
      q_rk = (DoubleYoVariable)getVariable("q_rk");
      qd_rk = (DoubleYoVariable)getVariable("qd_rk");
      qdd_rk = (DoubleYoVariable)getVariable("qdd_rk");
      tau_rk = (DoubleYoVariable)getVariable("tau_rk");
      tau_joint_limit_rk = (DoubleYoVariable)getVariable("tau_joint_limit_rk");
      q_ra = (DoubleYoVariable)getVariable("q_ra");
      qd_ra = (DoubleYoVariable)getVariable("qd_ra");
      qdd_ra = (DoubleYoVariable)getVariable("qdd_ra");
      tau_ra = (DoubleYoVariable)getVariable("tau_ra");
      q_lh = (DoubleYoVariable)getVariable("q_lh");
      qd_lh = (DoubleYoVariable)getVariable("qd_lh");
      qdd_lh = (DoubleYoVariable)getVariable("qdd_lh");
      tau_lh = (DoubleYoVariable)getVariable("tau_lh");
      q_lk = (DoubleYoVariable)getVariable("q_lk");
      qd_lk = (DoubleYoVariable)getVariable("qd_lk");
      qdd_lk = (DoubleYoVariable)getVariable("qdd_lk");
      tau_lk = (DoubleYoVariable)getVariable("tau_lk");
      tau_joint_limit_lk = (DoubleYoVariable)getVariable("tau_joint_limit_lk");
      q_la = (DoubleYoVariable)getVariable("q_la");
      qd_la = (DoubleYoVariable)getVariable("qd_la");
      qdd_la = (DoubleYoVariable)getVariable("qdd_la");
      tau_la = (DoubleYoVariable)getVariable("tau_la");

      gc_rheel_x = (DoubleYoVariable)getVariable("gc_rheel_x");
      gc_rheel_y = (DoubleYoVariable)getVariable("gc_rheel_y");
      gc_rheel_z = (DoubleYoVariable)getVariable("gc_rheel_z");
      gc_rheel_dx = (DoubleYoVariable)getVariable("gc_rheel_dx");
      gc_rheel_dy = (DoubleYoVariable)getVariable("gc_rheel_dy");
      gc_rheel_dz = (DoubleYoVariable)getVariable("gc_rheel_dz");
      gc_rheel_fx = (DoubleYoVariable)getVariable("gc_rheel_fx");
      gc_rheel_fy = (DoubleYoVariable)getVariable("gc_rheel_fy");
      gc_rheel_fz = (DoubleYoVariable)getVariable("gc_rheel_fz");
      gc_rheel_px = (DoubleYoVariable)getVariable("gc_rheel_px");
      gc_rheel_py = (DoubleYoVariable)getVariable("gc_rheel_py");
      gc_rheel_pz = (DoubleYoVariable)getVariable("gc_rheel_pz");
      gc_rheel_tdx = (DoubleYoVariable)getVariable("gc_rheel_tdx");
      gc_rheel_tdy = (DoubleYoVariable)getVariable("gc_rheel_tdy");
      gc_rheel_tdz = (DoubleYoVariable)getVariable("gc_rheel_tdz");
      gc_rheel_fs = (DoubleYoVariable)getVariable("gc_rheel_fs");
      gc_rheel_slip = (BooleanYoVariable)getVariable("gc_rheel_slip");
      gc_rtoe_x = (DoubleYoVariable)getVariable("gc_rtoe_x");
      gc_rtoe_y = (DoubleYoVariable)getVariable("gc_rtoe_y");
      gc_rtoe_z = (DoubleYoVariable)getVariable("gc_rtoe_z");
      gc_rtoe_dx = (DoubleYoVariable)getVariable("gc_rtoe_dx");
      gc_rtoe_dy = (DoubleYoVariable)getVariable("gc_rtoe_dy");
      gc_rtoe_dz = (DoubleYoVariable)getVariable("gc_rtoe_dz");
      gc_rtoe_fx = (DoubleYoVariable)getVariable("gc_rtoe_fx");
      gc_rtoe_fy = (DoubleYoVariable)getVariable("gc_rtoe_fy");
      gc_rtoe_fz = (DoubleYoVariable)getVariable("gc_rtoe_fz");
      gc_rtoe_px = (DoubleYoVariable)getVariable("gc_rtoe_px");
      gc_rtoe_py = (DoubleYoVariable)getVariable("gc_rtoe_py");
      gc_rtoe_pz = (DoubleYoVariable)getVariable("gc_rtoe_pz");
      gc_rtoe_tdx = (DoubleYoVariable)getVariable("gc_rtoe_tdx");
      gc_rtoe_tdy = (DoubleYoVariable)getVariable("gc_rtoe_tdy");
      gc_rtoe_tdz = (DoubleYoVariable)getVariable("gc_rtoe_tdz");
      gc_rtoe_fs = (DoubleYoVariable)getVariable("gc_rtoe_fs");
      gc_rtoe_slip = (BooleanYoVariable)getVariable("gc_rtoe_slip");
      gc_lheel_x = (DoubleYoVariable)getVariable("gc_lheel_x");
      gc_lheel_y = (DoubleYoVariable)getVariable("gc_lheel_y");
      gc_lheel_z = (DoubleYoVariable)getVariable("gc_lheel_z");
      gc_lheel_dx = (DoubleYoVariable)getVariable("gc_lheel_dx");
      gc_lheel_dy = (DoubleYoVariable)getVariable("gc_lheel_dy");
      gc_lheel_dz = (DoubleYoVariable)getVariable("gc_lheel_dz");
      gc_lheel_fx = (DoubleYoVariable)getVariable("gc_lheel_fx");
      gc_lheel_fy = (DoubleYoVariable)getVariable("gc_lheel_fy");
      gc_lheel_fz = (DoubleYoVariable)getVariable("gc_lheel_fz");
      gc_lheel_px = (DoubleYoVariable)getVariable("gc_lheel_px");
      gc_lheel_py = (DoubleYoVariable)getVariable("gc_lheel_py");
      gc_lheel_pz = (DoubleYoVariable)getVariable("gc_lheel_pz");
      gc_lheel_tdx = (DoubleYoVariable)getVariable("gc_lheel_tdx");
      gc_lheel_tdy = (DoubleYoVariable)getVariable("gc_lheel_tdy");
      gc_lheel_tdz = (DoubleYoVariable)getVariable("gc_lheel_tdz");
      gc_lheel_fs = (DoubleYoVariable)getVariable("gc_lheel_fs");
      gc_lheel_slip = (BooleanYoVariable)getVariable("gc_lheel_slip");
      gc_ltoe_x = (DoubleYoVariable)getVariable("gc_ltoe_x");
      gc_ltoe_y = (DoubleYoVariable)getVariable("gc_ltoe_y");
      gc_ltoe_z = (DoubleYoVariable)getVariable("gc_ltoe_z");
      gc_ltoe_dx = (DoubleYoVariable)getVariable("gc_ltoe_dx");
      gc_ltoe_dy = (DoubleYoVariable)getVariable("gc_ltoe_dy");
      gc_ltoe_dz = (DoubleYoVariable)getVariable("gc_ltoe_dz");
      gc_ltoe_fx = (DoubleYoVariable)getVariable("gc_ltoe_fx");
      gc_ltoe_fy = (DoubleYoVariable)getVariable("gc_ltoe_fy");
      gc_ltoe_fz = (DoubleYoVariable)getVariable("gc_ltoe_fz");
      gc_ltoe_px = (DoubleYoVariable)getVariable("gc_ltoe_px");
      gc_ltoe_py = (DoubleYoVariable)getVariable("gc_ltoe_py");
      gc_ltoe_pz = (DoubleYoVariable)getVariable("gc_ltoe_pz");
      gc_ltoe_tdx = (DoubleYoVariable)getVariable("gc_ltoe_tdx");
      gc_ltoe_tdy = (DoubleYoVariable)getVariable("gc_ltoe_tdy");
      gc_ltoe_tdz = (DoubleYoVariable)getVariable("gc_ltoe_tdz");
      gc_ltoe_fs = (DoubleYoVariable)getVariable("gc_ltoe_fs");
      gc_ltoe_slip = (BooleanYoVariable)getVariable("gc_ltoe_slip");

   }

   private Link body()
   {
      Link ret = new Link("body");

      ret.setMass(12.0);
      ret.setMomentOfInertia(0.10, 0.10, 0.10);

      ret.setComOffset(0.0, 0.0, BODY_CG_Z);

      // ret.setComOffset(0.0,0.0, 0.0);

      AppearanceDefinition bodyAppearance = YoAppearance.Red();

      if (SHOW_MASS_PROPERTIES_GRAPHICS)
      {
         ret.addEllipsoidFromMassProperties(YoAppearance.DarkRed());
         ret.addCoordinateSystemToCOM(0.2);
      }

      Graphics3DObject linkGraphics = new Graphics3DObject();
      if (SHOW_CARTOON_GRAPHICS)
      {
         /* For left half */
         linkGraphics.translate(0.0, HIP_OFFSET_Y, 0.0);
         linkGraphics.rotate(-55.0 * Math.PI / 180.0, Axis.Y);
         linkGraphics.addCube(BODY_X, BODY_Y, BODY_Z, bodyAppearance);

         linkGraphics.rotate(7.333 * Math.PI / 180.0, Axis.Y);
         linkGraphics.addCube(BODY_X, BODY_Y, BODY_Z, bodyAppearance);
         linkGraphics.rotate(7.333 * Math.PI / 180.0, Axis.Y);
         linkGraphics.addCube(BODY_X, BODY_Y, BODY_Z, bodyAppearance);
         linkGraphics.rotate(7.333 * Math.PI / 180.0, Axis.Y);
         linkGraphics.addCube(BODY_X, BODY_Y, BODY_Z, bodyAppearance);
         linkGraphics.rotate(7.333 * Math.PI / 180.0, Axis.Y);
         linkGraphics.addCube(BODY_X, BODY_Y, BODY_Z, bodyAppearance);
         linkGraphics.rotate(7.333 * Math.PI / 180.0, Axis.Y);
         linkGraphics.addCube(BODY_X, BODY_Y, BODY_Z, bodyAppearance);
         linkGraphics.rotate(7.333 * Math.PI / 180.0, Axis.Y);
         linkGraphics.addCube(BODY_X, BODY_Y, BODY_Z, bodyAppearance);
         linkGraphics.rotate(7.333 * Math.PI / 180.0, Axis.Y);
         linkGraphics.addCube(BODY_X, BODY_Y, BODY_Z, bodyAppearance);
         linkGraphics.rotate(7.333 * Math.PI / 180.0, Axis.Y);
         linkGraphics.addCube(BODY_X, BODY_Y, BODY_Z, bodyAppearance);
         linkGraphics.rotate(7.333 * Math.PI / 180.0, Axis.Y);
         linkGraphics.addCube(BODY_X, BODY_Y, BODY_Z, bodyAppearance);
         linkGraphics.rotate(7.333 * Math.PI / 180.0, Axis.Y);
         linkGraphics.addCube(BODY_X, BODY_Y, BODY_Z, bodyAppearance);
         linkGraphics.rotate(7.333 * Math.PI / 180.0, Axis.Y);
         linkGraphics.addCube(BODY_X, BODY_Y, BODY_Z, bodyAppearance);
         linkGraphics.rotate(7.333 * Math.PI / 180.0, Axis.Y);
         linkGraphics.addCube(BODY_X, BODY_Y, BODY_Z, bodyAppearance);
         linkGraphics.rotate(7.333 * Math.PI / 180.0, Axis.Y);
         linkGraphics.addCube(BODY_X, BODY_Y, BODY_Z, bodyAppearance);
         linkGraphics.rotate(7.333 * Math.PI / 180.0, Axis.Y);
         linkGraphics.addCube(BODY_X, BODY_Y, BODY_Z, bodyAppearance);
         linkGraphics.rotate(7.333 * Math.PI / 180.0, Axis.Y);
         linkGraphics.addCube(BODY_X, BODY_Y, BODY_Z, bodyAppearance);

         linkGraphics.identity();

         // For right half
         linkGraphics.translate(0.0, -HIP_OFFSET_Y, 0.0);
         linkGraphics.rotate(-55.0 * Math.PI / 180.0, Axis.Y);
         linkGraphics.addCube(BODY_X, BODY_Y, BODY_Z, bodyAppearance);

         linkGraphics.rotate(7.333 * Math.PI / 180.0, Axis.Y);
         linkGraphics.addCube(BODY_X, BODY_Y, BODY_Z, bodyAppearance);
         linkGraphics.rotate(7.333 * Math.PI / 180.0, Axis.Y);
         linkGraphics.addCube(BODY_X, BODY_Y, BODY_Z, bodyAppearance);
         linkGraphics.rotate(7.333 * Math.PI / 180.0, Axis.Y);
         linkGraphics.addCube(BODY_X, BODY_Y, BODY_Z, bodyAppearance);
         linkGraphics.rotate(7.333 * Math.PI / 180.0, Axis.Y);
         linkGraphics.addCube(BODY_X, BODY_Y, BODY_Z, bodyAppearance);
         linkGraphics.rotate(7.333 * Math.PI / 180.0, Axis.Y);
         linkGraphics.addCube(BODY_X, BODY_Y, BODY_Z, bodyAppearance);
         linkGraphics.rotate(7.333 * Math.PI / 180.0, Axis.Y);
         linkGraphics.addCube(BODY_X, BODY_Y, BODY_Z, bodyAppearance);
         linkGraphics.rotate(7.333 * Math.PI / 180.0, Axis.Y);
         linkGraphics.addCube(BODY_X, BODY_Y, BODY_Z, bodyAppearance);
         linkGraphics.rotate(7.333 * Math.PI / 180.0, Axis.Y);
         linkGraphics.addCube(BODY_X, BODY_Y, BODY_Z, bodyAppearance);
         linkGraphics.rotate(7.333 * Math.PI / 180.0, Axis.Y);
         linkGraphics.addCube(BODY_X, BODY_Y, BODY_Z, bodyAppearance);
         linkGraphics.rotate(7.333 * Math.PI / 180.0, Axis.Y);
         linkGraphics.addCube(BODY_X, BODY_Y, BODY_Z, bodyAppearance);
         linkGraphics.rotate(7.333 * Math.PI / 180.0, Axis.Y);
         linkGraphics.addCube(BODY_X, BODY_Y, BODY_Z, bodyAppearance);
         linkGraphics.rotate(7.333 * Math.PI / 180.0, Axis.Y);
         linkGraphics.addCube(BODY_X, BODY_Y, BODY_Z, bodyAppearance);
         linkGraphics.rotate(7.333 * Math.PI / 180.0, Axis.Y);
         linkGraphics.addCube(BODY_X, BODY_Y, BODY_Z, bodyAppearance);
         linkGraphics.rotate(7.333 * Math.PI / 180.0, Axis.Y);
         linkGraphics.addCube(BODY_X, BODY_Y, BODY_Z, bodyAppearance);
         linkGraphics.rotate(7.333 * Math.PI / 180.0, Axis.Y);
         linkGraphics.addCube(BODY_X, BODY_Y, BODY_Z, bodyAppearance);

         linkGraphics.identity();

         linkGraphics.rotate(Math.PI / 2.0, Axis.X);
         linkGraphics.translate(0.0, 0.0, -0.37 / 2.0);

         linkGraphics.addCylinder(0.37, 0.025);
      }

      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private Link upper_leg(String name)
   {
      Link ret = new Link(name);

      AppearanceDefinition pulleyAppearance = YoAppearance.Red();

      ret.setMass(UPPER_LEG_MASS);
      ret.setMomentOfInertia(UPPER_LEG_Ixx, UPPER_LEG_Iyy, UPPER_LEG_Izz);
      ret.setComOffset(0.0, 0.0, -0.3029);

      if (SHOW_MASS_PROPERTIES_GRAPHICS)
      {
         ret.addEllipsoidFromMassProperties(YoAppearance.Fuchsia());
         ret.addCoordinateSystemToCOM(0.2);
      }

      Graphics3DObject linkGraphics = new Graphics3DObject();
      if (SHOW_CARTOON_GRAPHICS)
      {
         linkGraphics.translate(0.0, 0.0, -UPPER_LINK_LENGTH);
         linkGraphics.addCube(0.025, 0.053, 0.054, YoAppearance.AluminumMaterial());

         linkGraphics.translate(0.0, 0.0, 0.054);
         linkGraphics.addCylinder(0.308, 0.0113, YoAppearance.BlackMetalMaterial());

         linkGraphics.translate(0.0, 0.0, 0.308);
         linkGraphics.addCube(0.025, 0.053, 0.047, YoAppearance.AluminumMaterial());

         linkGraphics.identity();
         linkGraphics.translate(0.0, 0.0, -UPPER_LINK_LENGTH);
         linkGraphics.rotate(Math.PI / 2.0, Axis.X);
         linkGraphics.translate(0.0, 0.0, -0.005);
         linkGraphics.addCylinder(0.01, 0.033, pulleyAppearance);    /* Pulley */

         linkGraphics.identity();
         linkGraphics.translate(0.0, 0.0, -UPPER_LINK_LENGTH);
         linkGraphics.rotate(Math.PI / 2.0, Axis.X);
         linkGraphics.translate(0.0, 0.0, -0.013);
         linkGraphics.addCylinder(0.025, 0.015, pulleyAppearance);    /* Pulley */
      }

      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private Link lower_leg(String name)
   {
      Link ret = new Link(name);

      ret.setMass(LOWER_LEG_MASS);
      ret.setMomentOfInertia(LOWER_LEG_Ixx, LOWER_LEG_Iyy, LOWER_LEG_Izz);
      ret.setComOffset(0.0, 0.0, -0.1818);

      if (SHOW_MASS_PROPERTIES_GRAPHICS)
      {
         ret.addEllipsoidFromMassProperties(YoAppearance.Yellow());
         ret.addCoordinateSystemToCOM(0.2);
      }

      if (SHOW_CARTOON_GRAPHICS)
      {
    	 Graphics3DObject linkGraphics = new Graphics3DObject();
    	 linkGraphics.translate(0.0, 0.0, -LOWER_LINK_LENGTH);
    	 linkGraphics.addCube(0.025, 0.04, 0.04, YoAppearance.AluminumMaterial());

    	 linkGraphics.translate(0.0, 0.0, 0.04);
    	 linkGraphics.addCylinder(0.308, 0.0113, YoAppearance.BlackMetalMaterial());

    	 linkGraphics.translate(0.0, 0.0, 0.308);
    	 linkGraphics.addCube(0.025, 0.04, 0.062, YoAppearance.AluminumMaterial());
    	 ret.setLinkGraphics(linkGraphics);
      }

      return ret;
   }

   private Link foot(String name)
   {
      Link ret = new Link(name);

      ret.setMass(FOOT_MASS);
      ret.setMomentOfInertia(FOOT_Ixx, FOOT_Iyy, FOOT_Izz);
      ret.setComOffset(-0.0458, 0.0, -0.0309);

      if (SHOW_MASS_PROPERTIES_GRAPHICS)
      {
         ret.addEllipsoidFromMassProperties(YoAppearance.Aqua());
         ret.addCoordinateSystemToCOM(0.2);
      }

      if (SHOW_CARTOON_GRAPHICS)
      {
     	 Graphics3DObject linkGraphics = new Graphics3DObject();
     	 linkGraphics.translate(FOOT_X * (FOOT_OFFSET_PERCENT - 0.5), 0.0, FOOT_ZMIN);
     	 linkGraphics.addCube(FOOT_X, FOOT_Y, FOOT_ZMAX - FOOT_ZMIN);

     	 linkGraphics.identity();
     	 linkGraphics.translate(0.0, 0.0, FOOT_ZMAX);
     	 linkGraphics.addCube(0.028, 0.028, -FOOT_ZMAX);
    	 ret.setLinkGraphics(linkGraphics);
      }

      return ret;
   }

   public double getHipAngle(RobotSide robotSide)
   {
      if (robotSide == RobotSide.LEFT)
         return q_lh.getDoubleValue();
      else
         return q_rh.getDoubleValue();
   }

   public double getKneeAngle(RobotSide robotSide)
   {
      if (robotSide == RobotSide.LEFT)
         return q_lk.getDoubleValue();
      else
         return q_rk.getDoubleValue();
   }

   public double getAnkleAngle(RobotSide robotSide)
   {
      if (robotSide == RobotSide.LEFT)
         return q_la.getDoubleValue();
      else
         return q_ra.getDoubleValue();
   }

   public double getHipVelocity(RobotSide robotSide)
   {
      if (robotSide == RobotSide.LEFT)
         return qd_lh.getDoubleValue();
      else
         return qd_rh.getDoubleValue();
   }

   public double getKneeVelocity(RobotSide robotSide)
   {
      if (robotSide == RobotSide.LEFT)
         return qd_lk.getDoubleValue();
      else
         return qd_rk.getDoubleValue();
   }

   public double getAnkleVelocity(RobotSide robotSide)
   {
      if (robotSide == RobotSide.LEFT)
         return qd_la.getDoubleValue();
      else
         return qd_ra.getDoubleValue();
   }

   public void initializeForFastWalking(RobotSide robotSide)
   {
      q_x.set(0.0);
      q_z.set(0.89);    // 0.859601;
      q_pitch.set(0.0);
      q_rh.set(-0.01);
      q_rk.set(0.0);
      q_ra.set(0.0);
      q_lh.set(-0.01);
      q_lk.set(0.0);
      qd_lk.set(0.0);
      q_la.set(0.0);
      qd_la.set(0.0);

      qd_x.set(0.0);
      qd_z.set(0.0);
      qd_pitch.set(0.0);
      qd_rh.set(0.0);
      qd_rk.set(0.0);
      qd_ra.set(0.0);
      qd_lh.set(0.0);

   }

   public void initializeForBallisticWalking()
   {
      t.set(0.0);
      q_x.set(-6.682748329552);
      q_z.set(0.8595491387990886);
      q_pitch.set(0.015960335262156407);
      qd_x.set(-0.7458267603119068);
      qd_z.set(-0.1316280037632628);
      qd_pitch.set(0.2936523063890944);
      qdd_x.set(-0.014403137733384336);
      qdd_z.set(-9.875003450455294);
      qdd_pitch.set(0.32073293177715856);
      q_rh.set(-0.21087503928827175);
      qd_rh.set(-1.1052064349476054);
      qdd_rh.set(-0.5410391265542851);
      tau_rh.set(0.0);
      q_rk.set(-3.169140834396399E-6);
      qd_rk.set(0.02809029011577098);
      qdd_rk.set(0.3794324112618696);
      tau_rk.set(0.0);
      tau_joint_limit_rk.set(0.0);
      q_ra.set(0.19325405584118394);
      qd_ra.set(0.6843781575753498);
      qdd_ra.set(-3.807468721908601);
      tau_ra.set(0.0);
      q_lh.set(0.4437190247188682);
      qd_lh.set(0.7274224893635938);
      qdd_lh.set(-0.6970637226618527);
      tau_lh.set(0.0);
      q_lk.set(-0.10164281312506038);
      qd_lk.set(-2.0991552566519283);
      qdd_lk.set(1.8896227665602998);
      tau_lk.set(0.0);
//      tau_lim_lk.set(0.0);
      q_la.set(0.032132090392145575);
      qd_la.set(-1.940528096295951);
      qdd_la.set(-8.770666789833381);
      tau_la.set(0.0);

      gc_rheel_x.set(-6.462486952576842);
      gc_rheel_y.set(-0.12);
      gc_rheel_z.set(-0.004448801109866819);
      gc_rheel_dx.set(-0.1433736927180039);
      gc_rheel_dy.set(0.0);
      gc_rheel_dz.set(0.004763952120601313);
      gc_rheel_fx.set(0.0);
      gc_rheel_fy.set(0.0);
      gc_rheel_fz.set(0.0);
      gc_rheel_px.set(0.0);
      gc_rheel_py.set(0.0);
      gc_rheel_pz.set(0.0);
      gc_rheel_tdx.set(0.0);
      gc_rheel_tdy.set(0.0);
      gc_rheel_tdz.set(0.0);
      gc_rheel_fs.set(0.0);
      gc_rheel_slip.set(false);
      gc_rtoe_x.set(-6.692486634223784);
      gc_rtoe_y.set(-0.12);
      gc_rtoe_z.set(-0.00483147891823249);
      gc_rtoe_dx.set(-0.14333578147742276);
      gc_rtoe_dy.set(0.0);
      gc_rtoe_dz.set(-0.018025722946184475);
      gc_rtoe_fx.set(0.0);
      gc_rtoe_fy.set(0.0);
      gc_rtoe_fz.set(0.0);
      gc_rtoe_px.set(0.0);
      gc_rtoe_py.set(0.0);
      gc_rtoe_pz.set(0.0);
      gc_rtoe_tdx.set(0.0);
      gc_rtoe_tdy.set(0.0);
      gc_rtoe_tdz.set(0.0);
      gc_rtoe_fs.set(0.0);
      gc_rtoe_slip.set(false);
      gc_lheel_x.set(-6.978304151952998);
      gc_lheel_y.set(0.12);
      gc_lheel_z.set(0.030917312707976694);
      gc_lheel_dx.set(-0.5871172504190455);
      gc_lheel_dy.set(0.0);
      gc_lheel_dz.set(0.01549948743431992);
      gc_lheel_fx.set(0.0);
      gc_lheel_fy.set(0.0);
      gc_lheel_fz.set(0.0);
      gc_lheel_px.set(0.0);
      gc_lheel_py.set(0.0);
      gc_lheel_pz.set(0.0);
      gc_lheel_tdx.set(0.0);
      gc_lheel_tdy.set(0.0);
      gc_lheel_tdz.set(0.0);
      gc_lheel_fs.set(0.0);
      gc_lheel_slip.set(false);
      gc_ltoe_x.set(-7.191018486491939);
      gc_ltoe_y.set(0.12);
      gc_ltoe_z.set(0.11839652099003362);
      gc_ltoe_dx.set(-0.8511831099789776);
      gc_ltoe_dy.set(0.0);
      gc_ltoe_dz.set(-0.6266016695928729);
      gc_ltoe_fx.set(0.0);
      gc_ltoe_fy.set(0.0);
      gc_ltoe_fz.set(0.0);
      gc_ltoe_px.set(0.0);
      gc_ltoe_py.set(0.0);
      gc_ltoe_pz.set(0.0);
      gc_ltoe_tdx.set(0.0);
      gc_ltoe_tdy.set(0.0);
      gc_ltoe_tdz.set(0.0);
      gc_ltoe_fs.set(0.0);
      gc_ltoe_slip.set(false);

   }
   
   public int getInputVectorLength()
   {
	   return 6;
   }
   
   public void setInputVector(double[] u)
   {
	   if (u.length!=6) throw new java.lang.RuntimeException("u is the wrong size");
	   tau_rh.set(u[0]);
	   tau_rk.set(u[1]);
	   tau_ra.set(u[2]);
	   tau_lh.set(u[3]);
	   tau_lk.set(u[4]);
	   tau_la.set(u[5]);
   }
   
   //////////////////////////////////////
   
   public double getBodyVelocityX()
   {
      double vel = this.qd_x.getDoubleValue();
      return vel;
   }
   
   public double getBodyPositionX()
   {
      double vel = this.q_x.getDoubleValue();
      return vel;
   }
}
