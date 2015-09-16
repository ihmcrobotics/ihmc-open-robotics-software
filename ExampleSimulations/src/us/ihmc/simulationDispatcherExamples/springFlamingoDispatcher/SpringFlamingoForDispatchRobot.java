package us.ihmc.simulationDispatcherExamples.springFlamingoDispatcher;

import java.util.ArrayList;

import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.simulationconstructionset.FloatingPlanarJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;

public class SpringFlamingoForDispatchRobot extends Robot    // implements java.io.Serializable
{
   private static final double UPPER_LEG_MASS = 0.4598;
   private static final double
      UPPER_LEG_Ixx = 0.01256, UPPER_LEG_Iyy = 0.01256, UPPER_LEG_Izz = 0.00013;

   private static final double LOWER_LEG_MASS = 0.306;
   private static final double
      LOWER_LEG_Ixx = 0.00952, LOWER_LEG_Iyy = 0.00952, LOWER_LEG_Izz = 5.67e-5;

   private static final double FOOT_MASS = 0.3466;
   private static final double
      FOOT_Ixx = 0.0015, FOOT_Iyy = 0.0015, FOOT_Izz = 7.15e-5;

   private static final double BODY_Z = .385;
   private static final double BODY_Y = .12;
   private static final double BODY_X = .05;
   private static final double BODY_CG_Z = 0.20;
   private static final double BODY_Z_LEGPLOT = 0.21;
   private static final double BODY_Y_LEGPLOT = 0.363;
   private static final double BODY_X_LEGPLOT = 0.45;
   private static final double UPPER_LINK_LENGTH = 0.42;
   private static final double UPPER_LEG_ZMAX = 0.0;
   private static final double UPPER_LEG_ZMIN = -0.42;
   private static final double UPPER_LEG_Y = 0.02175;
   private static final double UPPER_LEG_X = 0.02175;
   private static final double LOWER_LINK_LENGTH = 0.42;
   private static final double LOWER_LEG_ZMAX = 0.0;
   private static final double LOWER_LEG_ZMIN = -0.42;
   private static final double LOWER_LEG_Y = 0.02175;
   private static final double LOWER_LEG_X = 0.02175;
   private static final double FOOT_ZMIN = -0.04;
   private static final double FOOT_ZMAX = -0.01;
   private static final double FOOT_Y = 0.04;
   private static final double FOOT_X = 0.23;
   private static final double FOOT_H = (0.04);
   private static final double FOOT_OFFSET_PERCENT = 0.25;
   private static final double FOOT_FORWARD = (FOOT_X * FOOT_OFFSET_PERCENT);
   private static final double HIP_OFFSET_Y = 0.12;

   DoubleYoVariable gc_lheel_x, gc_lheel_z, gc_lheel_dx, gc_lheel_dz, gc_lheel_fs, gc_lheel_tdx, gc_lheel_tdz, gc_lheel_fx, gc_lheel_fz;
   DoubleYoVariable gc_ltoe_x, gc_ltoe_z, gc_ltoe_dx, gc_ltoe_dz, gc_ltoe_fs, gc_ltoe_tdx, gc_ltoe_tdz, gc_ltoe_fx, gc_ltoe_fz;
   DoubleYoVariable gc_rheel_x, gc_rheel_z, gc_rheel_dx, gc_rheel_dz, gc_rheel_fs, gc_rheel_tdx, gc_rheel_tdz, gc_rheel_fx, gc_rheel_fz;
   DoubleYoVariable gc_rtoe_x, gc_rtoe_z, gc_rtoe_dx, gc_rtoe_dz, gc_rtoe_fs, gc_rtoe_tdx, gc_rtoe_tdz, gc_rtoe_fx, gc_rtoe_fz;

   DoubleYoVariable gc_rheel_int_x, gc_rheel_ingt_z, gc_rtoe_int_x, gc_rtoe_int_z;

   FloatingPlanarJoint plane; 
   PinJoint rightHip, rightKnee, rightAnkle, leftHip, leftKnee, leftAnkle;

   ArrayList<GroundContactPoint> gcPoints = new ArrayList<GroundContactPoint>(4);

   public SpringFlamingoForDispatchRobot()
   {
      super("SpringFlamingo");
      this.setGravity(0.0, 0.0, -9.81);

      plane = new FloatingPlanarJoint("plane", this, FloatingPlanarJoint.XZ);
      Link body = body();
      plane.setLink(body);
      this.addRootJoint(plane);


      /** ************************ Right limb ********************************** */

      rightHip = new PinJoint("rh", new Vector3d(0.0, -HIP_OFFSET_Y, 0.0), this, Axis.Y);    // right hip joint
      Link r_upper_leg = upper_leg("r_upper_leg");
      rightHip.setLink(r_upper_leg);
      plane.addJoint(rightHip);

      rightKnee = new PinJoint("rk", new Vector3d(0.0, 0.0, -UPPER_LINK_LENGTH), this, Axis.Y);    // right knee joint
      Link r_lower_leg = lower_leg("r_lower_leg");
      rightKnee.setLink(r_lower_leg);
      rightHip.addJoint(rightKnee);
      ((PinJoint) rightKnee).setLimitStops(-Math.PI, 0.0, 1000.0, 40.0);

      rightAnkle = new PinJoint("ra", new Vector3d(0.0, 0.0, -LOWER_LINK_LENGTH), this, Axis.Y);    // right ankle joint
      Link r_foot = foot("r_foot");
      rightAnkle.setLink(r_foot);
      rightKnee.addJoint(rightAnkle);

      GroundContactPoint gc_rheel = new GroundContactPoint("gc_rheel", new Vector3d(FOOT_OFFSET_PERCENT * FOOT_X, 0.0, FOOT_ZMIN), this);
      GroundContactPoint gc_rtoe = new GroundContactPoint("gc_rtoe", new Vector3d(-(1.0 - FOOT_OFFSET_PERCENT) * FOOT_X, 0.0, FOOT_ZMIN), this);

      gcPoints.add(gc_rheel);
      gcPoints.add(gc_rtoe);

      rightAnkle.physics.addGroundContactPoint(gc_rheel);
      rightAnkle.physics.addGroundContactPoint(gc_rtoe);

      /** ************************ Left limb ********************************** */

      leftHip = new PinJoint("lh", new Vector3d(0.0, HIP_OFFSET_Y, 0.0), this, Axis.Y);    // left hip joint
      Link l_upper_leg = upper_leg("l_upper_leg");
      leftHip.setLink(l_upper_leg);
      plane.addJoint(leftHip);

      leftKnee = new PinJoint("lk", new Vector3d(0.0, 0.0, -UPPER_LINK_LENGTH), this, Axis.Y);    // left knee joint
      Link l_lower_leg = lower_leg("l_lower_leg");
      leftKnee.setLink(l_lower_leg);
      leftHip.addJoint(leftKnee);
      ((PinJoint) leftKnee).setLimitStops(-Math.PI, 0.0, 1000.0, 40.0);

      leftAnkle = new PinJoint("la", new Vector3d(0.0, 0.0, -LOWER_LINK_LENGTH), this, Axis.Y);    // left ankle joint
      Link l_foot = foot("l_foot");
      leftAnkle.setLink(l_foot);
      leftKnee.addJoint(leftAnkle);

      GroundContactPoint gc_lheel = new GroundContactPoint("gc_lheel", new Vector3d(FOOT_OFFSET_PERCENT * FOOT_X, 0.0, FOOT_ZMIN), this);
      GroundContactPoint gc_ltoe = new GroundContactPoint("gc_ltoe", new Vector3d(-(1.0 - FOOT_OFFSET_PERCENT) * FOOT_X, 0.0, FOOT_ZMIN), this);

      gcPoints.add(gc_lheel);
      gcPoints.add(gc_ltoe);

      leftAnkle.physics.addGroundContactPoint(gc_lheel);
      leftAnkle.physics.addGroundContactPoint(gc_ltoe);

   }

   private Link body()
   {
      Link ret = new Link("body");

      AppearanceDefinition bodyAppearance = YoAppearance.Red();

      ret.setMass(12.0);
      ret.setMomentOfInertia(0.10, 0.10, 0.10);

      ret.setComOffset(0.0, 0.0, BODY_CG_Z);

      // ret.setComOffset(0.0,0.0, 0.0);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      ret.setLinkGraphics(linkGraphics);
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

//    For right half
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

      return ret;
   }

   private Link upper_leg(String name)
   {
      Link ret = new Link(name);

      AppearanceDefinition pulleyAppearance = YoAppearance.Red();

      ret.setMass(UPPER_LEG_MASS);
      ret.setMomentOfInertia(UPPER_LEG_Ixx, UPPER_LEG_Iyy, UPPER_LEG_Izz);
      ret.setComOffset(0.0, 0.0, -0.3029);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      ret.setLinkGraphics(linkGraphics);
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

      return ret;

   }

   private Link lower_leg(String name)
   {
      Link ret = new Link(name);

      ret.setMass(LOWER_LEG_MASS);
      ret.setMomentOfInertia(LOWER_LEG_Ixx, LOWER_LEG_Iyy, LOWER_LEG_Izz);
      ret.setComOffset(0.0, 0.0, -0.1818);
      
      Graphics3DObject linkGraphics = new Graphics3DObject();
      ret.setLinkGraphics(linkGraphics);

      linkGraphics.translate(0.0, 0.0, -LOWER_LINK_LENGTH);
      linkGraphics.addCube(0.025, 0.04, 0.04, YoAppearance.AluminumMaterial());

      linkGraphics.translate(0.0, 0.0, 0.04);
      linkGraphics.addCylinder(0.308, 0.0113, YoAppearance.BlackMetalMaterial());

      linkGraphics.translate(0.0, 0.0, 0.308);
      linkGraphics.addCube(0.025, 0.04, 0.062, YoAppearance.AluminumMaterial());

      return ret;
   }

   private Link foot(String name)
   {
      Link ret = new Link(name);

      ret.setMass(FOOT_MASS);
      ret.setMomentOfInertia(FOOT_Ixx, FOOT_Iyy, FOOT_Izz);
      ret.setComOffset(-0.0458, 0.0, -0.0309);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      ret.setLinkGraphics(linkGraphics);
      
      linkGraphics.translate(FOOT_X * (FOOT_OFFSET_PERCENT - 0.5), 0.0, FOOT_ZMIN);
      linkGraphics.addCube(FOOT_X, FOOT_Y, FOOT_ZMAX - FOOT_ZMIN);

      linkGraphics.identity();
      linkGraphics.translate(0.0, 0.0, FOOT_ZMAX);
      linkGraphics.addCube(0.028, 0.028, -FOOT_ZMAX);

      return ret;
   }

}
