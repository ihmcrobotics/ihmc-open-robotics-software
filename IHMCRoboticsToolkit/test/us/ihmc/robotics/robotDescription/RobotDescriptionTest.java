package us.ihmc.robotics.robotDescription;

import java.util.ArrayList;

import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.robotics.Axis;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.robotSide.RobotSide;

public class RobotDescriptionTest
{

   double UPPER_LEG_MASS = 0.4598;
   double
      UPPER_LEG_Ixx = 0.01256, UPPER_LEG_Iyy = 0.01256, UPPER_LEG_Izz = 0.00013;

   double LOWER_LEG_MASS = 0.306;
   double
      LOWER_LEG_Ixx = 0.00952, LOWER_LEG_Iyy = 0.00952, LOWER_LEG_Izz = 5.67e-5;

   double FOOT_MASS = 0.3466;

   // private static final double FOOT_Ixx = 0.0015, FOOT_Iyy = 0.0015, FOOT_Izz = 7.15e-5;
   double
      FOOT_Ixx = 7.15e-5, FOOT_Iyy = 0.0015, FOOT_Izz = 0.0015;

   double BODY_Z = .385;
   double BODY_Y = .12;
   double BODY_X = .05;
   double BODY_CG_Z = 0.20;
   double BODY_Z_LEGPLOT = 0.21;
   double BODY_Y_LEGPLOT = 0.363;
   double BODY_X_LEGPLOT = 0.45;
   double UPPER_LINK_LENGTH = 0.42;
   double UPPER_LEG_ZMAX = 0.0;
   double UPPER_LEG_ZMIN = -0.42;
   double UPPER_LEG_Y = 0.02175;
   double UPPER_LEG_X = 0.02175;
   double LOWER_LINK_LENGTH = 0.42;
   double LOWER_LEG_ZMAX = 0.0;
   double LOWER_LEG_ZMIN = -0.42;
   double LOWER_LEG_Y = 0.02175;
   double LOWER_LEG_X = 0.02175;
   double FOOT_ZMIN = -0.04;
   double FOOT_ZMAX = -0.01;
   double FOOT_Y = 0.04;
   double FOOT_X = 0.23;
   double FOOT_H = (0.04);
   double FOOT_OFFSET_PERCENT = 0.25;
   double FOOT_FORWARD = (FOOT_X * FOOT_OFFSET_PERCENT);
   double FOOT_BEHIND = FOOT_X - FOOT_FORWARD;
   double HIP_OFFSET_Y = 0.12;

   @Test
   public void testUsingSpringFlamingoRobotDescription()
   {
      JointDescription plane, rightHip, rightKnee, rightAnkle, leftHip, leftKnee, leftAnkle;

         ArrayList<GroundContactPointDescription> gcPoints = new ArrayList<GroundContactPointDescription>(4);

        RobotDescription robotDescription = new RobotDescription("SpringFlamingo");

            plane = new FloatingPlanarJointDescription("plane", this, FloatingPlanarJointDescription.XZ);
            LinkDesription body = body();
            plane.setLink(body);
            robotDescription.addRootJoint(plane);

            RigidBodyTransform camRotation = new RigidBodyTransform();
            camRotation.setRotationYawAndZeroTranslation(Math.PI);

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

            GroundContactPointDescription gc_rheel = new GroundContactPointDescription("gc_rheel", new Vector3d(FOOT_OFFSET_PERCENT * FOOT_X, 0.0, FOOT_ZMIN), this);
            GroundContactPointDescription gc_rtoe = new GroundContactPointDescription("gc_rtoe", new Vector3d(-(1.0 - FOOT_OFFSET_PERCENT) * FOOT_X, 0.0, FOOT_ZMIN), this);

            gcPoints.add(gc_rheel);
            gcPoints.add(gc_rtoe);

            rightAnkle.addGroundContactPoint(gc_rheel);
            rightAnkle.addGroundContactPoint(gc_rtoe);

            JointWrenchSensorDescription rightAnkleWrenchSensor = new JointWrenchSensorDescription("rightAnkleWrenchSensor",  new Vector3d(), this);
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
         }

         private LinkDescription body()
         {
            LinkDescription ret = new LinkDescription("body");

            ret.setMass(12.0);
            ret.setMomentOfInertia(0.10, 0.10, 0.10);

            ret.setComOffset(0.0, 0.0, BODY_CG_Z);

            // ret.setComOffset(0.0,0.0, 0.0);

            AppearanceDefinition bodyAppearance = YoAppearance.Red();

            LinkGraphicsDescription linkGraphics = new LinkGraphicsDescription();
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

            if (SHOW_MASS_PROPERTIES_GRAPHICS)
            {
              ret.addEllipsoidFromMassProperties(YoAppearance.DarkRed());
              ret.addCoordinateSystemToCOM(0.2);
            }

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

            if (SHOW_MASS_PROPERTIES_GRAPHICS)
            {
               ret.addEllipsoidFromMassProperties(YoAppearance.Fuchsia());
               ret.addCoordinateSystemToCOM(0.2);
            }

            return ret;
         }

         private Link lower_leg(String name)
         {
            Link ret = new Link(name);

            ret.setMass(LOWER_LEG_MASS);
            ret.setMomentOfInertia(LOWER_LEG_Ixx, LOWER_LEG_Iyy, LOWER_LEG_Izz);
            ret.setComOffset(0.0, 0.0, -0.1818);

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

            if (SHOW_MASS_PROPERTIES_GRAPHICS)
            {
               ret.addEllipsoidFromMassProperties(YoAppearance.Yellow());
               ret.addCoordinateSystemToCOM(0.2);
            }
            return ret;
         }

         private Link foot(String name)
         {
            Link ret = new Link(name);

            ret.setMass(FOOT_MASS);
            ret.setMomentOfInertia(FOOT_Ixx, FOOT_Iyy, FOOT_Izz);
            ret.setComOffset(-0.0458, 0.0, -0.0309);

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

            if (SHOW_MASS_PROPERTIES_GRAPHICS)
            {
               ret.addEllipsoidFromMassProperties(YoAppearance.Aqua());
               ret.addCoordinateSystemToCOM(0.2);
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
//            tau_lim_lk.set(0.0);
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

   }

}
