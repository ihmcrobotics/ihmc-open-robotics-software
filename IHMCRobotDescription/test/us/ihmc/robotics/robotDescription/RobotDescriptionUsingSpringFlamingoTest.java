package us.ihmc.robotics.robotDescription;

import java.util.ArrayList;

import javax.vecmath.Vector3d;

import org.junit.Test;

import us.ihmc.continuousIntegration.ContinuousIntegrationAnnotations;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.Plane;
import us.ihmc.robotics.geometry.RigidBodyTransform;

public class RobotDescriptionUsingSpringFlamingoTest
{
   private static final boolean SHOW_CARTOON_GRAPHICS = true;
   private static final boolean SHOW_MASS_PROPERTIES_GRAPHICS = false;

   private static final double UPPER_LEG_MASS = 0.4598;
   private static final double UPPER_LEG_Ixx = 0.01256, UPPER_LEG_Iyy = 0.01256, UPPER_LEG_Izz = 0.00013;

   private static final double LOWER_LEG_MASS = 0.306;
   private static final double LOWER_LEG_Ixx = 0.00952, LOWER_LEG_Iyy = 0.00952, LOWER_LEG_Izz = 5.67e-5;

   private static final double FOOT_MASS = 0.3466;

   // private static final double FOOT_Ixx = 0.0015, FOOT_Iyy = 0.0015, FOOT_Izz = 7.15e-5;
   private static final double FOOT_Ixx = 7.15e-5, FOOT_Iyy = 0.0015, FOOT_Izz = 0.0015;

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
   private static final double FOOT_BEHIND = FOOT_X - FOOT_FORWARD;
   private static final double HIP_OFFSET_Y = 0.12;

   @ContinuousIntegrationAnnotations.ContinuousIntegrationTest(estimatedDuration = 300)
   @Test(timeout = 1000)
   public void testUsingSpringFlamingoRobotDescription()
   {
      FloatingPlanarJointDescription plane;
      PinJointDescription rightHip, rightKnee, rightAnkle, leftHip, leftKnee, leftAnkle;

      ArrayList<GroundContactPointDescription> gcPoints = new ArrayList<GroundContactPointDescription>(4);

      RobotDescription robotDescription = new RobotDescription("SpringFlamingo");

      plane = new FloatingPlanarJointDescription("plane", Plane.XZ);
      LinkDescription body = body();
      plane.setLink(body);
      robotDescription.addRootJoint(plane);

      RigidBodyTransform camRotation = new RigidBodyTransform();
      camRotation.setRotationYawAndZeroTranslation(Math.PI);

      CameraSensorDescription robotCam = new CameraSensorDescription("robot cam mount", camRotation);
      plane.addCameraSensor(robotCam);

      RigidBodyTransform imuTransform = new RigidBodyTransform();
      imuTransform.setTranslation(new Vector3d(0.0, 0.0, 0.2));
      IMUSensorDescription imuMount = new IMUSensorDescription("FlamingoIMU", imuTransform);
      plane.addIMUSensor(imuMount);

      /** ************************ Right limb ********************************** */

      rightHip = new PinJointDescription("rh", new Vector3d(0.0, -HIP_OFFSET_Y, 0.0), Axis.Y); // right hip joint
      LinkDescription r_upper_leg = upper_leg("r_upper_leg");
      rightHip.setLink(r_upper_leg);
      plane.addJoint(rightHip);

      JointWrenchSensorDescription rightHipWrenchSensor = new JointWrenchSensorDescription("rightHipWrenchSensor", new Vector3d());
      rightHip.addJointWrenchSensor(rightHipWrenchSensor);

      rightKnee = new PinJointDescription("rk", new Vector3d(0.0, 0.0, -UPPER_LINK_LENGTH), Axis.Y); // right knee joint
      LinkDescription r_lower_leg = lower_leg("r_lower_leg");
      rightKnee.setLink(r_lower_leg);
      rightHip.addJoint(rightKnee);
      rightKnee.setLimitStops(-Math.PI, 0.0, 1000.0, 40.0);

      JointWrenchSensorDescription rightKneeWrenchSensor = new JointWrenchSensorDescription("rightKneeWrenchSensor", new Vector3d());
      rightKnee.addJointWrenchSensor(rightKneeWrenchSensor);

      rightAnkle = new PinJointDescription("ra", new Vector3d(0.0, 0.0, -LOWER_LINK_LENGTH), Axis.Y); // right ankle joint
      LinkDescription r_foot = foot("r_foot");
      rightAnkle.setLink(r_foot);
      rightKnee.addJoint(rightAnkle);

      GroundContactPointDescription gc_rheel = new GroundContactPointDescription("gc_rheel", new Vector3d(FOOT_OFFSET_PERCENT * FOOT_X, 0.0, FOOT_ZMIN));
      GroundContactPointDescription gc_rtoe = new GroundContactPointDescription("gc_rtoe", new Vector3d(-(1.0 - FOOT_OFFSET_PERCENT) * FOOT_X, 0.0, FOOT_ZMIN));

      gcPoints.add(gc_rheel);
      gcPoints.add(gc_rtoe);

      rightAnkle.addGroundContactPoint(gc_rheel);
      rightAnkle.addGroundContactPoint(gc_rtoe);

      JointWrenchSensorDescription rightAnkleWrenchSensor = new JointWrenchSensorDescription("rightAnkleWrenchSensor", new Vector3d());
      rightAnkle.addJointWrenchSensor(rightAnkleWrenchSensor);

      /** ************************ Left limb ********************************** */

      leftHip = new PinJointDescription("lh", new Vector3d(0.0, HIP_OFFSET_Y, 0.0), Axis.Y); // left hip joint
      LinkDescription l_upper_leg = upper_leg("l_upper_leg");
      leftHip.setLink(l_upper_leg);
      plane.addJoint(leftHip);

      JointWrenchSensorDescription leftHipWrenchSensor = new JointWrenchSensorDescription("leftHipWrenchSensor", new Vector3d());
      leftHip.addJointWrenchSensor(leftHipWrenchSensor);

      leftKnee = new PinJointDescription("lk", new Vector3d(0.0, 0.0, -UPPER_LINK_LENGTH), Axis.Y); // left knee joint
      LinkDescription l_lower_leg = lower_leg("l_lower_leg");
      leftKnee.setLink(l_lower_leg);
      leftHip.addJoint(leftKnee);
      ((PinJointDescription) leftKnee).setLimitStops(-Math.PI, 0.0, 1000.0, 40.0);

      JointWrenchSensorDescription leftKneeWrenchSensor = new JointWrenchSensorDescription("leftKneeWrenchSensor", new Vector3d());
      leftKnee.addJointWrenchSensor(leftKneeWrenchSensor);

      leftAnkle = new PinJointDescription("la", new Vector3d(0.0, 0.0, -LOWER_LINK_LENGTH), Axis.Y); // left ankle joint
      LinkDescription l_foot = foot("l_foot");
      leftAnkle.setLink(l_foot);
      leftKnee.addJoint(leftAnkle);

      GroundContactPointDescription gc_lheel = new GroundContactPointDescription("gc_lheel", new Vector3d(FOOT_OFFSET_PERCENT * FOOT_X, 0.0, FOOT_ZMIN));
      GroundContactPointDescription gc_ltoe = new GroundContactPointDescription("gc_ltoe", new Vector3d(-(1.0 - FOOT_OFFSET_PERCENT) * FOOT_X, 0.0, FOOT_ZMIN));

      gcPoints.add(gc_lheel);
      gcPoints.add(gc_ltoe);

      leftAnkle.addGroundContactPoint(gc_lheel);
      leftAnkle.addGroundContactPoint(gc_ltoe);

      JointWrenchSensorDescription leftAnkleWrenchSensor = new JointWrenchSensorDescription("leftAnkleWrenchSensor", new Vector3d());
      leftAnkle.addJointWrenchSensor(leftAnkleWrenchSensor);
   }

   private LinkDescription body()
   {
      LinkDescription ret = new LinkDescription("body");

      ret.setMass(12.0);
      ret.setMomentOfInertia(0.10, 0.10, 0.10);

      ret.setCenterOfMassOffset(0.0, 0.0, BODY_CG_Z);

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

   private LinkDescription upper_leg(String name)
   {
      LinkDescription ret = new LinkDescription(name);

      AppearanceDefinition pulleyAppearance = YoAppearance.Red();

      ret.setMass(UPPER_LEG_MASS);
      ret.setMomentOfInertia(UPPER_LEG_Ixx, UPPER_LEG_Iyy, UPPER_LEG_Izz);
      ret.setCenterOfMassOffset(0.0, 0.0, -0.3029);

      LinkGraphicsDescription linkGraphics = new LinkGraphicsDescription();
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
         linkGraphics.addCylinder(0.01, 0.033, pulleyAppearance); /* Pulley */

         linkGraphics.identity();
         linkGraphics.translate(0.0, 0.0, -UPPER_LINK_LENGTH);
         linkGraphics.rotate(Math.PI / 2.0, Axis.X);
         linkGraphics.translate(0.0, 0.0, -0.013);
         linkGraphics.addCylinder(0.025, 0.015, pulleyAppearance); /* Pulley */
      }

      ret.setLinkGraphics(linkGraphics);

      if (SHOW_MASS_PROPERTIES_GRAPHICS)
      {
         ret.addEllipsoidFromMassProperties(YoAppearance.Fuchsia());
         ret.addCoordinateSystemToCOM(0.2);
      }

      return ret;
   }

   private LinkDescription lower_leg(String name)
   {
      LinkDescription ret = new LinkDescription(name);

      ret.setMass(LOWER_LEG_MASS);
      ret.setMomentOfInertia(LOWER_LEG_Ixx, LOWER_LEG_Iyy, LOWER_LEG_Izz);
      ret.setCenterOfMassOffset(0.0, 0.0, -0.1818);

      if (SHOW_CARTOON_GRAPHICS)
      {
         LinkGraphicsDescription linkGraphics = new LinkGraphicsDescription();
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

   private LinkDescription foot(String name)
   {
      LinkDescription ret = new LinkDescription(name);

      ret.setMass(FOOT_MASS);
      ret.setMomentOfInertia(FOOT_Ixx, FOOT_Iyy, FOOT_Izz);
      ret.setCenterOfMassOffset(-0.0458, 0.0, -0.0309);

      if (SHOW_CARTOON_GRAPHICS)
      {
         LinkGraphicsDescription linkGraphics = new LinkGraphicsDescription();
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

}
