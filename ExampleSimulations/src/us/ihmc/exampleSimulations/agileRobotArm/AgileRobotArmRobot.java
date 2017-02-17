package us.ihmc.exampleSimulations.agileRobotArm;

import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;


public class AgileRobotArmRobot extends Robot
{
   private static final long serialVersionUID = 7530820318717789935L;
   public static final double POUNDS = 1.0 / 2.2;    // Pound to Kg conversion.
   public static final double INCHES = 0.0254;    // Inch to Meter Conversion.

   public static final double BASE_HEIGHT = 1.0;
   public static final double BASE_RAD = 0.1;

   public static final double SHOULDER_DIFFERENTIAL_HEIGHT = 0.05;
   public static final double SHOULDER_DIFFERENTIAL_WIDTH = 0.075;

   public static final double UPPER_ARM_LENGTH = 23.29 * INCHES;
   public static final double UPPER_ARM_RAD = 0.05;
   public static final double UPPER_ARM_MASS = 6.7 * POUNDS;

   public static final double FOREARM_LENGTH = 9.1 * INCHES;
   public static final double FOREARM_RAD = 0.03;
   public static final double FOREARM_MASS = 1.0 * POUNDS;

   public static final double WRIST_DIFFERENTIAL_HEIGHT = 0.025;
   public static final double WRIST_DIFFERENTIAL_WIDTH = 0.0375;

   public static final double GRIPPER_LENGTH = 0.08;
   public static final double GRIPPER_COM_OFFSET = 3.0 * INCHES;
   public static final double GRIPPER_RAD = 0.05;
   public static final double GRIPPER_MASS = 3.0 * POUNDS;


   public AgileRobotArmRobot()
   {
      super("AgileRobotArm");

      /** ************************ Base ********************************** */

      Link baseLink = base();
      this.addStaticLink(baseLink);

      /** ************************ Shoulder ********************************** */

      PinJoint shoulder_yaw = new PinJoint("shoulder_yaw", new Vector3D(0.0, 0.0, BASE_HEIGHT), this, Axis.X);
      Link shoulder_differential = shoulder_differential();
      shoulder_yaw.setLink(shoulder_differential);
      this.addRootJoint(shoulder_yaw);

      PinJoint shoulder_pitch = new PinJoint("shoulder_pitch", new Vector3D(0.0, 0.0, SHOULDER_DIFFERENTIAL_HEIGHT), this, Axis.Y);
      Link upper_arm = upper_arm();
      shoulder_pitch.setLink(upper_arm);
      shoulder_yaw.addJoint(shoulder_pitch);


      /** ************************ Elbow ********************************** */

      PinJoint elbow_pitch = new PinJoint("elbow_pitch", new Vector3D(0.0, 0.0, UPPER_ARM_LENGTH), this, Axis.Y);
      Link forearm = forearm();
      elbow_pitch.setLink(forearm);
      shoulder_pitch.addJoint(elbow_pitch);

      /** ************************ Wrist ********************************** */

      PinJoint wrist_pitch = new PinJoint("wrist_pitch", new Vector3D(0.0, 0.0, FOREARM_LENGTH), this, Axis.Y);
      Link wrist_differential = wrist_differential();
      wrist_pitch.setLink(wrist_differential);
      elbow_pitch.addJoint(wrist_pitch);

      PinJoint wrist_yaw = new PinJoint("wrist_yaw", new Vector3D(0.0, 0.0, 0.0), this, Axis.X);
      Link wrist_differential2 = wrist_differential();
      wrist_yaw.setLink(wrist_differential2);
      wrist_pitch.addJoint(wrist_yaw);

      PinJoint wrist_roll = new PinJoint("wrist_roll", new Vector3D(0.0, 0.0, WRIST_DIFFERENTIAL_HEIGHT), this, Axis.Z);
      Link gripper = gripper();
      wrist_roll.setLink(gripper);
      wrist_yaw.addJoint(wrist_roll);

   }


   private Link base()
   {
      AppearanceDefinition baseAppearance = YoAppearance.Blue();

      Link ret = new Link("base");

      ret.setMass(100.0);
      ret.setMomentOfInertia(1.0, 1.0, 1.0);
      ret.setComOffset(0.0, 0.0, 0.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();

      linkGraphics.addCoordinateSystem(1.0);
      linkGraphics.addCylinder(BASE_HEIGHT, BASE_RAD, baseAppearance);
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private Link shoulder_differential()
   {
      Link ret = new Link("shoulder_differential");

      ret.setMass(0.1);
      ret.setMomentOfInertia(0.005, 0.005, 0.005);
      ret.setComOffset(0.0, 0.0, SHOULDER_DIFFERENTIAL_HEIGHT / 2.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();

      linkGraphics.addCube(SHOULDER_DIFFERENTIAL_WIDTH, SHOULDER_DIFFERENTIAL_WIDTH, SHOULDER_DIFFERENTIAL_HEIGHT);
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }


   private Link upper_arm()
   {
      AppearanceDefinition upperArmApp = YoAppearance.Green();

      Link ret = new Link("upper_arm");

      ret.setMass(UPPER_ARM_MASS);    // 2.35);
      ret.setMomentOfInertia(0.0437, 0.0437, 0.0054);
      ret.setComOffset(0.0, 0.0, UPPER_ARM_LENGTH / 2.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();

      linkGraphics.addCylinder(UPPER_ARM_LENGTH, UPPER_ARM_RAD, upperArmApp);
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }


   private Link forearm()
   {
      AppearanceDefinition forearmApp = YoAppearance.Red();

      Link ret = new Link("forearm");

      ret.setMass(FOREARM_MASS);    // 0.864);
      ret.setMomentOfInertia(0.00429, 0.00429, 0.00106);
      ret.setComOffset(0.0, 0.0, FOREARM_LENGTH / 2.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();

      linkGraphics.addCylinder(FOREARM_LENGTH, FOREARM_RAD, forearmApp);
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private Link wrist_differential()
   {
      Link ret = new Link("wrist_differential");

      ret.setMass(0.1);
      ret.setMomentOfInertia(0.005, 0.005, 0.005);
      ret.setComOffset(0.0, 0.0, WRIST_DIFFERENTIAL_HEIGHT / 2.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();

      linkGraphics.addCube(WRIST_DIFFERENTIAL_WIDTH, WRIST_DIFFERENTIAL_WIDTH, WRIST_DIFFERENTIAL_HEIGHT);
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private Link gripper()
   {
      AppearanceDefinition gripperApp = YoAppearance.PlaneMaterial();

      Link ret = new Link("gripper");

      ret.setMass(GRIPPER_MASS);    // 0.207);
      ret.setMomentOfInertia(0.00041, 0.00041, 0.00001689);
      ret.setComOffset(0.0, 0.0, GRIPPER_COM_OFFSET);

      Graphics3DObject linkGraphics = new Graphics3DObject();

      linkGraphics.addCylinder(GRIPPER_LENGTH, GRIPPER_RAD, gripperApp);

      linkGraphics.translate(0.05, 0.0, GRIPPER_LENGTH);
      linkGraphics.addCube(0.02, 0.1, 0.1, YoAppearance.Black());

      linkGraphics.translate(-0.1, 0.0, 0.0);
      linkGraphics.addCube(0.02, 0.1, 0.1, YoAppearance.Black());

      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

}
