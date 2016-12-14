package us.ihmc.exampleSimulations.skippy;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.graphics3DDescription.Graphics3DObject;
import us.ihmc.graphics3DDescription.appearance.AppearanceDefinition;
import us.ihmc.graphics3DDescription.appearance.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.geometry.RotationalInertiaCalculator;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;

public class SkippyRobotV2 extends Robot
{
   private static final boolean SHOW_MASS_ELIPSOIDS = false;
   private static final boolean SHOW_COORDINATE_SYSTEMS = false;

   private static final AppearanceDefinition SHOULDER_COLOR = YoAppearance.Red();
   private static final AppearanceDefinition TORSO_COLOR = YoAppearance.Blue();
   private static final AppearanceDefinition LEG_COLOR = YoAppearance.Blue();
   private static final AppearanceDefinition JOINT_COLOR = YoAppearance.LightSteelBlue();
   private static final double JOINT_RADIUS = 0.1;

   public static final double LEG_LENGTH = 1.0;
   public static final double LEG_MASS = 1.5;
   public static final double LEG_RADIUS = 0.05;

   public static final double TORSO_LENGTH = 2.0;
   public static final double TORSO_MASS = 1.0;
   public static final double TORSO_RADIUS = 0.05;

   public static final double SHOULDER_LENGTH = 3.0;
   public static final double SHOULDER_MASS = 0.5;
   public static final double SHOULDER_RADIUS = 0.05;

   private final GroundContactPoint footContactPoint;


   public SkippyRobotV2()
   {
      super("SkippyV2");

      // --- scs robot ---
      FloatingJoint rootJoint = new FloatingJoint("rootJoint", new Vector3d(), this);
      rootJoint.setLink(createTorsoSkippy());
      rootJoint.setPosition(0.0, 0.0, LEG_LENGTH + TORSO_LENGTH / 2.0);
      this.addRootJoint(rootJoint);
      PinJoint shoulderJoint = new PinJoint("shoulderJoint", new Vector3d(0.0, 0.0, TORSO_LENGTH / 2.0), this, Axis.Y);
      shoulderJoint.setLink(createArm());
      rootJoint.addJoint(shoulderJoint);
      PinJoint hipJoint = new PinJoint("hip", new Vector3d(0.0, 0.0, -TORSO_LENGTH / 2.0), this, Axis.X);
      hipJoint.setLink(createLeg());
      rootJoint.addJoint(hipJoint);

      // add ground contact points
      footContactPoint = new GroundContactPoint("gc_foot", new Vector3d(0.0, 0.0, -LEG_LENGTH), this);
      hipJoint.addGroundContactPoint(footContactPoint);
      GroundContactPoint hipContactPoint = new GroundContactPoint("gc_hip", new Vector3d(0.0, 0.0, 0.0), this);
      hipJoint.addGroundContactPoint(hipContactPoint);
      GroundContactPoint arm1ContactPoint = new GroundContactPoint("gc_arm1", new Vector3d(SHOULDER_LENGTH / 2.0, 0.0, 0.0), this);
      shoulderJoint.addGroundContactPoint(arm1ContactPoint);
      GroundContactPoint arm2ContactPoint = new GroundContactPoint("gc_arm2", new Vector3d(-SHOULDER_LENGTH / 2.0, 0.0, 0.0), this);
      shoulderJoint.addGroundContactPoint(arm2ContactPoint);

      // add ground contact model
      LinearGroundContactModel ground = new LinearGroundContactModel(this, this.getRobotsYoVariableRegistry());
      ground.setZStiffness(2000.0);
      ground.setZDamping(1500.0);
      ground.setXYStiffness(50000.0);
      ground.setXYDamping(2000.0);
      ground.setGroundProfile3D(new FlatGroundProfile());
      this.setGroundContactModel(ground);

      // add an external force point to easily push the robot in simulation
      ExternalForcePoint forcePoint = new ExternalForcePoint("forcePoint", new Vector3d(0.0, 0.0, TORSO_LENGTH / 2.0), this);
      rootJoint.addExternalForcePoint(forcePoint);
   }

   private Link createTorsoSkippy()
   {
      Link torso = new Link("torso");
      torso.setMass(TORSO_MASS);
      torso.setComOffset(0.0, 0.0, 0.0);
      torso.setMomentOfInertia(RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidCylinder(TORSO_MASS, TORSO_RADIUS, TORSO_LENGTH, Axis.Z));

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(0.0, 0.0, -TORSO_LENGTH / 2.0);
      linkGraphics.addCylinder(TORSO_LENGTH, TORSO_RADIUS, TORSO_COLOR);
      torso.setLinkGraphics(linkGraphics);

      if (SHOW_MASS_ELIPSOIDS)
         torso.addEllipsoidFromMassProperties();
      if (SHOW_COORDINATE_SYSTEMS)
         torso.addCoordinateSystemToCOM(0.3);

      return torso;
   }

   private Link createArm()
   {
      Link arms = new Link("arms");
      arms.setMass(SHOULDER_MASS);
      arms.setComOffset(0.0, 0.0, 0.0);
      arms.setMomentOfInertia(RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidCylinder(SHOULDER_MASS, SHOULDER_RADIUS, SHOULDER_LENGTH, Axis.X));

      // arm graphics:
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.rotate(Math.toRadians(90), Axis.Y);
      linkGraphics.translate(0.0, 0.0, -SHOULDER_LENGTH / 2.0);
      linkGraphics.addCylinder(SHOULDER_LENGTH, SHOULDER_RADIUS, SHOULDER_COLOR);
      // joint graphics:
      linkGraphics.rotate(Math.PI / 2.0, Axis.Y);
      linkGraphics.rotate(Math.PI / 2.0, Axis.X);
      linkGraphics.translate(-SHOULDER_LENGTH / 2.0, 0.0, -JOINT_RADIUS);
      linkGraphics.addCylinder(2.0 * JOINT_RADIUS, 2.0 * JOINT_RADIUS / 3.0, JOINT_COLOR);
      arms.setLinkGraphics(linkGraphics);

      if (SHOW_MASS_ELIPSOIDS)
         arms.addEllipsoidFromMassProperties();
      if (SHOW_COORDINATE_SYSTEMS)
         arms.addCoordinateSystemToCOM(0.3);

      return arms;
   }

   private Link createLeg()
   {
      Link leg = new Link("leg");
      leg.setMass(LEG_MASS);
      leg.setComOffset(0.0, 0.0, -LEG_LENGTH / 2.0);
      leg.setMomentOfInertia(RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidCylinder(LEG_MASS, LEG_RADIUS, LEG_LENGTH, Axis.Z));

      // leg graphics:
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(0.0, 0.0, -LEG_LENGTH);
      linkGraphics.addCylinder(LEG_LENGTH, LEG_RADIUS, LEG_COLOR);
      // joint graphics:
      linkGraphics.identity();
      linkGraphics.rotate(Math.PI / 2.0, Axis.Y);
      linkGraphics.translate(0.0, 0.0, -JOINT_RADIUS);
      linkGraphics.addCylinder(2.0 * JOINT_RADIUS, 2.0 * JOINT_RADIUS / 3.0, JOINT_COLOR);
      leg.setLinkGraphics(linkGraphics);

      if (SHOW_MASS_ELIPSOIDS)
         leg.addEllipsoidFromMassProperties();
      if (SHOW_COORDINATE_SYSTEMS)
         leg.addCoordinateSystemToCOM(0.3);

      return leg;
   }

   public Point3d getFootLocation()
   {
      return footContactPoint.getPositionPoint();
   }

}
