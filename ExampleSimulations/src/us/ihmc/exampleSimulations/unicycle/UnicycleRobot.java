package us.ihmc.exampleSimulations.unicycle;

import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.simulationconstructionset.FloatingPlanarJoint;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;

public class UnicycleRobot extends Robot
{
   // upper body dimensions
   private static final double body_x = 0.25;
   private static final double body_y = 0.25;
   private static final double body_z = 0.5;
   private static final double body_mass = 5.0;

   // fork dimensions
   private static final double fork_l = 0.3;
   private static final double fork_r = 0.01;
   private static final double fork_mass = 0.0;

   // wheel dimensions
   private static final double wheel_r = 0.2;
   private static final double wheel_d = 0.03;
   private static final double wheel_mass = 2.0;
   private static final int contactPoints = 50;

   public UnicycleRobot()
   {
      super("Unicycle");
      this.setGravity(0.0, 0.0, -9.81);

      FloatingPlanarJoint rootJoint = new FloatingPlanarJoint("floatingRootJoint", this);
      Link upperBody = createUpperBody();
      rootJoint.setLink(upperBody);
      rootJoint.setCartesianPosition(0.0, fork_l + wheel_r);
      rootJoint.setRotation(0.3);

      rootJoint.addGroundContactPoint(new GroundContactPoint("gcp_body_1", new Vector3d(-body_x/2.0, body_y/2.0, body_z), this));
      rootJoint.addGroundContactPoint(new GroundContactPoint("gcp_body_2", new Vector3d(body_x/2.0, body_y/2.0, body_z), this));
      rootJoint.addGroundContactPoint(new GroundContactPoint("gcp_body_3", new Vector3d(body_x/2.0, -body_y/2.0, body_z), this));
      rootJoint.addGroundContactPoint(new GroundContactPoint("gcp_body_4", new Vector3d(-body_x/2.0, -body_y/2.0, body_z), this));
      rootJoint.addGroundContactPoint(new GroundContactPoint("gcp_body_5", new Vector3d(-body_x/2.0, body_y/2.0, 0.0), this));
      rootJoint.addGroundContactPoint(new GroundContactPoint("gcp_body_6", new Vector3d(body_x/2.0, body_y/2.0, 0.0), this));
      rootJoint.addGroundContactPoint(new GroundContactPoint("gcp_body_7", new Vector3d(body_x/2.0, -body_y/2.0, 0.0), this));
      rootJoint.addGroundContactPoint(new GroundContactPoint("gcp_body_8", new Vector3d(-body_x/2.0, -body_y/2.0, 0.0), this));
      this.addRootJoint(rootJoint);

      PinJoint backJoint = new PinJoint("backJoint", new Vector3d(0.0, 0.0, 0.0), this, Axis.Y);
      Link fork = createFork();
      backJoint.setLink(fork);
      rootJoint.addJoint(backJoint);

      PinJoint wheelJoint = new PinJoint("wheelJoint", new Vector3d(0.0, 0.0, -fork_l), this, Axis.Y);
      Link wheel = createWheel();
      wheelJoint.setLink(wheel);
      for (int i = 0; i < contactPoints; i++)
      {
         double angle = 2.0 * Math.PI * (double) i / (double) contactPoints;
         Vector3d offset = new Vector3d(wheel_r * Math.sin(angle), 0.0, wheel_r * Math.cos(angle));
         wheelJoint.addGroundContactPoint(new GroundContactPoint("gcp_wheel_" + i, offset, this));
      }
      backJoint.addJoint(wheelJoint);

      GroundContactModel groundContactModel = new LinearGroundContactModel(this, this.getRobotsYoVariableRegistry());
      this.setGroundContactModel(groundContactModel);

      showCoordinatesRecursively(rootJoint, false);
   }

   private Link createWheel()
   {
      Link link = new Link("Wheel");
      link.setMass(wheel_mass);
      link.setComOffset(new Vector3d(0.0, 0.0, 0.0));
      double ixx = wheel_mass/12.0 * (3.0 * wheel_r*wheel_r + wheel_d*wheel_d);
      double iyy = wheel_mass/2.0 * wheel_r*wheel_r;
      double izz = ixx;
      link.setMomentOfInertia(ixx, iyy, izz);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(0.0, -wheel_d/2.0, 0.0);
      linkGraphics.rotate(Math.PI/2.0, Axis.X);
      linkGraphics.addCylinder(-wheel_d, wheel_r, YoAppearance.Red());
      link.setLinkGraphics(linkGraphics);

      return link;
   }

   private Link createFork()
   {
      Link link = new Link("Fork");
      link.setMass(fork_mass);
      link.setComOffset(new Vector3d(0.0, 0.0, -fork_l/2.0));
      double ixx = fork_mass/12.0 * (3.0 * fork_r*fork_r + fork_l*fork_l); // + fork_mass*fork_l*fork_l/4.0;
      double iyy = ixx;
      double izz = fork_mass/2.0 * fork_r*fork_r;
      link.setMomentOfInertia(ixx, iyy, izz);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCylinder(-fork_l, fork_r, YoAppearance.Red());
      link.setLinkGraphics(linkGraphics);

      return link;
   }

   private Link createUpperBody()
   {
      Link link = new Link("upperBody");
      link.setMass(body_mass);

      link.setComOffset(new Vector3d(0.0, 0.0, body_z/2.0));
      double ixx = body_mass/12.0 * (body_y*body_y + body_z*body_z); // + body_mass*body_z*body_z/4.0;
      double iyy = body_mass/12.0 * (body_x*body_x + body_z*body_z); // + body_mass*body_z*body_z/4.0;
      double izz = body_mass/12.0 * (body_x*body_x + body_y*body_y);
      link.setMomentOfInertia(ixx, iyy, izz);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCube(body_x, body_y, body_z, YoAppearance.Red());
      link.setLinkGraphics(linkGraphics);

      return link;
   }

   private void showCoordinatesRecursively(Joint joint, boolean drawEllipoids)
   {
      Graphics3DObject linkGraphics = joint.getLink().getLinkGraphics();
      linkGraphics.identity();
      linkGraphics.addCoordinateSystem(0.3);
      if (drawEllipoids)
      {
         joint.getLink().addEllipsoidFromMassProperties();
      }
      for (Joint child : joint.getChildrenJoints())
      {
         showCoordinatesRecursively(child, drawEllipoids);
      }
   }

   public double getWheelRadius()
   {
      return wheel_r;
   }
}
