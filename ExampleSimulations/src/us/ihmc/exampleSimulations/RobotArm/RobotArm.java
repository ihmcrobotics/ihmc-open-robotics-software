package us.ihmc.exampleSimulations.RobotArm;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.simulationconstructionset.*;

import javax.vecmath.Vector3d;

public class RobotArm extends Robot
{
   public static final double
         angle1 = Math.toRadians(90),
         angle2 = Math.toRadians(115),
         angle3 = Math.toRadians(65),
         angle4 = Math.toRadians(90),
         angle5 = Math.toRadians(90),
         angle6 = Math.toRadians(90),
         angle7 = Math.toRadians(80);
//   // Lengths, masses, radii & moments of inertia (not used)
//   public static final double L1 = 1.0, L2 = 2.0, M1 = 1.0, M2 = 1.0, R1 = 0.1, R2 = 0.05, Iyy1 = 0.083, Iyy2 = 0.33;
//
//   // For multiple arms (not used)
//   private final SideDependentList<PinJoint> joints = new SideDependentList<>();

   RobotArm()
   {
      // Create an instance of 7Bot
       super("RobotArm");

      // Create damping to make the transitions smoother
      double damping = 100.0;

      // Create axis 1 & base of 7Bot
      PinJoint axis1 = new PinJoint("axis1", new Vector3d(0.0, 0.0, 0.0), this, Axis.Z);
      axis1.setInitialState(Math.cos(angle1), Math.sin(angle1));
      axis1.setDamping(damping);
      axis1.setLimitStops(-0.25,3.15,0,0);

      Link link1 = linkCylinder1();
      axis1.setLink(link1);

      // Create axis 2 & upper arm of 7Bot
      PinJoint axis2 = new PinJoint("axis2", new Vector3d(0.2, 0.0, 0.5), this, Axis.Y);
      axis2.setInitialState(Math.cos(angle2), Math.sin(angle2));
      axis2.setDamping(damping);
      axis2.setLimitStops(-1.15,2.35,0,0);

      Link link2 = stick2();
      axis2.setLink(link2);
      axis1.addJoint(axis2);

      // Create axes 3 & 4 & forearm of 7Bot
      UniversalJoint axes34 = new UniversalJoint("axis3", "axis4", new Vector3d(0.0, 0.0, 1.5), this, Axis.Y, Axis.X);
      axes34.getFirstJoint().setInitialState(Math.cos(angle3), Math.sin(angle3));
      axes34.getFirstJoint().setDamping(damping);
      axes34.getFirstJoint().setLimitStops(-1.3,2.0,0,0);

         // link3 is not instantiated because it is hidden in the UniversalJoint

      axes34.getSecondJoint().setInitialState(Math.cos(angle4), Math.sin(angle4));
      axes34.getSecondJoint().setDamping(damping);
      axes34.getSecondJoint().setLimitStops(-1.3,2.0,0,0);

      Link link4 = linkCylinder4();
      axes34.setLink(link4);
      axis2.addJoint(axes34);

      // Create axis 5
      PinJoint axis5 = new PinJoint("axis5", new Vector3d(0.0, 0.0, 0.0), this, Axis.Y);
      axis5.setInitialState(Math.cos(angle5), Math.sin(angle5));
      axis5.setDamping(damping);
      // axis5.setLimitStops(-1.3,2.0,0,0);

      Link link5 = stick5();
      axis5.setLink(link5);
      axes34.addJoint(axis5);

      // Create axis 6
      PinJoint axis6 = new PinJoint("axis6", new Vector3d(0.5, 0.0, 0.0), this, Axis.X);
      axis6.setInitialState(Math.cos(angle6), Math.sin(angle6));
      axis6.setDamping(damping);
      // axis6.setLimitStops(-1.7,2.1,0,0);

      Link link6 = linkCylinder6();
      axis6.setLink(link6);
      axis5.addJoint(axis6);

      // Create axis 7
      PinJoint axis7 = new PinJoint("axis7", new Vector3d(0.0, 0.0, 0.0), this, Axis.Z);
      axis7.setInitialState(Math.cos(angle7), Math.sin(angle7));
      axis7.setDamping(damping);
      // axis7.setLimitStops(-1.3,2.0,0,0);

      Link link7 = link7();
      axis7.setLink(link7);
      axis6.addJoint(axis7);

      this.addRootJoint(axis1);
      // showCoordinatesRecursively(circle, true);
   }

   private Link linkCylinder1()
   {
      Link ret = new Link("linkCylinder1");
      double mass = 100.0;
      double length = 0.6;
      double radius = 0.35;

      ret.setMass(mass);
      ret.setComOffset(0.0, 0.0, length / 2.0);
      ret.setMomentOfInertia(mass*(3*radius*radius + length*length)/12.0, mass*(3*radius*radius + length*length)/12.0, mass*radius*radius/2.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCylinder(length, radius, YoAppearance.Blue());

      ret.setLinkGraphics(linkGraphics);
      return ret;
   }

   private Link stick2()
   {
      Link ret = new Link("stick2");
      double mass = 150.0;
      double length = 1.5;
      double radius = 0.1;

      ret.setMass(mass);
      ret.setComOffset( 0.0, 0.0, length / 2.0);
      ret.setMomentOfInertia(mass*(3*radius*radius + length*length)/12.0, mass*(3*radius*radius + length*length)/12.0, mass*radius*radius/2.0);
      // create a LinkGraphics object to manipulate the visual representation of the link
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCylinder(length, radius, YoAppearance.Red());

      // associate the linkGraphics object with the link object
      ret.setLinkGraphics(linkGraphics);
      return ret;
   }

//   private Link stick3()
//   {
//      Link ret = new Link("stick3");
//
//      double mass = 150.0;
//      double length = 1.5;
//      double radius = 0.1;
//
//      ret.setMass(mass);
//      ret.setComOffset(length / 2.0, 0.0, 0.0);
//
//      ret.setMomentOfInertia(mass*radius*radius/2.0,mass*(3*radius*radius + length*length)/12.0, mass*(3*radius*radius + length*length)/12.0);
//      Graphics3DObject linkGraphics = new Graphics3DObject();
//      linkGraphics.rotate(Math.PI/2.0, Axis.Y);
//      linkGraphics.addCylinder(length, radius, YoAppearance.Green());
//      ret.setLinkGraphics(linkGraphics);
//      return ret;
//   }

   private Link linkCylinder4()
   {
      Link linkCircle2 = new Link("linkCylinder4");
      double mass = 1.0;
      double length = 0.2;
      double radius = 0.2;

      linkCircle2.setMass(mass);
      linkCircle2.setComOffset(0.0, 0.0, 0.0);
      linkCircle2.setMomentOfInertia(mass*radius*radius/2.0, mass*(3*radius*radius + length*length)/12.0, mass*(3*radius*radius + length*length)/12.0);
      Graphics3DObject linkGraphics = new Graphics3DObject();

      linkGraphics.rotate(Math.PI/2.0, Axis.Y);
      linkGraphics.translate(0.0, 0.0, -0.1);
      linkGraphics.addCylinder(length, radius, YoAppearance.White());
      linkCircle2.setLinkGraphics(linkGraphics);
      return linkCircle2;
   }

   private Link stick5()
   {
      Link ret = new Link("stick5");

      double mass = 150.0;
      double length = 0.5;
      double radius = 0.05;

      ret.setMass(mass);
      ret.setComOffset(length / 2.0, 0.0, 0.0);
      ret.setMomentOfInertia(mass*radius*radius/2.0,mass*(3*radius*radius + length*length)/12.0, mass*(3*radius*radius + length*length)/12.0);


      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.rotate(Math.PI/2.0, Axis.Y);
      linkGraphics.addCylinder(length, radius, YoAppearance.Red());

      ret.setLinkGraphics(linkGraphics);
      return ret;
   }

   private Link linkCylinder6()
   {
      double mass = 1.0;
      double length = 0.2;
      double radius = 0.2;

      Link linkCircle3 = new Link("linkCylinder6");
      linkCircle3.setMass(mass);
      linkCircle3.setComOffset(0.0, 0.0, 0.0);
      linkCircle3.setMomentOfInertia(mass*radius*radius/2.0, mass*(3*radius*radius + length*length)/12.0, mass*(3*radius*radius + length*length)/12.0);
      Graphics3DObject linkGraphics = new Graphics3DObject();

      linkGraphics.rotate(Math.PI/2.0, Axis.Y);
      linkGraphics.translate(0.0, 0.0, -0.1);
      linkGraphics.addCylinder(length, radius, YoAppearance.White());
      linkCircle3.setLinkGraphics(linkGraphics);
      return linkCircle3;
   }

   private Link link7()
   {
      Link ret = new Link("link7");

      double mass = 150.0;
      double length = 0.5;
      double radius = 0.05;

      ret.setMass(mass);
      ret.setComOffset(length / 2.0, 0.0, 0.0);
      ret.setMomentOfInertia(mass*radius*radius/2.0,mass*(3*radius*radius + length*length)/12.0, mass*(3*radius*radius + length*length)/12.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.rotate(Math.PI/2.0, Axis.Y);
      linkGraphics.addCylinder(length, radius, YoAppearance.Blue());
      ret.setLinkGraphics(linkGraphics);
      return ret;
   }

   private void showCoordinatesRecursively(Joint joint, boolean drawEllipsoids)
   {
      Graphics3DObject linkGraphics = joint.getLink().getLinkGraphics();
      linkGraphics.identity();
      linkGraphics.addCoordinateSystem(1.6);
      if (drawEllipsoids)
      {
         joint.getLink().addEllipsoidFromMassProperties();
      }
      for (Joint child : joint.getChildrenJoints())
      {
         showCoordinatesRecursively(child, drawEllipsoids);
      }
   }

}
