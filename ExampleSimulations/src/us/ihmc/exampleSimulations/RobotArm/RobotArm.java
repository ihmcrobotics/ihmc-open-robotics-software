package us.ihmc.exampleSimulations.RobotArm;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.simulationconstructionset.*;

import javax.vecmath.Vector3d;

public class RobotArm extends Robot
{
   public static final double
         L1 = 1.0, L2 = 2.0, M1 = 1.0, M2 = 1.0, R1 = 0.1, R2 = 0.05, Iyy1 = 0.083, Iyy2 = 0.33;
   private final SideDependentList<PinJoint> joints = new SideDependentList<>();

   RobotArm()
   {
      super("RobotArm"); // create an instance of Robot
      // Create joints and assign links. Pin joints have a single axis of rotation.

//      NullJoint rootJoint = new NullJoint("CeilingJoint", new Vector3d(), this);

//      Link ceiling = new Link("link1");
//      ceiling.setMass(100.0);
//      Graphics3DObject linkGraphics = new Graphics3DObject();
//      linkGraphics.addCube(1, 2, -0.5);

//      ceiling.setLinkGraphics(linkGraphics);
//      rootJoint.setLink(ceiling);

      double damping = 100.0;

      PinJoint circle = new PinJoint("circle", new Vector3d(0.0, 0.0, 0.0), this, Axis.Z);
      circle.setInitialState(1.55, 0.0);
      circle.setDamping(damping);
      circle.setLimitStops(-0.25,3.15,0,0);
      Link link1 = linkCircle();
      circle.setLink(link1); // a
//      rootJoint.addJoint(circle);
//////////////////////////////////////////////////////
      PinJoint stick1 = new PinJoint("stick1", new Vector3d(0.2, 0.0, 0.5), this, Axis.Y);
            stick1.setInitialState(1.5, 0.0);
            stick1.setDamping(damping);
            circle.setLimitStops(-1.15,2.35,0,0);
            Link link = link1();
            stick1.setLink(link); // associate link1 with the joint pin1
            circle.addJoint(stick1);

      PinJoint stick2 = new PinJoint("stick2", new Vector3d(0.0, 0.03, 1.5), this, Axis.Y);
            stick2.setInitialState(0.05, 0.0);

            stick2.setDamping(damping);
           circle.setLimitStops(-1.3,2.0,0,0);
            Link link3 = link2();
            stick2.setLink(link3); // associate link1 with the joint pin1
            stick1.addJoint(stick2);

      PinJoint circle2 = new PinJoint("circle2", new Vector3d(1.5, 0.0, 0.0), this, Axis.X);
      circle2.setDamping(damping);
            Link link2 = linkCircle2();
            circle2.setLink(link2);
            stick2.addJoint(circle2);

      PinJoint stick3 = new PinJoint("stick3", new Vector3d(0.2, 0.0, 0.0), this, Axis.Y);
             stick2.setInitialState(0.05, 0.0);
             stick2.setDamping(damping);
           // stick2.setLimitStops(-1.3,2.0,0,0);
            Link link4 = link3();
            stick3.setLink(link4); // associate link1 with the joint pin1
           circle2.addJoint(stick3);

      PinJoint circle3 = new PinJoint("circle3", new Vector3d(0.2, 0.0, 0.0), this, Axis.X);
            Link link5 = linkCircle3();
            //circle.setLimitStops(-1.7,2.1,0,0);
            circle3.setDamping(damping);
            circle3.setLink(link5);
            stick3.addJoint(circle3);

      PinJoint stick4 = new PinJoint("stick4", new Vector3d(0.0, 0.0, 0.0), this, Axis.Z);
            Link link6 = link4();
            stick4.setDamping(damping);
            stick4.setLink(link6);
            circle3.addJoint(stick4);




  //////////////////////////////////////////////////////

//
//      PinJoint stick1 = new PinJoint("stick1", new Vector3d(0.2, 0.0, 0.5), this, Axis.Y);
//      stick1.setInitialState(1.5, 0.0);
//      stick1.setDamping(damping);
//      circle.setLimitStops(-1.15,2.35,0,0);
//      Link link = link1();
//      stick1.setLink(link); // associate link1 with the joint pin1
//      circle.addJoint(stick1);
//      /*
//       *  The second joint is initiated with the offset vector (0.0,0.0,L1) since
//       *  it should be placed a distance of L1 in the Z direction from the previous joint.
//       */
//      PinJoint circle2 = new PinJoint("circle2", new Vector3d(0.0, 0.03, 1.5), this, Axis.X);
//      circle2.setDamping(damping);
//      Link link2 = linkCircle2();
//      circle2.setLink(link2);
//      stick1.addJoint(circle2);
////
//      PinJoint stick2 = new PinJoint("stick2", new Vector3d(0.0, 0.0, 0.0), this, Axis.Y);
//      stick2.setInitialState(0.05, 0.0);
//
//      stick2.setDamping(damping);
//     circle.setLimitStops(-1.3,2.0,0,0);
//      Link link3 = link2();
//      stick2.setLink(link3); // associate link1 with the joint pin1
//      circle2.addJoint(stick2);
//
//      PinJoint stick3 = new PinJoint("stick3", new Vector3d(1.5, 0.0, 0.0), this, Axis.Y);
//       stick2.setInitialState(0.05, 0.0);
//       stick2.setDamping(damping);
//     // stick2.setLimitStops(-1.3,2.0,0,0);
//      Link link4 = link3();
//      stick3.setLink(link4); // associate link1 with the joint pin1
//     stick2.addJoint(stick3);
//
//      PinJoint circle3 = new PinJoint("circle3", new Vector3d(0.2, 0.0, 0.0), this, Axis.X);
//      Link link5 = linkCircle3();
//      //circle.setLimitStops(-1.7,2.1,0,0);
//      circle3.setDamping(damping);
//      circle3.setLink(link5);
//      stick3.addJoint(circle3);
////
//      PinJoint stick4 = new PinJoint("stick4", new Vector3d(0.0, 0.0, 0.0), this, Axis.Z);
//      Link link6 = link4();
//      stick4.setDamping(damping);
//      stick4.setLink(link6);
//      circle3.addJoint(stick4);


      this.addRootJoint(circle);
      showCoordinatesRecursively(circle, true);
   }

   /**
    * Create the first link for the DoublePendulumRobot.
    */
   private Link link1()
   {
      Link ret = new Link("link1");
      double m = 150.0;
      double h = 1.5;
      double r = 0.1;

      ret.setMass(m);
      ret.setComOffset(0.0, 0.0, h / 2.0);
      ret.setMomentOfInertia(m*(3*r*r + h*h)/12.0, m*(3*r*r + h*h)/12.0, m*r*r/2.0);
      // create a LinkGraphics object to manipulate the visual representation of the link
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCylinder(h, r, YoAppearance.Red());

      // associate the linkGraphics object with the link object
      ret.setLinkGraphics(linkGraphics);
      return ret;
   }
   /**
    * Create the second link for the DoublePendulumRobot.
    */
   private Link linkCircle()
   {
      Link ret = new Link("linkCircle");
      double m = 100.0;
      double h = 0.6;
      double r = 0.35;

      ret.setMass(m);
      ret.setComOffset(0.0, 0.0, h / 2.0);
      ret.setMomentOfInertia(m*(3*r*r + h*h)/12.0, m*(3*r*r + h*h)/12.0, m*r*r/2.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCylinder(h, r, YoAppearance.Blue());

      ret.setLinkGraphics(linkGraphics);
      return ret;
   }
   private Link link3()
   {
      Link ret = new Link("link3");

      double m = 150.0;
      double h = 0.2;
      double r = 0.05;

      ret.setMass(m);
      ret.setComOffset(h / 2.0, 0.0, 0.0);
      ret.setMomentOfInertia(m*r*r/2.0,m*(3*r*r + h*h)/12.0, m*(3*r*r + h*h)/12.0);


      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.rotate(Math.PI/2.0, Axis.Y);

      linkGraphics.addCylinder(h, r, YoAppearance.Red());



      ret.setLinkGraphics(linkGraphics);
      return ret;
   }
   private Link link4()
   {
      Link ret = new Link("link4");

      double m = 150.0;
      double h = 0.5;
      double r = 0.05;

      ret.setMass(m);
      ret.setComOffset(h / 2.0, 0.0, 0.0);
      ret.setMomentOfInertia(m*r*r/2.0,m*(3*r*r + h*h)/12.0, m*(3*r*r + h*h)/12.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.rotate(Math.PI/2.0, Axis.Y);
      linkGraphics.addCylinder(h, r, YoAppearance.Blue());
      ret.setLinkGraphics(linkGraphics);
      return ret;
   }
   private Link link2()
   {
      Link ret = new Link("link2");

      double m = 150.0;
      double h = 1.5;
      double r = 0.1;

      ret.setMass(m);
      ret.setComOffset(h / 2.0, 0.0, 0.0);

      ret.setMomentOfInertia(m*r*r/2.0,m*(3*r*r + h*h)/12.0, m*(3*r*r + h*h)/12.0);
      Graphics3DObject linkGraphics = new Graphics3DObject();
       linkGraphics.rotate(Math.PI/2.0, Axis.Y);
      linkGraphics.addCylinder(h, r, YoAppearance.Green());
      ret.setLinkGraphics(linkGraphics);
      return ret;
   }
   private Link linkCircle2()
   {
      Link linkCircle2 = new Link("linkCircle2");
      double m = 1.0;
      double h = 0.2;
      double r = 0.2;

      linkCircle2.setMass(m);
      linkCircle2.setComOffset(0.0, 0.0, 0.0);
      linkCircle2.setMomentOfInertia(m*r*r/2.0, m*(3*r*r + h*h)/12.0, m*(3*r*r + h*h)/12.0);
      Graphics3DObject linkGraphics = new Graphics3DObject();

      linkGraphics.rotate(Math.PI/2.0, Axis.Y);
      linkGraphics.translate(0.0, 0.0, -0.1);
      linkGraphics.addCylinder(h, r, YoAppearance.White());
      linkCircle2.setLinkGraphics(linkGraphics);
      return linkCircle2;

//
//      double m = 1.0;
//      double h = 0.2;
//      double r = 0.2;
//
//
//
//      Link linkCircle2 = new Link("linkCircle3");
//      linkCircle2.setMass(m);
//      linkCircle2.setComOffset(0.0, 0.0, 0.0);
//      linkCircle2.setMomentOfInertia(m*r*r/2.0, m*(3*r*r + h*h)/12.0, m*(3*r*r + h*h)/12.0);
//      Graphics3DObject linkGraphics = new Graphics3DObject();
//
//      linkGraphics.rotate(Math.PI/2.0, Axis.Y);
//      linkGraphics.translate(0.0, 0.0, -0.1);
//      linkGraphics.addCylinder(h, r, YoAppearance.White());
//      linkCircle2.setLinkGraphics(linkGraphics);
//      return linkCircle2;
   }
   private Link linkCircle3()
   {

      double m = 1.0;
      double h = 0.2;
      double r = 0.2;



      Link linkCircle3 = new Link("linkCircle3");
      linkCircle3.setMass(m);
      linkCircle3.setComOffset(0.0, 0.0, 0.0);
      linkCircle3.setMomentOfInertia(m*r*r/2.0, m*(3*r*r + h*h)/12.0, m*(3*r*r + h*h)/12.0);
      Graphics3DObject linkGraphics = new Graphics3DObject();

      linkGraphics.rotate(Math.PI/2.0, Axis.Y);
      linkGraphics.translate(0.0, 0.0, -0.1);
      linkGraphics.addCylinder(h, r, YoAppearance.White());
      linkCircle3.setLinkGraphics(linkGraphics);
      return linkCircle3;
   }

   private void showCoordinatesRecursively(Joint joint, boolean drawEllipoids)
   {
      Graphics3DObject linkGraphics = joint.getLink().getLinkGraphics();
      linkGraphics.identity();
      linkGraphics.addCoordinateSystem(1.6);
      if (drawEllipoids)
      {
         joint.getLink().addEllipsoidFromMassProperties();
      }
      for (Joint child : joint.getChildrenJoints())
      {
         showCoordinatesRecursively(child, drawEllipoids);
      }
   }

}
