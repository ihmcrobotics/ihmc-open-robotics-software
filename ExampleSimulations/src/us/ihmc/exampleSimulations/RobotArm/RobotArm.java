package us.ihmc.exampleSimulations.RobotArm;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.simulationconstructionset.*;

import javax.vecmath.Vector3d;

public class RobotArm extends Robot
{
   // Angles of every respective axis
   private static final double
         angle1 = Math.toRadians(90),
         angle2 = Math.toRadians(115),
         angle3 = Math.toRadians(65),
         angle4 = Math.toRadians(90),
         angle5 = Math.toRadians(90),
         angle6 = Math.toRadians(90),
         angle7 = Math.toRadians(80);

   // Lengths (L*), masses (M*) & radii (R*)
   private static final double
         L1 = 0.0254, M1 = 10.0, R1 = 0.03175,
         L2 = 0.1524, M2 = 10.0, R2 = 0.0127,
//         L3 = 0.0, M3 = 0.0, R3 = 0.0,
         L4 = 0.2286, M4 = 10.0, R4 = 0.0127,
//         L5 = 0.0, M5 = 0.0, R5 = 0.0,
         L6 = 0.0508, M6 = 10.0, R6 = 0.0127,
         L7 = 0.0635, M7 = 10.0, R7 = 0.0127;

   RobotArm()
   {
      // Create an instance of 7Bot
      super("RobotArm");

      // Create damping to make the transitions smoother
      double damping = 10.0;

      // Create axis 1 & base of 7Bot
      PinJoint axis1 = new PinJoint("axis1", new Vector3d(0.0, 0.0, 0.0), this, Axis.Z);
      axis1.setInitialState(Math.cos(angle1), Math.sin(angle1));
      axis1.setDamping(damping);
      axis1.setLimitStops(-0.25, 3.15, 0, 0);

      Link link1 = rotatingLink1();
      axis1.setLink(link1);
      this.addRootJoint(axis1);

      // Create axis 2 + upper arm of 7Bot
      PinJoint axis2 = new PinJoint("axis2", new Vector3d(0.0, 0.0, L1), this, Axis.Y);
      axis2.setInitialState(Math.cos(angle2), Math.sin(angle2));
      axis2.setDamping(damping);
      axis2.setLimitStops(-1.15, 2.35, 0, 0);

      Link link2 = translatingLink2();
      axis2.setLink(link2);
      axis1.addJoint(axis2);

      // Create axes 3 & 4 + forearm of 7Bot
      UniversalJoint axes34 = new UniversalJoint("axis3", "axis4", new Vector3d(0.0, 0.0, L2 * Math.sin(angle2)), this, Axis.Y, Axis.X);
      axes34.getFirstJoint().setInitialState(Math.cos(angle3), Math.sin(angle3));
      axes34.getFirstJoint().setDamping(damping);
      axes34.getFirstJoint().setLimitStops(-1.3, 2.0, 0, 0);

      // link3 is not instantiated because it is hidden in the UniversalJoint

      axes34.getSecondJoint().setInitialState(Math.cos(angle4), Math.sin(angle4));
      axes34.getSecondJoint().setDamping(damping);
      axes34.getSecondJoint().setLimitStops(-1.3, 2.0, 0, 0);

      Link link4 = translatingAndRotatingLink4();
      axes34.setLink(link4);
      axis2.addJoint(axes34);

      // Create axes 5 & 6 + neck of 7Bot
      UniversalJoint axes56 = new UniversalJoint("axis5", "axis6", new Vector3d(L4, 0.0, 0.0), this, Axis.Y, Axis.X);
      axes56.getFirstJoint().setInitialState(Math.cos(angle5), Math.sin(angle5));
      axes56.getFirstJoint().setDamping(damping);

      // link5 is not instantiated because it is hidden in the UniversalJoint

      axes56.getSecondJoint().setInitialState(Math.cos(angle6), Math.sin(angle6));
      axes56.getSecondJoint().setDamping(damping);

      Link link6 = rotatingLink6();
      axes56.setLink(link6);
      axes34.addJoint(axes56);

      // Create axis 7 (left grip)
      PinJoint axis7Left = new PinJoint("axis7Left", new Vector3d(L6, 0.0, 0.0), this, Axis.Z);
      axis7Left.setInitialState(Math.cos(angle7), Math.sin(angle7));
      axis7Left.setDamping(damping);

      Link link7LeftBase = link7LeftBase();
      axis7Left.setLink(link7LeftBase);
      axes56.addJoint(axis7Left);
      
//      PinJoint gripLeft = new PinJoint("gripLeft", new Vector3d(0.0, -L7 * Math.sin(angle7), 0.0), this, Axis.Z);
////      gripLeft.setInitialState(Math.PI / 2 - Math.cos(angle7), Math.PI / 2 - Math.sin(angle7));
//      gripLeft.setDamping(damping);
//
//      Link link7LeftTip = link7LeftTip();
//      gripLeft.setLink(link7LeftTip);
//      axis7Left.addJoint(gripLeft);
      
      // Create axis 7 (right grip)
      PinJoint axis7Right = new PinJoint("axis7Right", new Vector3d(L6, 0.0, 0.0), this, Axis.Z);
      axis7Right.setInitialState(Math.cos(angle7), Math.sin(angle7));
      axis7Right.setDamping(damping);

      Link link7RightBase = link7RightBase();
      axis7Right.setLink(link7RightBase);
      axes56.addJoint(axis7Right);

//      PinJoint gripRight = new PinJoint("gripRight", new Vector3d(0.0, -L7 * Math.sin(angle7), 0.0), this, Axis.Z);
////      gripRight.setInitialState(Math.PI / 2 - Math.cos(angle7), Math.PI / 2 - Math.sin(angle7));
//      gripRight.setDamping(damping);
//
//      Link link7RightTip = link7RightTip();
//      gripRight.setLink(link7RightTip);
//      axis7Right.addJoint(gripRight);

      //  showCoordinatesRecursively(axis1, true);
   }

   private Link rotatingLink1()
   {
      Link ret = new Link("rotatingLink1");

      ret.setMass(M1);
      ret.setComOffset(0.0, 0.0, L1 / 2.0);
      ret.setMomentOfInertia(M1 * (3 * R1 * R1 + L1 * L1) / 12.0, M1 * (3 * R1 * R1 + L1 * L1) / 12.0, M1 * R1 * R1 / 2.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCylinder(L1, R1, YoAppearance.Violet());

      ret.setLinkGraphics(linkGraphics);
      return ret;
   }

   private Link translatingLink2()
   {
      Link ret = new Link("translatingLink2");

      ret.setMass(M2);
      ret.setComOffset(0.0, 0.0, L2 / 2.0);
      ret.setMomentOfInertia(M2 * (3 * R2 * R2 + L2 * L2) / 12.0, M2 * (3 * R2 * R2 + L2 * L2) / 12.0, M2 * R2 * R2 / 2.0);
      // create a LinkGraphics object to manipulate the visual representation of the link
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(0.0, 0.0, -0.01);
      linkGraphics.addCylinder(L2, R2, YoAppearance.Aqua());

      // associate the linkGraphics object with the link object
      ret.setLinkGraphics(linkGraphics);
      return ret;
   }

   private Link translatingAndRotatingLink4()
   {
      Link ret = new Link("translatingAndRotatingLink4");

      ret.setMass(M4);
      ret.setComOffset(0.0, 0.0, 0.0);
      ret.setMomentOfInertia(M4 * R4 * R4 / 2.0, M4 * (3 * R4 * R4 + L4 * L4) / 12.0, M4 * (3 * R4 * R4 + L4 * L4) / 12.0);
      Graphics3DObject linkGraphics = new Graphics3DObject();

      linkGraphics.rotate(Math.PI / 2.0, Axis.Y);
      linkGraphics.translate(0.0, 0.0, -0.01);
      linkGraphics.addCylinder(L4, R4, YoAppearance.Green());
      ret.setLinkGraphics(linkGraphics);
      return ret;
   }

   private Link rotatingLink6()
   {
      Link ret = new Link("rotatingLink6");

      ret.setMass(M6);
      ret.setComOffset(0.0, 0.0, 0.0);
      ret.setMomentOfInertia(M6 * R6 * R6 / 2.0, M6 * (3 * R6 * R6 + L6 * L6) / 12.0, M6 * (3 * R6 * R6 + L6 * L6) / 12.0);
      Graphics3DObject linkGraphics = new Graphics3DObject();

      linkGraphics.rotate(Math.PI / 2.0, Axis.Y);
      linkGraphics.translate(0.0, 0.0, -0.01);
      linkGraphics.addCylinder(L6, R6, YoAppearance.Yellow());
      ret.setLinkGraphics(linkGraphics);
      return ret;
   }

   private Link link7LeftBase()
   {
      Link ret = new Link("link7LeftBase");

      ret.setMass(M7);
      ret.setComOffset(L7 / 2.0, 0.0, 0.0);
      ret.setMomentOfInertia(M7 * R7 * R7 / 2.0, M7 * (3 * R7 * R7 + L7 * L7) / 12.0, M7 * (3 * R7 * R7 + L7 * L7) / 12.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.rotate(Math.PI / 2.0, Axis.Y);
      linkGraphics.rotate(-Math.PI / 4.0, Axis.X);
      linkGraphics.translate(0.0, 0.0, -0.01);
      linkGraphics.addCylinder(L7, R7, YoAppearance.White());
      ret.setLinkGraphics(linkGraphics);
      return ret;
   }

   private Link link7LeftTip()
   {
      Link ret = new Link("link7LeftTip");

      ret.setMass(M7);
      ret.setComOffset(L7 / 2.0, 0.0, 0.0);
      ret.setMomentOfInertia(M7 * R7 * R7 / 2.0, M7 * (3 * R7 * R7 + L7 * L7) / 12.0, M7 * (3 * R7 * R7 + L7 * L7) / 12.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.rotate(Math.PI / 2.0, Axis.Y);
//      linkGraphics.rotate(Math.PI / 4.0, Axis.X);
//      linkGraphics.translate(0.0, 0.0, -0.02);
      linkGraphics.addCylinder(L7, R7, YoAppearance.White());
      ret.setLinkGraphics(linkGraphics);
      return ret;
   }

   private Link link7RightBase()
   {
      Link ret = new Link("link7RightBase");

      ret.setMass(M7);
      ret.setComOffset(L7 / 2.0, 0.0, 0.0);
      ret.setMomentOfInertia(M7 * R7 * R7 / 2.0, M7 * (3 * R7 * R7 + L7 * L7) / 12.0, M7 * (3 * R7 * R7 + L7 * L7) / 12.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.rotate(Math.PI / 2.0, Axis.Y);
      linkGraphics.rotate(Math.PI / 4.0, Axis.X);
      linkGraphics.translate(0.0, 0.0, -0.01);
      linkGraphics.addCylinder(L7, R7, YoAppearance.Black());
      ret.setLinkGraphics(linkGraphics);
      return ret;
   }

   private Link link7RightTip()
   {
      Link ret = new Link("link7RightTip");

      ret.setMass(M7);
      ret.setComOffset(L7 / 2.0, 0.0, 0.0);
      ret.setMomentOfInertia(M7 * R7 * R7 / 2.0, M7 * (3 * R7 * R7 + L7 * L7) / 12.0, M7 * (3 * R7 * R7 + L7 * L7) / 12.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.rotate(Math.PI / 2.0, Axis.Y);
//      linkGraphics.rotate(-Math.PI / 4.0, Axis.X);
//      linkGraphics.translate(0.0, 0.0, -0.04);
      linkGraphics.addCylinder(L7, R7, YoAppearance.Black());
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
