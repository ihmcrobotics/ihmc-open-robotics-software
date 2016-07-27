package us.ihmc.exampleSimulations.doublePendulum2;

import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.dataStructures.YoVariableHolder;
import us.ihmc.simulationconstructionset.*;

/**
 * This class DoublePendulumRobot is a public class that extends Robot. The class Robot is
 * included in the Simulation Construction Set and has built in graphics, dynamics, etc.
 * Extending the class is an easy way to make a new type of robot, in this case a DoublePendulumRobot.
 */
public class DoublePendulumRobot extends Robot
{
   private static final long serialVersionUID = -7671864179791904256L;
   private double mass = 1.0;
   private double length = 2.0;
   public static final double
         L1 = 1.0, L2 = 2.0, M1 = 1.0, M2 = 1.0, R1 = 0.1, R2 = 0.05, Iyy1 = 0.083, Iyy2 = 0.33;
   public DoublePendulumRobot(String name)
   {
      super(name);

//      NullJoint nullJoint = new NullJoint("nullJoint", new Vector3d(0.0,0.0,2.0),this);
//
//
//      Link ceiling = new Link("link1");
//      Graphics3DObject linkGraphics = new Graphics3DObject();
//      linkGraphics.addCube(5, 5, 0.1);
//      ceiling.setLinkGraphics(linkGraphics);
//      nullJoint.setLink(ceiling);





      PinJoint pin1 = new PinJoint("joint1", new Vector3d(0.0, 0.0, 0.0), this, Axis.Y);

//      pin1.setInitialState(0.001, 0.00);
//      pin1.setDamping(0.0001);

      Link link1 = link1();
      pin1.setLink(link1); // associate link1 with the joint pin1
      //nullJoint.addJoint(pin1);
      this.addRootJoint(pin1);


      //this.addStaticLink(nullJoint);
//      Joint pin2 = new PinJoint("joint2", new Vector3d(0.0, 0.0, L1), this, Axis.Y);
//      Link link2 = link2();
//      pin2.setLink(link2);
//      pin1.addJoint(pin2);
//
//      Joint pin3 = new PinJoint("joint3", new Vector3d(0.0, 0.0, L2), this, Axis.Y);
//      Link link3 = link1();
//      pin3.setLink(link3);
//      pin1.addJoint(pin3);

     // this.addRootJoint(pin2);



   }
   /**
    * Create the first link for the DoublePendulumRobot.
    */
   private Link link1()
   {
      Link ret = new Link("link1");
      ret.setMass(M1);
      ret.setComOffset(0.0, 0.0, L1 / 2.0);
      ret.setMomentOfInertia(0.0, Iyy1, 0.0);
      // create a LinkGraphics object to manipulate the visual representation of the link
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCylinder(L1, R1, YoAppearance.Red());


      // associate the linkGraphics object with the link object
      ret.setLinkGraphics(linkGraphics);
      return ret;
   }
   private Link link2()
   {
      Link ret = new Link("link2");
      ret.setMass(M2);
      ret.setComOffset(0.0, 0.0, L2 / 2.0);
      ret.setMomentOfInertia(0.0, Iyy2, 0.0);
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCylinder(L2, R2, YoAppearance.Green());
      ret.setLinkGraphics(linkGraphics);
      return ret;
   }

}