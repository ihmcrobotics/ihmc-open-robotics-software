package us.ihmc.exampleSimulations.doublePendulum;

import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;

/**
 * This class DoublePendulumRobot is a public class that extends Robot. The class Robot is
 * included in the Simulation Construction Set and has built in graphics, dynamics, etc.
 * Extending the class is an easy way to make a new type of robot, in this case a DoublePendulumRobot.
 */
public class DoublePendulumRobot extends Robot
{
   private static final long serialVersionUID = -7671864179791904256L;

   /* L1 and L2 are the link lengths, M1 and M2 are the link masses, and R1 and R2 are the radii of the links,
    * Iyy1 and Iyy2 are the moments of inertia of the links. The moments of inertia are defined about the COM
    * for each link.
    */
   public static final double
         L1 = 1.0, L2 = 2.0, M1 = 1.0, M2 = 1.0, R1 = 0.1, R2 = 0.05, Iyy1 = 0.083, Iyy2 = 0.33;

   public DoublePendulumRobot()
   {
      super("DoublePendulum"); // create an instance of Robot
      // Create joints and assign links. Pin joints have a single axis of rotation.
      PinJoint pin1 = new PinJoint("joint1", new Vector3d(0.0, 0.0, 0.0), this, Axis.Y);
       pin1.setInitialState(0.05, 0.0);
      Link link1 = link1();
      pin1.setLink(link1); // associate link1 with the joint pin1
      this.addRootJoint(pin1);
      /*
       *  The second joint is initiated with the offset vector (0.0,0.0,L1) since
       *  it should be placed a distance of L1 in the Z direction from the previous joint.
       */
      Joint pin2 = new PinJoint("joint2", new Vector3d(0.0, 0.0, L1), this, Axis.Y);
      Link link2 = link2();
      pin2.setLink(link2);
      pin1.addJoint(pin2);
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
   /**
    * Create the second link for the DoublePendulumRobot.
    */
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