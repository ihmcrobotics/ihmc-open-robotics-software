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

   /* L1 and L2 are the link lengths, M1 and M2 are the link masses, and R1 and R2 are the radii of the links,
    * Iyy1 and Iyy2 are the moments of inertia of the links. The moments of inertia are defined about the COM
    * for each link.
    */
   public static final double
         L1 = 6.0,  M1 = 4.0,  R1 = 0.1,  Iyy1 = 0.083;
   PinJoint pin1 = new PinJoint("joint1", new Vector3d(0.0, 0.0, 0.0), this, Axis.Y);

   public DoublePendulumRobot(String yoVariable)
   {


      super(yoVariable); // create and instance of Robot
      FloatingPlanarJoint floatingPlanarJoint = new FloatingPlanarJoint(yoVariable,this);

      Link ceiling = new Link("link1");
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCube(5, 5, 0.1);
      ceiling.setLinkGraphics(linkGraphics);
      floatingPlanarJoint.setLink(ceiling);



      // Create joints and assign links. Pin joints have a single axis of rotation.

     // pin1.setLimitStops(0,12, 2, 3);
       pin1.setInitialState(0.01, 0.01);
      Link link1 = link1();
      pin1.setLink(link1); // associate link1 with the joint pin1
      floatingPlanarJoint.addJoint(pin1);
      this.addRootJoint(floatingPlanarJoint);


   }

   /**
    * Create the first link for the DoublePendulumRobot.
    */
   public void setLocation(double x, double y, double z)
   {
      pin1.changeOffsetVector(x,y,z);
   }
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

}