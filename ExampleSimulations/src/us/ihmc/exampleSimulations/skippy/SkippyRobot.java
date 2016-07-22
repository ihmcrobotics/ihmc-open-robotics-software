package us.ihmc.exampleSimulations.skippy;

import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.simulationconstructionset.*;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;

import javax.vecmath.Vector3d;
import java.util.ArrayList;

/**
 * This class SkippyRobot is a public class that extends Robot. The class Robot is
 * included in the Simulation Construction Set and has built in graphics, dynamics, etc.
 * Extending the class is an easy way to make a new type of robot, in this case a SkippyRobot.
 */
public class SkippyRobot extends Robot
{
   private static final long serialVersionUID = -7671864179791904256L;
   private Joint foot;
   private Joint hip;
   private Joint shoulder;
   private ArrayList<GroundContactPoint> groundContactPoints = new ArrayList();

   /* L1 and L2 are the link lengths, M1 and M2 are the link masses, and R1 and R2 are the radii of the links,
    * Iyy1 and Iyy2 are the moments of inertia of the links. The moments of inertia are defined about the COM
    * for each link.
    */
   public static final double
         L1 = 1.0, M1 = 1.0, R1 = 0.05, Iyy1 = 0.083,
         L2 = 2.0, M2 = 1.0, R2 = 0.05, Iyy2 = 0.33,
         L3 = 3.0, M3 = 1.0, R3 = 0.05, Iyy3 = 0.15;

   public SkippyRobot()
   {
      super("Skippy"); // create an instance of Robot

      this.setGravity(0.0,0.0,-9.81);

      GroundContactModel ground = new LinearGroundContactModel(this, 1422, 150.6, 50.0, 1000.0, this.getRobotsYoVariableRegistry());
      GroundProfile3D profile = new FlatGroundProfile();
      ground.setGroundProfile3D(profile);
      this.setGroundContactModel(ground);

      // create GroundContactPoints to distinguish when robot touches the ground
      GroundContactPoint footContact = new GroundContactPoint("foot", new Vector3d(0.0, 0.0, 0.0), this);
      GroundContactPoint hipContact = new GroundContactPoint("hip", new Vector3d(0.0, 0.0, 0.0), this);
      GroundContactPoint shoulderContact = new GroundContactPoint("shoulder", new Vector3d(0.0, 0.0, 0.0), this);

      groundContactPoints.add(footContact);
      groundContactPoints.add(hipContact);
      groundContactPoints.add(shoulderContact);

      // Create joints and assign links. Pin joints have a single axis of rotation.
      Link leg = createLeg();
      foot = new FloatingPlanarJoint("foot", this, 3);
      foot.setLink(leg); // associate createLeg with the joint pin1
      this.addRootJoint(foot);
      foot.addGroundContactPoint(footContact);

      /*
       *  The second joint is initiated with the offset vector (0.0,0.0,L1) since
       *  it should be placed a distance of L1 in the Z direction from the previous joint.
       */
      hip = new PinJoint("hip", new Vector3d(0.0, 0.0, L1), this, Axis.X);
      Link torso = createTorso();
      hip.setLink(torso);
      this.foot.addJoint(hip);
      hip.addGroundContactPoint(hipContact);

      /*
       *  The third joint is initiated with the offset vector (0.0,0.0,L1+L2) since
       *  it should be placed a distance of L1 + L2 in the Z direction from the previous joint.
       */
      shoulder = new PinJoint("shoulder", new Vector3d(0.0, 0.0, L1 + L2), this, Axis.Y);
      Link crossBar = createCrossbar();
      shoulder.setLink(crossBar);
      this.foot.addJoint(shoulder);
      shoulder.addGroundContactPoint(shoulderContact);
   }

   /**
    * Create the first link for the SkippyRobot.
    */
   private Link createLeg()
   {
      Link leg = new Link("leg");
      leg.setMass(M1);
      leg.setComOffset(0.0, 0.0, L1 / 2.0);
      leg.setMomentOfInertia(0.0, Iyy1, 0.0);

      // create a LinkGraphics object to manipulate the visual representation of the link
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCube(R1, R1, L1, YoAppearance.Crimson());

      // associate the linkGraphics object with the link object
      leg.setLinkGraphics(linkGraphics);

      return leg;
   }

   /**
    * Create the second link for the SkippyRobot.
    */
   private Link createTorso()
   {
      Link torso = new Link("torso");
      torso.setMass(M2);
      torso.setComOffset(0.0, 0.0, L2 / 2.0);
      torso.setMomentOfInertia(0.0, Iyy2, 0.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCylinder(L2, R2, YoAppearance.LightSkyBlue());

      torso.setLinkGraphics(linkGraphics);

      return torso;
   }

   /**
    * Create the third link for the SkippyRobot.
    */
   private Link createCrossbar()
   {
      Link crossbar = new Link("crossbar");
      crossbar.setMass(M3);
      crossbar.setComOffset(0.0, 0.0, L3 / 2.0);
      crossbar.setMomentOfInertia(0.0, Iyy3, 0.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.rotate(Math.toRadians(90), Axis.X);
      linkGraphics.translate(0.0, 0.0, -L3 / 2.0);
      linkGraphics.addCylinder(L3, R3, YoAppearance.DarkBlue());

      crossbar.setLinkGraphics(linkGraphics);

      return crossbar;
   }

   public PinJoint getHipJoint()
   {
      return (PinJoint) hip;
   }

   public PinJoint getShoulderJoint()
   {
      return (PinJoint) shoulder;
   }
}
