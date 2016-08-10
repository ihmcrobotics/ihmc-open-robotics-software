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
   public static FloatingJoint mainJoint;
   public static UniversalJoint foot;
   private PinJoint hip;
   private PinJoint shoulder;
   private ArrayList<GroundContactPoint> groundContactPoints = new ArrayList();

   /* L* are the link lengths, M* are the link masses, and R* are the radii of the links.
    * Iyy* are the moments of inertia of the links, which are defined about the COM for each link.
    */
   public static final double
           LEG_LENGTH = 1.0, LEG_MASS = 1.5, LEG_CUBE_LENGTH = 0.1, LEG_MOI = (1.0/3.0)* LEG_MASS *Math.pow(LEG_LENGTH,2), // Leg
           TORSO_LENGTH = 2.0, TORSO_MASS = 1.0, TORSO_RADIUS = 0.05, TORSO_MOI = (1.0/3.0)* TORSO_MASS *Math.pow(TORSO_LENGTH,2), // Torso
           SHOULDER_LENGTH = 3.0, SHOULDER_MASS = 0.5, SHOULDER_RADIUS = 0.05, SHOULDER_MOI = (1.0/12.0)* SHOULDER_MASS *Math.pow(SHOULDER_LENGTH,2); // Crossbar

   private ExternalForcePoint balanceForce;

   public SkippyRobot()
   {
      super("Skippy");

      this.setGravity(0.0,0.0,-9.81); // m/s^2

      // Create GroundContactPoints to distinguish when robot touches the ground
      GroundContactPoint footContact = new GroundContactPoint("rootContactPoint", new Vector3d(0.0, 0.0, 0.0), this);
      GroundContactPoint footContact1 = new GroundContactPoint("rootContactPoint1", new Vector3d(-LEG_CUBE_LENGTH, 0.0, 0.0), this);
//      GroundContactPoint footContact2 = new GroundContactPoint("rootContactPoint2", new Vector3d(LEG_CUBE_LENGTH, 0.0, 0.0), this);
//      GroundContactPoint footContact3 = new GroundContactPoint("rootContactPoint3", new Vector3d(0.0, LEG_CUBE_LENGTH, 0.0), this);
//      GroundContactPoint footContact4 = new GroundContactPoint("rootContactPoint4", new Vector3d(0.0, -LEG_CUBE_LENGTH, 0.0), this);
//      GroundContactPoint footContact5 = new GroundContactPoint("rootContactPoint5", new Vector3d(-0.25, -LEG_CUBE_LENGTH, 0.0), this);
//      GroundContactPoint footContact6 = new GroundContactPoint("rootContactPoint6", new Vector3d(LEG_CUBE_LENGTH, LEG_CUBE_LENGTH, 0.0), this);
//      GroundContactPoint footContact7 = new GroundContactPoint("rootContactPoint7", new Vector3d(LEG_CUBE_LENGTH, -LEG_CUBE_LENGTH, 0.0), this);
//      GroundContactPoint footContact8 = new GroundContactPoint("rootContactPoint8", new Vector3d(-LEG_CUBE_LENGTH, LEG_CUBE_LENGTH, 0.0), this);

      GroundContactPoint hipContact = new GroundContactPoint("hipContactPoint", new Vector3d(0.0, 0.0, 0.0), this);
      GroundContactPoint shoulderContact = new GroundContactPoint("shoulderContactPoint", new Vector3d(0.0, 0.0, 0.0), this);
      GroundContactPoint leftContact = new GroundContactPoint("leftContactPoint", new Vector3d(-SHOULDER_LENGTH / 2.0, 0.0, 0.0), this);
      GroundContactPoint rightContact = new GroundContactPoint("rightContactPoint", new Vector3d(SHOULDER_LENGTH / 2.0, 0.0, 0.0), this);

      groundContactPoints.add(footContact);
      groundContactPoints.add(footContact1);
      groundContactPoints.add(hipContact);
      groundContactPoints.add(shoulderContact);
      groundContactPoints.add(leftContact);
      groundContactPoints.add(rightContact);


//      mainJoint = new FloatingJoint("mainJoint", new Vector3d(0.0,0.0,0.0), this);
//      Link mainJointLink = new Link("mainJointLink");
//      mainJointLink.setMass(1.5);
//      mainJointLink.setMomentOfInertia(90,90,9990);
//      mainJoint.setLink(mainJointLink);
//      mainJoint.addGroundContactPoint(footContact);
//      this.addRootJoint(mainJoint);
//      Graphics3DObject mainJointLinkGraphics = new Graphics3DObject();
//      mainJointLinkGraphics.addCube(0.25,0.25,0.25,YoAppearance.White());
//      mainJointLink.setLinkGraphics(mainJointLinkGraphics);
//      mainJoint.addGroundContactPoint(footContact1);
//      mainJoint.addGroundContactPoint(footContact2);
//      mainJoint.addGroundContactPoint(footContact3);
//      mainJoint.addGroundContactPoint(footContact4);
//      mainJoint.addGroundContactPoint(footContact5);
//      mainJoint.addGroundContactPoint(footContact6);
//      mainJoint.addGroundContactPoint(footContact7);
//      mainJoint.addGroundContactPoint(footContact8);



       // Create joints and assign links. Each joint should be placed L* distance away from its previous joint.
      foot = new UniversalJoint("foot_X", "foot_Y", new Vector3d(0.0, 0.0, 0.0), this, Axis.X, Axis.Y);
      foot.setInitialState(Math.PI/7.0, 0.0, Math.PI/9.0, 0.0); // initial position "q" of foot
      Link leg = createLeg();
      foot.setLink(leg);
      this.addRootJoint(foot);
      foot.addGroundContactPoint(footContact);

      hip = new PinJoint("hip", new Vector3d(0.0, 0.0, LEG_LENGTH), this, Axis.X);
      Link torso = createTorso();
      hip.setLink(torso);
      hip.setInitialState(-2.0*Math.PI/8.0,0.0);
      this.foot.addJoint(hip);
      //hip.addGroundContactPoint(hipContact);

      shoulder = new PinJoint("shoulder", new Vector3d(0.0, 0.0, TORSO_LENGTH), this, Axis.Y);
      Link arms = createArms();
      shoulder.setLink(arms);
      shoulder.setInitialState(0.0,0.0);

      balanceForce = new ExternalForcePoint("BalanceForce", this);
      //balanceForce.setForce(0.0,0.0,0.0);
      shoulder.addExternalForcePoint(balanceForce);

      //shoulder.setDamping(0.3);
      this.hip.addJoint(shoulder);
      //shoulder.addGroundContactPoint(shoulderContact);
      //shoulder.addGroundContactPoint(leftContact);
      //shoulder.addGroundContactPoint(rightContact);

      GroundContactModel ground = new LinearGroundContactModel(this, 1422, 150.6, 50.0, 1000.0, this.getRobotsYoVariableRegistry());
      GroundProfile3D profile = new FlatGroundProfile();
      ground.setGroundProfile3D(profile);
      this.setGroundContactModel(ground);
   }

   private Link createLeg()
   {
      Link leg = new Link("leg");
      leg.setMass(LEG_MASS);
      leg.setComOffset(0.0, 0.0, LEG_LENGTH / 2.0);
      leg.setMomentOfInertia(LEG_MOI, LEG_MOI, 0.00001);

      // create a LinkGraphics object to manipulate the visual representation of the link
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCube(LEG_CUBE_LENGTH, LEG_CUBE_LENGTH, LEG_LENGTH, YoAppearance.Crimson());

      // associate the linkGraphics object with the link object
      leg.setLinkGraphics(linkGraphics);

      return leg;
   }

   private Link createTorso()
   {
      Link torso = new Link("torso");
      torso.setMass(TORSO_MASS);
      torso.setComOffset(0.0, 0.0, TORSO_LENGTH / 2.0);
      torso.setMomentOfInertia(TORSO_MOI, 0.0001, 0.0001);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCylinder(TORSO_LENGTH, TORSO_RADIUS, YoAppearance.LightSkyBlue());
      linkGraphics.addSphere(0.10,YoAppearance.White());

      torso.setLinkGraphics(linkGraphics);

      return torso;
   }

   private Link createArms()
   {
      Link arms = new Link("arms");
      arms.setMass(SHOULDER_MASS);
      arms.setComOffset(0.0, 0.0, SHOULDER_LENGTH / 2.0);
      arms.setMomentOfInertia(0.0001, SHOULDER_MOI, 0.0001);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.rotate(Math.toRadians(90), Axis.Y);
      linkGraphics.translate(0.0, 0.0, -SHOULDER_LENGTH / 2.0);
      linkGraphics.addCylinder(SHOULDER_LENGTH, SHOULDER_RADIUS, YoAppearance.DarkBlue());

      arms.setLinkGraphics(linkGraphics);

      return arms;
   }
   public UniversalJoint getLegJoint()
   {
      return foot;
   }
   public double getLegMass()
   {
      return LEG_MASS;
   }
   public double getLegLength()
   {
      return LEG_LENGTH;
   }
   public PinJoint getHipJoint()
   {
      return hip;
   }
   public double getHipMass()
   {
      return TORSO_MASS;
   }
   public double getHipLength()
   {
      return TORSO_LENGTH;
   }
   public PinJoint getShoulderJoint()
   {
      return shoulder;
   }
   public double getShoulderMass()
   {
      return SHOULDER_MASS;
   }
   public double getShoulderLength() {
      return SHOULDER_LENGTH;
   }
   public void setBalanceForce(double x, double y, double z)
   {
      balanceForce.setForce(x, y, z);
   }
   public ArrayList<GroundContactPoint> getGroundContactPoints()
   {
      return groundContactPoints;
   }
}
