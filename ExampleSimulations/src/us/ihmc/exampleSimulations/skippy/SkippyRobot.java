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
   private static final boolean SHOW_MOI_ELLIPSOIDS = true;

   private static final long serialVersionUID = -7671864179791904256L;

   private ArrayList<GroundContactPoint> groundContactPoints = new ArrayList();

   private int robotType = 0;

   //acrobot
   private PinJoint hip;

   //skippy
   private FloatingJoint rootJoint;
   private ExternalForcePoint rootJointForce;

   //all
   private UniversalJoint foot;
   private PinJoint shoulder;


   /* L* are the link lengths, M* are the link masses, and R* are the radii of the links.
    * Iyy* are the moments of inertia of the links, which are defined about the COM for each link.
    */
   public static final double
           LEG_LENGTH = 1.0, LEG_MASS = 1.5, LEG_CUBE_LENGTH = 0.1, LEG_MOI = (1.0/4.0)* LEG_MASS *Math.pow(LEG_LENGTH,2), // Leg
           TORSO_LENGTH = 2.0, TORSO_MASS = 1.0, TORSO_RADIUS = 0.05, TORSO_MOI = (1.0/4.0)* TORSO_MASS *Math.pow(TORSO_LENGTH,2), // Torso
           SHOULDER_LENGTH = 3.0, SHOULDER_MASS = 0.5, SHOULDER_RADIUS = 0.05, SHOULDER_MOI = (1.0/2.0)* SHOULDER_MASS *Math.pow(SHOULDER_LENGTH,2); // Crossbar

   private ExternalForcePoint balanceForce;

   public SkippyRobot(int typeOfRobot)
   {
      super("Skippy");
      robotType = typeOfRobot;

      this.setGravity(0.0,0.0,-9.81); // m/s^2

      if(typeOfRobot==0)
      {
         // Create GroundContactPoints to distinguish when robot touches the ground
         GroundContactPoint footContact = new GroundContactPoint("rootContactPoint", new Vector3d(0.0, 0.0, 0.0), this);
         GroundContactPoint footContact1 = new GroundContactPoint("rootContactPoint1", new Vector3d(-LEG_CUBE_LENGTH, 0.0, 0.0), this);

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

         // Create joints and assign links. Each joint should be placed L* distance away from its previous joint.
         foot = new UniversalJoint("foot_X", "foot_Y", new Vector3d(0.0, 0.0, 0.0), this, Axis.X, Axis.Y);
         foot.setInitialState(Math.PI/7.0, 0.0, Math.PI/24, 0.0); // initial position "q" of foot
         Link leg = createLegAcrobot();
         foot.setLink(leg);
         this.addRootJoint(foot);
         foot.addGroundContactPoint(footContact);

         hip = new PinJoint("hipJoint", new Vector3d(0.0, 0.0, LEG_LENGTH), this, Axis.X);
         Link torso = createTorsoAcrobot();
         hip.setLink(torso);
         hip.setInitialState(-2.0*Math.PI/8.0,0.0);
         this.foot.addJoint(hip);
         //hip.addGroundContactPoint(hipContact);

         shoulder = new PinJoint("shoulderJoint", new Vector3d(0.0, 0.0, TORSO_LENGTH), this, Axis.Y);
         Link arms = createArmsAcrobot();
         shoulder.setLink(arms);
         shoulder.setInitialState(Math.PI/6,0.0);

         balanceForce = new ExternalForcePoint("BalanceForce", this);
         //balanceForce.setForce(0.0,0.0,0.0);
         shoulder.addExternalForcePoint(balanceForce);

         //shoulder.setDamping(0.3);
         this.hip.addJoint(shoulder);
         //shoulder.addGroundContactPoint(shoulderContact);
         //shoulder.addGroundContactPoint(leftContact);
         //shoulder.addGroundContactPoint(rightContact);
      }
      else if(typeOfRobot==1)
      {
         rootJoint = new FloatingJoint("rootJoint", new Vector3d(0.0, 0.0, LEG_LENGTH + TORSO_LENGTH/2), this);
         rootJointForce = new ExternalForcePoint("rootJointForce", new Vector3d(0.0, 0.0, TORSO_LENGTH/2), this);
         rootJoint.addExternalForcePoint(rootJointForce);
         Link torso = createTorsoSkippy();
         rootJoint.setLink(torso);
         this.addRootJoint(rootJoint);
         
         shoulder = new PinJoint("shoulderJoint", new Vector3d(0.0, 0.0, TORSO_LENGTH/2), this, Axis.Y);
         shoulder.setInitialState(Math.PI/12, 0.0);
         Link arms = createArmsAcrobot();
         shoulder.setLink(arms);
         GroundContactPoint shoulderContactPointLeft = new GroundContactPoint("shoulderContactPointLeft", new Vector3d(-SHOULDER_LENGTH/2, 0.0, 0.0), this);
         GroundContactPoint shoulderContactPointRight = new GroundContactPoint("shoulderContactPointRight", new Vector3d(SHOULDER_LENGTH/2, 0.0, 0.0), this);
         shoulder.addGroundContactPoint(shoulderContactPointLeft);
         shoulder.addGroundContactPoint(shoulderContactPointRight);
         rootJoint.addJoint(shoulder);

         foot = new UniversalJoint("foot_X", "foot_Y", new Vector3d(0.0, 0.0, -TORSO_LENGTH/2), this, Axis.X, Axis.Y);
         foot.setInitialState((Math.PI+Math.PI/12), 0.0, 0.0, 0.0);
         Link leg = createLegSkippy();
         foot.setLink(leg);
         GroundContactPoint rootContactPoint = new GroundContactPoint("rootContactPoint", new Vector3d(0.0, 0.0, 0.0), this);
         foot.addGroundContactPoint(rootContactPoint);
         GroundContactPoint hipContactPoint = new GroundContactPoint("hipContactPoint", new Vector3d(0.0, 0.0, LEG_LENGTH), this);
         foot.addGroundContactPoint(hipContactPoint);
         groundContactPoints.add(hipContactPoint);
         groundContactPoints.add(rootContactPoint);
         rootJoint.addJoint(foot);
      }



      GroundContactModel ground = new LinearGroundContactModel(this, 1422, 150.6, 50.0, 1000.0, this.getRobotsYoVariableRegistry());
      GroundProfile3D profile = new FlatGroundProfile();
      ground.setGroundProfile3D(profile);
      this.setGroundContactModel(ground);
   }

   private Link createLegAcrobot()
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

      if (SHOW_MOI_ELLIPSOIDS)
      {
         leg.addEllipsoidFromMassProperties(YoAppearance.Gold());
      }

      return leg;
   }
   private Link createLegSkippy()
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

      if (SHOW_MOI_ELLIPSOIDS)
      {
         leg.addEllipsoidFromMassProperties(YoAppearance.Gold());
      }

      return leg;
   }

   private Link createTorsoAcrobot()
   {
      Link torso = new Link("torso");
      torso.setMass(TORSO_MASS);
      torso.setComOffset(0.0, 0.0, TORSO_LENGTH / 2.0);
      torso.setMomentOfInertia(TORSO_MOI, TORSO_MOI, 0.0001);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCylinder(TORSO_LENGTH, TORSO_RADIUS, YoAppearance.LightSkyBlue());
      linkGraphics.addSphere(0.10,YoAppearance.White());

      torso.setLinkGraphics(linkGraphics);

      if (SHOW_MOI_ELLIPSOIDS)
      {
         torso.addEllipsoidFromMassProperties(YoAppearance.Gold());
      }

      return torso;
   }

   private Link createTorsoSkippy()
   {
      Link torso = new Link("torso");
      torso.setMass(TORSO_MASS);
      torso.setComOffset(0.0, 0.0, 0.0);
      torso.setMomentOfInertia(TORSO_MOI, TORSO_MOI, 0.0001);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(0.0, 0.0, -TORSO_LENGTH/2);
      linkGraphics.addCylinder(TORSO_LENGTH, TORSO_RADIUS, YoAppearance.LightSkyBlue());

      torso.setLinkGraphics(linkGraphics);

      if (SHOW_MOI_ELLIPSOIDS)
      {
         torso.addEllipsoidFromMassProperties(YoAppearance.Gold());
      }

      return torso;
   }

   private Link createArmsAcrobot()
   {
      Link arms = new Link("arms");
      arms.setMass(SHOULDER_MASS);
      arms.setComOffset(0.0, 0.0, 0.0);
      arms.setMomentOfInertia(0.0001, SHOULDER_MOI, SHOULDER_MOI);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.rotate(Math.toRadians(90), Axis.Y);
      linkGraphics.translate(0.0, 0.0, -SHOULDER_LENGTH / 2.0);
      linkGraphics.addCylinder(SHOULDER_LENGTH, SHOULDER_RADIUS, YoAppearance.DarkBlue());
      arms.setLinkGraphics(linkGraphics);

      if (SHOW_MOI_ELLIPSOIDS)
      {
         arms.addEllipsoidFromMassProperties(YoAppearance.Gold());
      }

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
   public PinJoint getHipJointAcrobot()
   {
      return hip;
   }
   public FloatingJoint getHipJointSkippy()
   {
      return rootJoint;
   }
   public void setRootJointForce(double x, double y, double z)
   {
      rootJointForce.setForce(x, y, z);
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
