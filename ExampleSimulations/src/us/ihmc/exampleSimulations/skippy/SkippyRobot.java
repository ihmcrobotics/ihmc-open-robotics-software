package us.ihmc.exampleSimulations.skippy;

import java.util.ArrayList;

import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.graphics3DAdapter.GroundProfile3D;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.UniversalJoint;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;

/**
 * This class SkippyRobot is a public class that extends Robot. The class Robot is
 * included in the Simulation Construction Set and has built in graphics, dynamics, etc.
 * Extending the class is an easy way to make a new type of robot, in this case a SkippyRobot.
 */
public class SkippyRobot extends Robot
{
   private static final boolean SHOW_MOI_ELLIPSOIDS = true;

   private static final long serialVersionUID = -7671864179791904256L;

   private final GroundContactPoint footGroundContactPoint;

   private final ArrayList<GroundContactPoint> groundContactPoints = new ArrayList();

   private final RobotType robotType;

   public enum RobotType
   {
      TIPPY, SKIPPY;

      public double negateIfTippy(double value)
      {
         if (this == TIPPY) return -value;
         return value;
      }

      public double negativeIfSkippy(double value)
      {
         if (this == RobotType.SKIPPY) return -value;
         return value;
      }
   }

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
   //4, 1/4
   private ExternalForcePoint balanceForce;

   public SkippyRobot(RobotType typeOfRobot)
   {
      super("Skippy");
      robotType = typeOfRobot;

      this.setGravity(0.0,0.0,-9.81); // m/s^2

      if(typeOfRobot==RobotType.TIPPY)
      {
         // Create GroundContactPoints to distinguish when robot touches the ground
         GroundContactPoint footContact = new GroundContactPoint("gc_foot", new Vector3d(0.0, 0.0, 0.0), this);
         footGroundContactPoint = footContact;
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
         foot.setInitialState(-Math.PI/7.0, 0.0, 0.0 * Math.PI/24, 0.0); // initial position "q" of foot
         Link leg = createLegAcrobot();
         foot.setLink(leg);
         this.addRootJoint(foot);
         foot.addGroundContactPoint(footContact);

         //Negative X axis so that corresponds with Tippy, which makes joints from the foot up, rather than the torso outward.
         hip = new PinJoint("hipJoint", new Vector3d(0.0, 0.0, LEG_LENGTH), this, new Vector3d(-1.0, 0.0, 0.0));
         Link torso = createTorsoAcrobot();
         hip.setLink(torso);
         hip.setInitialState(-2.0*Math.PI/8.0,0.0);
         this.foot.addJoint(hip);
         //hip.addGroundContactPoint(hipContact);

         shoulder = new PinJoint("shoulderJoint", new Vector3d(0.0, 0.0, TORSO_LENGTH), this, Axis.Y);
         Link arms = createArmsAcrobot();
         shoulder.setLink(arms);
         shoulder.setInitialState(0.0 * Math.PI/6,0.0);

         balanceForce = new ExternalForcePoint("BalanceForce", this);
         //balanceForce.setForce(0.0,0.0,0.0);
         shoulder.addExternalForcePoint(balanceForce);

         //shoulder.setDamping(0.3);
         this.hip.addJoint(shoulder);
         //shoulder.addGroundContactPoint(shoulderContact);
         //shoulder.addGroundContactPoint(leftContact);
         //shoulder.addGroundContactPoint(rightContact);
      }
      else if(typeOfRobot==RobotType.SKIPPY)
      {
         rootJoint = new FloatingJoint("rootJoint", new Vector3d(0.0, 0.0, 0.0), this);
         //double offsetAngle = Math.PI/6;
         //rootJoint.setPosition(convertHipJointAngleToVector(offsetAngle));
         rootJointForce = new ExternalForcePoint("rootJointForce", new Vector3d(0.0, 0.0, TORSO_LENGTH/2), this);
         rootJoint.addExternalForcePoint(rootJointForce);
         Link torso = createTorsoSkippy();
         rootJoint.setLink(torso);
         this.addRootJoint(rootJoint);

         RigidBodyTransform transform = new RigidBodyTransform();
         transform.setRotationEulerAndZeroTranslation(-Math.PI/7.0 + 2.0 * Math.PI/8.0, 0.0, 0.0);
         transform.setTranslation(new Vector3d(0.0, 0.0, 1.85));
         rootJoint.setRotationAndTranslation(transform);

         shoulder = new PinJoint("shoulderJoint", new Vector3d(0.0, 0.0, TORSO_LENGTH/2), this, Axis.Y);
         shoulder.setInitialState(0*Math.PI/12, 0.0);
         Link arms = createArmsAcrobot();
         shoulder.setLink(arms);
         GroundContactPoint shoulderContactPointLeft = new GroundContactPoint("shoulderContactPointLeft", new Vector3d(-SHOULDER_LENGTH/2, 0.0, 0.0), this);
         GroundContactPoint shoulderContactPointRight = new GroundContactPoint("shoulderContactPointRight", new Vector3d(SHOULDER_LENGTH/2, 0.0, 0.0), this);
         shoulder.addGroundContactPoint(shoulderContactPointLeft);
         shoulder.addGroundContactPoint(shoulderContactPointRight);
         shoulderContactPointLeft.disable();
         shoulderContactPointRight.disable();
         rootJoint.addJoint(shoulder);

         hip = new PinJoint("hip", new Vector3d(0.0, 0.0, -TORSO_LENGTH/2.0), this, Axis.X);
         hip.setInitialState(-2.0*Math.PI/8.0,0.0);
         Link leg = createLegSkippy();
         hip.setLink(leg);
         GroundContactPoint footContactPoint = new GroundContactPoint("gc_foot", new Vector3d(0.0, 0.0, -LEG_LENGTH), this);
         hip.addGroundContactPoint(footContactPoint);

         footGroundContactPoint = footContactPoint;

         GroundContactPoint hipContactPoint = new GroundContactPoint("hipContactPoint", new Vector3d(0.0, 0.0, 0.0), this);
         hip.addGroundContactPoint(hipContactPoint);

         ExternalForcePoint glueDownToGroundPoint = new GroundContactPoint("glueDownToGroundPoint", new Vector3d(0.0, 0.0, -LEG_LENGTH), this);
         hip.addExternalForcePoint(glueDownToGroundPoint);
         glueDownToGroundPoint.setForce(new Vector3d(0.0, 0.0, -8000.0));

         rootJoint.addJoint(hip);

         //use as reference
         groundContactPoints.add(hipContactPoint);  //0
         groundContactPoints.add(footContactPoint);  //1
         groundContactPoints.add(shoulderContactPointRight);  //2

      }

      else throw new RuntimeException("No such robot " + robotType);

      GroundContactModel ground = new LinearGroundContactModel(this, 10000.0, 5000.0, 50.0, 5000.0, this.getRobotsYoVariableRegistry());
      GroundProfile3D profile = new FlatGroundProfile();
      ground.setGroundProfile3D(profile);
      this.setGroundContactModel(ground);
   }

   private Vector3d convertHipJointAngleToVector(double angle)
   {
      //hipjoint only on YZ plane
      double dz = TORSO_LENGTH/2.0 - TORSO_LENGTH/2.0 * Math.cos(Math.abs(angle));
      double dy = TORSO_LENGTH/2.0 - TORSO_LENGTH/2.0 * Math.sin(Math.abs(angle));
      if(angle < 0)
         dy = dy * -1;

      Vector3d newPosition = new Vector3d();
      this.getHipJointSkippy().getPosition(newPosition);
      newPosition.setY(newPosition.getY()+dy);
      newPosition.setZ(newPosition.getZ()+dz);
      return newPosition;
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
      leg.setComOffset(0.0, 0.0, -LEG_LENGTH / 2.0);

      leg.setMomentOfInertia(LEG_MOI, LEG_MOI, 0.00001);

      // create a LinkGraphics object to manipulate the visual representation of the link
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(0.0, 0.0, -LEG_LENGTH);
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

   public Point3d computeFootLocation()
   {
      return footGroundContactPoint.getPositionPoint();
   }
}
