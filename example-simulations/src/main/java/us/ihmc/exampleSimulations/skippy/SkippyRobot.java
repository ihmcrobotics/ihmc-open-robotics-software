package us.ihmc.exampleSimulations.skippy;

import java.util.ArrayList;

import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.*;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple3D.interfaces.Vector3DBasics;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.jMonkeyEngineToolkit.GroundProfile3D;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.GroundContactModel;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.KinematicPoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.UniversalJoint;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;

/**
 *
 * SkippyRobot is the extension of Robot that serves as the main robot for the Skippy Simulation.
 *
 * The simulation models Skippy, a simple theoretical robot developed by Roy Featherstone:
 *    http://royfeatherstone.org/skippy/index.html
 *
 * Other variations of Skippy also have been created, including a model resembling an Acrobot; different
 * controllers have also been implemented for positioning/jumping.
 *
 */

public class SkippyRobot extends Robot
{
   private static final boolean SHOW_MOI_ELLIPSOIDS = false;

   private static final long serialVersionUID = -7671864179791904256L;

   private final PoseReferenceFrame bodyZUpFrame;

   public final GroundContactPoint footGroundContactPoint;

   private final ArrayList<GroundContactPoint> groundContactPoints = new ArrayList();

   private final RobotType robotType;

    /**
     * Skippy: A simple robot with two degrees of freedom, balanced on a single point
     *   - Skippy is used for JUMPING_FORWARD, JUMPING_SIDEWAYS and BALANCE (see: SkippyController.java)
     * Tippy: A variation of Skippy rooted to the ground
     *   - Tippy is used for BALANCE (see: SkippyController.java)
     */

   public enum RobotType
   {
      TIPPY,
      SKIPPY;

      public double negateIfTippy(double value)
      {
         if (this == TIPPY)
            return -value;
         return value;
      }

      public double negativeIfSkippy(double value)
      {
         if (this == RobotType.SKIPPY)
            return -value;
         return value;
      }
   }

   /**
    *
    * Robot Outline:
    *
    *    There are three major parts to Tippy/Skippy:
    *       1. LEGs
    *       2. TORSO
    *       3. SHOULDER
    *
    *    The following is specific terminology for each robot:
    *       Tippy:
    *          LEG - Referred to as FOOT interchangeably, represented by a UniversalJoint (footJointIfTippy)
    *          TORSO - Referred to as HIP interchangeably, represented by a PinJoint (hipJoint)
    *          SHOULDER - Referred to as ARM interchangeably, represented by a PinJoint (shoulderJoint)
    *       Skippy:
    *          LEG - Referred to as FOOT interchangeably, represented by a PinJoint (hipJoint)
    *          TORSO - Referred to as HIP interchangeably, represented by a FloatingJoint (rootJointIfSkippy)
    *          SHOULDER - Referred to as ARM interchangeably, represented by a PinJoint (shoulderJoint)
    *
    */

   //tippy
   private final UniversalJoint footJointIfTippy;

   //skippy
   private final FloatingJoint rootJointIfSkippy;
   private ExternalForcePoint rootJointForce;

   //all
   private final PinJoint shoulderJoint;
   private final PinJoint hipJoint;

   public final YoDouble t;  //used for JUMP controller

   private final KinematicPoint bodyPoint;

   private final YoDouble yaw = new YoDouble("yaw", this.getRobotsYoRegistry());

   public static final double LEG_LENGTH = 1.0, LEG_MASS = 1.5, LEG_CUBE_LENGTH = 0.1, LEG_MOI = (1.0 / 4.0) * LEG_MASS * Math.pow(LEG_LENGTH, 2), // Leg
         TORSO_LENGTH = 2.0, TORSO_MASS = 1.0, TORSO_RADIUS = 0.05, TORSO_MOI = (1.0 / 4.0) * TORSO_MASS * Math.pow(TORSO_LENGTH, 2), // Torso
         SHOULDER_LENGTH = 3.0, SHOULDER_MASS = 0.5, SHOULDER_RADIUS = 0.05, SHOULDER_MOI = (1.0 / 2.0) * SHOULDER_MASS * Math.pow(SHOULDER_LENGTH, 2); // Crossbar

   private ExternalForcePoint balanceForce;
   public static ExternalForcePoint glueDownToGroundPoint;

//   private final double initialBodySidewaysLean = 0.13783;   //Limit sideways lean for IcpBasedControl to balance
   private final double initialBodySidewaysLean = 0.0; //Limit sideways lean for FeedbackPostureControl to balance
   private final double initialShoulderJointAngle = 0.0 * Math.PI / 6.0;
   private final double initialYawIfSkippy = 0.0* Math.PI * 0.8;

   public SkippyRobot(RobotType typeOfRobot)
   {
      super("Skippy");
      robotType = typeOfRobot;

      t = (YoDouble)findVariable("t");

      this.setGravity(0.0, 0.0, -9.81); // m/s^2

      if (typeOfRobot == RobotType.TIPPY)
      {
         // Create GroundContactPoints to distinguish when robot touches the ground
         GroundContactPoint footContact = new GroundContactPoint("gc_foot", new Vector3D(0.0, 0.0, 0.0), this);
         footGroundContactPoint = footContact;
         GroundContactPoint footContact1 = new GroundContactPoint("rootContactPoint1", new Vector3D(-LEG_CUBE_LENGTH, 0.0, 0.0), this);

         GroundContactPoint hipContact = new GroundContactPoint("hipContactPoint", new Vector3D(0.0, 0.0, 0.0), this);
         GroundContactPoint shoulderContact = new GroundContactPoint("shoulderContactPoint", new Vector3D(0.0, 0.0, 0.0), this);
         GroundContactPoint leftContact = new GroundContactPoint("leftContactPoint", new Vector3D(-SHOULDER_LENGTH / 2.0, 0.0, 0.0), this);
         GroundContactPoint rightContact = new GroundContactPoint("rightContactPoint", new Vector3D(SHOULDER_LENGTH / 2.0, 0.0, 0.0), this);

         groundContactPoints.add(footContact);
         groundContactPoints.add(footContact1);
         groundContactPoints.add(hipContact);
         groundContactPoints.add(shoulderContact);
         groundContactPoints.add(leftContact);
         groundContactPoints.add(rightContact);

         // Create joints and assign links. Each joint should be placed L* distance away from its previous joint.
         footJointIfTippy = new UniversalJoint("foot_X", "foot_Y", new Vector3D(0.0, 0.0, 0.0), this, Axis3D.X, Axis3D.Y);
         footJointIfTippy.setInitialState(-Math.PI / 7.0, 0.0, initialBodySidewaysLean, 0.0); // initial position "q" of foot
         Link leg = createLegTippy();
         footJointIfTippy.setLink(leg);
         this.addRootJoint(footJointIfTippy);
         footJointIfTippy.addGroundContactPoint(footContact);

         //Negative X axis so that corresponds with Tippy, which makes joints from the foot up, rather than the torso outward.
         hipJoint = new PinJoint("hipJoint", new Vector3D(0.0, 0.0, LEG_LENGTH), this, new Vector3D(-1.0, 0.0, 0.0));
         Link torso = createTorsoTippy();
         hipJoint.setLink(torso);
         hipJoint.setInitialState(- Math.PI / 6.0, 0.0);

         bodyPoint = new KinematicPoint("bodyPoint", new Vector3D(0.0, 0.0, TORSO_LENGTH / 2.0), this);
         hipJoint.addKinematicPoint(bodyPoint);

         this.footJointIfTippy.addJoint(hipJoint);
         //hip.addGroundContactPoint(hipContact);

         shoulderJoint = new PinJoint("shoulderJoint", new Vector3D(0.0, 0.0, TORSO_LENGTH), this, Axis3D.Y);
         Link arms = createArmsTippy();
         shoulderJoint.setLink(arms);
         shoulderJoint.setInitialState(initialShoulderJointAngle, 0.0);

         balanceForce = new ExternalForcePoint("BalanceForce", this);
         shoulderJoint.addExternalForcePoint(balanceForce);

         //shoulder.setDamping(0.3);
         this.hipJoint.addJoint(shoulderJoint);

         rootJointIfSkippy = null;
      }

      else if (typeOfRobot == RobotType.SKIPPY)
      {
         rootJointIfSkippy = new FloatingJoint("rootJoint", new Vector3D(0.0, 0.0, 0.0), this);
         rootJointForce = new ExternalForcePoint("rootJointForce", new Vector3D(0.0, 0.0, TORSO_LENGTH / 2), this);
         rootJointIfSkippy.addExternalForcePoint(rootJointForce);
         Link torso = createTorsoSkippy();
         rootJointIfSkippy.setLink(torso);

         bodyPoint = new KinematicPoint("bodyPoint", new Vector3D(0.0, 0.0, TORSO_LENGTH / 2.0), this);
         rootJointIfSkippy.addKinematicPoint(bodyPoint);

         this.addRootJoint(rootJointIfSkippy);

         RigidBodyTransform transform = new RigidBodyTransform();
         transform.setRotationEulerAndZeroTranslation(/*0.0*/Math.PI / 7.0 - 2.0 * Math.PI / 8.0, -initialBodySidewaysLean, initialYawIfSkippy);
         transform.getTranslation().set(new Vector3D(0.0, 0.0, /*2.0*/2.0-0.15975+0.0032));
         rootJointIfSkippy.setRotationAndTranslation(transform);

         shoulderJoint = new PinJoint("shoulderJoint", new Vector3D(0.0, 0.0, TORSO_LENGTH / 2), this, Axis3D.Y);
         shoulderJoint.setDamping(0.0);
         shoulderJoint.setInitialState(initialShoulderJointAngle, 0.0);
         Link arms = createArmsTippy();
         shoulderJoint.setLink(arms);
         GroundContactPoint shoulderContactPointLeft = new GroundContactPoint("shoulderContactPointLeft", new Vector3D(-SHOULDER_LENGTH / 2, 0.0, 0.0), this);
         GroundContactPoint shoulderContactPointRight = new GroundContactPoint("shoulderContactPointRight", new Vector3D(SHOULDER_LENGTH / 2, 0.0, 0.0), this);
         shoulderJoint.addGroundContactPoint(shoulderContactPointLeft);
         shoulderJoint.addGroundContactPoint(shoulderContactPointRight);
         shoulderContactPointLeft.disable();
         shoulderContactPointRight.disable();
         rootJointIfSkippy.addJoint(shoulderJoint);

         hipJoint = new PinJoint("hip", new Vector3D(0.0, 0.0, -TORSO_LENGTH / 2.0), this, Axis3D.X);
         hipJoint.setDamping(0.0);
         hipJoint.setInitialState(/*0.0*/2.0 * Math.PI / 8.0, 0.0);
         Link leg = createLegSkippy();
         hipJoint.setLink(leg);


         GroundContactPoint footContactPoint = new GroundContactPoint("gc_foot", new Vector3D(0.0, 0.0, -LEG_LENGTH), this);
         hipJoint.addGroundContactPoint(footContactPoint);

         footGroundContactPoint = footContactPoint;

         GroundContactPoint hipContactPoint = new GroundContactPoint("hipContactPoint", new Vector3D(0.0, 0.0, 0.0), this);
         hipJoint.addGroundContactPoint(hipContactPoint);

         glueDownToGroundPoint = new ExternalForcePoint("glueDownToGroundPoint", new Vector3D(0.0, 0.0, -LEG_LENGTH), this);
         hipJoint.addExternalForcePoint(glueDownToGroundPoint);

         rootJointIfSkippy.addJoint(hipJoint);

         //use as reference
         groundContactPoints.add(hipContactPoint); //0
         groundContactPoints.add(footContactPoint); //1
         groundContactPoints.add(shoulderContactPointRight); //2

         footJointIfTippy = null;
      }

      else
         throw new RuntimeException("No such robot " + robotType);

      bodyZUpFrame = new PoseReferenceFrame("bodyFrame", ReferenceFrame.getWorldFrame());

      GroundContactModel ground = new LinearGroundContactModel(this, 10000.0, 5000.0, 50.0, 5000.0, this.getRobotsYoRegistry());//50.0, 5000.0, this.getRobotsYoVariableRegistry());
      GroundProfile3D profile = new FlatGroundProfile();
      ground.setGroundProfile3D(profile);
      this.setGroundContactModel(ground);
   }

   private Link createLegTippy()
   {
      Link leg = new Link("leg");
      leg.setMass(LEG_MASS);
      leg.setComOffset(0.0, 0.0, LEG_LENGTH / 2.0);
      leg.setMomentOfInertia(LEG_MOI, LEG_MOI, 0.00001);

      // create a LinkGraphics object to manipulate the visual representation of the link
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCube(LEG_CUBE_LENGTH, LEG_CUBE_LENGTH, LEG_LENGTH, YoAppearance.Crimson());
      /*
       * Associate the linkGraphics object with the link object
       */
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
      linkGraphics.addCube(LEG_CUBE_LENGTH, LEG_CUBE_LENGTH, LEG_LENGTH, YoAppearance.Blue());
      /*
       * Joint
       */
      linkGraphics.identity();
      linkGraphics.rotate(Math.PI / 2.0, Axis3D.Y);
      linkGraphics.translate(0.0, 0.0, -LEG_CUBE_LENGTH);
      linkGraphics.addCylinder(2*LEG_CUBE_LENGTH, 2*LEG_CUBE_LENGTH/3, YoAppearance.LightSteelBlue());
      /*
       * Associate the linkGraphics object with the link object
       */
      leg.setLinkGraphics(linkGraphics);

      if (SHOW_MOI_ELLIPSOIDS)
      {
         leg.addEllipsoidFromMassProperties(YoAppearance.Gold());
      }

      return leg;
   }

   private Link createTorsoTippy()
   {
      Link torso = new Link("torso");
      torso.setMass(TORSO_MASS);
      torso.setComOffset(0.0, 0.0, TORSO_LENGTH / 2.0);
      torso.setMomentOfInertia(TORSO_MOI, TORSO_MOI, 0.0001);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.addCylinder(TORSO_LENGTH, TORSO_RADIUS, YoAppearance.Glass(0.75));
      linkGraphics.addSphere(0.10, YoAppearance.White());
      /*
       * Associate the linkGraphics object with the link object
       */
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
      linkGraphics.translate(0.0, 0.0, -TORSO_LENGTH / 2);
      linkGraphics.addCylinder(TORSO_LENGTH, TORSO_RADIUS, YoAppearance.Blue());
      /*
       * Associate the linkGraphics object with the link object
       */
      torso.setLinkGraphics(linkGraphics);

      if (SHOW_MOI_ELLIPSOIDS)
      {
         torso.addEllipsoidFromMassProperties(YoAppearance.Gold());
      }

      return torso;
   }

   private Link createArmsTippy()
   {
      Link arms = new Link("arms");
      arms.setMass(SHOULDER_MASS);
      arms.setComOffset(0.0, 0.0, 0.0);
      arms.setMomentOfInertia(0.0001, SHOULDER_MOI, SHOULDER_MOI);

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.rotate(Math.toRadians(90), Axis3D.Y);
      linkGraphics.translate(0.0, 0.0, -SHOULDER_LENGTH / 2.0);
      linkGraphics.addCylinder(SHOULDER_LENGTH, SHOULDER_RADIUS, YoAppearance.Red());
      /*
       * Joint
       */
      linkGraphics.rotate(Math.PI/2,Axis3D.Y);
      linkGraphics.rotate(Math.PI/2,Axis3D.X);
      linkGraphics.translate(-SHOULDER_LENGTH/2, 0.0, -LEG_CUBE_LENGTH);
      linkGraphics.addCylinder(2*LEG_CUBE_LENGTH, 2*LEG_CUBE_LENGTH/3, YoAppearance.LightSteelBlue());
      /*
       * Associate the linkGraphics object with the link object
       */
      arms.setLinkGraphics(linkGraphics);

      if (SHOW_MOI_ELLIPSOIDS)
      {
         arms.addEllipsoidFromMassProperties(YoAppearance.Gold());
      }

      return arms;
   }

   public UniversalJoint getLegJoint()
   {
      return footJointIfTippy;
   }

   public double getLegMass()
   {
      return LEG_MASS;
   }

   public double getLegLength()
   {
      return LEG_LENGTH;
   }

   public PinJoint getHipJointTippy()
   {
      return hipJoint;
   }

   public FloatingJoint getHipJointSkippy()
   {
      return rootJointIfSkippy;
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
      return shoulderJoint;
   }

   public PinJoint getHipJoint()
   {
      return hipJoint;
   }

   public double getShoulderMass()
   {
      return SHOULDER_MASS;
   }

   public double getShoulderLength()
   {
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

   public Point3D computeFootLocation()
   {
      return footGroundContactPoint.getPositionCopy();
   }

   public void computeFootContactForce(Vector3DBasics tempForce){
	  footGroundContactPoint.getForce(tempForce);
   }

   private RigidBodyTransform transform = new RigidBodyTransform();

   public ReferenceFrame updateAndGetBodyFrame()
   {
      if (robotType == RobotType.TIPPY)
      {
         transform.getTranslation().set(new Vector3D(bodyPoint.getPositionCopy()));
      }

      else if (robotType == RobotType.SKIPPY)
      {
         rootJointIfSkippy.getTransformToWorld(transform);

         // Change to ZUp by getting rid of x and y rotation.
         Vector3D translation = new Vector3D();
         translation.set(transform.getTranslation());
         Vector3D rotationEuler = new Vector3D();
         transform.getRotation().getEuler(rotationEuler);
         rotationEuler.setX(0.0);
         rotationEuler.setY(0.0);
         //         rotationEuler.setZ(0.0);
         yaw.set(rotationEuler.getZ());

         transform.setRotationEulerAndZeroTranslation(rotationEuler);
         transform.getTranslation().set(translation);
      }

      this.bodyZUpFrame.setTransformAndUpdate(transform);
      return bodyZUpFrame;
   }

   public double getGravity(){
      return this.getGravityZ();
   }

   public boolean getFootFS()
   {
      return footGroundContactPoint.isInContact();
   }

   public double getMass()
   {
      return SHOULDER_MASS+LEG_MASS+TORSO_MASS;
   }

   public YoDouble getQdd_z()
   {
      return rootJointIfSkippy.qdd_z;
   }

   public YoDouble getQ_hip()
   {
      return hipJoint.getQYoVariable();
   }

   public YoDouble getQd_hip()
   {
      return hipJoint.getQDYoVariable();
   }
   public YoDouble getQdd_hip()
   {
      return hipJoint.getQDDYoVariable();
   }

   Point3D tempCOMPosition = new Point3D();
   Vector3D tempLinearMomentum = new Vector3D();
   Vector3D tempAngularMomentum = new Vector3D();
   /**
    * Computes the CoM position and velocity and the ICP
    */
   public void computeComAndICP(FramePoint3D comToPack, FrameVector3D comVelocityToPack, FramePoint3D icpToPack, FrameVector3D angularMomentumToPack)
   {
      double totalMass = computeCOMMomentum(tempCOMPosition, tempLinearMomentum, tempAngularMomentum);
      angularMomentumToPack.set(tempAngularMomentum);

      comToPack.set(tempCOMPosition);
      tempLinearMomentum.scale(1.0 / totalMass);
      comVelocityToPack.set(tempLinearMomentum);

      double omega0 = Math.sqrt(comToPack.getZ() / Math.abs(getGravity()));

      icpToPack.scaleAdd(omega0, comVelocityToPack, comToPack);
      icpToPack.setZ(0.0);
   }

   Vector3D tempJointAxis = new Vector3D();
   RigidBodyTransform transformJointToWorld = new RigidBodyTransform();
   public void getHipJointAxis(FrameVector3D hipAxisToPack)
   {
      hipJoint.getJointAxis(tempJointAxis);
      hipJoint.getTransformToWorld(transformJointToWorld);
      transformJointToWorld.transform(tempJointAxis);
      hipAxisToPack.set(tempJointAxis);
      hipAxisToPack.normalize();
   }

   public void getShoulderJointAxis(FrameVector3D shoulderAxisToPack)
   {
      shoulderJoint.getJointAxis(tempJointAxis);
      shoulderJoint.getTransformToWorld(transformJointToWorld);
      transformJointToWorld.transform(tempJointAxis);
      shoulderAxisToPack.set(tempJointAxis);
      shoulderAxisToPack.normalize();
   }

}
