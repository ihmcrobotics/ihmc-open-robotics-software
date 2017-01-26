package us.ihmc.exampleSimulations.skippy;

import java.util.EnumMap;

import javax.vecmath.Matrix3d;
import javax.vecmath.Point3d;
import javax.vecmath.Vector3d;

import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePoint;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.RotationalInertiaCalculator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.Twist;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.GroundContactPoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.util.LinearGroundContactModel;
import us.ihmc.simulationconstructionset.util.ground.FlatGroundProfile;

public class SkippyRobotV2 extends Robot
{

   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private final ReferenceFrame footReferenceFrame;

   private final DoubleYoVariable kCapture = new DoubleYoVariable("kCapture", yoVariableRegistry);
   private final DoubleYoVariable totalMass = new DoubleYoVariable("totalMass", yoVariableRegistry);

   private final RigidBody elevator;
   private final SixDoFJoint rootJoint;

   private static final boolean SHOW_MASS_ELIPSOIDS = false;
   private static final boolean SHOW_COORDINATE_SYSTEMS = false;

   private static final AppearanceDefinition SHOULDER_COLOR = YoAppearance.Red();
   private static final AppearanceDefinition TORSO_COLOR = YoAppearance.Blue();
   private static final AppearanceDefinition LEG_COLOR = YoAppearance.Blue();
   private static final AppearanceDefinition JOINT_COLOR = YoAppearance.LightSteelBlue();
   private static final double JOINT_RADIUS = 0.1;

   public static final double LEG_LENGTH = 1.0;
   public static final double LEG_MASS = 1.5;
   public static final double LEG_RADIUS = 0.05;

   public static final double TORSO_LENGTH = 2.0;
   public static final double TORSO_MASS = 1.0;
   public static final double TORSO_RADIUS = 0.05;

   public static final double SHOULDER_LENGTH = 3.0;
   public static final double SHOULDER_MASS = 0.5;
   public static final double SHOULDER_RADIUS = 0.05;

   private final GroundContactPoint footContactPoint;

   private enum SkippyJoint
   {
      HIP_PITCH, SHOULDER_ROLL;
   }

   private enum SkippyBody
   {
      ELEVATOR, LEG, TORSO, SHOULDER
   }

   private final EnumMap<SkippyJoint, OneDoFJoint> jointMap = new EnumMap<>(SkippyJoint.class);
   private final EnumMap<SkippyJoint, PinJoint> scsJointMap = new EnumMap<>(SkippyJoint.class);
   private final EnumMap<SkippyBody, RigidBody> bodyMap = new EnumMap<>(SkippyBody.class);
   private final FloatingJoint scsRootJoint;

   public SkippyRobotV2()
   {
      super("SkippyV2");

      // --- id robot ---
      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new RigidBodyTransform());
      elevator = new RigidBody("elevator", elevatorFrame);
      bodyMap.put(SkippyBody.ELEVATOR, elevator);

      rootJoint = new SixDoFJoint("rootJoint", elevator, elevatorFrame);
      Matrix3d inertiaTorso = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidCylinder(TORSO_MASS, TORSO_RADIUS, TORSO_LENGTH, Axis.Z);
      RigidBody torso = ScrewTools.addRigidBody("torso", rootJoint, inertiaTorso, TORSO_MASS, new Vector3d(0.0, 0.0, 0.0));
      bodyMap.put(SkippyBody.TORSO, torso);

      RevoluteJoint idHipJoint = ScrewTools.addRevoluteJoint("idHipJoint", torso, new Vector3d(0.0, 0.0, -TORSO_LENGTH / 2.0), new Vector3d(1.0, 0.0, 0.0));
      Matrix3d inertiaLeg = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidCylinder(LEG_MASS, LEG_RADIUS, LEG_LENGTH, Axis.Z);
      RigidBody leg = ScrewTools.addRigidBody("leg", idHipJoint, inertiaLeg, LEG_MASS, new Vector3d(0.0, 0.0, -LEG_LENGTH / 2.0));
      bodyMap.put(SkippyBody.LEG, leg);
      jointMap.put(SkippyJoint.HIP_PITCH, idHipJoint);

      RigidBodyTransform legToFoot = new RigidBodyTransform();
      legToFoot.setTranslation(0.0, 0.0, -LEG_LENGTH / 2.0);
      footReferenceFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("footFrame", leg.getBodyFixedFrame(), legToFoot);

      RevoluteJoint idShoulderJoint = ScrewTools.addRevoluteJoint("idShoulderJoint", torso, new Vector3d(0.0, 0.0, TORSO_LENGTH / 2.0),
                                                                  new Vector3d(0.0, 1.0, 0.0));
      Matrix3d inertiaShoulder = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidCylinder(SHOULDER_MASS, SHOULDER_RADIUS, SHOULDER_LENGTH, Axis.X);
      RigidBody shoulder = ScrewTools.addRigidBody("shoulder", idShoulderJoint, inertiaShoulder, SHOULDER_MASS, new Vector3d(0.0, 0.0, 0.0));
      jointMap.put(SkippyJoint.SHOULDER_ROLL, idShoulderJoint);
      bodyMap.put(SkippyBody.SHOULDER, shoulder);

      // --- scs robot ---
      scsRootJoint = new FloatingJoint("rootJoint", new Vector3d(), this);
      scsRootJoint.setLink(createTorsoSkippy());
      scsRootJoint.setPosition(0.0, 0.0, LEG_LENGTH + TORSO_LENGTH / 2.0);
      this.addRootJoint(scsRootJoint);

      PinJoint shoulderJoint = new PinJoint("shoulderJoint", new Vector3d(0.0, 0.0, TORSO_LENGTH / 2.0), this, Axis.Y);
      shoulderJoint.setLink(createArm());
      scsRootJoint.addJoint(shoulderJoint);
      scsJointMap.put(SkippyJoint.SHOULDER_ROLL, shoulderJoint);

      PinJoint hipJoint = new PinJoint("hip", new Vector3d(0.0, 0.0, -TORSO_LENGTH / 2.0), this, Axis.X);
      hipJoint.setLink(createLeg());
      scsRootJoint.addJoint(hipJoint);
      scsJointMap.put(SkippyJoint.HIP_PITCH, hipJoint);

      totalMass.set(computeCenterOfMass(new Point3d()));

      // add ground contact points
      footContactPoint = new GroundContactPoint("gc_foot", new Vector3d(0.0, 0.0, -LEG_LENGTH), this);
      hipJoint.addGroundContactPoint(footContactPoint);
      GroundContactPoint hipContactPoint = new GroundContactPoint("gc_hip", new Vector3d(0.0, 0.0, 0.0), this);
      hipJoint.addGroundContactPoint(hipContactPoint);
      GroundContactPoint arm1ContactPoint = new GroundContactPoint("gc_arm1", new Vector3d(SHOULDER_LENGTH / 2.0, 0.0, 0.0), this);
      shoulderJoint.addGroundContactPoint(arm1ContactPoint);
      GroundContactPoint arm2ContactPoint = new GroundContactPoint("gc_arm2", new Vector3d(-SHOULDER_LENGTH / 2.0, 0.0, 0.0), this);
      shoulderJoint.addGroundContactPoint(arm2ContactPoint);

      // add ground contact model
      LinearGroundContactModel ground = new LinearGroundContactModel(this, this.getRobotsYoVariableRegistry());
      ground.setZStiffness(2000.0);
      ground.setZDamping(1500.0);
      ground.setXYStiffness(50000.0);
      ground.setXYDamping(2000.0);
      ground.setGroundProfile3D(new FlatGroundProfile());
      this.setGroundContactModel(ground);

      // add an external force point to easily push the robot in simulation
      ExternalForcePoint forcePoint = new ExternalForcePoint("forcePoint", new Vector3d(0.0, 0.0, TORSO_LENGTH / 2.0), this);
      scsRootJoint.addExternalForcePoint(forcePoint);
   }

   private Link createTorsoSkippy()
   {
      Link torso = new Link("torso");
      torso.setMass(TORSO_MASS);
      torso.setComOffset(0.0, 0.0, 0.0);
      torso.setMomentOfInertia(RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidCylinder(TORSO_MASS, TORSO_RADIUS, TORSO_LENGTH, Axis.Z));

      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(0.0, 0.0, -TORSO_LENGTH / 2.0);
      linkGraphics.addCylinder(TORSO_LENGTH, TORSO_RADIUS, TORSO_COLOR);
      torso.setLinkGraphics(linkGraphics);

      if (SHOW_MASS_ELIPSOIDS)
         torso.addEllipsoidFromMassProperties();
      if (SHOW_COORDINATE_SYSTEMS)
         torso.addCoordinateSystemToCOM(0.3);

      return torso;
   }

   private Link createArm()
   {
      Link arms = new Link("arms");
      arms.setMass(SHOULDER_MASS);
      arms.setComOffset(0.0, 0.0, 0.0);
      arms.setMomentOfInertia(RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidCylinder(SHOULDER_MASS, SHOULDER_RADIUS, SHOULDER_LENGTH, Axis.X));

      // arm graphics:
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.rotate(Math.toRadians(90), Axis.Y);
      linkGraphics.translate(0.0, 0.0, -SHOULDER_LENGTH / 2.0);
      linkGraphics.addCylinder(SHOULDER_LENGTH, SHOULDER_RADIUS, SHOULDER_COLOR);
      // joint graphics:
      linkGraphics.rotate(Math.PI / 2.0, Axis.Y);
      linkGraphics.rotate(Math.PI / 2.0, Axis.X);
      linkGraphics.translate(-SHOULDER_LENGTH / 2.0, 0.0, -JOINT_RADIUS);
      linkGraphics.addCylinder(2.0 * JOINT_RADIUS, 2.0 * JOINT_RADIUS / 3.0, JOINT_COLOR);
      arms.setLinkGraphics(linkGraphics);

      if (SHOW_MASS_ELIPSOIDS)
         arms.addEllipsoidFromMassProperties();
      if (SHOW_COORDINATE_SYSTEMS)
         arms.addCoordinateSystemToCOM(0.3);

      return arms;
   }

   private Link createLeg()
   {
      Link leg = new Link("leg");
      leg.setMass(LEG_MASS);
      leg.setComOffset(0.0, 0.0, -LEG_LENGTH / 2.0);
      leg.setMomentOfInertia(RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidCylinder(LEG_MASS, LEG_RADIUS, LEG_LENGTH, Axis.Z));

      // leg graphics:
      Graphics3DObject linkGraphics = new Graphics3DObject();
      linkGraphics.translate(0.0, 0.0, -LEG_LENGTH);
      linkGraphics.addCylinder(LEG_LENGTH, LEG_RADIUS, LEG_COLOR);
      // joint graphics:
      linkGraphics.identity();
      linkGraphics.rotate(Math.PI / 2.0, Axis.Y);
      linkGraphics.translate(0.0, 0.0, -JOINT_RADIUS);
      linkGraphics.addCylinder(2.0 * JOINT_RADIUS, 2.0 * JOINT_RADIUS / 3.0, JOINT_COLOR);
      leg.setLinkGraphics(linkGraphics);

      if (SHOW_MASS_ELIPSOIDS)
         leg.addEllipsoidFromMassProperties();
      if (SHOW_COORDINATE_SYSTEMS)
         leg.addCoordinateSystemToCOM(0.3);

      return leg;
   }

   public void updateInverseDynamicsStructureFromSimulation()
   {
      // update joint angles and velocities
      for (SkippyJoint joint : SkippyJoint.values())
      {
         OneDoFJoint idJoint = jointMap.get(joint);
         OneDegreeOfFreedomJoint scsJoint = scsJointMap.get(joint);
         idJoint.setQ(scsJoint.getQYoVariable().getDoubleValue());
         idJoint.setQd(scsJoint.getQDYoVariable().getDoubleValue());
      }

      // update root joint position
      RigidBodyTransform rootJointTransform = new RigidBodyTransform();
      scsRootJoint.getTransformToWorld(rootJointTransform);
      rootJointTransform.normalizeRotationPart();
      rootJoint.setPositionAndRotation(rootJointTransform);

      // update root joint velocity
      FrameVector linearVelocity = new FrameVector();
      FrameVector angularVelocity = new FrameVector();
      ReferenceFrame elevatorFrame = rootJoint.getFrameBeforeJoint();
      ReferenceFrame rootBodyFrame = rootJoint.getFrameAfterJoint();
      scsRootJoint.getVelocity(linearVelocity);
      linearVelocity.changeFrame(rootBodyFrame);
      scsRootJoint.getAngularVelocity(angularVelocity, rootBodyFrame);
      Twist rootJointTwist = new Twist(rootBodyFrame, elevatorFrame, rootBodyFrame, linearVelocity.getVector(), angularVelocity.getVector());
      rootJoint.setJointTwist(rootJointTwist);

      // update all the frames
      bodyMap.get(SkippyBody.ELEVATOR).updateFramesRecursively();
   }

   public void updateSimulationFromInverseDynamicsTorques()
   {
      for (SkippyJoint joint : SkippyJoint.values())
      {
         OneDoFJoint idJoint = jointMap.get(joint);
         OneDegreeOfFreedomJoint scsJoint = scsJointMap.get(joint);
         scsJoint.setTau(idJoint.getTau());
      }
   }

   public Point3d getFootLocation()
   {
      return footContactPoint.getPositionPoint();
   }

   public RigidBody getSkippyFoot()
   {
      return bodyMap.get(SkippyBody.LEG);
   }

   private final Point3d tempCOMPosition = new Point3d();
   private final Vector3d tempLinearMomentum = new Vector3d();
   private final Vector3d tempAngularMomentum = new Vector3d();

   public void computeComAndICP(FramePoint comToPack, FrameVector comVelocityToPack, FramePoint icpToPack, FrameVector angularMomentumToPack)
   {
      totalMass.set(computeCOMMomentum(tempCOMPosition, tempLinearMomentum, tempAngularMomentum));
      angularMomentumToPack.set(tempAngularMomentum);

      comToPack.set(tempCOMPosition);
      tempLinearMomentum.scale(1.0 / totalMass.getDoubleValue());
      comVelocityToPack.set(tempLinearMomentum);

      double omega0 = Math.sqrt(comToPack.getZ() / Math.abs(getGravityZ()));

      icpToPack.scaleAdd(omega0, comVelocityToPack, comToPack);
      icpToPack.setZ(0.0);
   }

   public Point3d computeFootLocation()
   {
      return footContactPoint.getPositionPoint();
   }

   public boolean getFootFS()
   {
      return footContactPoint.isInContact();
   }

   public void computeFootContactForce(Vector3d tempForce)
   {
      footContactPoint.getForce(tempForce);
   }

   public void cmpFromIcpDynamics(FramePoint icp, FramePoint footLocation, FramePoint desiredCMPToPack)
   {
      FrameVector icpToFoot = new FrameVector();
      icpToFoot.sub(icp, footLocation);
      desiredCMPToPack.scaleAdd(kCapture.getDoubleValue(), icpToFoot, icp);
      desiredCMPToPack.setZ(0.0);
   }

   public double getGravity()
   {
      return this.getGravityZ();
   }

   public double getMass()
   {
      return totalMass.getDoubleValue();
   }

   public ReferenceFrame getEndEffectorFrame()
   {
      return footReferenceFrame;
   }

   public RigidBody getEndEffectorBody()
   {
      return bodyMap.get(SkippyBody.LEG);
   }

   public RigidBody getElevator()
   {
      return elevator;
   }

   public RigidBody getTorso()
   {
      return bodyMap.get(SkippyBody.TORSO);
   }

}
