package us.ihmc.exampleSimulations.controllerCore.robotArmWithFixedBase;

import java.util.ArrayList;
import java.util.List;

import org.apache.commons.lang3.tuple.ImmutablePair;
import org.apache.commons.lang3.tuple.Pair;

import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.geometry.RotationalInertiaCalculator;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;

public class RobotArm extends Robot
{
   private static final double SMALL_MASS = 0.2;
   private final Vector3D X_AXIS = new Vector3D(1.0, 0.0, 0.0);
   private final Vector3D Y_AXIS = new Vector3D(0.0, 1.0, 0.0);
   private final Vector3D Z_AXIS = new Vector3D(0.0, 0.0, 1.0);
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double gravity = 9.81;

   private final ReferenceFrame elevatorFrame;
   private final RigidBody elevator;
   private final Vector3D shoulderYawOffset = new Vector3D(0.0, 0.0, 0.3);
   private final Vector3D shoulderRollOffset = new Vector3D(0.0, 0.0, 0.0);
   private final Vector3D shoulderPitchOffset = new Vector3D(0.0, 0.0, 0.0);

   private final double upperArmMass = 2.2;
   private final double upperArmLength = 0.35;
   private final double upperArmRadius = 0.025;
   private final Matrix3D upperArmInertia = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidCylinder(upperArmMass, upperArmRadius, upperArmRadius,
                                                                                                                  Axis.Z);
   private final Vector3D upperArmCoM = new Vector3D(0.0, 0.0, upperArmLength / 2.0);

   private final Vector3D elbowPitchOffset = new Vector3D(0.0, 0.0, upperArmLength);

   private final double lowerArmMass = 2.2;
   private final double lowerArmLength = 0.35;
   private final double lowerArmRadius = 0.025;
   private final Matrix3D lowerArmInertia = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidCylinder(lowerArmMass, lowerArmRadius, lowerArmRadius,
                                                                                                                  Axis.Z);
   private final Vector3D lowerArmCoM = new Vector3D(0.0, 0.0, lowerArmLength / 2.0);

   private final Vector3D wristPitchOffset = new Vector3D(0.0, 0.0, lowerArmLength);
   private final Vector3D wristRollOffset = new Vector3D(0.0, 0.0, 0.0);
   private final Vector3D wristYawOffset = new Vector3D(0.0, 0.0, 0.0);

   private final double handMass = 1.2;
   private final Matrix3D handInertia = RotationalInertiaCalculator.getRotationalInertiaFromRadiiOfGyration(handMass, 0.08, 0.08, 0.08);
   private final Vector3D handCoM = new Vector3D(0.0, 0.0, 0.05);

   private final RevoluteJoint shoulderYaw;
   private final RigidBody shoulderYawLink;
   private final RevoluteJoint shoulderRoll;
   private final RigidBody shoulderRollLink;
   private final RevoluteJoint shoulderPitch;
   private final RigidBody upperArm;
   private final RevoluteJoint elbowPitch;
   private final RigidBody lowerArm;
   private final RevoluteJoint wristPitch;
   private final RigidBody wristPitchLink;
   private final RevoluteJoint wristRoll;
   private final RigidBody wristRollLink;
   private final RevoluteJoint wristYaw;
   private final RigidBody hand;

   private final RigidBodyTransform controlFrameTransform = new RigidBodyTransform(new AxisAngle(), new Vector3D(0.0, 0.0, 0.2));
   private final ReferenceFrame handControlFrame;

   private final List<Pair<OneDoFJoint, OneDegreeOfFreedomJoint>> idToSCSJointPairs = new ArrayList<>();

   public RobotArm()
   {
      super(RobotArm.class.getSimpleName());
      this.setGravity(0.0, 0.0, -gravity);

      elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new RigidBodyTransform());
      elevator = new RigidBody("elevator", elevatorFrame);

      shoulderYaw = ScrewTools.addRevoluteJoint("shoulderYaw", elevator, shoulderYawOffset, Z_AXIS);
      shoulderYawLink = ScrewTools.addRigidBody("shoulderYawLink", shoulderYaw, createNullMOI(), SMALL_MASS, new RigidBodyTransform());
      shoulderRoll = ScrewTools.addRevoluteJoint("shoulderRoll", shoulderYawLink, shoulderRollOffset, X_AXIS);
      shoulderRollLink = ScrewTools.addRigidBody("shoulderRollLink", shoulderRoll, createNullMOI(), SMALL_MASS, new RigidBodyTransform());
      shoulderPitch = ScrewTools.addRevoluteJoint("shoulderPitch", shoulderRollLink, shoulderPitchOffset, Y_AXIS);

      upperArm = ScrewTools.addRigidBody("upperArm", shoulderPitch, upperArmInertia, upperArmMass, upperArmCoM);

      elbowPitch = ScrewTools.addRevoluteJoint("elbowPitch", upperArm, elbowPitchOffset, Y_AXIS);

      lowerArm = ScrewTools.addRigidBody("lowerArm", elbowPitch, lowerArmInertia, lowerArmMass, lowerArmCoM);

      wristPitch = ScrewTools.addRevoluteJoint("wristPitch", lowerArm, wristPitchOffset, Y_AXIS);
      wristPitchLink = ScrewTools.addRigidBody("wristPitchLink", wristPitch, createNullMOI(), SMALL_MASS, new RigidBodyTransform());
      wristRoll = ScrewTools.addRevoluteJoint("wristRoll", wristPitchLink, wristRollOffset, X_AXIS);
      wristRollLink = ScrewTools.addRigidBody("wristRollLink", wristRoll, createNullMOI(), SMALL_MASS, new RigidBodyTransform());
      wristYaw = ScrewTools.addRevoluteJoint("wristYaw", wristRollLink, wristYawOffset, Z_AXIS);

      hand = ScrewTools.addRigidBody("hand", wristYaw, handInertia, handMass, handCoM);

      handControlFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent("handControlFrame", hand.getBodyFixedFrame(), controlFrameTransform);

      setJointLimits();

      setupSCSRobot();
   }

   private void setJointLimits()
   {
      RevoluteJoint[] allJoints = ScrewTools.filterJoints(ScrewTools.computeSubtreeJoints(elevator), RevoluteJoint.class);
      for (RevoluteJoint revoluteJoint : allJoints)
      {
         revoluteJoint.setJointLimitUpper(Math.PI);
         revoluteJoint.setJointLimitLower(-Math.PI);
      }
   }

   private void setupSCSRobot()
   {
      PinJoint scsShoulderYaw = new PinJoint("shoulderYaw", shoulderYawOffset, this, Axis.Z);
      PinJoint scsShoulderRoll = new PinJoint("shoulderRoll", shoulderRollOffset, this, Axis.X);
      PinJoint scsShoulderPitch = new PinJoint("shoulderPitch", shoulderPitchOffset, this, Axis.Y);
      PinJoint scsElbowPitch = new PinJoint("elbowPitch", elbowPitchOffset, this, Axis.Y);
      PinJoint scsWristPitch = new PinJoint("wristPitch", wristPitchOffset, this, Axis.Y);
      PinJoint scsWristRoll = new PinJoint("wristRoll", wristRollOffset, this, Axis.X);
      PinJoint scsWristYaw = new PinJoint("wristYaw", wristYawOffset, this, Axis.Z);

      Link scsShoulderYawLink = new Link("shoulderYawLink");
      scsShoulderYawLink.setMass(SMALL_MASS);
      scsShoulderYawLink.setMomentOfInertia(createNullMOI());
      scsShoulderYawLink.setComOffset(0.0, 0.0, 0.0);

      Link scsShoulderRollLink = new Link("shoulderRollLink");
      scsShoulderRollLink.setMass(SMALL_MASS);
      scsShoulderRollLink.setMomentOfInertia(createNullMOI());
      scsShoulderRollLink.setComOffset(0.0, 0.0, 0.0);

      Link scsUpperArmLink = new Link("upperArmLink");
      scsUpperArmLink.setMass(upperArmMass);
      scsUpperArmLink.setMomentOfInertia(upperArmInertia);
      scsUpperArmLink.setComOffset(upperArmCoM);
      scsUpperArmLink.setLinkGraphics(createArmGraphic(upperArmLength, upperArmRadius, YoAppearance.Red()));

      Link scsLowerArmLink = new Link("lowerArmLink");
      scsLowerArmLink.setMass(lowerArmMass);
      scsLowerArmLink.setMomentOfInertia(lowerArmInertia);
      scsLowerArmLink.setComOffset(lowerArmCoM);
      scsLowerArmLink.setLinkGraphics(createArmGraphic(lowerArmLength, lowerArmRadius, YoAppearance.Green()));

      Link scsWristPitchLink = new Link("wristPitchLink");
      scsWristPitchLink.setMass(SMALL_MASS);
      scsWristPitchLink.setMomentOfInertia(createNullMOI());
      scsWristPitchLink.setComOffset(0.0, 0.0, 0.0);

      Link scsWristRollLink = new Link("wristRollLink");
      scsWristRollLink.setMass(SMALL_MASS);
      scsWristRollLink.setMomentOfInertia(createNullMOI());
      scsWristRollLink.setComOffset(0.0, 0.0, 0.0);

      Link scsHandLink = new Link("handLink");
      scsHandLink.setMass(handMass);
      scsHandLink.setMomentOfInertia(handInertia);
      scsHandLink.setComOffset(handCoM);
      setupHandGraphics(scsHandLink);

      addRootJoint(scsShoulderYaw);
      scsShoulderYaw.setLink(scsShoulderYawLink);
      scsShoulderYaw.addJoint(scsShoulderRoll);
      scsShoulderRoll.setLink(scsShoulderRollLink);
      scsShoulderRoll.addJoint(scsShoulderPitch);
      scsShoulderPitch.setLink(scsUpperArmLink);
      scsShoulderPitch.addJoint(scsElbowPitch);
      scsElbowPitch.setLink(scsLowerArmLink);
      scsElbowPitch.addJoint(scsWristPitch);
      scsWristPitch.setLink(scsWristPitchLink);
      scsWristPitch.addJoint(scsWristRoll);
      scsWristRoll.setLink(scsWristRollLink);
      scsWristRoll.addJoint(scsWristYaw);
      scsWristYaw.setLink(scsHandLink);

      idToSCSJointPairs.add(new ImmutablePair<>(shoulderYaw, scsShoulderYaw));
      idToSCSJointPairs.add(new ImmutablePair<>(shoulderRoll, scsShoulderRoll));
      idToSCSJointPairs.add(new ImmutablePair<>(shoulderPitch, scsShoulderPitch));
      idToSCSJointPairs.add(new ImmutablePair<>(elbowPitch, scsElbowPitch));
      idToSCSJointPairs.add(new ImmutablePair<>(wristPitch, scsWristPitch));
      idToSCSJointPairs.add(new ImmutablePair<>(wristRoll, scsWristRoll));
      idToSCSJointPairs.add(new ImmutablePair<>(wristYaw, scsWristYaw));
   }

   public void setupHandGraphics(Link scsHandLink)
   {
      Graphics3DObject graphics3dObject = new Graphics3DObject();
      graphics3dObject.addSphere(0.025, YoAppearance.Grey());
      graphics3dObject.translate(handCoM);
      graphics3dObject.addEllipsoid(0.04, 0.01, 0.1);
      graphics3dObject.transform(controlFrameTransform);
      graphics3dObject.addCoordinateSystem(0.1);
      scsHandLink.setLinkGraphics(graphics3dObject);
   }

   private Graphics3DObject createArmGraphic(double length, double radius, AppearanceDefinition appearance)
   {
      Graphics3DObject graphics3dObject = new Graphics3DObject();
      graphics3dObject.addSphere(1.2 * radius, YoAppearance.Grey());
      graphics3dObject.addCylinder(length, radius, appearance);
      return graphics3dObject;
   }

   private Matrix3D createNullMOI()
   {
      Matrix3D momentOfInertia = new Matrix3D();
      momentOfInertia.setIdentity();
      momentOfInertia.scale(1.0e-4);
      return momentOfInertia;
   }

   public void updateSCSRobot()
   {
      for (Pair<OneDoFJoint,OneDegreeOfFreedomJoint> pair : idToSCSJointPairs)
      {
         pair.getRight().setTau(pair.getLeft().getTau());
      }
   }

   public void updateIDRobot()
   {
      for (Pair<OneDoFJoint,OneDegreeOfFreedomJoint> pair : idToSCSJointPairs)
      {
         pair.getLeft().setQ(pair.getRight().getQ());
         pair.getLeft().setQd(pair.getRight().getQD());
      }
      elevator.updateFramesRecursively();
   }

   public RigidBody getElevator()
   {
      return elevator;
   }

   public RigidBody getHand()
   {
      return hand;
   }

   public ReferenceFrame getHandControlFrame()
   {
      return handControlFrame;
   }

   public double getGravity()
   {
      return gravity;
   }
}
