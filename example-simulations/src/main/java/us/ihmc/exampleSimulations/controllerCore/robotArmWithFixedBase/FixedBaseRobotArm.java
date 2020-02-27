package us.ihmc.exampleSimulations.controllerCore.robotArmWithFixedBase;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.Axis;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.geometry.RotationalInertiaCalculator;
import us.ihmc.robotics.math.filters.FilteredVelocityYoFrameVector;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.simulationconstructionset.*;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Random;

import static us.ihmc.robotics.math.filters.FilteredVelocityYoFrameVector.createFilteredVelocityYoFrameVector;

public class FixedBaseRobotArm extends Robot
{
   private static final double SMALL_MASS = 0.2;
   private static final Vector3D X_AXIS = new Vector3D(1.0, 0.0, 0.0);
   private static final Vector3D Y_AXIS = new Vector3D(0.0, 1.0, 0.0);
   private static final Vector3D Z_AXIS = new Vector3D(0.0, 0.0, 1.0);
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double gravity = 9.81;

   private final RigidBodyBasics elevator;
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
   private final RigidBodyBasics shoulderYawLink;
   private final RevoluteJoint shoulderRoll;
   private final RigidBodyBasics shoulderRollLink;
   private final RevoluteJoint shoulderPitch;
   private final RigidBodyBasics upperArm;
   private final RevoluteJoint elbowPitch;
   private final RigidBodyBasics lowerArm;
   private final RevoluteJoint wristPitch;
   private final RigidBodyBasics wristPitchLink;
   private final RevoluteJoint wristRoll;
   private final RigidBodyBasics wristRollLink;
   private final RevoluteJoint wristYaw;
   private final RigidBodyBasics hand;

   private final RigidBodyTransform controlFrameTransform = new RigidBodyTransform(new AxisAngle(), new Vector3D(0.0, 0.0, 0.4));
   private final ReferenceFrame handControlFrame;
   private final KinematicPoint controlFrameTracker = new KinematicPoint("controlFrameTracker", controlFrameTransform.getTranslationVector(), this);
   private final YoDouble dummyAlpha = new YoDouble("dummy", new YoVariableRegistry("dummy"));
   private final FilteredVelocityYoFrameVector controlFrameLinearAcceleration;
   private final FilteredVelocityYoFrameVector controlFrameAngularAcceleration;

   private final Map<OneDoFJointBasics, OneDegreeOfFreedomJoint> idToSCSJointMap = new HashMap<>();

   public FixedBaseRobotArm(double dt)
   {
      super(FixedBaseRobotArm.class.getSimpleName());
      this.setGravity(0.0, 0.0, -gravity);

      elevator = new RigidBody("elevator", worldFrame);

      shoulderYaw = new RevoluteJoint("shoulderYaw", elevator, shoulderYawOffset, Z_AXIS);
      shoulderYawLink = new RigidBody("shoulderYawLink", shoulderYaw, createNullMOI(), SMALL_MASS, new RigidBodyTransform());
      shoulderRoll = new RevoluteJoint("shoulderRoll", shoulderYawLink, shoulderRollOffset, X_AXIS);
      shoulderRollLink = new RigidBody("shoulderRollLink", shoulderRoll, createNullMOI(), SMALL_MASS, new RigidBodyTransform());
      shoulderPitch = new RevoluteJoint("shoulderPitch", shoulderRollLink, shoulderPitchOffset, Y_AXIS);

      upperArm = new RigidBody("upperArm", shoulderPitch, upperArmInertia, upperArmMass, upperArmCoM);

      elbowPitch = new RevoluteJoint("elbowPitch", upperArm, elbowPitchOffset, Y_AXIS);

      lowerArm = new RigidBody("lowerArm", elbowPitch, lowerArmInertia, lowerArmMass, lowerArmCoM);

      wristPitch = new RevoluteJoint("wristPitch", lowerArm, wristPitchOffset, Y_AXIS);
      wristPitchLink = new RigidBody("wristPitchLink", wristPitch, createNullMOI(), SMALL_MASS, new RigidBodyTransform());
      wristRoll = new RevoluteJoint("wristRoll", wristPitchLink, wristRollOffset, X_AXIS);
      wristRollLink = new RigidBody("wristRollLink", wristRoll, createNullMOI(), SMALL_MASS, new RigidBodyTransform());
      wristYaw = new RevoluteJoint("wristYaw", wristRollLink, wristYawOffset, Z_AXIS);

      hand = new RigidBody("hand", wristYaw, handInertia, handMass, handCoM);

      handControlFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("handControlFrame", hand.getBodyFixedFrame(), controlFrameTransform);

      controlFrameLinearAcceleration = createFilteredVelocityYoFrameVector("controlFrameLinearAcceleration", "", dummyAlpha, dt, yoVariableRegistry,
                                                                           controlFrameTracker.getYoVelocity());
      controlFrameAngularAcceleration = createFilteredVelocityYoFrameVector("controlFrameAngularAcceleration", "", dummyAlpha, dt, yoVariableRegistry,
                                                                            controlFrameTracker.getYoAngularVelocity());

      setJointLimits();

      setupSCSRobot();
   }

   public void updateControlFrameAcceleration()
   {
      controlFrameLinearAcceleration.update();
      controlFrameAngularAcceleration.update();
   }

   private void setJointLimits()
   {
      RevoluteJoint[] allJoints = MultiBodySystemTools.filterJoints(MultiBodySystemTools.collectSubtreeJoints(elevator), RevoluteJoint.class);
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

      double b_damp = 0.025;
      scsShoulderYaw.setDamping(b_damp);
      scsShoulderRoll.setDamping(b_damp);
      scsShoulderPitch.setDamping(b_damp);
      scsElbowPitch.setDamping(b_damp);
      scsWristPitch.setDamping(b_damp);
      scsWristRoll.setDamping(b_damp);
      scsWristYaw.setDamping(b_damp);

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

      scsWristYaw.addKinematicPoint(controlFrameTracker);

      idToSCSJointMap.put(shoulderYaw, scsShoulderYaw);
      idToSCSJointMap.put(shoulderRoll, scsShoulderRoll);
      idToSCSJointMap.put(shoulderPitch, scsShoulderPitch);
      idToSCSJointMap.put(elbowPitch, scsElbowPitch);
      idToSCSJointMap.put(wristPitch, scsWristPitch);
      idToSCSJointMap.put(wristRoll, scsWristRoll);
      idToSCSJointMap.put(wristYaw, scsWristYaw);
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

   public void updateJointTaus(JointDesiredOutputListReadOnly lowLevelOneDoFJointDesiredDataHolder)
   {
      for (Entry<OneDoFJointBasics, OneDegreeOfFreedomJoint> pair : idToSCSJointMap.entrySet())
      {
         OneDoFJointBasics oneDoFJoint = pair.getKey();
         JointDesiredOutputReadOnly data = lowLevelOneDoFJointDesiredDataHolder.getJointDesiredOutput(oneDoFJoint); 
         if (data.hasDesiredTorque())
         {
            double tau = data.getDesiredTorque();
            pair.getKey().setTau(tau);
            pair.getValue().setTau(tau);
         }
      }
   }

   public void updateSCSRobotJointConfiguration(JointDesiredOutputListReadOnly lowLevelOneDoFJointDesiredDataHolder)
   {
      for (Entry<OneDoFJointBasics, OneDegreeOfFreedomJoint> pair : idToSCSJointMap.entrySet())
      {
         OneDoFJointBasics oneDoFJoint = pair.getKey();
         JointDesiredOutputReadOnly data = lowLevelOneDoFJointDesiredDataHolder.getJointDesiredOutput(oneDoFJoint); 

         if (data.hasDesiredPosition())
         {
            double q = data.getDesiredPosition();
            pair.getValue().setQ(q);
         }

         if (data.hasDesiredVelocity())
         {
            double qd = data.getDesiredVelocity();
            pair.getValue().setQd(qd);
         }
      }
   }

   public void updateIDRobot()
   {
      for (Entry<OneDoFJointBasics, OneDegreeOfFreedomJoint> pair : idToSCSJointMap.entrySet())
      {
         pair.getKey().setQ(pair.getValue().getQ());
         pair.getKey().setQd(pair.getValue().getQD());
      }
      elevator.updateFramesRecursively();
   }

   public void setRandomConfiguration()
   {
      Random random = new Random();

      for (Entry<OneDoFJointBasics, OneDegreeOfFreedomJoint> pair : idToSCSJointMap.entrySet())
      {
         OneDegreeOfFreedomJoint joint = pair.getValue();

         double lowerLimit = joint.getJointLowerLimit();
         if (!Double.isFinite(lowerLimit))
            lowerLimit = -Math.PI;
         double upperLimit = joint.getJointUpperLimit();
         if (!Double.isFinite(upperLimit))
            upperLimit = Math.PI;

         joint.setQ(RandomNumbers.nextDouble(random, lowerLimit, upperLimit));
      }
   }

   public RigidBodyBasics getElevator()
   {
      return elevator;
   }

   public RigidBodyBasics getHand()
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

   public RevoluteJoint getShoulderYaw()
   {
      return shoulderYaw;
   }

   public RevoluteJoint getShoulderRoll()
   {
      return shoulderRoll;
   }

   public RevoluteJoint getShoulderPitch()
   {
      return shoulderPitch;
   }

   public RevoluteJoint getElbowPitch()
   {
      return elbowPitch;
   }

   public RevoluteJoint getWristPitch()
   {
      return wristPitch;
   }

   public RevoluteJoint getWristRoll()
   {
      return wristRoll;
   }

   public RevoluteJoint getWristYaw()
   {
      return wristYaw;
   }

   public OneDegreeOfFreedomJoint getSCSJointFromIDJoint(OneDoFJointBasics oneDoFJoint)
   {
      return idToSCSJointMap.get(oneDoFJoint);
   }
}
