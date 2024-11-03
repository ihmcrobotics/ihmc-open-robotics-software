package us.ihmc.exampleSimulations.controllerCore.robotArmWithMovingBase;

import static us.ihmc.robotics.math.filters.FilteredVelocityYoFrameVector.createFilteredVelocityYoFrameVector;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Random;

import us.ihmc.commons.RandomNumbers;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.axisAngle.AxisAngle;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.referenceFrame.tools.ReferenceFrameTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.mecano.multiBodySystem.PrismaticJoint;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.geometry.RotationalInertiaCalculator;
import us.ihmc.robotics.math.filters.FilteredVelocityYoFrameVector;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.simulationconstructionset.KinematicPoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SliderJoint;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

public class MovingBaseRobotArm extends Robot
{
   private static final double SMALL_MASS = 0.2;
   private static final Vector3D X_AXIS = new Vector3D(1.0, 0.0, 0.0);
   private static final Vector3D Y_AXIS = new Vector3D(0.0, 1.0, 0.0);
   private static final Vector3D Z_AXIS = new Vector3D(0.0, 0.0, 1.0);
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private static final double gravity = 9.81;

   private final RigidBodyBasics elevator;

   private final Vector3D baseXOffset = new Vector3D(0.0, 0.0, 0.3);

   private final double baseMass = 10.0;
   private final Matrix3D baseInertia = RotationalInertiaCalculator.getRotationalInertiaFromDiagonal(1.0, 1.0, 1.0);

   private final Vector3D shoulderYawOffset = new Vector3D(0.0, 0.0, 0.0);
   private final Vector3D shoulderRollOffset = new Vector3D(0.0, 0.0, 0.0);
   private final Vector3D shoulderPitchOffset = new Vector3D(0.0, 0.0, 0.0);

   private final double upperArmMass = 2.2;
   private final double upperArmLength = 0.35;
   private final double upperArmRadius = 0.025;
   private final Matrix3D upperArmInertia = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidCylinder(upperArmMass, upperArmRadius, upperArmRadius,
                                                                                                                  Axis3D.Z);
   private final Vector3D upperArmCoM = new Vector3D(0.0, 0.0, upperArmLength / 2.0);

   private final Vector3D elbowPitchOffset = new Vector3D(0.0, 0.0, upperArmLength);

   private final double lowerArmMass = 2.2;
   private final double lowerArmLength = 0.35;
   private final double lowerArmRadius = 0.025;
   private final Matrix3D lowerArmInertia = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidCylinder(lowerArmMass, lowerArmRadius, lowerArmRadius,
                                                                                                                  Axis3D.Z);
   private final Vector3D lowerArmCoM = new Vector3D(0.0, 0.0, lowerArmLength / 2.0);

   private final Vector3D wristPitchOffset = new Vector3D(0.0, 0.0, lowerArmLength);
   private final Vector3D wristRollOffset = new Vector3D(0.0, 0.0, 0.0);
   private final Vector3D wristYawOffset = new Vector3D(0.0, 0.0, 0.0);

   private final double handMass = 1.2;
   private final Matrix3D handInertia = RotationalInertiaCalculator.getRotationalInertiaFromRadiiOfGyration(handMass, 0.08, 0.08, 0.08);
   private final Vector3D handCoM = new Vector3D(0.0, 0.0, 0.05);

   private final PrismaticJoint baseX;
   private final RigidBodyBasics baseXLink;
   private final PrismaticJoint baseY;
   private final RigidBodyBasics baseYLink;
   private final PrismaticJoint baseZ;
   private final RigidBodyBasics base;
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
   private final KinematicPoint controlFrameTracker = new KinematicPoint("controlFrameTracker", controlFrameTransform.getTranslation(), this);
   private final YoDouble dummyAlpha = new YoDouble("dummy", new YoRegistry("dummy"));
   private final FilteredVelocityYoFrameVector controlFrameLinearAcceleration;
   private final FilteredVelocityYoFrameVector controlFrameAngularAcceleration;

   private final Map<OneDoFJointBasics, OneDegreeOfFreedomJoint> idToSCSJointMap = new HashMap<>();

   public MovingBaseRobotArm(double dt)
   {
      super(MovingBaseRobotArm.class.getSimpleName());
      this.setGravity(0.0, 0.0, -gravity);

      elevator = new RigidBody("elevator", worldFrame);

      // Moving (and actuated) base 
      baseX = new PrismaticJoint("baseX", elevator, baseXOffset, X_AXIS);
      baseXLink = new RigidBody("baseXLink", baseX, createNullMOI(), SMALL_MASS, new RigidBodyTransform());
      baseY = new PrismaticJoint("baseY", baseXLink, new Vector3D(), Y_AXIS);
      baseYLink = new RigidBody("baseYLink", baseY, createNullMOI(), SMALL_MASS, new RigidBodyTransform());
      baseZ = new PrismaticJoint("baseZ", baseYLink, new Vector3D(), Z_AXIS);
      base = new RigidBody("base", baseZ, baseInertia, baseMass, new RigidBodyTransform());
      
      // Arm
      shoulderYaw = new RevoluteJoint("shoulderYaw", base, shoulderYawOffset, Z_AXIS);
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

      handControlFrame = ReferenceFrameTools.constructFrameWithUnchangingTransformToParent("handControlFrame", hand.getBodyFixedFrame(), controlFrameTransform);

      controlFrameLinearAcceleration = createFilteredVelocityYoFrameVector("controlFrameLinearAcceleration", "", dummyAlpha, dt, yoRegistry,
                                                                           controlFrameTracker.getYoVelocity());
      controlFrameAngularAcceleration = createFilteredVelocityYoFrameVector("controlFrameAngularAcceleration", "", dummyAlpha, dt, yoRegistry,
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
      SliderJoint scsBaseX = new SliderJoint("baseX", baseXOffset, this, Axis3D.X);
      SliderJoint scsBaseY = new SliderJoint("baseY", new Vector3D(), this, Axis3D.Y);
      SliderJoint scsBaseZ = new SliderJoint("baseZ", new Vector3D(), this, Axis3D.Z);
      PinJoint scsShoulderYaw = new PinJoint("shoulderYaw", shoulderYawOffset, this, Axis3D.Z);
      PinJoint scsShoulderRoll = new PinJoint("shoulderRoll", shoulderRollOffset, this, Axis3D.X);
      PinJoint scsShoulderPitch = new PinJoint("shoulderPitch", shoulderPitchOffset, this, Axis3D.Y);
      PinJoint scsElbowPitch = new PinJoint("elbowPitch", elbowPitchOffset, this, Axis3D.Y);
      PinJoint scsWristPitch = new PinJoint("wristPitch", wristPitchOffset, this, Axis3D.Y);
      PinJoint scsWristRoll = new PinJoint("wristRoll", wristRollOffset, this, Axis3D.X);
      PinJoint scsWristYaw = new PinJoint("wristYaw", wristYawOffset, this, Axis3D.Z);

      double b_damp = 0.0;
      scsBaseX.setDamping(b_damp);
      scsBaseY.setDamping(b_damp);
      scsBaseZ.setDamping(b_damp);
      scsShoulderYaw.setDamping(b_damp);
      scsShoulderRoll.setDamping(b_damp);
      scsShoulderPitch.setDamping(b_damp);
      scsElbowPitch.setDamping(b_damp);
      scsWristPitch.setDamping(b_damp);
      scsWristRoll.setDamping(b_damp);
      scsWristYaw.setDamping(b_damp);

      Link scsBaseXLink = new Link("baseXLink");
      scsBaseXLink.setMass(SMALL_MASS);
      scsBaseXLink.setMomentOfInertia(createNullMOI());
      scsBaseXLink.setComOffset(0.0, 0.0, 0.0);

      Link scsBaseYLink = new Link("baseYLink");
      scsBaseYLink.setMass(SMALL_MASS);
      scsBaseYLink.setMomentOfInertia(createNullMOI());
      scsBaseYLink.setComOffset(0.0, 0.0, 0.0);
      
      Link scsBase = new Link("base");
      scsBase.setMass(baseMass);
      scsBase.setMomentOfInertia(baseInertia);
      scsBase.setComOffset(0.0, 0.0, 0.0);
      scsBase.setLinkGraphics(createBaseGraphic(0.3, YoAppearance.AluminumMaterial()));
      
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

      addRootJoint(scsBaseX);
      scsBaseX.setLink(scsBaseXLink);
      scsBaseX.addJoint(scsBaseY);
      scsBaseY.setLink(scsBaseYLink);
      scsBaseY.addJoint(scsBaseZ);
      scsBaseZ.setLink(scsBase);
      scsBaseZ.addJoint(scsShoulderYaw);
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

      idToSCSJointMap.put(baseX, scsBaseX);
      idToSCSJointMap.put(baseY, scsBaseY);
      idToSCSJointMap.put(baseZ, scsBaseZ);
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
   
   private Graphics3DObject createBaseGraphic(double size, AppearanceDefinition appearance)
   {
      Graphics3DObject graphics3dObject = new Graphics3DObject();
      double height = 0.005;
      graphics3dObject.translate(0.0, 0.0, -height);
      graphics3dObject.addCube(size, size, height, appearance);
      return graphics3dObject;
   }

   private Matrix3D createNullMOI()
   {
      Matrix3D momentOfInertia = new Matrix3D();
      momentOfInertia.setIdentity();
      momentOfInertia.scale(1.0e-4);
      return momentOfInertia;
   }

   public void updateSCSRobotJointTaus(JointDesiredOutputListReadOnly lowLevelOneDoFJointDesiredDataHolder)
   {
      for (Entry<OneDoFJointBasics, OneDegreeOfFreedomJoint> pair : idToSCSJointMap.entrySet())
      {
         OneDoFJointBasics oneDoFJoint = pair.getKey();
         JointDesiredOutputReadOnly data = lowLevelOneDoFJointDesiredDataHolder.getJointDesiredOutput(oneDoFJoint); 

         if (data.hasDesiredTorque())
         {
            double tau = data.getDesiredTorque();
            pair.getValue().setTau(tau);
            pair.getKey().setTau(tau);
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

   public RigidBodyBasics getBase()
   {
      return base;
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
