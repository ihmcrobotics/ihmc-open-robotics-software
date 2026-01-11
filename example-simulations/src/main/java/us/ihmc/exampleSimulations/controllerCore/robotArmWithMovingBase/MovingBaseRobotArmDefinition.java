package us.ihmc.exampleSimulations.controllerCore.robotArmWithMovingBase;

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
import us.ihmc.scs2.definition.robot.RevoluteJointDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.simulationconstructionset.KinematicPoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.OneDegreeOfFreedomJoint;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SliderJoint;
import us.ihmc.yoVariables.euclid.filters.FilteredFiniteDifferenceYoFrameVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;

import java.util.HashMap;
import java.util.Map;
import java.util.Map.Entry;
import java.util.Random;

public class MovingBaseRobotArmDefinition extends RobotDefinition
{
   private static final double SMALL_MASS = 0.2;
   private static final Vector3D X_AXIS = new Vector3D(1.0, 0.0, 0.0);
   private static final Vector3D Y_AXIS = new Vector3D(0.0, 1.0, 0.0);
   private static final Vector3D Z_AXIS = new Vector3D(0.0, 0.0, 1.0);

   public static final String elevatorName = "elevator";
   private final Vector3D baseXOffset = new Vector3D(0.0, 0.0, 0.3);

   public static final String baseName = "base";
   private static final double baseMass = 10.0;
   private static final Matrix3D baseInertia = RotationalInertiaCalculator.getRotationalInertiaFromDiagonal(1.0, 1.0, 1.0);

   private final Vector3D shoulderYawOffset = new Vector3D(0.0, 0.0, 0.0);
   private final Vector3D shoulderRollOffset = new Vector3D(0.0, 0.0, 0.0);
   private final Vector3D shoulderPitchOffset = new Vector3D(0.0, 0.0, 0.0);

   public static final String elbowPitchName = "elbowPitch";
   public static final String wristYawName = "wristYaw";

   private final double upperArmMass = 2.2;
   private final double upperArmLength = 0.35;
   private final double upperArmRadius = 0.025;
   private final Matrix3D upperArmInertia = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidCylinder(upperArmMass,
                                                                                                                  upperArmRadius,
                                                                                                                  upperArmRadius,
                                                                                                                  Axis3D.Z);
   private final Vector3D upperArmCoM = new Vector3D(0.0, 0.0, upperArmLength / 2.0);

   private final Vector3D elbowPitchOffset = new Vector3D(0.0, 0.0, upperArmLength);

   private final double lowerArmMass = 2.2;
   private final double lowerArmLength = 0.35;
   private final double lowerArmRadius = 0.025;
   private final Matrix3D lowerArmInertia = RotationalInertiaCalculator.getRotationalInertiaMatrixOfSolidCylinder(lowerArmMass,
                                                                                                                  lowerArmRadius,
                                                                                                                  lowerArmRadius,
                                                                                                                  Axis3D.Z);
   private final Vector3D lowerArmCoM = new Vector3D(0.0, 0.0, lowerArmLength / 2.0);

   private final Vector3D wristPitchOffset = new Vector3D(0.0, 0.0, lowerArmLength);
   private final Vector3D wristRollOffset = new Vector3D(0.0, 0.0, 0.0);
   private final Vector3D wristYawOffset = new Vector3D(0.0, 0.0, 0.0);

   public static final String handName = "hand";
   private final double handMass = 1.2;
   private final Matrix3D handInertia = RotationalInertiaCalculator.getRotationalInertiaFromRadiiOfGyration(handMass, 0.08, 0.08, 0.08);
   private final Vector3D handCoM = new Vector3D(0.0, 0.0, 0.05);

   private final RigidBodyTransform controlFrameTransform = new RigidBodyTransform(new AxisAngle(), new Vector3D(0.0, 0.0, 0.4));

   public MovingBaseRobotArmDefinition()
   {
      super("movingArm");

      RigidBodyDefinition elevator = new RigidBodyDefinition(elevatorName);
      setRootBodyDefinition(elevator);

      // Moving (and actuated) base
      RevoluteJointDefinition baseX = new RevoluteJointDefinition("baseX", baseXOffset, X_AXIS);
      RevoluteJointDefinition baseY = new RevoluteJointDefinition("baseY", new Vector3D(), Y_AXIS);
      RevoluteJointDefinition baseZ = new RevoluteJointDefinition("baseZ", new Vector3D(), Z_AXIS);

      RigidBodyDefinition baseXLink = new RigidBodyDefinition("baseXLink");
      baseXLink.getMomentOfInertia().set(createNullMOI());
      baseXLink.setMass(SMALL_MASS);
      RigidBodyDefinition baseYLink = new RigidBodyDefinition("baseYLink");
      baseYLink.getMomentOfInertia().set(createNullMOI());
      baseYLink.setMass(SMALL_MASS);
      RigidBodyDefinition base = new RigidBodyDefinition(baseName);
      base.getMomentOfInertia().set(baseInertia);
      base.setMass(baseMass);
      setupBaseGraphic(0.3, ColorDefinitions.DarkKhaki(), base);

      elevator.addChildJoint(baseX);

      baseX.setSuccessor(baseXLink);
      baseXLink.addChildJoint(baseY);
      baseY.setSuccessor(baseYLink);
      baseYLink.addChildJoint(baseZ);
      baseZ.setSuccessor(base);

      // Arm
      RevoluteJointDefinition shoulderYaw = new RevoluteJointDefinition("shoulderYaw", shoulderYawOffset, Z_AXIS);
      RevoluteJointDefinition shoulderRoll = new RevoluteJointDefinition("shoulderRoll", shoulderRollOffset, X_AXIS);
      RevoluteJointDefinition shoulderPitch = new RevoluteJointDefinition("shoulderPitch", shoulderPitchOffset, Y_AXIS);
      RevoluteJointDefinition elbowPitch = new RevoluteJointDefinition(elbowPitchName, elbowPitchOffset, Y_AXIS);
      RevoluteJointDefinition wristPitch = new RevoluteJointDefinition("wristPitch", wristPitchOffset, Y_AXIS);
      RevoluteJointDefinition wristRoll = new RevoluteJointDefinition("wristRoll", wristRollOffset, X_AXIS);
      RevoluteJointDefinition wristYaw = new RevoluteJointDefinition(wristYawName, wristYawOffset, Z_AXIS);

      RigidBodyDefinition shoulderYawLink = new RigidBodyDefinition("shoulderYawLink");
      shoulderYawLink.getMomentOfInertia().set(createNullMOI());
      shoulderYawLink.setMass(SMALL_MASS);

      RigidBodyDefinition shoulderRollLink = new RigidBodyDefinition("shoulderRollLink");
      shoulderRollLink.getMomentOfInertia().set(createNullMOI());
      shoulderRollLink.setMass(SMALL_MASS);

      RigidBodyDefinition upperArm = new RigidBodyDefinition("upperArm");
      upperArm.getMomentOfInertia().set(upperArmInertia);
      upperArm.setMass(upperArmMass);
      upperArm.setCenterOfMassOffset(upperArmCoM);
      setupArmGraphic(upperArmLength, upperArmRadius, ColorDefinitions.Red(), upperArm);

      RigidBodyDefinition lowerArm = new RigidBodyDefinition("lowerArm");
      lowerArm.getMomentOfInertia().set(lowerArmInertia);
      lowerArm.setMass(lowerArmMass);
      lowerArm.setCenterOfMassOffset(lowerArmCoM);
      setupArmGraphic(lowerArmLength, lowerArmRadius, ColorDefinitions.Green(), lowerArm);

      RigidBodyDefinition wristPitchLink = new RigidBodyDefinition("wristPitchLink");
      wristPitchLink.getMomentOfInertia().set(createNullMOI());
      wristPitchLink.setMass(SMALL_MASS);

      RigidBodyDefinition wristRollLink = new RigidBodyDefinition("wristRollLink");
      wristRollLink.getMomentOfInertia().set(createNullMOI());
      wristRollLink.setMass(SMALL_MASS);

      RigidBodyDefinition hand = new RigidBodyDefinition(handName);
      hand.getMomentOfInertia().set(handInertia);
      hand.setMass(handMass);
      hand.setCenterOfMassOffset(handCoM);
      setupHandGraphics(hand);

      base.addChildJoint(shoulderYaw);
      shoulderYaw.setSuccessor(shoulderYawLink);
      shoulderYawLink.addChildJoint(shoulderRoll);
      shoulderRoll.setSuccessor(shoulderRollLink);
      shoulderRollLink.addChildJoint(shoulderPitch);
      shoulderPitch.setSuccessor(upperArm);
      upperArm.addChildJoint(elbowPitch);
      elbowPitch.setSuccessor(lowerArm);
      lowerArm.addChildJoint(wristPitch);
      wristPitch.setSuccessor(wristPitchLink);
      wristPitchLink.addChildJoint(wristRoll);
      wristRoll.setSuccessor(wristRollLink);
      wristRollLink.addChildJoint(wristYaw);
      wristYaw.setSuccessor(hand);
   }

   private static void setupBaseGraphic(double size, ColorDefinition color, RigidBodyDefinition base)
   {
      VisualDefinitionFactory visualDefinitionFactory = new VisualDefinitionFactory();
      double height = 0.005;
      visualDefinitionFactory.appendTranslation(0.0, 0.0, -height);
      visualDefinitionFactory.addBox(size, size, height, color);
      base.addVisualDefinitions(visualDefinitionFactory.getVisualDefinitions());
   }

   private static void setupArmGraphic(double length, double radius, ColorDefinition color, RigidBodyDefinition arm)
   {
      VisualDefinitionFactory visualDefinitionFactory = new VisualDefinitionFactory();
      visualDefinitionFactory.addSphere(1.2 * radius, ColorDefinitions.Grey());
      visualDefinitionFactory.addCylinder(length, radius, color);
      arm.addVisualDefinitions(visualDefinitionFactory.getVisualDefinitions());
   }

   private void setupHandGraphics(RigidBodyDefinition hand)
   {
      VisualDefinitionFactory visualDefinitionFactory = new VisualDefinitionFactory();
      visualDefinitionFactory.addSphere(0.025, ColorDefinitions.Grey());
      visualDefinitionFactory.appendTranslation(handCoM);
      visualDefinitionFactory.addEllipsoid(0.04, 0.01, 0.1);
      visualDefinitionFactory.appendTransform(controlFrameTransform);
      visualDefinitionFactory.addCoordinateSystem(0.1);

      hand.addVisualDefinitions(visualDefinitionFactory.getVisualDefinitions());
   }

   private static Matrix3D createNullMOI()
   {
      Matrix3D momentOfInertia = new Matrix3D();
      momentOfInertia.setIdentity();
      momentOfInertia.scale(1.0e-4);
      return momentOfInertia;
   }
}
