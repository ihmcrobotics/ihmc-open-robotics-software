package us.ihmc.commonWalkingControlModules.virtualModelControl;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;

import us.ihmc.commonWalkingControlModules.bipedSupportPolygons.ListOfPointsContactableFoot;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ContactableBodiesFactory;
import us.ihmc.commonWalkingControlModules.momentumBasedController.GeometricJacobianHolder;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.matrix.RotationMatrix;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple2D.Point2D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.controllers.YoPIDGains;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.DoubleYoVariable;
import us.ihmc.robotics.geometry.FramePose;
import us.ihmc.robotics.geometry.FrameVector;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.robotics.math.frames.YoFrameVector;
import us.ihmc.robotics.math.frames.YoWrench;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.RobotSpecificJointNames;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.referenceFrames.CenterOfMassReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RevoluteJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.screwTheory.RigidBodyInertia;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;
import us.ihmc.robotics.screwTheory.Wrench;
import us.ihmc.robotics.sensors.ContactSensorDefinition;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Joint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.RobotTools.SCSRobotFromInverseDynamicsRobotModel;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.bambooTools.SimulationTestingParameters;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;

public class VirtualModelControllerTestHelper
{
   private static final Vector3D X = new Vector3D(1.0, 0.0, 0.0);
   private static final Vector3D Y = new Vector3D(0.0, 1.0, 0.0);
   private static final Vector3D Z = new Vector3D(0.0, 0.0, 1.0);

   public static final double toFootCenterX = 0.05042;
   public static final double toFootCenterY = 0.082;

   public static final double footWidth = 0.11;
   public static final double footBack = 0.085;
   public static final double footLength = 0.22;
   public static final double toeWidth = 0.085;
   public static final double ankleHeight = 0.0875;

   public static final double POUNDS = 1.0 / 2.2;    // Pound to Kg conversion.
   public static final double INCHES = 0.0254;    // Inch to Meter Conversion.

   public static final double PELVIS_HEIGHT = 1.0;
   public static final double PELVIS_RAD = 0.1;

   public static final double HIP_WIDTH = 0.2;

   public static final double HIP_DIFFERENTIAL_HEIGHT = 0.05;
   public static final double HIP_DIFFERENTIAL_WIDTH = 0.075;

   public static final double THIGH_LENGTH = 23.29 * INCHES;
   public static final double THIGH_RAD = 0.05;
   public static final double THIGH_MASS = 6.7 * POUNDS;

   public static final double SHIN_LENGTH = 23.29 * INCHES;
   public static final double SHIN_RAD = 0.03;
   public static final double SHIN_MASS = 6.7 * POUNDS;

   public static final double ANKLE_DIFFERENTIAL_HEIGHT = 0.025;
   public static final double ANKLE_DIFFERENTIAL_WIDTH = 0.0375;

   public static final double FOOT_LENGTH = 0.08;
   public static final double FOOT_COM_OFFSET = 3.0 * INCHES;
   public static final double FOOT_RAD = 0.05;
   public static final double FOOT_MASS = 3.0 * POUNDS;

   private static final Random random = new Random(100L);

   private VirtualModelControllerTestHelper()
   {
   }

   public static void createVirtualModelControlTest(SCSRobotFromInverseDynamicsRobotModel robotModel, FullRobotModel controllerModel, ReferenceFrame centerOfMassFrame,
         List<RigidBody> endEffectors, List<Vector3D> desiredForces, List<Vector3D> desiredTorques, List<ExternalForcePoint> externalForcePoints, DenseMatrix64F selectionMatrix, SimulationTestingParameters simulationTestingParameters) throws Exception
   {
      double simulationDuration = 20.0;

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      YoVariableRegistry registry = new YoVariableRegistry("robert");

      GeometricJacobianHolder geometricJacobianHolder = new GeometricJacobianHolder();
      VirtualModelController virtualModelController = new VirtualModelController(geometricJacobianHolder, controllerModel.getElevator(),
            controllerModel.getOneDoFJoints(), registry, yoGraphicsListRegistry);

      List<ReferenceFrame> endEffectorFrames = new ArrayList<>();
      List<FramePose> desiredEndEffectorPoses = new ArrayList<>();

      List<YoWrench> desiredWrenches = new ArrayList<>();
      List<ForcePointController> forcePointControllers = new ArrayList<>();

      for (int i = 0; i < endEffectors.size(); i++)
      {
         RigidBody endEffector = endEffectors.get(i);
         ReferenceFrame endEffectorFrame = endEffector.getBodyFixedFrame();

         FramePose desiredEndEffectorPose = new FramePose(endEffectorFrame);
         desiredEndEffectorPose.setToZero();
         desiredEndEffectorPose.changeFrame(ReferenceFrame.getWorldFrame());

         endEffectorFrames.add(endEffectorFrame);
         desiredEndEffectorPoses.add(desiredEndEffectorPose);

         virtualModelController.registerControlledBody(endEffector, controllerModel.getElevator());

         Wrench desiredWrench = new Wrench(endEffectorFrame, endEffectorFrame);
         Vector3D desiredForce = desiredForces.get(i);
         Vector3D desiredTorque = desiredTorques.get(i);
         FrameVector forceFrameVector = new FrameVector(ReferenceFrame.getWorldFrame(), desiredForce);
         FrameVector torqueFrameVector = new FrameVector(ReferenceFrame.getWorldFrame(), desiredTorque);
         forceFrameVector.changeFrame(endEffectorFrame);
         torqueFrameVector.changeFrame(endEffectorFrame);
         desiredWrench.set(forceFrameVector, torqueFrameVector);
         desiredWrench.changeFrame(ReferenceFrame.getWorldFrame());

         YoWrench yoDesiredWrench = new YoWrench("desiredWrench" + i, endEffectorFrame, ReferenceFrame.getWorldFrame(), registry);
         yoDesiredWrench.set(desiredWrench);

         desiredWrenches.add(yoDesiredWrench);

         Vector3D contactForce = new Vector3D();
         Vector3D contactTorque = new Vector3D();
         contactForce.set(desiredForce);
         contactTorque.set(desiredTorque);
         contactForce.scale(-1.0);
         contactTorque.scale(-1.0);

         ForcePointController forcePointController = new ForcePointController("" + i, externalForcePoints.get(i), endEffectorFrame, desiredEndEffectorPose);
         forcePointController.setInitialForce(contactForce, contactTorque);
         forcePointController.setLinearGains(250, 10, 0);
         forcePointControllers.add(forcePointController);
      }

      DummyArmController armController = new DummyArmController(robotModel, controllerModel, controllerModel.getOneDoFJoints(), forcePointControllers, virtualModelController,
            geometricJacobianHolder, endEffectors, desiredWrenches, selectionMatrix);

      SimulationConstructionSet scs = new SimulationConstructionSet(robotModel, simulationTestingParameters);
      robotModel.setController(armController);
      scs.getRootRegistry().addChild(registry);
      for (ForcePointController forcePointController : forcePointControllers)
         yoGraphicsListRegistry.registerYoGraphicsList(forcePointController.getYoGraphicsList());
      scs.addYoGraphicsListRegistry(yoGraphicsListRegistry);

      BlockingSimulationRunner blockingSimulationRunner = new BlockingSimulationRunner(scs, 1500.0);
      scs.startOnAThread();

      Vector3D currentPosition = new Vector3D();
      Quaternion currentOrientation = new Quaternion();
      Vector3D currentForce = new Vector3D();
      Vector3D currentTorque = new Vector3D();

      List<Vector3D> desiredPositions = new ArrayList<>();
      List<Quaternion> desiredOrientations = new ArrayList<>();

      for (int i = 0; i < endEffectors.size(); i++)
      {
         desiredPositions.add(armController.getDesiredPosition(i));
         desiredOrientations.add(armController.getDesiredOrientation(i));
      }

      // check that the end effector doesn't move, and that the desired force is very close to what we want
      double timeIncrement = 1.0;
      while (scs.getTime() < simulationDuration)
      {
         blockingSimulationRunner.simulateAndBlock(timeIncrement);
         for (int i = 0; i < endEffectors.size(); i++)
         {
            currentPosition.set(armController.getCurrentPosition(i));
            currentOrientation.set(armController.getCurrentOrientation(i));
            currentForce.set(armController.getCurrentForce(i));
            currentTorque.set(armController.getCurrentTorque(i));

            EuclidCoreTestTools.assertTuple3DEquals("", currentPosition, desiredPositions.get(i), 0.01);
            EuclidCoreTestTools.assertQuaternionEquals(currentOrientation, desiredOrientations.get(i), 0.01);
            EuclidCoreTestTools.assertTuple3DEquals("", desiredForces.get(i), currentForce, 0.5);
            EuclidCoreTestTools.assertTuple3DEquals("", desiredTorques.get(i), currentTorque, 0.5);
         }
      }
   }

   public static RobotLegs createRobotLeg(double gravity)
   {
      RobotLegs robotLeg = new RobotLegs("robotLegs");
      robotLeg.setGravity(gravity);
      HashMap<InverseDynamicsJoint, Joint> jointMap = new HashMap<>();

      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", ReferenceFrame.getWorldFrame(), new RigidBodyTransform());
      RigidBody elevator = new RigidBody("elevator", elevatorFrame);

      FloatingJoint floatingJoint = new FloatingJoint("pelvis", new Vector3D(), robotLeg);
      robotLeg.addRootJoint(floatingJoint);
      SixDoFJoint rootJoint = new SixDoFJoint("pelvis", elevator, elevatorFrame);
      jointMap.put(rootJoint, floatingJoint);

      Link pelvisLink = pelvis();
      floatingJoint.setLink(pelvisLink);
      RigidBody pelvisBody = copyLinkAsRigidBody(pelvisLink, rootJoint, "pelvis");

      Vector3D leftHipYawOffset = new Vector3D(0.0, HIP_WIDTH, 0.0);
      Vector3D rightHipYawOffset = new Vector3D(0.0, -HIP_WIDTH, 0.0);
      PinJoint l_hip_yaw = new PinJoint("l_leg_hpz", leftHipYawOffset, robotLeg, Axis.Z);
      PinJoint r_hip_yaw = new PinJoint("r_leg_hpz", rightHipYawOffset, robotLeg, Axis.Z);
      l_hip_yaw.setQ(random.nextDouble());
      r_hip_yaw.setQ(random.nextDouble());

      Link l_hip_differential = hip_differential();
      Link r_hip_differential = hip_differential();
      l_hip_yaw.setLink(l_hip_differential);
      r_hip_yaw.setLink(r_hip_differential);
      floatingJoint.addJoint(l_hip_yaw);
      floatingJoint.addJoint(r_hip_yaw);

      RevoluteJoint l_leg_hpz = ScrewTools.addRevoluteJoint("l_leg_hpz", pelvisBody, leftHipYawOffset, Z);
      RevoluteJoint r_leg_hpz = ScrewTools.addRevoluteJoint("r_leg_hpz", pelvisBody, rightHipYawOffset, Z);
      l_leg_hpz.setQ(l_hip_yaw.getQYoVariable().getDoubleValue());
      r_leg_hpz.setQ(r_hip_yaw.getQYoVariable().getDoubleValue());
      RigidBody leftHipDifferentialBody = copyLinkAsRigidBody(l_hip_differential, l_leg_hpz, "l_hip_differential");
      RigidBody rightHipDifferentialBody = copyLinkAsRigidBody(r_hip_differential, r_leg_hpz, "r_hip_differential");
      jointMap.put(l_leg_hpz, l_hip_yaw);
      jointMap.put(r_leg_hpz, r_hip_yaw);

      Vector3D leftHipRollOffset = new Vector3D();
      Vector3D rightHipRollOffset = new Vector3D();
      PinJoint l_hip_roll = new PinJoint("l_leg_hpx", leftHipRollOffset, robotLeg, Axis.X);
      PinJoint r_hip_roll = new PinJoint("r_leg_hpx", rightHipRollOffset, robotLeg, Axis.X);
      l_hip_roll.setQ(random.nextDouble());
      r_hip_roll.setQ(random.nextDouble());

      Link l_hip_differential2 = hip_differential();
      Link r_hip_differential2 = hip_differential();
      l_hip_roll.setLink(l_hip_differential2);
      r_hip_roll.setLink(r_hip_differential2);
      l_hip_yaw.addJoint(l_hip_roll);
      r_hip_yaw.addJoint(r_hip_roll);

      RevoluteJoint l_leg_hpx = ScrewTools.addRevoluteJoint("l_leg_hpx", leftHipDifferentialBody, leftHipRollOffset, X);
      RevoluteJoint r_leg_hpx = ScrewTools.addRevoluteJoint("r_leg_hpx", rightHipDifferentialBody, rightHipRollOffset, X);
      l_leg_hpx.setQ(l_hip_roll.getQYoVariable().getDoubleValue());
      r_leg_hpx.setQ(r_hip_roll.getQYoVariable().getDoubleValue());
      RigidBody leftHipDifferentialBody2 = copyLinkAsRigidBody(l_hip_differential2, l_leg_hpx, "l_hip_differential");
      RigidBody rightHipDifferentialBody2 = copyLinkAsRigidBody(r_hip_differential2, r_leg_hpx, "r_hip_differential");
      jointMap.put(l_leg_hpx, l_hip_roll);
      jointMap.put(r_leg_hpx, r_hip_roll);

      Vector3D leftHipPitchOffset = new Vector3D();
      Vector3D rightHipPitchOffset = new Vector3D();
      PinJoint l_hip_pitch = new PinJoint("l_leg_hpy", leftHipPitchOffset, robotLeg, Axis.Y);
      PinJoint r_hip_pitch = new PinJoint("r_leg_hpy", rightHipPitchOffset, robotLeg, Axis.Y);
      l_hip_pitch.setQ(random.nextDouble());
      r_hip_pitch.setQ(random.nextDouble());

      Link leftThigh = thigh();
      Link rightThigh = thigh();
      l_hip_pitch.setLink(leftThigh);
      r_hip_pitch.setLink(rightThigh);
      l_hip_roll.addJoint(l_hip_pitch);
      r_hip_roll.addJoint(r_hip_pitch);

      RevoluteJoint l_leg_hpy = ScrewTools.addRevoluteJoint("l_leg_hpy", leftHipDifferentialBody2, leftHipPitchOffset, Y);
      RevoluteJoint r_leg_hpy = ScrewTools.addRevoluteJoint("r_leg_hpy", rightHipDifferentialBody2, rightHipPitchOffset, Y);
      l_leg_hpy.setQ(l_hip_pitch.getQYoVariable().getDoubleValue());
      r_leg_hpy.setQ(r_hip_pitch.getQYoVariable().getDoubleValue());
      RigidBody leftThighBody = copyLinkAsRigidBody(leftThigh, l_leg_hpy, "l_thigh");
      RigidBody rightThighBody = copyLinkAsRigidBody(rightThigh, r_leg_hpy, "r_thigh");
      jointMap.put(l_leg_hpy, l_hip_pitch);
      jointMap.put(r_leg_hpy, r_hip_pitch);

      Vector3D leftKneePitchOffset = new Vector3D(0.0, 0.0, -THIGH_LENGTH);
      Vector3D rightKneePitchOffset = new Vector3D(0.0, 0.0, -THIGH_LENGTH);
      PinJoint l_knee_pitch = new PinJoint("l_leg_kny", leftKneePitchOffset, robotLeg, Axis.Y);
      PinJoint r_knee_pitch = new PinJoint("r_leg_kny", rightKneePitchOffset, robotLeg, Axis.Y);
      l_knee_pitch.setQ(random.nextDouble());
      r_knee_pitch.setQ(random.nextDouble());

      Link l_shin = shin();
      Link r_shin = shin();
      l_knee_pitch.setLink(l_shin);
      r_knee_pitch.setLink(r_shin);
      l_hip_pitch.addJoint(l_knee_pitch);
      r_hip_pitch.addJoint(r_knee_pitch);

      RevoluteJoint l_leg_kny = ScrewTools.addRevoluteJoint("l_leg_kny", leftThighBody, leftKneePitchOffset, Y);
      RevoluteJoint r_leg_kny = ScrewTools.addRevoluteJoint("r_leg_kny", rightThighBody, rightKneePitchOffset, Y);
      l_leg_kny.setQ(l_knee_pitch.getQYoVariable().getDoubleValue());
      r_leg_kny.setQ(r_knee_pitch.getQYoVariable().getDoubleValue());
      RigidBody leftShinBody = copyLinkAsRigidBody(l_shin, l_leg_kny, "l_shin");
      RigidBody rightShinBody = copyLinkAsRigidBody(r_shin, r_leg_kny, "r_shin");
      jointMap.put(l_leg_kny, l_knee_pitch);
      jointMap.put(r_leg_kny, r_knee_pitch);

      Vector3D leftAnklePitchOffset = new Vector3D(0.0, 0.0, -SHIN_LENGTH);
      Vector3D rightAnklePitchOffset = new Vector3D(0.0, 0.0, -SHIN_LENGTH);
      PinJoint l_ankle_pitch = new PinJoint("l_leg_aky", leftAnklePitchOffset, robotLeg, Axis.Y);
      PinJoint r_ankle_pitch = new PinJoint("r_leg_aky", rightAnklePitchOffset, robotLeg, Axis.Y);
      l_ankle_pitch.setQ(random.nextDouble());
      r_ankle_pitch.setQ(random.nextDouble());

      Link l_ankle_differential = ankle_differential();
      Link r_ankle_differential = ankle_differential();
      l_ankle_pitch.setLink(l_ankle_differential);
      r_ankle_pitch.setLink(r_ankle_differential);
      l_knee_pitch.addJoint(r_ankle_pitch);
      l_knee_pitch.addJoint(r_ankle_pitch);

      RevoluteJoint l_leg_aky = ScrewTools.addRevoluteJoint("l_leg_aky", leftShinBody, leftAnklePitchOffset, Y);
      RevoluteJoint r_leg_aky = ScrewTools.addRevoluteJoint("r_leg_aky", rightShinBody, rightAnklePitchOffset, Y);
      l_leg_aky.setQ(l_ankle_pitch.getQYoVariable().getDoubleValue());
      r_leg_aky.setQ(r_ankle_pitch.getQYoVariable().getDoubleValue());
      RigidBody leftAnkleDifferentialBody = copyLinkAsRigidBody(l_ankle_differential, l_leg_aky, "l_ankle_differential");
      RigidBody rightAnkleDifferentialBody = copyLinkAsRigidBody(r_ankle_differential, r_leg_aky, "r_ankle_differential");
      jointMap.put(l_leg_aky, l_ankle_pitch);
      jointMap.put(r_leg_aky, r_ankle_pitch);

      Vector3D leftAnkleRollOffset = new Vector3D();
      Vector3D rightAnkleRollOffset = new Vector3D();
      PinJoint l_ankle_roll = new PinJoint("l_leg_akx", leftAnkleRollOffset, robotLeg, Axis.X);
      PinJoint r_ankle_roll = new PinJoint("r_leg_akx", rightAnkleRollOffset, robotLeg, Axis.X);
      l_ankle_roll.setQ(random.nextDouble());
      r_ankle_roll.setQ(random.nextDouble());

      Link l_foot = foot();
      Link r_foot = foot();
      l_ankle_roll.setLink(l_foot);
      r_ankle_roll.setLink(r_foot);
      l_ankle_pitch.addJoint(l_ankle_roll);
      r_ankle_pitch.addJoint(r_ankle_roll);

      RevoluteJoint l_leg_akx = ScrewTools.addRevoluteJoint("l_leg_akx", leftAnkleDifferentialBody, leftAnkleRollOffset, X);
      RevoluteJoint r_leg_akx = ScrewTools.addRevoluteJoint("r_leg_akx", rightAnkleDifferentialBody, rightAnkleRollOffset, X);
      l_leg_akx.setQ(l_ankle_roll.getQYoVariable().getDoubleValue());
      r_leg_akx.setQ(r_ankle_roll.getQYoVariable().getDoubleValue());
      RigidBody leftFootBody = copyLinkAsRigidBody(l_foot, l_leg_akx, "l_foot");
      RigidBody rightFootBody = copyLinkAsRigidBody(r_foot, r_leg_akx, "r_foot");
      jointMap.put(l_leg_akx, l_ankle_roll);
      jointMap.put(r_leg_akx, r_ankle_roll);

      RigidBodyTransform leftSoleToAnkleFrame = TransformTools.createTranslationTransform(footLength / 2.0 - footBack + toFootCenterX,
            toFootCenterY, -ankleHeight);
      RigidBodyTransform rightSoleToAnkleFrame = TransformTools.createTranslationTransform(footLength / 2.0 - footBack + toFootCenterX,
            -toFootCenterY, -ankleHeight);
      ReferenceFrame leftSoleFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent("Left_Sole",
            leftFootBody.getBodyFixedFrame(), leftSoleToAnkleFrame);
      ReferenceFrame rightSoleFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent("Right_Sole",
            rightFootBody.getBodyFixedFrame(), rightSoleToAnkleFrame);

      SideDependentList<RigidBody> feet = new SideDependentList<>();
      feet.put(RobotSide.LEFT, leftFootBody);
      feet.put(RobotSide.RIGHT, rightFootBody);
      SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();
      soleFrames.put(RobotSide.LEFT, leftSoleFrame);
      soleFrames.put(RobotSide.RIGHT, rightSoleFrame);

      OneDoFJoint[] joints = {l_leg_hpz, l_leg_hpx, l_leg_hpy, l_leg_kny, l_leg_akx, l_leg_aky, r_leg_hpz, r_leg_hpx, r_leg_hpy, r_leg_kny, r_leg_akx, r_leg_aky};

      robotLeg.setPelvis(pelvisBody);
      robotLeg.setFeet(feet);
      robotLeg.setElevator(elevator);
      robotLeg.setRootJoint(rootJoint);
      robotLeg.setSoleFrames(soleFrames);
      robotLeg.createReferenceFrames();
      robotLeg.setOneDoFJoints(joints);

      return robotLeg;
   }

   private static Link pelvis()
   {
      AppearanceDefinition pelvisAppearance = YoAppearance.Blue();

      Link ret = new Link("pelvis");

      ret.setMass(100.0);
      ret.setMomentOfInertia(1.0, 1.0, 1.0);
      ret.setComOffset(0.0, 0.0, 0.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();

      linkGraphics.addCoordinateSystem(1.0);
      linkGraphics.addCylinder(PELVIS_HEIGHT, PELVIS_RAD, pelvisAppearance);
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private static Link hip_differential()
   {
      Link ret = new Link("hip_differential");

      ret.setMass(0.1);
      ret.setMomentOfInertia(0.005, 0.005, 0.005);
      ret.setComOffset(0.0, 0.0, 0.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();

      linkGraphics.addCube(HIP_DIFFERENTIAL_WIDTH, HIP_DIFFERENTIAL_WIDTH, HIP_DIFFERENTIAL_HEIGHT);
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }


   private static Link thigh()
   {
      AppearanceDefinition thighApp = YoAppearance.Green();

      Link ret = new Link("thigh");

      ret.setMass(THIGH_MASS);    // 2.35);
      ret.setMomentOfInertia(0.0437, 0.0437, 0.0054);
      ret.setComOffset(0.0, 0.0, -THIGH_LENGTH / 2.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();

      linkGraphics.addCylinder(THIGH_LENGTH, THIGH_RAD, thighApp);
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }


   private static Link shin()
   {
      AppearanceDefinition shinApp = YoAppearance.Red();

      Link ret = new Link("shin");

      ret.setMass(SHIN_MASS);    // 0.864);
      ret.setMomentOfInertia(0.00429, 0.00429, 0.00106);
      ret.setComOffset(0.0, 0.0, -SHIN_LENGTH / 2.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();

      linkGraphics.addCylinder(SHIN_LENGTH, SHIN_RAD, shinApp);
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private static Link ankle_differential()
   {
      Link ret = new Link("ankle_differential");

      ret.setMass(0.1);
      ret.setMomentOfInertia(0.005, 0.005, 0.005);
      ret.setComOffset(0.0, 0.0, 0.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();

      linkGraphics.addCube(ANKLE_DIFFERENTIAL_WIDTH, ANKLE_DIFFERENTIAL_WIDTH, ANKLE_DIFFERENTIAL_HEIGHT);
      ret.setLinkGraphics(linkGraphics);

      return ret;
   }

   private static Link foot()
   {
      AppearanceDefinition footApp = YoAppearance.PlaneMaterial();

      Link ret = new Link("foot");

      ret.setMass(FOOT_MASS);    // 0.207);
      ret.setMomentOfInertia(0.00041, 0.00041, 0.00001689);
      ret.setComOffset(FOOT_COM_OFFSET, 0.0, 0.0);

      Graphics3DObject linkGraphics = new Graphics3DObject();

      linkGraphics.addCylinder(FOOT_LENGTH, FOOT_RAD, footApp);

      linkGraphics.translate(0.05, 0.0, FOOT_LENGTH);
      linkGraphics.addCube(0.02, 0.1, 0.1, YoAppearance.Black());

      linkGraphics.translate(-0.1, 0.0, 0.0);
      linkGraphics.addCube(0.02, 0.1, 0.1, YoAppearance.Black());

      ret.setLinkGraphics(linkGraphics);

      return ret;
   }
   private static RigidBody copyLinkAsRigidBody(Link link, InverseDynamicsJoint currentInverseDynamicsJoint, String bodyName)
   {
      Vector3D comOffset = new Vector3D();
      link.getComOffset(comOffset);
      Matrix3D momentOfInertia = new Matrix3D();
      link.getMomentOfInertia(momentOfInertia);
      ReferenceFrame nextFrame = createOffsetFrame(currentInverseDynamicsJoint, comOffset, bodyName);
      nextFrame.update();
      RigidBodyInertia inertia = new RigidBodyInertia(nextFrame, momentOfInertia, link.getMass());
      RigidBody rigidBody = new RigidBody(bodyName, inertia, currentInverseDynamicsJoint);

      return rigidBody;
   }

   public static ReferenceFrame createOffsetFrame(InverseDynamicsJoint currentInverseDynamicsJoint, Vector3D offset, String frameName)
   {
      ReferenceFrame parentFrame = currentInverseDynamicsJoint.getFrameAfterJoint();
      RigidBodyTransform transformToParent = new RigidBodyTransform();
      transformToParent.setTranslationAndIdentityRotation(offset);
      ReferenceFrame beforeJointFrame = ReferenceFrame.constructBodyFrameWithUnchangingTransformToParent(frameName, parentFrame, transformToParent);

      return beforeJointFrame;
   }

   public static void compareWrenches(Wrench inputWrench, Wrench outputWrench, DenseMatrix64F selectionMatrix)
   {
      inputWrench.getBodyFrame().checkReferenceFrameMatch(outputWrench.getBodyFrame());
      outputWrench.changeFrame(inputWrench.getExpressedInFrame());
      inputWrench.getExpressedInFrame().checkReferenceFrameMatch(outputWrench.getExpressedInFrame());

      DenseMatrix64F inputWrenchMatrix = new DenseMatrix64F(Wrench.SIZE, 1);
      DenseMatrix64F outputWrenchMatrix = new DenseMatrix64F(Wrench.SIZE, 1);
      DenseMatrix64F selectedValues = new DenseMatrix64F(Wrench.SIZE, 1);

      inputWrench.getMatrix(inputWrenchMatrix);
      outputWrench.getMatrix(outputWrenchMatrix);

      double epsilon = 1e-4;
      int taskSize = selectionMatrix.getNumRows();
      int colIndex = 0;
      for (int i = 0; i < taskSize; i++)
         for (int j = colIndex; j < Wrench.SIZE; j++)
            if (selectionMatrix.get(i, j) == 1)
               selectedValues.set(j, 0, 1);

      // only compare the selected values
      for (int i = 0; i < Wrench.SIZE; i++)
      {
         if (selectedValues.get(i, 0) == 1)
            Assert.assertEquals(inputWrenchMatrix.get(i, 0), outputWrenchMatrix.get(i, 0), epsilon);
      }
   }

   public static void compareWrenches(Wrench inputWrench, Wrench outputWrench)
   {
      inputWrench.getBodyFrame().checkReferenceFrameMatch(outputWrench.getBodyFrame());
      outputWrench.changeFrame(inputWrench.getExpressedInFrame());
      inputWrench.getExpressedInFrame().checkReferenceFrameMatch(outputWrench.getExpressedInFrame());

      double epsilon = 1e-4;
      EuclidCoreTestTools.assertTuple3DEquals(inputWrench.getAngularPartCopy(), outputWrench.getAngularPartCopy(), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(inputWrench.getLinearPartCopy(), outputWrench.getLinearPartCopy(), epsilon);
   }

   public static RobotArm createRobotArm()
   {
      return new RobotArm();
   }

   public static ForkedRobotArm createForkedRobotArm()
   {
      return new ForkedRobotArm();
   }

   public static PlanarRobotArm createPlanarArm()
   {
      return new PlanarRobotArm();
   }

   public static PlanarForkedRobotArm createPlanarForkedRobotArm()
   {
      return new PlanarForkedRobotArm();
   }

   public static class PlanarRobotArm implements FullRobotModel
   {
      private final SCSRobotFromInverseDynamicsRobotModel scsRobotArm;

      private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      private final ReferenceFrame elevatorFrame;
      private final ReferenceFrame centerOfMassFrame;

      private final RigidBody elevator;
      private final RigidBody upperArm;
      private final RigidBody lowerArm;
      private final RigidBody hand;

      private ExternalForcePoint externalForcePoint;

      private final OneDoFJoint[] oneDoFJoints;

      public PlanarRobotArm()
      {
         elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new RigidBodyTransform());
         elevator = new RigidBody("elevator", elevatorFrame);
         centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMassFrame", ReferenceFrame.getWorldFrame(), elevator);

         upperArm = createUpperArm(elevator);
         lowerArm = createLowerArm(upperArm);
         hand = createHand(lowerArm);

         scsRobotArm = new SCSRobotFromInverseDynamicsRobotModel("robotArm", elevator.getChildrenJoints().get(0));
         scsRobotArm.setGravity(0);
         scsRobotArm.updateJointPositions_ID_to_SCS();
         scsRobotArm.update();

         addLinkGraphics();
         addForcePoint();

         oneDoFJoints = ScrewTools.createOneDoFJointPath(elevator, hand);
      }

      private void addLinkGraphics()
      {
         AppearanceDefinition upperArmAppearance = YoAppearance.Red();
         AppearanceDefinition lowerArmAppearance = YoAppearance.Red();
         AppearanceDefinition handAppearance = YoAppearance.Red();

         Link upperArmLink = scsRobotArm.getLink("upperArm");
         Link lowerArmLink = scsRobotArm.getLink("lowerArm");
         Link handLink = scsRobotArm.getLink("hand");

         Graphics3DObject upperArmGraphics = new Graphics3DObject();
         Graphics3DObject lowerArmGraphics = new Graphics3DObject();
         Graphics3DObject handGraphics = new Graphics3DObject();

         upperArmGraphics.addCylinder(THIGH_LENGTH, THIGH_RAD, upperArmAppearance);
         lowerArmGraphics.addCylinder(SHIN_LENGTH, SHIN_RAD, lowerArmAppearance);
         handGraphics.addCylinder(SHIN_LENGTH / 2.0, FOOT_RAD, handAppearance);

         upperArmLink.setLinkGraphics(upperArmGraphics);
         lowerArmLink.setLinkGraphics(lowerArmGraphics);
         handLink.setLinkGraphics(handGraphics);
      }

      private void addForcePoint()
      {
         externalForcePoint = new ExternalForcePoint("handForcePoint", scsRobotArm.getLink("hand").getComOffset(), scsRobotArm);

         scsRobotArm.getLink("hand").getParentJoint().addExternalForcePoint(externalForcePoint);
      }

      public ExternalForcePoint getExternalForcePoint()
      {
         return externalForcePoint;
      }

      public SCSRobotFromInverseDynamicsRobotModel getSCSRobotArm()
      {
         return scsRobotArm;
      }

      public RobotSpecificJointNames getRobotSpecificJointNames()
      {
         return null;
      }

      public void updateFrames()
      {
         worldFrame.update();
         elevator.updateFramesRecursively();
      }

      public ReferenceFrame getWorldFrame()
      {
         return worldFrame;
      }

      public ReferenceFrame getElevatorFrame()
      {
         return elevatorFrame;
      }

      public ReferenceFrame getCenterOfMassFrame()
      {
         return centerOfMassFrame;
      }

      public ReferenceFrame getHandFrame()
      {
         return hand.getBodyFixedFrame();
      }

      public InverseDynamicsJoint getBaseJoint()
      {
         return upperArm.getParentJoint();
      }

      public SixDoFJoint getRootJoint()
      {
         return null;
      }

      public RigidBody getElevator()
      {
         return elevator;
      }

      public RigidBody getHand()
      {
         return hand;
      }

      public OneDoFJoint getSpineJoint(SpineJointName spineJointName)
      {
         return null;
      }

      public OneDoFJoint getNeckJoint(NeckJointName neckJointName)
      {
         return null;
      }

      public InverseDynamicsJoint getLidarJoint(String lidarName)
      {
         return null;
      }

      public RigidBody getPelvis()
      {
         return null;
      }

      public RigidBody getChest()
      {
         return null;
      }

      public RigidBody getHead()
      {
         return null;
      }

      public OneDoFJoint[] getOneDoFJoints()
      {
         return oneDoFJoints;
      }

      public void getOneDoFJoints(ArrayList<OneDoFJoint> oneDoFJointsToPack)
      {
         List<OneDoFJoint> list = Arrays.asList(oneDoFJoints);

         for (int i = 0; i < list.size(); i++)
            oneDoFJointsToPack.set(i, list.get(i));
      }

      public IMUDefinition[] getIMUDefinitions()
      {
         return null;
      }

      public ForceSensorDefinition[] getForceSensorDefinitions()
      {
         return null;
      }

      public ContactSensorDefinition[] getContactSensorDefinitions()
      {
         return null;
      }

      private RigidBody createUpperArm(RigidBody parentBody)
      {
         RevoluteJoint joint = ScrewTools.addRevoluteJoint("shoulderPitch_y", parentBody, new Vector3D(0.0, 0.0, 0.0), Y);
         joint.setQ(Math.toRadians(20));

         Vector3D comOffset = new Vector3D(0.0, 0.0, THIGH_LENGTH / 2.0);
         ReferenceFrame nextFrame = createOffsetFrame(joint, comOffset, "upperArmFrame");
         nextFrame.update();

         Matrix3D momentOfInertia = new Matrix3D();
         momentOfInertia.setElement(0, 0, 0.0437);
         momentOfInertia.setElement(1, 1, 0.0437);
         momentOfInertia.setElement(2, 2, 0.0054);

         RigidBodyInertia inertia = new RigidBodyInertia(nextFrame, momentOfInertia, THIGH_MASS);
         RigidBody rigidBody = new RigidBody("upperArm", inertia, joint);

         return rigidBody;
      }

      private RigidBody createLowerArm(RigidBody parentBody)
      {
         RevoluteJoint joint = ScrewTools.addRevoluteJoint("elbow_y", parentBody, new Vector3D(0.0, 0.0, THIGH_LENGTH), Y);
         joint.setQ(Math.toRadians(40));

         Vector3D comOffset = new Vector3D(0.0, 0.0, SHIN_LENGTH / 2.0);
         ReferenceFrame nextFrame = createOffsetFrame(joint, comOffset, "lowerArmFrame");
         nextFrame.update();

         Matrix3D momentOfInertia = new Matrix3D();
         momentOfInertia.setElement(0, 0, 0.0437);
         momentOfInertia.setElement(1, 1, 0.0437);
         momentOfInertia.setElement(2, 2, 0.0054);

         RigidBodyInertia inertia = new RigidBodyInertia(nextFrame, momentOfInertia, SHIN_MASS);
         RigidBody rigidBody = new RigidBody("lowerArm", inertia, joint);

         return rigidBody;
      }

      private RigidBody createHand(RigidBody parentBody)
      {
         RevoluteJoint joint = ScrewTools.addRevoluteJoint("wristPitch_y", parentBody, new Vector3D(0.0, 0.0, SHIN_LENGTH), Y);
         joint.setQ(Math.toRadians(30));

         Vector3D comOffset = new Vector3D(0.0, 0.0, SHIN_LENGTH / 4.0);
         ReferenceFrame nextFrame = createOffsetFrame(joint, comOffset, "handFrame");
         nextFrame.update();

         Matrix3D momentOfInertia = new Matrix3D();
         momentOfInertia.setElement(0, 0, 0.0437);
         momentOfInertia.setElement(1, 1, 0.0437);
         momentOfInertia.setElement(2, 2, 0.0054);

         RigidBodyInertia inertia = new RigidBodyInertia(nextFrame, momentOfInertia, FOOT_MASS);
         RigidBody rigidBody = new RigidBody("hand", inertia, joint);

         return rigidBody;
      }

      @Override
      public OneDoFJoint[] getControllableOneDoFJoints()
      {
         return null;
      }

      @Override
      public void getControllableOneDoFJoints(ArrayList<OneDoFJoint> oneDoFJointsToPack)
      {
      }

      @Override
      public OneDoFJoint getOneDoFJointByName(String name)
      {
         return null;
      }

      @Override
      public RigidBody getEndEffector(Enum<?> segmentEnum)
      {
         return null;
      }

      @Override
      public double getTotalMass()
      {
         return Double.NaN;
      }

      @Override
      public ReferenceFrame getLidarBaseFrame(String name)
      {
         return null;
      }

      @Override
      public ReferenceFrame getCameraFrame(String name)
      {
         return null;
      }

      @Override
      public ReferenceFrame getHeadBaseFrame()
      {
         return null;
      }

      @Override
      public Map<String, OneDoFJoint> getOneDoFJointsAsMap()
      {
         return null;
      }

      @Override
      public void getOneDoFJointsFromRootToHere(OneDoFJoint oneDoFJointAtEndOfChain, ArrayList<OneDoFJoint> oneDoFJointsToPack)
      {
      }

      @Override
      public RigidBodyTransform getLidarBaseToSensorTransform(String name)
      {
         return null;
      }
   }

   public static class RobotArm implements FullRobotModel
   {
      private final SCSRobotFromInverseDynamicsRobotModel scsRobotArm;

      private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      private final ReferenceFrame elevatorFrame;
      private final ReferenceFrame centerOfMassFrame;

      private final RigidBody elevator;
      private final RigidBody hand;
      private final RigidBody shoulderDifferentialYaw;

      private ExternalForcePoint externalForcePoint;

      private final OneDoFJoint[] oneDoFJoints;

      public RobotArm()
      {
         elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new RigidBodyTransform());
         elevator = new RigidBody("elevator", elevatorFrame);
         centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMass", ReferenceFrame.getWorldFrame(), elevator);

         shoulderDifferentialYaw = createDifferential("shoulderDifferential", elevator, new Vector3D(), Z);
         RigidBody shoulderDifferentialRoll = createDifferential("shoulderDifferential", shoulderDifferentialYaw, new Vector3D(), X);

         RigidBody upperArm = createUpperArm(shoulderDifferentialRoll);

         RigidBody lowerArm = createLowerArm(upperArm);

         RigidBody wristDifferentialRoll = createDifferential("wristDifferential", lowerArm, new Vector3D(0.0, 0.0, SHIN_LENGTH), X);
         //RigidBody wristDifferentialYaw = createDifferential("wristDifferential", wristDifferentialRoll, new Vector3d(), Z);

         hand = createHand(wristDifferentialRoll);

         scsRobotArm = new SCSRobotFromInverseDynamicsRobotModel("robotArm", elevator.getChildrenJoints().get(0));
         scsRobotArm.setGravity(0);
         scsRobotArm.updateJointPositions_ID_to_SCS();
         scsRobotArm.update();

         addLinkGraphics();
         addForcePoint();

         oneDoFJoints = ScrewTools.createOneDoFJointPath(elevator, hand);
      }

      private void addLinkGraphics()
      {
         AppearanceDefinition upperArmAppearance = YoAppearance.Red();
         AppearanceDefinition lowerArmAppearance = YoAppearance.Red();
         AppearanceDefinition handAppearance = YoAppearance.Red();

         Link upperArmLink = scsRobotArm.getLink("upperArm");
         Link lowerArmLink = scsRobotArm.getLink("lowerArm");
         Link handLink = scsRobotArm.getLink("hand");

         Graphics3DObject upperArmGraphics = new Graphics3DObject();
         Graphics3DObject lowerArmGraphics = new Graphics3DObject();
         Graphics3DObject handGraphics = new Graphics3DObject();

         upperArmGraphics.addCylinder(THIGH_LENGTH, THIGH_RAD, upperArmAppearance);
         lowerArmGraphics.addCylinder(SHIN_LENGTH, SHIN_RAD, lowerArmAppearance);
         handGraphics.addCylinder(SHIN_LENGTH / 2.0, FOOT_RAD, handAppearance);

         upperArmLink.setLinkGraphics(upperArmGraphics);
         lowerArmLink.setLinkGraphics(lowerArmGraphics);
         handLink.setLinkGraphics(handGraphics);
      }

      private void addForcePoint()
      {
         externalForcePoint = new ExternalForcePoint("handForcePoint", scsRobotArm.getLink("hand").getComOffset(), scsRobotArm);

         scsRobotArm.getLink("hand").getParentJoint().addExternalForcePoint(externalForcePoint);
      }

      public ExternalForcePoint getExternalForcePoint()
      {
         return externalForcePoint;
      }

      public SCSRobotFromInverseDynamicsRobotModel getSCSRobotArm()
      {
         return scsRobotArm;
      }

      public RobotSpecificJointNames getRobotSpecificJointNames()
      {
         return null;
      }

      public void updateFrames()
      {
         worldFrame.update();
         elevator.updateFramesRecursively();
      }

      public ReferenceFrame getWorldFrame()
      {
         return worldFrame;
      }

      public ReferenceFrame getElevatorFrame()
      {
         return elevatorFrame;
      }

      public ReferenceFrame getCenterOfMassFrame()
      {
         return centerOfMassFrame;
      }

      public ReferenceFrame getHandFrame()
      {
         return hand.getBodyFixedFrame();
      }

      public InverseDynamicsJoint getBaseJoint()
      {
         return shoulderDifferentialYaw.getParentJoint();
      }

      public SixDoFJoint getRootJoint()
      {
         return null;
      }

      public RigidBody getElevator()
      {
         return elevator;
      }

      public RigidBody getHand()
      {
         return hand;
      }

      public OneDoFJoint getSpineJoint(SpineJointName spineJointName)
      {
         return null;
      }

      public OneDoFJoint getNeckJoint(NeckJointName neckJointName)
      {
         return null;
      }

      public InverseDynamicsJoint getLidarJoint(String lidarName)
      {
         return null;
      }

      public RigidBody getPelvis()
      {
         return null;
      }

      public RigidBody getChest()
      {
         return null;
      }

      public RigidBody getHead()
      {
         return null;
      }

      public OneDoFJoint[] getOneDoFJoints()
      {
         return oneDoFJoints;
      }

      public void getOneDoFJoints(ArrayList<OneDoFJoint> oneDoFJointsToPack)
      {
         List<OneDoFJoint> list = Arrays.asList(oneDoFJoints);

         for (int i = 0; i < list.size(); i++)
            oneDoFJointsToPack.set(i, list.get(i));
      }

      public IMUDefinition[] getIMUDefinitions()
      {
         return null;
      }

      public ForceSensorDefinition[] getForceSensorDefinitions()
      {
         return null;
      }

      public ContactSensorDefinition[] getContactSensorDefinitions()
      {
         return null;
      }

      private RigidBody createDifferential(String name, RigidBody parentBody, Vector3D jointOffset, Vector3D jointAxis)
      {
         String jointName;
         if (jointAxis == X)
            jointName = name + "_x";
         else if (jointAxis == Y)
            jointName = name + "_y";
         else
            jointName = name + "_z";
         RevoluteJoint joint = ScrewTools.addRevoluteJoint(jointName, parentBody, jointOffset, jointAxis);
         joint.setQ(random.nextDouble());

         Vector3D comOffset = new Vector3D();
         ReferenceFrame nextFrame = createOffsetFrame(joint, comOffset, name + "Frame");
         nextFrame.update();

         Matrix3D momentOfInertia = new Matrix3D();
         momentOfInertia.setElement(0, 0, 0.005);
         momentOfInertia.setElement(1, 1, 0.005);
         momentOfInertia.setElement(2, 2, 0.005);

         RigidBodyInertia inertia = new RigidBodyInertia(nextFrame, momentOfInertia, 0.1);
         RigidBody rigidBody = new RigidBody(name, inertia, joint);

         return rigidBody;
      }

      private RigidBody createUpperArm(RigidBody parentBody)
      {
         RevoluteJoint joint = ScrewTools.addRevoluteJoint("shoulderPitch_y", parentBody, new Vector3D(0.0, 0.0, 0.0), Y);
         joint.setQ(random.nextDouble());

         Vector3D comOffset = new Vector3D(0.0, 0.0, THIGH_LENGTH / 2.0);
         ReferenceFrame nextFrame = createOffsetFrame(joint, comOffset, "upperArmFrame");
         nextFrame.update();

         Matrix3D momentOfInertia = new Matrix3D();
         momentOfInertia.setElement(0, 0, 0.0437);
         momentOfInertia.setElement(1, 1, 0.0437);
         momentOfInertia.setElement(2, 2, 0.0054);

         RigidBodyInertia inertia = new RigidBodyInertia(nextFrame, momentOfInertia, THIGH_MASS);
         RigidBody rigidBody = new RigidBody("upperArm", inertia, joint);

         return rigidBody;
      }

      private RigidBody createLowerArm(RigidBody parentBody)
      {
         RevoluteJoint joint = ScrewTools.addRevoluteJoint("elbow_y", parentBody, new Vector3D(0.0, 0.0, THIGH_LENGTH), Y);
         joint.setQ(random.nextDouble());

         Vector3D comOffset = new Vector3D(0.0, 0.0, SHIN_LENGTH / 2.0);
         ReferenceFrame nextFrame = createOffsetFrame(joint, comOffset, "lowerArmFrame");
         nextFrame.update();

         Matrix3D momentOfInertia = new Matrix3D();
         momentOfInertia.setElement(0, 0, 0.0437);
         momentOfInertia.setElement(1, 1, 0.0437);
         momentOfInertia.setElement(2, 2, 0.0054);

         RigidBodyInertia inertia = new RigidBodyInertia(nextFrame, momentOfInertia, SHIN_MASS);
         RigidBody rigidBody = new RigidBody("lowerArm", inertia, joint);

         return rigidBody;
      }

      private RigidBody createHand(RigidBody parentBody)
      {
         RevoluteJoint joint = ScrewTools.addRevoluteJoint("wristPitch_y", parentBody, new Vector3D(0.0, 0.0, 0.0), Y);
         joint.setQ(random.nextDouble());

         Vector3D comOffset = new Vector3D(0.0, 0.0, SHIN_LENGTH / 4.0);
         ReferenceFrame nextFrame = createOffsetFrame(joint, comOffset, "handFrame");
         nextFrame.update();

         Matrix3D momentOfInertia = new Matrix3D();
         momentOfInertia.setElement(0, 0, 0.0437);
         momentOfInertia.setElement(1, 1, 0.0437);
         momentOfInertia.setElement(2, 2, 0.0054);

         RigidBodyInertia inertia = new RigidBodyInertia(nextFrame, momentOfInertia, FOOT_MASS);
         RigidBody rigidBody = new RigidBody("hand", inertia, joint);

         return rigidBody;
      }

      @Override
      public OneDoFJoint[] getControllableOneDoFJoints()
      {
         return null;
      }

      @Override
      public void getControllableOneDoFJoints(ArrayList<OneDoFJoint> oneDoFJointsToPack)
      {
      }

      @Override
      public OneDoFJoint getOneDoFJointByName(String name)
      {
         return null;
      }

      @Override
      public RigidBody getEndEffector(Enum<?> segmentEnum)
      {
         return null;
      }

      @Override
      public double getTotalMass()
      {
         return Double.NaN;
      }

      @Override
      public ReferenceFrame getLidarBaseFrame(String name)
      {
         return null;
      }

      @Override
      public ReferenceFrame getCameraFrame(String name)
      {
         return null;
      }

      @Override
      public ReferenceFrame getHeadBaseFrame()
      {
         return null;
      }

      @Override
      public Map<String, OneDoFJoint> getOneDoFJointsAsMap()
      {
         return null;
      }

      @Override
      public void getOneDoFJointsFromRootToHere(OneDoFJoint oneDoFJointAtEndOfChain, ArrayList<OneDoFJoint> oneDoFJointsToPack)
      {
      }

      @Override
      public RigidBodyTransform getLidarBaseToSensorTransform(String name)
      {
         return null;
      }
   }

   public static class ForkedRobotArm implements FullRobotModel
   {
      private final SCSRobotFromInverseDynamicsRobotModel scsRobotArm;

      private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      private final ReferenceFrame elevatorFrame;
      private final ReferenceFrame centerOfMassFrame;

      private final RigidBody elevator;
      private final RigidBody hand1;
      private final RigidBody hand2;
      private final RigidBody shoulderDifferentialYaw;
      private final SideDependentList<RigidBody> hands = new SideDependentList<>();
      private final List<InverseDynamicsJoint> joints = new ArrayList<>();

      private ExternalForcePoint externalForcePoint1;
      private ExternalForcePoint externalForcePoint2;

      private final OneDoFJoint[] oneDoFJoints;

      public ForkedRobotArm()
      {
         elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new RigidBodyTransform());
         elevator = new RigidBody("elevator", elevatorFrame);
         centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMass", ReferenceFrame.getWorldFrame(), elevator);

         shoulderDifferentialYaw = createDifferential("shoulderDifferential", elevator, new Vector3D(), Z);

         RigidBody upperArm = createUpperArm(shoulderDifferentialYaw);
         joints.add(upperArm.getParentJoint());

         RigidBody elbowDifferentialYaw1 = createDifferential("elbowDifferentialYaw1", upperArm, new Vector3D(0.0, 0.0, THIGH_LENGTH), Z);
         RigidBody elbowDifferentialYaw2 = createDifferential("elbowDifferentialYaw2", upperArm, new Vector3D(0.0, 0.0, THIGH_LENGTH), Z);
         RevoluteJoint differential1 = (RevoluteJoint) elbowDifferentialYaw1.getParentJoint();
         RevoluteJoint differential2 = (RevoluteJoint) elbowDifferentialYaw2.getParentJoint();
         differential1.setQ(2.5 * random.nextDouble());
         differential2.setQ(-2.5 * random.nextDouble());
         joints.add(differential1);
         joints.add(differential2);

         RigidBody lowerArm1 = createLowerArm("1", elbowDifferentialYaw1);
         RigidBody lowerArm2 = createLowerArm("2", elbowDifferentialYaw2);
         joints.add(lowerArm1.getParentJoint());
         joints.add(lowerArm2.getParentJoint());

         RigidBody wristDifferentialRoll1 = createDifferential("wristDifferential1", lowerArm1, new Vector3D(0.0, 0.0, SHIN_LENGTH), X);
         RigidBody wristDifferentialRoll2 = createDifferential("wristDifferential2", lowerArm2, new Vector3D(0.0, 0.0, SHIN_LENGTH), X);
         joints.add(wristDifferentialRoll1.getParentJoint());
         joints.add(wristDifferentialRoll2.getParentJoint());

         hand1 = createHand("1", wristDifferentialRoll1);
         hand2 = createHand("2", wristDifferentialRoll2);
         hands.put(RobotSide.LEFT, hand1);
         hands.put(RobotSide.RIGHT, hand2);
         joints.add(hand1.getParentJoint());
         joints.add(hand2.getParentJoint());

         InverseDynamicsJoint[] jointArray = new InverseDynamicsJoint[joints.size()];
         joints.toArray(jointArray);
         oneDoFJoints = ScrewTools.filterJoints(jointArray, OneDoFJoint.class);

         scsRobotArm = new SCSRobotFromInverseDynamicsRobotModel("robotArm", elevator.getChildrenJoints().get(0));
         scsRobotArm.setGravity(0);
         scsRobotArm.updateJointPositions_ID_to_SCS();
         scsRobotArm.update();

         addLinkGraphics();
         addForcePoint();
      }

      private void addLinkGraphics()
      {
         AppearanceDefinition upperArmAppearance = YoAppearance.Red();
         AppearanceDefinition lowerArmAppearance = YoAppearance.Red();
         AppearanceDefinition handAppearance = YoAppearance.Red();

         Link upperArmLink = scsRobotArm.getLink("upperArm");
         Link lowerArmLink1 = scsRobotArm.getLink("lowerArm1");
         Link lowerArmLink2 = scsRobotArm.getLink("lowerArm2");
         Link handLink1 = scsRobotArm.getLink("hand1");
         Link handLink2 = scsRobotArm.getLink("hand2");

         Graphics3DObject upperArmGraphics = new Graphics3DObject();
         Graphics3DObject lowerArmGraphics1 = new Graphics3DObject();
         Graphics3DObject lowerArmGraphics2 = new Graphics3DObject();
         Graphics3DObject handGraphics1 = new Graphics3DObject();
         Graphics3DObject handGraphics2 = new Graphics3DObject();

         upperArmGraphics.addCylinder(THIGH_LENGTH, THIGH_RAD, upperArmAppearance);
         lowerArmGraphics1.addCylinder(SHIN_LENGTH, SHIN_RAD, lowerArmAppearance);
         lowerArmGraphics2.addCylinder(SHIN_LENGTH, SHIN_RAD, lowerArmAppearance);
         handGraphics1.addCylinder(SHIN_LENGTH / 2.0, FOOT_RAD, handAppearance);
         handGraphics2.addCylinder(SHIN_LENGTH / 2.0, FOOT_RAD, handAppearance);

         upperArmLink.setLinkGraphics(upperArmGraphics);
         lowerArmLink1.setLinkGraphics(lowerArmGraphics1);
         lowerArmLink2.setLinkGraphics(lowerArmGraphics2);
         handLink1.setLinkGraphics(handGraphics1);
         handLink2.setLinkGraphics(handGraphics2);
      }

      private void addForcePoint()
      {
         externalForcePoint1 = new ExternalForcePoint("hand1ForcePoint", scsRobotArm.getLink("hand1").getComOffset(), scsRobotArm);
         externalForcePoint2 = new ExternalForcePoint("hand2ForcePoint", scsRobotArm.getLink("hand2").getComOffset(), scsRobotArm);

         scsRobotArm.getLink("hand1").getParentJoint().addExternalForcePoint(externalForcePoint1);
         scsRobotArm.getLink("hand2").getParentJoint().addExternalForcePoint(externalForcePoint2);
      }

      public SideDependentList<ExternalForcePoint> getExternalForcePoints()
      {
         SideDependentList<ExternalForcePoint> externalForcePoints = new SideDependentList<>(externalForcePoint1, externalForcePoint2);

         return externalForcePoints;
      }

      public SCSRobotFromInverseDynamicsRobotModel getSCSRobotArm()
      {
         return scsRobotArm;
      }

      public RobotSpecificJointNames getRobotSpecificJointNames()
      {
         return null;
      }

      public void updateFrames()
      {
         worldFrame.update();
         elevator.updateFramesRecursively();
      }

      public ReferenceFrame getWorldFrame()
      {
         return worldFrame;
      }

      public ReferenceFrame getElevatorFrame()
      {
         return elevatorFrame;
      }

      public ReferenceFrame getCenterOfMassFrame()
      {
         return centerOfMassFrame;
      }

      public ReferenceFrame getHandFrame(RobotSide robotSide)
      {
         return hands.get(robotSide).getBodyFixedFrame();
      }

      public InverseDynamicsJoint getBaseJoint()
      {
         return shoulderDifferentialYaw.getParentJoint();
      }

      public SixDoFJoint getRootJoint()
      {
         return null;
      }

      public RigidBody getElevator()
      {
         return elevator;
      }

      public RigidBody getHand(RobotSide robotSide)
      {
         return hands.get(robotSide);
      }

      public SideDependentList<RigidBody> getHands()
      {
         return hands;
      }
      public OneDoFJoint getSpineJoint(SpineJointName spineJointName)
      {
         return null;
      }

      public OneDoFJoint getNeckJoint(NeckJointName neckJointName)
      {
         return null;
      }

      public InverseDynamicsJoint getLidarJoint(String lidarName)
      {
         return null;
      }

      public RigidBody getPelvis()
      {
         return null;
      }

      public RigidBody getChest()
      {
         return null;
      }

      public RigidBody getHead()
      {
         return null;
      }

      public OneDoFJoint[] getOneDoFJoints()
      {
         return oneDoFJoints;
      }

      public void getOneDoFJoints(ArrayList<OneDoFJoint> oneDoFJointsToPack)
      {
         List<OneDoFJoint> list = Arrays.asList(oneDoFJoints);

         for (int i = 0; i < list.size(); i++)
            oneDoFJointsToPack.set(i, list.get(i));
      }

      public IMUDefinition[] getIMUDefinitions()
      {
         return null;
      }

      public ForceSensorDefinition[] getForceSensorDefinitions()
      {
         return null;
      }

      public ContactSensorDefinition[] getContactSensorDefinitions()
      {
         return null;
      }

      private RigidBody createDifferential(String name, RigidBody parentBody, Vector3D jointOffset, Vector3D jointAxis)
      {
         String jointName;
         if (jointAxis == X)
            jointName = name + "_x";
         else if (jointAxis == Y)
            jointName = name + "_y";
         else
            jointName = name + "_z";
         RevoluteJoint joint = ScrewTools.addRevoluteJoint(jointName, parentBody, jointOffset, jointAxis);
         joint.setQ(random.nextDouble());

         Vector3D comOffset = new Vector3D();
         ReferenceFrame nextFrame = createOffsetFrame(joint, comOffset, name + "Frame");
         nextFrame.update();

         Matrix3D momentOfInertia = new Matrix3D();
         momentOfInertia.setElement(0, 0, 0.005);
         momentOfInertia.setElement(1, 1, 0.005);
         momentOfInertia.setElement(2, 2, 0.005);

         RigidBodyInertia inertia = new RigidBodyInertia(nextFrame, momentOfInertia, 0.1);
         RigidBody rigidBody = new RigidBody(name, inertia, joint);

         return rigidBody;
      }

      private RigidBody createUpperArm(RigidBody parentBody)
      {
         RevoluteJoint joint = ScrewTools.addRevoluteJoint("shoulderPitch_y", parentBody, new Vector3D(0.0, 0.0, 0.0), Y);
         joint.setQ(random.nextDouble());

         Vector3D comOffset = new Vector3D(0.0, 0.0, THIGH_LENGTH / 2.0);
         ReferenceFrame nextFrame = createOffsetFrame(joint, comOffset, "upperArmFrame");
         nextFrame.update();

         Matrix3D momentOfInertia = new Matrix3D();
         momentOfInertia.setElement(0, 0, 0.0437);
         momentOfInertia.setElement(1, 1, 0.0437);
         momentOfInertia.setElement(2, 2, 0.0054);

         RigidBodyInertia inertia = new RigidBodyInertia(nextFrame, momentOfInertia, THIGH_MASS);
         RigidBody rigidBody = new RigidBody("upperArm", inertia, joint);

         return rigidBody;
      }

      private RigidBody createLowerArm(String suffix, RigidBody parentBody)
      {
         RevoluteJoint joint = ScrewTools.addRevoluteJoint("elbow_y_" + suffix, parentBody, new Vector3D(0.0, 0.0, 0.0), Y);
         joint.setQ(random.nextDouble());

         Vector3D comOffset = new Vector3D(0.0, 0.0, SHIN_LENGTH / 2.0);
         ReferenceFrame nextFrame = createOffsetFrame(joint, comOffset, "lowerArmFrame" + suffix);
         nextFrame.update();

         Matrix3D momentOfInertia = new Matrix3D();
         momentOfInertia.setElement(0, 0, 0.0437);
         momentOfInertia.setElement(1, 1, 0.0437);
         momentOfInertia.setElement(2, 2, 0.0054);

         RigidBodyInertia inertia = new RigidBodyInertia(nextFrame, momentOfInertia, SHIN_MASS);
         RigidBody rigidBody = new RigidBody("lowerArm" + suffix, inertia, joint);

         return rigidBody;
      }

      private RigidBody createHand(String suffix, RigidBody parentBody)
      {
         RevoluteJoint joint = ScrewTools.addRevoluteJoint("wristPitch_y_" + suffix, parentBody, new Vector3D(0.0, 0.0, 0.0), Y);
         joint.setQ(random.nextDouble());

         Vector3D comOffset = new Vector3D(0.0, 0.0, SHIN_LENGTH / 4.0);
         ReferenceFrame nextFrame = createOffsetFrame(joint, comOffset, "handFrame" + suffix);
         nextFrame.update();

         Matrix3D momentOfInertia = new Matrix3D();
         momentOfInertia.setElement(0, 0, 0.0437);
         momentOfInertia.setElement(1, 1, 0.0437);
         momentOfInertia.setElement(2, 2, 0.0054);

         RigidBodyInertia inertia = new RigidBodyInertia(nextFrame, momentOfInertia, FOOT_MASS);
         RigidBody rigidBody = new RigidBody("hand" + suffix, inertia, joint);

         return rigidBody;
      }

      @Override
      public OneDoFJoint[] getControllableOneDoFJoints()
      {
         return null;
      }

      @Override
      public void getControllableOneDoFJoints(ArrayList<OneDoFJoint> oneDoFJointsToPack)
      {
      }

      @Override
      public OneDoFJoint getOneDoFJointByName(String name)
      {
         return null;
      }

      @Override
      public RigidBody getEndEffector(Enum<?> segmentEnum)
      {
         return null;
      }

      @Override
      public double getTotalMass()
      {
         return Double.NaN;
      }

      @Override
      public ReferenceFrame getLidarBaseFrame(String name)
      {
         return null;
      }

      @Override
      public ReferenceFrame getCameraFrame(String name)
      {
         return null;
      }

      @Override
      public ReferenceFrame getHeadBaseFrame()
      {
         return null;
      }

      @Override
      public Map<String, OneDoFJoint> getOneDoFJointsAsMap()
      {
         return null;
      }

      @Override
      public void getOneDoFJointsFromRootToHere(OneDoFJoint oneDoFJointAtEndOfChain, ArrayList<OneDoFJoint> oneDoFJointsToPack)
      {
      }

      @Override
      public RigidBodyTransform getLidarBaseToSensorTransform(String name)
      {
         return null;
      }
   }

   public static class PlanarForkedRobotArm implements FullRobotModel
   {
      private final SCSRobotFromInverseDynamicsRobotModel scsRobotArm;

      private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      private final ReferenceFrame elevatorFrame;
      private final ReferenceFrame centerOfMassFrame;

      private final RigidBody elevator;
      private final RigidBody upperArm;
      private final RigidBody hand1;
      private final RigidBody hand2;
      private final SideDependentList<RigidBody> hands = new SideDependentList<>();

      private ExternalForcePoint externalForcePoint1;
      private ExternalForcePoint externalForcePoint2;

      private final OneDoFJoint[] oneDoFJoints;

      public PlanarForkedRobotArm()
      {
         elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new RigidBodyTransform());
         elevator = new RigidBody("elevator", elevatorFrame);
         centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMassFrame", ReferenceFrame.getWorldFrame(), elevator);

         List<InverseDynamicsJoint> joints = new ArrayList<>();

         upperArm = createUpperArm(elevator);
         joints.add(upperArm.getParentJoint());

         RigidBody lowerArm1 = createLowerArm("1", upperArm);
         RigidBody lowerArm2 = createLowerArm("2", upperArm);
         RevoluteJoint elbow1 = (RevoluteJoint) lowerArm1.getParentJoint();
         RevoluteJoint elbow2 = (RevoluteJoint) lowerArm2.getParentJoint();
         elbow1.setQ(1.5 * random.nextDouble());
         elbow2.setQ(-0.5 * random.nextDouble());
         joints.add(elbow1);
         joints.add(elbow2);

         hand1 = createHand("1", lowerArm1);
         hand2 = createHand("2", lowerArm2);
         hands.put(RobotSide.LEFT, hand1);
         hands.put(RobotSide.RIGHT, hand2);
         joints.add(hand1.getParentJoint());
         joints.add(hand2.getParentJoint());

         InverseDynamicsJoint[] jointArray = new InverseDynamicsJoint[joints.size()];
         joints.toArray(jointArray);
         oneDoFJoints = ScrewTools.filterJoints(jointArray, OneDoFJoint.class);

         scsRobotArm = new SCSRobotFromInverseDynamicsRobotModel("robotArm", elevator.getChildrenJoints().get(0));
         scsRobotArm.setGravity(0);
         scsRobotArm.updateJointPositions_ID_to_SCS();
         scsRobotArm.update();

         addLinkGraphics();
         addForcePoint();
      }

      private void addLinkGraphics()
      {
         AppearanceDefinition upperArmAppearance = YoAppearance.Red();
         AppearanceDefinition lowerArmAppearance = YoAppearance.Red();
         AppearanceDefinition handAppearance = YoAppearance.Red();

         Link upperArmLink = scsRobotArm.getLink("upperArm");
         Link lowerArmLink1 = scsRobotArm.getLink("lowerArm1");
         Link lowerArmLink2 = scsRobotArm.getLink("lowerArm2");
         Link handLink1 = scsRobotArm.getLink("hand1");
         Link handLink2 = scsRobotArm.getLink("hand2");

         Graphics3DObject upperArmGraphics = new Graphics3DObject();
         Graphics3DObject lowerArmGraphics1 = new Graphics3DObject();
         Graphics3DObject lowerArmGraphics2 = new Graphics3DObject();
         Graphics3DObject handGraphics1 = new Graphics3DObject();
         Graphics3DObject handGraphics2 = new Graphics3DObject();

         upperArmGraphics.addCylinder(THIGH_LENGTH, THIGH_RAD, upperArmAppearance);
         lowerArmGraphics1.addCylinder(SHIN_LENGTH, SHIN_RAD, lowerArmAppearance);
         lowerArmGraphics2.addCylinder(SHIN_LENGTH, SHIN_RAD, lowerArmAppearance);
         handGraphics1.addCylinder(SHIN_LENGTH / 2.0, FOOT_RAD, handAppearance);
         handGraphics2.addCylinder(SHIN_LENGTH / 2.0, FOOT_RAD, handAppearance);

         upperArmLink.setLinkGraphics(upperArmGraphics);
         lowerArmLink1.setLinkGraphics(lowerArmGraphics1);
         lowerArmLink2.setLinkGraphics(lowerArmGraphics2);
         handLink1.setLinkGraphics(handGraphics1);
         handLink2.setLinkGraphics(handGraphics2);
      }

      private void addForcePoint()
      {
         externalForcePoint1 = new ExternalForcePoint("hand1ForcePoint", scsRobotArm.getLink("hand1").getComOffset(), scsRobotArm);
         externalForcePoint2 = new ExternalForcePoint("hand2ForcePoint", scsRobotArm.getLink("hand2").getComOffset(), scsRobotArm);

         scsRobotArm.getLink("hand1").getParentJoint().addExternalForcePoint(externalForcePoint1);
         scsRobotArm.getLink("hand2").getParentJoint().addExternalForcePoint(externalForcePoint2);
      }

      public SideDependentList<ExternalForcePoint> getExternalForcePoints()
      {
         SideDependentList<ExternalForcePoint> externalForcePoints = new SideDependentList<>(externalForcePoint1, externalForcePoint2);

         return externalForcePoints;
      }

      public SCSRobotFromInverseDynamicsRobotModel getSCSRobotArm()
      {
         return scsRobotArm;
      }

      public RobotSpecificJointNames getRobotSpecificJointNames()
      {
         return null;
      }

      public void updateFrames()
      {
         worldFrame.update();
         elevator.updateFramesRecursively();
      }

      public ReferenceFrame getWorldFrame()
      {
         return worldFrame;
      }

      public ReferenceFrame getElevatorFrame()
      {
         return elevatorFrame;
      }

      public ReferenceFrame getCenterOfMassFrame()
      {
         return centerOfMassFrame;
      }

      public ReferenceFrame getHandFrame(RobotSide robotSide)
      {
         return hands.get(robotSide).getBodyFixedFrame();
      }

      public InverseDynamicsJoint getBaseJoint()
      {
         return upperArm.getParentJoint();
      }

      public SixDoFJoint getRootJoint()
      {
         return null;
      }

      public RigidBody getElevator()
      {
         return elevator;
      }

      public RigidBody getHand(RobotSide robotSide)
      {
         return hands.get(robotSide);
      }

      public SideDependentList<RigidBody> getHands()
      {
         return hands;
      }
      public OneDoFJoint getSpineJoint(SpineJointName spineJointName)
      {
         return null;
      }

      public OneDoFJoint getNeckJoint(NeckJointName neckJointName)
      {
         return null;
      }

      public InverseDynamicsJoint getLidarJoint(String lidarName)
      {
         return null;
      }

      public RigidBody getPelvis()
      {
         return null;
      }

      public RigidBody getChest()
      {
         return null;
      }

      public RigidBody getHead()
      {
         return null;
      }

      public OneDoFJoint[] getOneDoFJoints()
      {
         return oneDoFJoints;
      }

      public void getOneDoFJoints(ArrayList<OneDoFJoint> oneDoFJointsToPack)
      {
         List<OneDoFJoint> list = Arrays.asList(oneDoFJoints);

         for (int i = 0; i < list.size(); i++)
            oneDoFJointsToPack.set(i, list.get(i));
      }

      public IMUDefinition[] getIMUDefinitions()
      {
         return null;
      }

      public ForceSensorDefinition[] getForceSensorDefinitions()
      {
         return null;
      }

      public ContactSensorDefinition[] getContactSensorDefinitions()
      {
         return null;
      }

      private RigidBody createDifferential(String name, RigidBody parentBody, Vector3D jointOffset, Vector3D jointAxis)
      {
         String jointName;
         if (jointAxis == X)
            jointName = name + "_x";
         else if (jointAxis == Y)
            jointName = name + "_y";
         else
            jointName = name + "_z";
         RevoluteJoint joint = ScrewTools.addRevoluteJoint(jointName, parentBody, jointOffset, jointAxis);
         joint.setQ(random.nextDouble());

         Vector3D comOffset = new Vector3D();
         ReferenceFrame nextFrame = createOffsetFrame(joint, comOffset, name + "Frame");
         nextFrame.update();

         Matrix3D momentOfInertia = new Matrix3D();
         momentOfInertia.setElement(0, 0, 0.005);
         momentOfInertia.setElement(1, 1, 0.005);
         momentOfInertia.setElement(2, 2, 0.005);

         RigidBodyInertia inertia = new RigidBodyInertia(nextFrame, momentOfInertia, 0.1);
         RigidBody rigidBody = new RigidBody(name, inertia, joint);

         return rigidBody;
      }

      private RigidBody createUpperArm(RigidBody parentBody)
      {
         RevoluteJoint joint = ScrewTools.addRevoluteJoint("shoulderPitch_y", parentBody, new Vector3D(0.0, 0.0, 0.0), Y);
         joint.setQ(random.nextDouble());

         Vector3D comOffset = new Vector3D(0.0, 0.0, THIGH_LENGTH / 2.0);
         ReferenceFrame nextFrame = createOffsetFrame(joint, comOffset, "upperArmFrame");
         nextFrame.update();

         Matrix3D momentOfInertia = new Matrix3D();
         momentOfInertia.setElement(0, 0, 0.0437);
         momentOfInertia.setElement(1, 1, 0.0437);
         momentOfInertia.setElement(2, 2, 0.0054);

         RigidBodyInertia inertia = new RigidBodyInertia(nextFrame, momentOfInertia, THIGH_MASS);
         RigidBody rigidBody = new RigidBody("upperArm", inertia, joint);

         return rigidBody;
      }

      private RigidBody createLowerArm(String suffix, RigidBody parentBody)
      {
         RevoluteJoint joint = ScrewTools.addRevoluteJoint("elbow_y_" + suffix, parentBody, new Vector3D(0.0, 0.0, THIGH_LENGTH), Y);
         joint.setQ(random.nextDouble());

         Vector3D comOffset = new Vector3D(0.0, 0.0, SHIN_LENGTH / 2.0);
         ReferenceFrame nextFrame = createOffsetFrame(joint, comOffset, "lowerArmFrame" + suffix);
         nextFrame.update();

         Matrix3D momentOfInertia = new Matrix3D();
         momentOfInertia.setElement(0, 0, 0.0437);
         momentOfInertia.setElement(1, 1, 0.0437);
         momentOfInertia.setElement(2, 2, 0.0054);

         RigidBodyInertia inertia = new RigidBodyInertia(nextFrame, momentOfInertia, SHIN_MASS);
         RigidBody rigidBody = new RigidBody("lowerArm" + suffix, inertia, joint);

         return rigidBody;
      }

      private RigidBody createHand(String suffix, RigidBody parentBody)
      {
         RevoluteJoint joint = ScrewTools.addRevoluteJoint("wristPitch_y_" + suffix, parentBody, new Vector3D(0.0, 0.0, SHIN_LENGTH), Y);
         joint.setQ(random.nextDouble());

         Vector3D comOffset = new Vector3D(0.0, 0.0, SHIN_LENGTH / 4.0);
         ReferenceFrame nextFrame = createOffsetFrame(joint, comOffset, "handFrame" + suffix);
         nextFrame.update();

         Matrix3D momentOfInertia = new Matrix3D();
         momentOfInertia.setElement(0, 0, 0.0437);
         momentOfInertia.setElement(1, 1, 0.0437);
         momentOfInertia.setElement(2, 2, 0.0054);

         RigidBodyInertia inertia = new RigidBodyInertia(nextFrame, momentOfInertia, FOOT_MASS);
         RigidBody rigidBody = new RigidBody("hand" + suffix, inertia, joint);

         return rigidBody;
      }

      @Override
      public OneDoFJoint[] getControllableOneDoFJoints()
      {
         return null;
      }

      @Override
      public void getControllableOneDoFJoints(ArrayList<OneDoFJoint> oneDoFJointsToPack)
      {
      }

      @Override
      public OneDoFJoint getOneDoFJointByName(String name)
      {
         return null;
      }

      @Override
      public RigidBody getEndEffector(Enum<?> segmentEnum)
      {
         return null;
      }

      @Override
      public double getTotalMass()
      {
         return Double.NaN;
      }

      @Override
      public ReferenceFrame getLidarBaseFrame(String name)
      {
         return null;
      }

      @Override
      public ReferenceFrame getCameraFrame(String name)
      {
         return null;
      }

      @Override
      public ReferenceFrame getHeadBaseFrame()
      {
         return null;
      }

      @Override
      public Map<String, OneDoFJoint> getOneDoFJointsAsMap()
      {
         return null;
      }

      @Override
      public void getOneDoFJointsFromRootToHere(OneDoFJoint oneDoFJointAtEndOfChain, ArrayList<OneDoFJoint> oneDoFJointsToPack)
      {
      }

      @Override
      public RigidBodyTransform getLidarBaseToSensorTransform(String name)
      {
         return null;
      }
   }

   public static class RobotLegs extends Robot implements FullRobotModel
   {
      private RigidBody elevator;
      private RigidBody pelvis;

      private SideDependentList<RigidBody> feet = new SideDependentList<>();
      private SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();
      private SixDoFJoint rootJoint;
      private OneDoFJoint[] joints;

      private CommonHumanoidReferenceFrames referenceFrames;
      private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      private final SideDependentList<ArrayList<Point2D>> controllerFootGroundContactPoints = new SideDependentList<>();
      private final SideDependentList<Point2D> controllerToeContactPoints = new SideDependentList<>();

      private final ContactableBodiesFactory contactableBodiesFactory = new ContactableBodiesFactory();
      private final SideDependentList<ContactableFoot> footContactableBodies = new SideDependentList<>();

      public RobotLegs(String name)
      {
         super(name);
      }

      public void updateFrames()
      {
         worldFrame.update();
         referenceFrames.updateFrames();
      }

      public void createContactPoints()
      {
         for (RobotSide robotSide : RobotSide.values)
         {
            controllerFootGroundContactPoints.put(robotSide, new ArrayList<Point2D>());
            controllerFootGroundContactPoints.get(robotSide).add(new Point2D(-footLength / 2.0, -footWidth / 2.0));
            controllerFootGroundContactPoints.get(robotSide).add(new Point2D(-footLength / 2.0, footWidth / 2.0));
            controllerFootGroundContactPoints.get(robotSide).add(new Point2D(footLength / 2.0, -toeWidth / 2.0));
            controllerFootGroundContactPoints.get(robotSide).add(new Point2D(footLength / 2.0, toeWidth / 2.0));
            controllerToeContactPoints.put(robotSide, new Point2D(footLength / 2.0, 0.0));
         }

         contactableBodiesFactory.addFootContactParameters(controllerFootGroundContactPoints, controllerToeContactPoints);

         for (RobotSide robotSide : RobotSide.values)
         {
            RigidBody foot = feet.get(robotSide);
            ReferenceFrame soleFrame = referenceFrames.getSoleFrame(robotSide);
            List<Point2D> contactPointsInSoleFrame = controllerFootGroundContactPoints.get(robotSide);
            ListOfPointsContactableFoot footContactableBody = new ListOfPointsContactableFoot(foot, soleFrame, contactPointsInSoleFrame, controllerToeContactPoints.get(robotSide));
            footContactableBodies.put(robotSide, footContactableBody);
         }
      }

      public void setRootJoint(SixDoFJoint rootJoint)
      {
         this.rootJoint = rootJoint;
      }

      public void setElevator(RigidBody elevator)
      {
         this.elevator = elevator;
      }

      public void setPelvis(RigidBody pelvis)
      {
         this.pelvis = pelvis;
      }

      public void setFeet(SideDependentList<RigidBody> feet)
      {
         this.feet.set(feet);
      }

      public void setSoleFrames(SideDependentList<ReferenceFrame> soleFrames)
      {
         this.soleFrames.set(soleFrames);
      }

      public void setOneDoFJoints(OneDoFJoint[] joints)
      {
         this.joints = joints;
      }

      public void createReferenceFrames()
      {
         referenceFrames = new LegReferenceFrames(pelvis, elevator, feet, soleFrames);
      }

      public RobotSpecificJointNames getRobotSpecificJointNames()
      {
         return null;
      }

      public ReferenceFrame getWorldFrame()
      {
         return worldFrame;
      }

      public ReferenceFrame getElevatorFrame()
      {
         return elevator.getBodyFixedFrame();
      }

      public SixDoFJoint getRootJoint()
      {
         return rootJoint;
      }

      public RigidBody getElevator()
      {
         return elevator;
      }

      public OneDoFJoint getSpineJoint(SpineJointName spineJointName)
      {
         return null;
      }

      public OneDoFJoint getNeckJoint(NeckJointName neckJointName)
      {
         return null;
      }

      public InverseDynamicsJoint getLidarJoint(String lidarName)
      {
         return null;
      }

      public RigidBody getPelvis()
      {
         return pelvis;
      }

      public RigidBody getChest()
      {
         return null;
      }

      public RigidBody getHead()
      {
         return null;
      }

      public OneDoFJoint[] getOneDoFJoints()
      {
         return joints;
      }

      public void getOneDoFJoints(ArrayList<OneDoFJoint> oneDoFJointsToPack)
      {
         oneDoFJointsToPack.clear();
         for (OneDoFJoint joint : joints)
            oneDoFJointsToPack.add(joint);
      }

      public IMUDefinition[] getIMUDefinitions()
      {
         return null;
      }

      public ForceSensorDefinition[] getForceSensorDefinitions()
      {
         return null;
      }

      public ContactSensorDefinition[] getContactSensorDefinitions()
      {
         return null;
      }

      public RigidBody getFoot(RobotSide robotSide)
      {
         return feet.get(robotSide);
      }

      public ReferenceFrame getSoleFrame(RobotSide robotSide)
      {
         return soleFrames.get(robotSide);
      }

      public CommonHumanoidReferenceFrames getReferenceFrames()
      {
         return referenceFrames;
      }

      public SideDependentList<ArrayList<Point2D>> getControllerFootGroundContactPoints()
      {
         return controllerFootGroundContactPoints;
      }

      public ContactableBodiesFactory getContactableBodiesFactory()
      {
         return contactableBodiesFactory;
      }

      public SideDependentList<ContactableFoot> getFootContactableBodies()
      {
         return footContactableBodies;
      }

      @Override
      public OneDoFJoint[] getControllableOneDoFJoints()
      {
         return null;
      }

      @Override
      public void getControllableOneDoFJoints(ArrayList<OneDoFJoint> oneDoFJointsToPack)
      {
      }

      @Override
      public OneDoFJoint getOneDoFJointByName(String name)
      {
         return null;
      }

      @Override
      public RigidBody getEndEffector(Enum<?> segmentEnum)
      {
         return null;
      }

      @Override
      public double getTotalMass()
      {
         return Double.NaN;
      }

      @Override
      public ReferenceFrame getLidarBaseFrame(String name)
      {
         return null;
      }

      @Override
      public ReferenceFrame getCameraFrame(String name)
      {
         return null;
      }

      @Override
      public ReferenceFrame getHeadBaseFrame()
      {
         return null;
      }

      @Override
      public Map<String, OneDoFJoint> getOneDoFJointsAsMap()
      {
         return null;
      }

      @Override
      public void getOneDoFJointsFromRootToHere(OneDoFJoint oneDoFJointAtEndOfChain, ArrayList<OneDoFJoint> oneDoFJointsToPack)
      {
      }

      @Override
      public RigidBodyTransform getLidarBaseToSensorTransform(String name)
      {
         return null;
      }
   }

   private static class LegReferenceFrames implements CommonHumanoidReferenceFrames
   {
      private final ReferenceFrame pelvisFrame;
      private final ReferenceFrame centerOfMassFrame;

      private final SideDependentList<ReferenceFrame> footReferenceFrames = new SideDependentList<>();
      private final SideDependentList<ReferenceFrame> soleReferenceFrames = new SideDependentList<>();

      public LegReferenceFrames(RigidBody pelvis, RigidBody elevator, SideDependentList<RigidBody> feet, SideDependentList<ReferenceFrame> soleFrames)
      {
         pelvisFrame = pelvis.getBodyFixedFrame();
         centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMass", ReferenceFrame.getWorldFrame(), elevator);

         for (RobotSide robotSide : RobotSide.values)
         {
            footReferenceFrames.put(robotSide, feet.get(robotSide).getBodyFixedFrame());
            soleReferenceFrames.put(robotSide, soleFrames.get(robotSide));
         }
      }

      public void updateFrames()
      {
         centerOfMassFrame.update();
         for (RobotSide robotSide : RobotSide.values())
         {
            footReferenceFrames.get(robotSide).update();
            soleReferenceFrames.get(robotSide).update();
         }
      }

      public ReferenceFrame getABodyAttachedZUpFrame()
      {
         return null;
      }

      public ReferenceFrame getMidFeetZUpFrame()
      {
         return null;
      }

      public ReferenceFrame getMidFeetUnderPelvisFrame()
      {
         return null;
      }


      public SideDependentList<ReferenceFrame> getAnkleZUpReferenceFrames()
      {
         return null;
      }

      public SideDependentList<ReferenceFrame> getFootReferenceFrames()
      {
         return footReferenceFrames;
      }

      public SideDependentList<ReferenceFrame> getSoleFrames()
      {
         return soleReferenceFrames;
      }

      public ReferenceFrame getPelvisFrame()
      {
         return pelvisFrame;
      }

      public ReferenceFrame getAnkleZUpFrame(RobotSide robotSide)
      {
         return null;
      }

      public ReferenceFrame getFootFrame(RobotSide robotSide)
      {
         return footReferenceFrames.get(robotSide);
      }

      public ReferenceFrame getLegJointFrame(RobotSide robotSide, LegJointName legJointName)
      {
         return null;
      }

      public EnumMap<LegJointName, ReferenceFrame> getLegJointFrames(RobotSide robotSide)
      {
         return null;
      }

      public ReferenceFrame getIMUFrame()
      {
         return null;
      }

      public ReferenceFrame getCenterOfMassFrame()
      {
         return centerOfMassFrame;
      }

      public ReferenceFrame getPelvisZUpFrame()
      {
         return null;
      }

      public ReferenceFrame getSoleFrame(RobotSide robotSide)
      {
         return soleReferenceFrames.get(robotSide);
      }

      public ReferenceFrame getSoleZUpFrame(RobotSide robotSide)
      {
         return null;
      }

      public SideDependentList<ReferenceFrame> getSoleZUpFrames()
      {
         return null;
      }
   }

   private static class ForcePointController implements RobotController
   {
      private static final double linearKp = 50.0;
      private static final double linearKi = 0.0;
      private static final double linearKd = 50.0;
      private static final double linearDeadband = 0.00;
      private static final double angularKp = 0.0;
      private static final double angularKi = 0.0;
      private static final double angularKd = 10.0;
      private static final double angularDeadband = 0.00;
      private static final double linearMaxIntegral = 50.0;
      private static final double angularMaxIntegral = 50.0;

      private final YoVariableRegistry registry;

      private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      private final YoPIDGains linearPidGains;
      private final YoPIDGains angularPidGains;

      private final PIDController pidControllerLinearX;
      private final PIDController pidControllerLinearY;
      private final PIDController pidControllerLinearZ;
      private final PIDController pidControllerAngularX;
      private final PIDController pidControllerAngularY;
      private final PIDController pidControllerAngularZ;

      private final ExternalForcePoint forcePoint;

      private final DoubleYoVariable desiredLinearX;
      private final DoubleYoVariable desiredLinearY;
      private final DoubleYoVariable desiredLinearZ;
      private final DoubleYoVariable currentLinearX;
      private final DoubleYoVariable currentLinearY;
      private final DoubleYoVariable currentLinearZ;

      private final DoubleYoVariable desiredAngularX;
      private final DoubleYoVariable desiredAngularY;
      private final DoubleYoVariable desiredAngularZ;
      private final DoubleYoVariable currentAngularX;
      private final DoubleYoVariable currentAngularY;
      private final DoubleYoVariable currentAngularZ;

      private final ReferenceFrame handFrame;
      private final FramePose currentPose = new FramePose();
      private final Vector3D desiredPosition = new Vector3D();
      private final Vector3D currentPosition = new Vector3D();
      private final Quaternion desiredOrientation = new Quaternion();
      private final Quaternion currentOrientation = new Quaternion();
      private final Vector3D initialForce = new Vector3D();
      private final Vector3D initialTorque = new Vector3D();

      private final YoFrameVector contactForce;
      private final YoFrameVector contactTorque;

      private final YoGraphicVector forceVisualizer;
      private final YoGraphicsList yoGraphicsList;

      private boolean hasInitialForce = false;

      public ForcePointController(ExternalForcePoint forcePoint, ReferenceFrame handFrame, FramePose desiredPose)
      {
         this("", forcePoint, handFrame, desiredPose);
      }

      public ForcePointController(String suffix, ExternalForcePoint forcePoint, ReferenceFrame handFrame, FramePose desiredPose)
      {
         this.forcePoint = forcePoint;
         this.handFrame = handFrame;
         this.currentPose.setToZero(handFrame);
         desiredPose.getPosition(desiredPosition);
         desiredPose.getOrientation(desiredOrientation);

         registry = new YoVariableRegistry("forcePointController" + suffix);

         desiredLinearX = new DoubleYoVariable("desiredLinearX" + suffix, registry);
         desiredLinearY = new DoubleYoVariable("desiredLinearY" + suffix, registry);
         desiredLinearZ = new DoubleYoVariable("desiredLinearZ" + suffix, registry);
         currentLinearX = new DoubleYoVariable("currentLinearX" + suffix, registry);
         currentLinearY = new DoubleYoVariable("currentLinearY" + suffix, registry);
         currentLinearZ = new DoubleYoVariable("currentLinearZ" + suffix, registry);

         desiredAngularX = new DoubleYoVariable("desiredAngularX" + suffix, registry);
         desiredAngularY = new DoubleYoVariable("desiredAngularY" + suffix, registry);
         desiredAngularZ = new DoubleYoVariable("desiredAngularZ" + suffix, registry);
         currentAngularX = new DoubleYoVariable("currentAngularX" + suffix, registry);
         currentAngularY = new DoubleYoVariable("currentAngularY" + suffix, registry);
         currentAngularZ = new DoubleYoVariable("currentAngularZ" + suffix, registry);

         contactForce = new YoFrameVector("contactForce" + suffix, worldFrame, registry);
         contactTorque = new YoFrameVector("contactTorque" + suffix, worldFrame, registry);

         yoGraphicsList = new YoGraphicsList("forceGraphicsList" + suffix);

         linearPidGains = new YoPIDGains(forcePoint.getName() + "_linear" + suffix, registry);
         linearPidGains.setKp(linearKp);
         linearPidGains.setKi(linearKi);
         linearPidGains.setKd(linearKd);
         linearPidGains.setMaximumIntegralError(linearMaxIntegral);
         linearPidGains.setPositionDeadband(linearDeadband);

         angularPidGains = new YoPIDGains(forcePoint.getName() + "_angular" + suffix, registry);
         angularPidGains.setKp(angularKp);
         angularPidGains.setKi(angularKi);
         angularPidGains.setKd(angularKd);
         angularPidGains.setMaximumIntegralError(angularMaxIntegral);
         angularPidGains.setPositionDeadband(angularDeadband);

         pidControllerAngularX = new PIDController(angularPidGains, "angular_X" + suffix, registry);
         pidControllerAngularY = new PIDController(angularPidGains, "angular_Y" + suffix, registry);
         pidControllerAngularZ = new PIDController(angularPidGains, "angular_Z" + suffix, registry);

         pidControllerLinearX = new PIDController(linearPidGains, "linear_X" + suffix, registry);
         pidControllerLinearY = new PIDController(linearPidGains, "linear_Y" + suffix, registry);
         pidControllerLinearZ = new PIDController(linearPidGains, "linear_Z" + suffix, registry);

         AppearanceDefinition forceAppearance = YoAppearance.Red();
         forcePoint.getYoPosition().getFramePointCopy();
         forceVisualizer = new YoGraphicVector("contactForceVisualizer" + suffix, forcePoint.getYoPosition(), contactForce, 0.05, forceAppearance);

         yoGraphicsList.add(forceVisualizer);
      }

      public YoGraphicsList getYoGraphicsList()
      {
         return yoGraphicsList;
      }

      public void setLinearGains(double kp, double ki, double kd)
      {
         linearPidGains.setKp(kp);
         linearPidGains.setKi(ki);
         linearPidGains.setKd(kd);
      }

      public void setAngularGains(double kp, double ki, double kd)
      {
         angularPidGains.setKp(kp);
         angularPidGains.setKi(ki);
         angularPidGains.setKd(kd);
      }

      public void setInitialForce(Vector3D initialForce, Vector3D initialTorque)
      {
         forcePoint.setForce(initialForce);
         forcePoint.setMoment(initialTorque);
         this.initialForce.set(initialForce);
         this.initialTorque.set(initialTorque);
         hasInitialForce = true;
      }

      public void initialize()
      {
         if (!hasInitialForce)
            forcePoint.setForce(0.0, 0.0, 0.0);

         pidControllerAngularX.resetIntegrator();
         pidControllerAngularY.resetIntegrator();
         pidControllerAngularZ.resetIntegrator();

         pidControllerLinearX.resetIntegrator();
         pidControllerLinearY.resetIntegrator();
         pidControllerLinearZ.resetIntegrator();
      }

      public void doControl()
      {
         currentPose.setToZero(handFrame);
         currentPose.changeFrame(worldFrame);
         currentPose.getPosition(currentPosition);
         currentPose.getOrientation(currentOrientation);

         desiredLinearX.set(desiredPosition.getX());
         desiredLinearY.set(desiredPosition.getY());
         desiredLinearZ.set(desiredPosition.getZ());

         desiredAngularX.set(desiredOrientation.getX());
         desiredAngularY.set(desiredOrientation.getY());
         desiredAngularZ.set(desiredOrientation.getZ());

         currentLinearX.set(currentPosition.getX());
         currentLinearY.set(currentPosition.getY());
         currentLinearZ.set(currentPosition.getZ());

         currentAngularX.set(currentOrientation.getX());
         currentAngularY.set(currentOrientation.getY());
         currentAngularZ.set(currentOrientation.getZ());

         double linearForceX = pidControllerLinearX.compute(currentPosition.getX(), desiredPosition.getX(), 0.0, 0.0, 0.0);
         double linearForceY = pidControllerLinearY.compute(currentPosition.getY(), desiredPosition.getY(), 0.0, 0.0, 0.0);
         double linearForceZ = pidControllerLinearZ.compute(currentPosition.getZ(), desiredPosition.getZ(), 0.0, 0.0, 0.0);

         linearForceX += initialForce.getX();
         linearForceY += initialForce.getY();
         linearForceZ += initialForce.getZ();

         contactForce.setX(linearForceX);
         contactForce.setY(linearForceY);
         contactForce.setZ(linearForceZ);

         double torqueX = pidControllerAngularX.computeForAngles(currentOrientation.getX(), desiredOrientation.getX(), 0.0, 0.0, 0.0);
         double torqueY = pidControllerAngularY.computeForAngles(currentOrientation.getY(), desiredOrientation.getY(), 0.0, 0.0, 0.0);
         double torqueZ = pidControllerAngularZ.computeForAngles(currentOrientation.getZ(), desiredOrientation.getZ(), 0.0, 0.0, 0.0);

         torqueX += initialTorque.getX();
         torqueY += initialTorque.getY();
         torqueZ += initialTorque.getZ();

         contactTorque.setX(torqueX);
         contactTorque.setY(torqueY);
         contactTorque.setZ(torqueZ);

         forcePoint.setForce(linearForceX, linearForceY, linearForceZ);
         forcePoint.setMoment(torqueX, torqueY, torqueZ);

         forceVisualizer.update();
      }

      public Vector3D getDesiredPosition()
      {
         return desiredPosition;
      }

      public Quaternion getDesiredOrientation()
      {
         return desiredOrientation;
      }

      public Vector3D getCurrentPosition()
      {
         return currentPosition;
      }

      public Vector3D getCurrentTorque()
      {
         Vector3D contactTorque = new Vector3D();
         this.contactTorque.get(contactTorque);
         contactTorque.negate();
         return contactTorque;
      }

      public Vector3D getCurrentForce()
      {
         Vector3D contactForce = new Vector3D();
         this.contactForce.get(contactForce);
         contactForce.negate();
         return contactForce;
      }

      public Quaternion getCurrentOrientation()
      {
         return currentOrientation;
      }

      public String getName()
      {
         return "robotArmController";
      }

      public String getDescription()
      {
         return getName();
      }

      public YoVariableRegistry getYoVariableRegistry()
      {
         return registry;
      }
   }

   private static class DummyArmController implements RobotController
   {
      private final YoVariableRegistry registry = new YoVariableRegistry("controller");

      private final Map<InverseDynamicsJoint, DoubleYoVariable> yoJointTorques = new HashMap<>();

      private final SCSRobotFromInverseDynamicsRobotModel scsRobot;
      private final FullRobotModel controllerModel;
      private final OneDoFJoint[] controlledJoints;

      private final VirtualModelController virtualModelController;
      private final GeometricJacobianHolder geometricJacobianHolder;

      private Wrench desiredWrench = new Wrench();

      private List<ForcePointController> forcePointControllers = new ArrayList<>();
      private List<YoWrench> yoDesiredWrenches = new ArrayList<>();
      private List<RigidBody> endEffectors = new ArrayList<>();
      private final DenseMatrix64F selectionMatrix;

      private boolean firstTick = true;

      public DummyArmController(SCSRobotFromInverseDynamicsRobotModel scsRobot, FullRobotModel controllerModel, OneDoFJoint[] controlledJoints,
            List<ForcePointController> forcePointControllers, VirtualModelController virtualModelController, GeometricJacobianHolder geometricJacobianHolder,
            List<RigidBody> endEffectors, List<YoWrench> yoDesiredWrenches, DenseMatrix64F selectionMatrix)
      {
         this.scsRobot = scsRobot;
         this.controllerModel = controllerModel;
         this.controlledJoints = controlledJoints;
         this.forcePointControllers = forcePointControllers;
         this.virtualModelController = virtualModelController;
         this.geometricJacobianHolder = geometricJacobianHolder;
         this.endEffectors = endEffectors;
         this.selectionMatrix = selectionMatrix;
         this.yoDesiredWrenches = yoDesiredWrenches;

         for (InverseDynamicsJoint joint : controlledJoints)
            yoJointTorques.put(joint, new DoubleYoVariable(joint.getName() + "solutionTorque", registry));

         for (ForcePointController forcePointController : forcePointControllers)
            registry.addChild(forcePointController.getYoVariableRegistry());
      }

      public void initialize()
      {
         for (ForcePointController forcePointController : forcePointControllers)
            forcePointController.initialize();
      }

      public void doControl()
      {
         // copy from scs
         scsRobot.updateJointPositions_SCS_to_ID();
         scsRobot.updateJointVelocities_SCS_to_ID();
         scsRobot.update();
         controllerModel.updateFrames();

         for (ForcePointController forcePointController : forcePointControllers)
            forcePointController.doControl();
         geometricJacobianHolder.compute();

         // compute forces
         VirtualModelControlSolution virtualModelControlSolution = new VirtualModelControlSolution();

         virtualModelController.clear();
         for (int i = 0; i < endEffectors.size(); i++)
         {
            desiredWrench = yoDesiredWrenches.get(i).getWrench();
            virtualModelController.submitControlledBodyVirtualWrench(endEffectors.get(i), desiredWrench, selectionMatrix);
         }
         virtualModelController.compute(virtualModelControlSolution);

         Map<InverseDynamicsJoint, Double> jointTorques = virtualModelControlSolution.getJointTorques();
         for (OneDoFJoint joint : controlledJoints)
         {
            yoJointTorques.get(joint).set(jointTorques.get(joint));
            joint.setTau(jointTorques.get(joint));
         }

         // write to scs
         scsRobot.updateJointPositions_ID_to_SCS();
         scsRobot.updateJointVelocities_ID_to_SCS();
         scsRobot.updateJointTorques_ID_to_SCS();
      }

      public Vector3D getDesiredPosition(int index)
      {
         return forcePointControllers.get(index).getDesiredPosition();
      }

      public Quaternion getDesiredOrientation(int index)
      {
         return forcePointControllers.get(index).getDesiredOrientation();
      }

      public Vector3D getCurrentPosition(int index)
      {
         return forcePointControllers.get(index).getCurrentPosition();
      }

      public Quaternion getCurrentOrientation(int index)
      {
         return forcePointControllers.get(index).getCurrentOrientation();
      }

      public Vector3D getCurrentForce(int index)
      {
         return forcePointControllers.get(index).getCurrentForce();
      }

      public Vector3D getCurrentTorque(int index)
      {
         return forcePointControllers.get(index).getCurrentTorque();
      }

      public String getName()
      {
         return "robotArmController";
      }

      public String getDescription()
      {
         return getName();
      }

      public YoVariableRegistry getYoVariableRegistry()
      {
         return registry;
      }
   }
}
