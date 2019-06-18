package us.ihmc.commonWalkingControlModules.virtualModelControl;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.EnumMap;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.Random;

import org.ejml.data.DenseMatrix64F;

import us.ihmc.euclid.Axis;
import us.ihmc.euclid.matrix.Matrix3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.graphicsDescription.Graphics3DObject;
import us.ihmc.graphicsDescription.appearance.AppearanceDefinition;
import us.ihmc.graphicsDescription.appearance.YoAppearance;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicVector;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsList;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.mecano.frames.CenterOfMassReferenceFrame;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.multiBodySystem.RevoluteJoint;
import us.ihmc.mecano.multiBodySystem.RigidBody;
import us.ihmc.mecano.multiBodySystem.SixDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.spatial.Wrench;
import us.ihmc.mecano.spatial.interfaces.WrenchReadOnly;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.mecano.yoVariables.spatial.YoFixedFrameWrench;
import us.ihmc.robotModels.FullRobotModel;
import us.ihmc.robotics.Assert;
import us.ihmc.robotics.controllers.PIDController;
import us.ihmc.robotics.controllers.pidGains.implementations.YoPIDGains;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.robotics.partNames.LegJointName;
import us.ihmc.robotics.partNames.NeckJointName;
import us.ihmc.robotics.partNames.RobotSpecificJointNames;
import us.ihmc.robotics.partNames.SpineJointName;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.SelectionMatrix6D;
import us.ihmc.robotics.sensors.ContactSensorDefinition;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationConstructionSetTools.tools.RobotTools.SCSRobotFromInverseDynamicsRobotModel;
import us.ihmc.simulationconstructionset.ExternalForcePoint;
import us.ihmc.simulationconstructionset.FloatingJoint;
import us.ihmc.simulationconstructionset.Link;
import us.ihmc.simulationconstructionset.PinJoint;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.RobotController;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoFrameVector3D;

public class VirtualModelControllerTestHelper
{
   private static final Vector3D X = new Vector3D(1.0, 0.0, 0.0);
   private static final Vector3D Y = new Vector3D(0.0, 1.0, 0.0);
   private static final Vector3D Z = new Vector3D(0.0, 0.0, 1.0);

   private static final double toFootCenterX = 0.05042;
   private static final double toFootCenterY = 0.082;

   private static final double footBack = 0.085;
   private static final double footLength = 0.22;
   private static final double ankleHeight = 0.0875;

   private static final double POUNDS = 1.0 / 2.2;    // Pound to Kg conversion.
   private static final double INCHES = 0.0254;    // Inch to Meter Conversion.

   private static final double PELVIS_HEIGHT = 1.0;
   private static final double PELVIS_RAD = 0.1;

   private static final double HIP_WIDTH = 0.2;

   private static final double HIP_DIFFERENTIAL_HEIGHT = 0.05;
   private static final double HIP_DIFFERENTIAL_WIDTH = 0.075;

   private static final double THIGH_LENGTH = 23.29 * INCHES;
   private static final double THIGH_RAD = 0.05;
   private static final double THIGH_MASS = 6.7 * POUNDS;

   private static final double SHIN_LENGTH = 23.29 * INCHES;
   private static final double SHIN_RAD = 0.03;
   private static final double SHIN_MASS = 6.7 * POUNDS;

   private static final double ANKLE_DIFFERENTIAL_HEIGHT = 0.025;
   private static final double ANKLE_DIFFERENTIAL_WIDTH = 0.0375;

   private static final double FOOT_LENGTH = 0.08;
   private static final double FOOT_COM_OFFSET = 3.0 * INCHES;
   private static final double FOOT_RAD = 0.05;
   private static final double FOOT_MASS = 3.0 * POUNDS;

   private static final Random random = new Random(100L);

   static void createVirtualModelControlTest(SCSRobotFromInverseDynamicsRobotModel robotModel, FullRobotModel controllerModel, ReferenceFrame centerOfMassFrame,
         List<RigidBodyBasics> endEffectors, List<Vector3D> desiredForces, List<Vector3D> desiredTorques, List<ExternalForcePoint> externalForcePoints, DenseMatrix64F selectionMatrix, SimulationTestingParameters simulationTestingParameters) throws Exception
   {
      double simulationDuration = 20.0;

      YoGraphicsListRegistry yoGraphicsListRegistry = new YoGraphicsListRegistry();
      YoVariableRegistry registry = new YoVariableRegistry("robert");

      VirtualModelController virtualModelController = new VirtualModelController(controllerModel.getElevator(),
                                                                                 centerOfMassFrame, registry, yoGraphicsListRegistry);

      List<YoFixedFrameWrench> desiredWrenches = new ArrayList<>();
      List<ForcePointController> forcePointControllers = new ArrayList<>();

      for (int i = 0; i < endEffectors.size(); i++)
      {
         RigidBodyBasics endEffector = endEffectors.get(i);
         ReferenceFrame endEffectorFrame = endEffector.getBodyFixedFrame();

         FramePose3D desiredEndEffectorPose = new FramePose3D(endEffectorFrame);
         desiredEndEffectorPose.setToZero();
         desiredEndEffectorPose.changeFrame(ReferenceFrame.getWorldFrame());

         virtualModelController.registerControlledBody(endEffector, controllerModel.getElevator());

         Wrench desiredWrench = new Wrench(endEffectorFrame, endEffectorFrame);
         Vector3D desiredForce = desiredForces.get(i);
         Vector3D desiredTorque = desiredTorques.get(i);
         FrameVector3D forceFrameVector = new FrameVector3D(ReferenceFrame.getWorldFrame(), desiredForce);
         FrameVector3D torqueFrameVector = new FrameVector3D(ReferenceFrame.getWorldFrame(), desiredTorque);
         forceFrameVector.changeFrame(endEffectorFrame);
         torqueFrameVector.changeFrame(endEffectorFrame);
         desiredWrench.set(torqueFrameVector, forceFrameVector);
         desiredWrench.changeFrame(ReferenceFrame.getWorldFrame());

         YoFixedFrameWrench yoDesiredWrench = new YoFixedFrameWrench("desiredWrench" + i, endEffectorFrame, ReferenceFrame.getWorldFrame(), registry);
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
            endEffectors, desiredWrenches, selectionMatrix);

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

            EuclidCoreTestTools.assertTuple3DEquals("", currentPosition, desiredPositions.get(i), 0.25);
            EuclidCoreTestTools.assertQuaternionEquals(currentOrientation, desiredOrientations.get(i), 0.25);
            EuclidCoreTestTools.assertTuple3DEquals("", desiredForces.get(i), currentForce, 0.5);
            EuclidCoreTestTools.assertTuple3DEquals("", desiredTorques.get(i), currentTorque, 0.5);
         }
      }

      scs.closeAndDispose();
      blockingSimulationRunner = null;
   }

   public static RobotLegs createRobotLeg(double gravity)
   {
      RobotLegs robotLeg = new RobotLegs("robotLegs");
      robotLeg.setGravity(gravity);

      RigidBodyBasics elevator = new RigidBody("elevator", ReferenceFrame.getWorldFrame());

      FloatingJoint floatingJoint = new FloatingJoint("pelvis", new Vector3D(), robotLeg);
      robotLeg.addRootJoint(floatingJoint);
      SixDoFJoint rootJoint = new SixDoFJoint("pelvis", elevator);

      Link pelvisLink = pelvis();
      floatingJoint.setLink(pelvisLink);
      RigidBodyBasics pelvisBody = copyLinkAsRigidBody(pelvisLink, rootJoint, "pelvis");

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

      RevoluteJoint l_leg_hpz = new RevoluteJoint("l_leg_hpz", pelvisBody, leftHipYawOffset, Z);
      RevoluteJoint r_leg_hpz = new RevoluteJoint("r_leg_hpz", pelvisBody, rightHipYawOffset, Z);
      l_leg_hpz.setQ(l_hip_yaw.getQYoVariable().getDoubleValue());
      r_leg_hpz.setQ(r_hip_yaw.getQYoVariable().getDoubleValue());
      RigidBodyBasics leftHipDifferentialBody = copyLinkAsRigidBody(l_hip_differential, l_leg_hpz, "l_hip_differential");
      RigidBodyBasics rightHipDifferentialBody = copyLinkAsRigidBody(r_hip_differential, r_leg_hpz, "r_hip_differential");

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

      RevoluteJoint l_leg_hpx = new RevoluteJoint("l_leg_hpx", leftHipDifferentialBody, leftHipRollOffset, X);
      RevoluteJoint r_leg_hpx = new RevoluteJoint("r_leg_hpx", rightHipDifferentialBody, rightHipRollOffset, X);
      l_leg_hpx.setQ(l_hip_roll.getQYoVariable().getDoubleValue());
      r_leg_hpx.setQ(r_hip_roll.getQYoVariable().getDoubleValue());
      RigidBodyBasics leftHipDifferentialBody2 = copyLinkAsRigidBody(l_hip_differential2, l_leg_hpx, "l_hip_differential");
      RigidBodyBasics rightHipDifferentialBody2 = copyLinkAsRigidBody(r_hip_differential2, r_leg_hpx, "r_hip_differential");

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

      RevoluteJoint l_leg_hpy = new RevoluteJoint("l_leg_hpy", leftHipDifferentialBody2, leftHipPitchOffset, Y);
      RevoluteJoint r_leg_hpy = new RevoluteJoint("r_leg_hpy", rightHipDifferentialBody2, rightHipPitchOffset, Y);
      l_leg_hpy.setQ(l_hip_pitch.getQYoVariable().getDoubleValue());
      r_leg_hpy.setQ(r_hip_pitch.getQYoVariable().getDoubleValue());
      RigidBodyBasics leftThighBody = copyLinkAsRigidBody(leftThigh, l_leg_hpy, "l_thigh");
      RigidBodyBasics rightThighBody = copyLinkAsRigidBody(rightThigh, r_leg_hpy, "r_thigh");

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

      RevoluteJoint l_leg_kny = new RevoluteJoint("l_leg_kny", leftThighBody, leftKneePitchOffset, Y);
      RevoluteJoint r_leg_kny = new RevoluteJoint("r_leg_kny", rightThighBody, rightKneePitchOffset, Y);
      l_leg_kny.setQ(l_knee_pitch.getQYoVariable().getDoubleValue());
      r_leg_kny.setQ(r_knee_pitch.getQYoVariable().getDoubleValue());
      RigidBodyBasics leftShinBody = copyLinkAsRigidBody(l_shin, l_leg_kny, "l_shin");
      RigidBodyBasics rightShinBody = copyLinkAsRigidBody(r_shin, r_leg_kny, "r_shin");

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

      RevoluteJoint l_leg_aky = new RevoluteJoint("l_leg_aky", leftShinBody, leftAnklePitchOffset, Y);
      RevoluteJoint r_leg_aky = new RevoluteJoint("r_leg_aky", rightShinBody, rightAnklePitchOffset, Y);
      l_leg_aky.setQ(l_ankle_pitch.getQYoVariable().getDoubleValue());
      r_leg_aky.setQ(r_ankle_pitch.getQYoVariable().getDoubleValue());
      RigidBodyBasics leftAnkleDifferentialBody = copyLinkAsRigidBody(l_ankle_differential, l_leg_aky, "l_ankle_differential");
      RigidBodyBasics rightAnkleDifferentialBody = copyLinkAsRigidBody(r_ankle_differential, r_leg_aky, "r_ankle_differential");

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

      RevoluteJoint l_leg_akx = new RevoluteJoint("l_leg_akx", leftAnkleDifferentialBody, leftAnkleRollOffset, X);
      RevoluteJoint r_leg_akx = new RevoluteJoint("r_leg_akx", rightAnkleDifferentialBody, rightAnkleRollOffset, X);
      l_leg_akx.setQ(l_ankle_roll.getQYoVariable().getDoubleValue());
      r_leg_akx.setQ(r_ankle_roll.getQYoVariable().getDoubleValue());
      RigidBodyBasics leftFootBody = copyLinkAsRigidBody(l_foot, l_leg_akx, "l_foot");
      RigidBodyBasics rightFootBody = copyLinkAsRigidBody(r_foot, r_leg_akx, "r_foot");

      RigidBodyTransform leftSoleToAnkleFrame = TransformTools.createTranslationTransform(footLength / 2.0 - footBack + toFootCenterX,
            toFootCenterY, -ankleHeight);
      RigidBodyTransform rightSoleToAnkleFrame = TransformTools.createTranslationTransform(footLength / 2.0 - footBack + toFootCenterX,
            -toFootCenterY, -ankleHeight);
      MovingReferenceFrame leftSoleFrame = MovingReferenceFrame.constructFrameFixedInParent("Left_Sole",
            leftFootBody.getBodyFixedFrame(), leftSoleToAnkleFrame);
      MovingReferenceFrame rightSoleFrame = MovingReferenceFrame.constructFrameFixedInParent("Right_Sole",
            rightFootBody.getBodyFixedFrame(), rightSoleToAnkleFrame);

      SideDependentList<RigidBodyBasics> feet = new SideDependentList<>();
      feet.put(RobotSide.LEFT, leftFootBody);
      feet.put(RobotSide.RIGHT, rightFootBody);
      SideDependentList<MovingReferenceFrame> soleFrames = new SideDependentList<>();
      soleFrames.put(RobotSide.LEFT, leftSoleFrame);
      soleFrames.put(RobotSide.RIGHT, rightSoleFrame);

      OneDoFJointBasics[] joints = {l_leg_hpz, l_leg_hpx, l_leg_hpy, l_leg_kny, l_leg_akx, l_leg_aky, r_leg_hpz, r_leg_hpx, r_leg_hpy, r_leg_kny, r_leg_akx, r_leg_aky};

      robotLeg.setPelvis(pelvisBody);
      robotLeg.setFeet(feet);
      robotLeg.setElevator(elevator);
      robotLeg.setRootJoint(rootJoint);
      robotLeg.setSoleFrames(soleFrames);
      robotLeg.createReferenceFrames();
      robotLeg.setOneDoFJoints(joints);
      elevator.updateFramesRecursively();
      robotLeg.referenceFrames.updateFrames();

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
   private static RigidBodyBasics copyLinkAsRigidBody(Link link, JointBasics currentInverseDynamicsJoint, String bodyName)
   {
      Vector3D comOffset = new Vector3D();
      link.getComOffset(comOffset);
      Matrix3D momentOfInertia = new Matrix3D();
      link.getMomentOfInertia(momentOfInertia);

      return new RigidBody(bodyName, currentInverseDynamicsJoint, momentOfInertia, link.getMass(), comOffset);
   }

   static void compareWrenches(WrenchReadOnly inputWrench, Wrench outputWrench, DenseMatrix64F selectionMatrix)
   {
      inputWrench.getBodyFrame().checkReferenceFrameMatch(outputWrench.getBodyFrame());
      outputWrench.changeFrame(inputWrench.getReferenceFrame());
      inputWrench.getReferenceFrame().checkReferenceFrameMatch(outputWrench.getReferenceFrame());

      DenseMatrix64F inputWrenchMatrix = new DenseMatrix64F(Wrench.SIZE, 1);
      DenseMatrix64F outputWrenchMatrix = new DenseMatrix64F(Wrench.SIZE, 1);
      DenseMatrix64F selectedValues = new DenseMatrix64F(Wrench.SIZE, 1);

      inputWrench.get(inputWrenchMatrix);
      outputWrench.get(outputWrenchMatrix);

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

   public static void compareWrenches(WrenchReadOnly inputWrench, Wrench outputWrench, SelectionMatrix6D selectionMatrix)
   {
      inputWrench.getBodyFrame().checkReferenceFrameMatch(outputWrench.getBodyFrame());
      outputWrench.changeFrame(inputWrench.getReferenceFrame());
      inputWrench.getReferenceFrame().checkReferenceFrameMatch(outputWrench.getReferenceFrame());

      DenseMatrix64F inputWrenchMatrix = new DenseMatrix64F(Wrench.SIZE, 1);
      DenseMatrix64F outputWrenchMatrix = new DenseMatrix64F(Wrench.SIZE, 1);

      inputWrench.get(inputWrenchMatrix);
      outputWrench.get(outputWrenchMatrix);

      double epsilon = 1e-4;

      if (selectionMatrix.isAngularXSelected())
         Assert.assertEquals(inputWrenchMatrix.get(0, 0), outputWrenchMatrix.get(0, 0), epsilon);
      if (selectionMatrix.isAngularYSelected())
         Assert.assertEquals(inputWrenchMatrix.get(1, 0), outputWrenchMatrix.get(1, 0), epsilon);
      if (selectionMatrix.isAngularZSelected())
         Assert.assertEquals(inputWrenchMatrix.get(2, 0), outputWrenchMatrix.get(2, 0), epsilon);
      if (selectionMatrix.isLinearXSelected())
         Assert.assertEquals(inputWrenchMatrix.get(3, 0), outputWrenchMatrix.get(3, 0), epsilon);
      if (selectionMatrix.isLinearYSelected())
         Assert.assertEquals(inputWrenchMatrix.get(4, 0), outputWrenchMatrix.get(4, 0), epsilon);
      if (selectionMatrix.isLinearZSelected())
         Assert.assertEquals(inputWrenchMatrix.get(5, 0), outputWrenchMatrix.get(5, 0), epsilon);

   }

   public static void compareWrenches(WrenchReadOnly inputWrench, Wrench outputWrench)
   {
      inputWrench.getBodyFrame().checkReferenceFrameMatch(outputWrench.getBodyFrame());
      outputWrench.changeFrame(inputWrench.getReferenceFrame());
      inputWrench.getReferenceFrame().checkReferenceFrameMatch(outputWrench.getReferenceFrame());

      double epsilon = 1e-4;
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(inputWrench.getAngularPart()), new Vector3D(outputWrench.getAngularPart()), epsilon);
      EuclidCoreTestTools.assertTuple3DEquals(new Vector3D(inputWrench.getLinearPart()), new Vector3D(outputWrench.getLinearPart()), epsilon);
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
      private final MovingReferenceFrame elevatorFrame;
      private final ReferenceFrame centerOfMassFrame;

      private final RigidBodyBasics elevator;
      private final RigidBodyBasics upperArm;
      private final RigidBodyBasics lowerArm;
      private final RigidBodyBasics hand;

      private ExternalForcePoint externalForcePoint;

      private final OneDoFJointBasics[] oneDoFJoints;

      PlanarRobotArm()
      {
         elevator = new RigidBody("elevator", worldFrame);
         elevatorFrame = elevator.getBodyFixedFrame();
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

         oneDoFJoints = MultiBodySystemTools.createOneDoFJointPath(elevator, hand);
         elevator.updateFramesRecursively();
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

      @Override
      public RobotSpecificJointNames getRobotSpecificJointNames()
      {
         return null;
      }

      @Override
      public void updateFrames()
      {
         worldFrame.update();
         elevator.updateFramesRecursively();
      }

      @Override
      public MovingReferenceFrame getElevatorFrame()
      {
         return elevatorFrame;
      }

      public ReferenceFrame getCenterOfMassFrame()
      {
         return centerOfMassFrame;
      }

      @Override
      public SixDoFJoint getRootJoint()
      {
         return null;
      }

      @Override
      public RigidBodyBasics getElevator()
      {
         return elevator;
      }

      public RigidBodyBasics getHand()
      {
         return hand;
      }

      @Override
      public OneDoFJointBasics getSpineJoint(SpineJointName spineJointName)
      {
         return null;
      }

      @Override
      public OneDoFJointBasics getNeckJoint(NeckJointName neckJointName)
      {
         return null;
      }

      @Override
      public JointBasics getLidarJoint(String lidarName)
      {
         return null;
      }

      @Override
      public RigidBodyBasics getRootBody()
      {
         return getElevator();
      }

      @Override
      public RigidBodyBasics getHead()
      {
         return null;
      }

      @Override
      public OneDoFJointBasics[] getOneDoFJoints()
      {
         return oneDoFJoints;
      }

      @Override
      public void getOneDoFJoints(List<OneDoFJointBasics> oneDoFJointsToPack)
      {
         List<OneDoFJointBasics> list = Arrays.asList(oneDoFJoints);

         for (int i = 0; i < list.size(); i++)
            oneDoFJointsToPack.set(i, list.get(i));
      }

      @Override
      public IMUDefinition[] getIMUDefinitions()
      {
         return null;
      }

      @Override
      public ForceSensorDefinition[] getForceSensorDefinitions()
      {
         return null;
      }

      @Override
      public ContactSensorDefinition[] getContactSensorDefinitions()
      {
         return null;
      }

      private RigidBodyBasics createUpperArm(RigidBodyBasics parentBody)
      {
         RevoluteJoint joint = new RevoluteJoint("shoulderPitch_y", parentBody, new Vector3D(0.0, 0.0, 0.0), Y);
         joint.setQ(Math.toRadians(20));

         Vector3D comOffset = new Vector3D(0.0, 0.0, THIGH_LENGTH / 2.0);
         return new RigidBody("upperArm", joint, 0.0437, 0.0437, 0.0054, THIGH_MASS, comOffset);
      }

      private RigidBodyBasics createLowerArm(RigidBodyBasics parentBody)
      {
         RevoluteJoint joint = new RevoluteJoint("elbow_y", parentBody, new Vector3D(0.0, 0.0, THIGH_LENGTH), Y);
         joint.setQ(Math.toRadians(40));

         Vector3D comOffset = new Vector3D(0.0, 0.0, SHIN_LENGTH / 2.0);

         return new RigidBody("lowerArm", joint, 0.0437, 0.0437, 0.0054, SHIN_MASS, comOffset);
      }

      private RigidBodyBasics createHand(RigidBodyBasics parentBody)
      {
         RevoluteJoint joint = new RevoluteJoint("wristPitch_y", parentBody, new Vector3D(0.0, 0.0, SHIN_LENGTH), Y);
         joint.setQ(Math.toRadians(30));

         Vector3D comOffset = new Vector3D(0.0, 0.0, SHIN_LENGTH / 4.0);

         return new RigidBody("hand", joint, 0.0437, 0.0437, 0.0054, FOOT_MASS, comOffset);
      }

      @Override
      public OneDoFJointBasics[] getControllableOneDoFJoints()
      {
         return null;
      }

      @Override
      public void getControllableOneDoFJoints(List<OneDoFJointBasics> oneDoFJointsToPack)
      {
      }

      @Override
      public OneDoFJointBasics getOneDoFJointByName(String name)
      {
         return null;
      }

      @Override
      public RigidBodyBasics getEndEffector(Enum<?> segmentEnum)
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
      public Map<String, OneDoFJointBasics> getOneDoFJointsAsMap()
      {
         return null;
      }

      @Override
      public void getOneDoFJointsFromRootToHere(OneDoFJointBasics oneDoFJointAtEndOfChain, List<OneDoFJointBasics> oneDoFJointsToPack)
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
      private final MovingReferenceFrame elevatorFrame;
      private final ReferenceFrame centerOfMassFrame;

      private final RigidBodyBasics elevator;
      private final RigidBodyBasics hand;
      private final RigidBodyBasics shoulderDifferentialYaw;

      private ExternalForcePoint externalForcePoint;

      private final OneDoFJointBasics[] oneDoFJoints;

      RobotArm()
      {
         elevator = new RigidBody("elevator", worldFrame);
         elevatorFrame = elevator.getBodyFixedFrame();
         centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMass", ReferenceFrame.getWorldFrame(), elevator);

         shoulderDifferentialYaw = createDifferential("shoulderDifferential", elevator, new Vector3D(), Z);
         RigidBodyBasics shoulderDifferentialRoll = createDifferential("shoulderDifferential", shoulderDifferentialYaw, new Vector3D(), X);

         RigidBodyBasics upperArm = createUpperArm(shoulderDifferentialRoll);

         RigidBodyBasics lowerArm = createLowerArm(upperArm);

         RigidBodyBasics wristDifferentialRoll = createDifferential("wristDifferential", lowerArm, new Vector3D(0.0, 0.0, SHIN_LENGTH), X);
         //RigidBody wristDifferentialYaw = createDifferential("wristDifferential", wristDifferentialRoll, new Vector3d(), Z);

         hand = createHand(wristDifferentialRoll);

         scsRobotArm = new SCSRobotFromInverseDynamicsRobotModel("robotArm", elevator.getChildrenJoints().get(0));
         scsRobotArm.setGravity(0);
         scsRobotArm.updateJointPositions_ID_to_SCS();
         scsRobotArm.update();

         addLinkGraphics();
         addForcePoint();

         oneDoFJoints = MultiBodySystemTools.createOneDoFJointPath(elevator, hand);
         elevator.updateFramesRecursively();
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

      @Override
      public RobotSpecificJointNames getRobotSpecificJointNames()
      {
         return null;
      }

      @Override
      public void updateFrames()
      {
         worldFrame.update();
         elevator.updateFramesRecursively();
      }

      @Override
      public MovingReferenceFrame getElevatorFrame()
      {
         return elevatorFrame;
      }

      public ReferenceFrame getCenterOfMassFrame()
      {
         return centerOfMassFrame;
      }

      @Override
      public SixDoFJoint getRootJoint()
      {
         return null;
      }

      @Override
      public RigidBodyBasics getElevator()
      {
         return elevator;
      }

      public RigidBodyBasics getHand()
      {
         return hand;
      }

      @Override
      public OneDoFJointBasics getSpineJoint(SpineJointName spineJointName)
      {
         return null;
      }

      @Override
      public OneDoFJointBasics getNeckJoint(NeckJointName neckJointName)
      {
         return null;
      }

      @Override
      public JointBasics getLidarJoint(String lidarName)
      {
         return null;
      }

      @Override
      public RigidBodyBasics getRootBody()
      {
         return getElevator();
      }

      @Override
      public RigidBodyBasics getHead()
      {
         return null;
      }

      @Override
      public OneDoFJointBasics[] getOneDoFJoints()
      {
         return oneDoFJoints;
      }

      @Override
      public void getOneDoFJoints(List<OneDoFJointBasics> oneDoFJointsToPack)
      {
         List<OneDoFJointBasics> list = Arrays.asList(oneDoFJoints);

         for (int i = 0; i < list.size(); i++)
            oneDoFJointsToPack.set(i, list.get(i));
      }

      @Override
      public IMUDefinition[] getIMUDefinitions()
      {
         return null;
      }

      @Override
      public ForceSensorDefinition[] getForceSensorDefinitions()
      {
         return null;
      }

      @Override
      public ContactSensorDefinition[] getContactSensorDefinitions()
      {
         return null;
      }

      private RigidBodyBasics createDifferential(String name, RigidBodyBasics parentBody, Vector3D jointOffset, Vector3D jointAxis)
      {
         String jointName;
         if (jointAxis == X)
            jointName = name + "_x";
         else if (jointAxis == Y)
            jointName = name + "_y";
         else
            jointName = name + "_z";
         RevoluteJoint joint = new RevoluteJoint(jointName, parentBody, jointOffset, jointAxis);
         joint.setQ(random.nextDouble());

         Vector3D comOffset = new Vector3D();

         return new RigidBody(name, joint, 0.005, 0.005, 0.005, 0.1, comOffset);
      }

      private RigidBodyBasics createUpperArm(RigidBodyBasics parentBody)
      {
         RevoluteJoint joint = new RevoluteJoint("shoulderPitch_y", parentBody, new Vector3D(0.0, 0.0, 0.0), Y);
         joint.setQ(random.nextDouble());

         Vector3D comOffset = new Vector3D(0.0, 0.0, THIGH_LENGTH / 2.0);
         return new RigidBody("upperArm", joint, 0.0437, 0.0437, 0.0054, THIGH_MASS, comOffset);
      }

      private RigidBodyBasics createLowerArm(RigidBodyBasics parentBody)
      {
         RevoluteJoint joint = new RevoluteJoint("elbow_y", parentBody, new Vector3D(0.0, 0.0, THIGH_LENGTH), Y);
         joint.setQ(random.nextDouble());

         Vector3D comOffset = new Vector3D(0.0, 0.0, SHIN_LENGTH / 2.0);

         return new RigidBody("lowerArm", joint, 0.0437, 0.0437, 0.0054, SHIN_MASS, comOffset);
      }

      private RigidBodyBasics createHand(RigidBodyBasics parentBody)
      {
         RevoluteJoint joint = new RevoluteJoint("wristPitch_y", parentBody, new Vector3D(0.0, 0.0, 0.0), Y);
         joint.setQ(random.nextDouble());

         Vector3D comOffset = new Vector3D(0.0, 0.0, SHIN_LENGTH / 4.0);

         return new RigidBody("hand", joint, 0.0437, 0.0437, 0.0054, FOOT_MASS, comOffset);
      }

      @Override
      public OneDoFJointBasics[] getControllableOneDoFJoints()
      {
         return null;
      }

      @Override
      public void getControllableOneDoFJoints(List<OneDoFJointBasics> oneDoFJointsToPack)
      {
      }

      @Override
      public OneDoFJointBasics getOneDoFJointByName(String name)
      {
         return null;
      }

      @Override
      public RigidBodyBasics getEndEffector(Enum<?> segmentEnum)
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
      public Map<String, OneDoFJointBasics> getOneDoFJointsAsMap()
      {
         return null;
      }

      @Override
      public void getOneDoFJointsFromRootToHere(OneDoFJointBasics oneDoFJointAtEndOfChain, List<OneDoFJointBasics> oneDoFJointsToPack)
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
      private final MovingReferenceFrame elevatorFrame;
      private final ReferenceFrame centerOfMassFrame;

      private final RigidBodyBasics elevator;
      private final RigidBodyBasics hand1;
      private final RigidBodyBasics hand2;
      private final RigidBodyBasics shoulderDifferentialYaw;
      private final SideDependentList<RigidBodyBasics> hands = new SideDependentList<>();
      private final List<JointBasics> joints = new ArrayList<>();

      private ExternalForcePoint externalForcePoint1;
      private ExternalForcePoint externalForcePoint2;

      private final OneDoFJointBasics[] oneDoFJoints;

      ForkedRobotArm()
      {
         elevator = new RigidBody("elevator", worldFrame);
         elevatorFrame = elevator.getBodyFixedFrame();
         centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMass", ReferenceFrame.getWorldFrame(), elevator);

         shoulderDifferentialYaw = createDifferential("shoulderDifferential", elevator, new Vector3D(), Z);

         RigidBodyBasics upperArm = createUpperArm(shoulderDifferentialYaw);
         joints.add(upperArm.getParentJoint());

         RigidBodyBasics elbowDifferentialYaw1 = createDifferential("elbowDifferentialYaw1", upperArm, new Vector3D(0.0, 0.0, THIGH_LENGTH), Z);
         RigidBodyBasics elbowDifferentialYaw2 = createDifferential("elbowDifferentialYaw2", upperArm, new Vector3D(0.0, 0.0, THIGH_LENGTH), Z);
         RevoluteJoint differential1 = (RevoluteJoint) elbowDifferentialYaw1.getParentJoint();
         RevoluteJoint differential2 = (RevoluteJoint) elbowDifferentialYaw2.getParentJoint();
         differential1.setQ(2.5 * random.nextDouble());
         differential2.setQ(-2.5 * random.nextDouble());
         joints.add(differential1);
         joints.add(differential2);

         RigidBodyBasics lowerArm1 = createLowerArm("1", elbowDifferentialYaw1);
         RigidBodyBasics lowerArm2 = createLowerArm("2", elbowDifferentialYaw2);
         joints.add(lowerArm1.getParentJoint());
         joints.add(lowerArm2.getParentJoint());

         RigidBodyBasics wristDifferentialRoll1 = createDifferential("wristDifferential1", lowerArm1, new Vector3D(0.0, 0.0, SHIN_LENGTH), X);
         RigidBodyBasics wristDifferentialRoll2 = createDifferential("wristDifferential2", lowerArm2, new Vector3D(0.0, 0.0, SHIN_LENGTH), X);
         joints.add(wristDifferentialRoll1.getParentJoint());
         joints.add(wristDifferentialRoll2.getParentJoint());

         hand1 = createHand("1", wristDifferentialRoll1);
         hand2 = createHand("2", wristDifferentialRoll2);
         hands.put(RobotSide.LEFT, hand1);
         hands.put(RobotSide.RIGHT, hand2);
         joints.add(hand1.getParentJoint());
         joints.add(hand2.getParentJoint());

         JointBasics[] jointArray = new JointBasics[joints.size()];
         joints.toArray(jointArray);
         oneDoFJoints = MultiBodySystemTools.filterJoints(jointArray, OneDoFJointBasics.class);
         elevator.updateFramesRecursively();

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
         return new SideDependentList<>(externalForcePoint1, externalForcePoint2);
      }

      public SCSRobotFromInverseDynamicsRobotModel getSCSRobotArm()
      {
         return scsRobotArm;
      }

      @Override
      public RobotSpecificJointNames getRobotSpecificJointNames()
      {
         return null;
      }

      @Override
      public void updateFrames()
      {
         worldFrame.update();
         elevator.updateFramesRecursively();
      }

      @Override
      public MovingReferenceFrame getElevatorFrame()
      {
         return elevatorFrame;
      }

      public ReferenceFrame getCenterOfMassFrame()
      {
         return centerOfMassFrame;
      }

      @Override
      public SixDoFJoint getRootJoint()
      {
         return null;
      }

      @Override
      public RigidBodyBasics getElevator()
      {
         return elevator;
      }

      public RigidBodyBasics getHand(RobotSide robotSide)
      {
         return hands.get(robotSide);
      }

      @Override
      public OneDoFJointBasics getSpineJoint(SpineJointName spineJointName)
      {
         return null;
      }

      @Override
      public OneDoFJointBasics getNeckJoint(NeckJointName neckJointName)
      {
         return null;
      }

      @Override
      public JointBasics getLidarJoint(String lidarName)
      {
         return null;
      }

      @Override
      public RigidBodyBasics getRootBody()
      {
         return getElevator();
      }

      @Override
      public RigidBodyBasics getHead()
      {
         return null;
      }

      @Override
      public OneDoFJointBasics[] getOneDoFJoints()
      {
         return oneDoFJoints;
      }

      @Override
      public void getOneDoFJoints(List<OneDoFJointBasics> oneDoFJointsToPack)
      {
         List<OneDoFJointBasics> list = Arrays.asList(oneDoFJoints);

         for (int i = 0; i < list.size(); i++)
            oneDoFJointsToPack.set(i, list.get(i));
      }

      @Override
      public IMUDefinition[] getIMUDefinitions()
      {
         return null;
      }

      @Override
      public ForceSensorDefinition[] getForceSensorDefinitions()
      {
         return null;
      }

      @Override
      public ContactSensorDefinition[] getContactSensorDefinitions()
      {
         return null;
      }

      private RigidBodyBasics createDifferential(String name, RigidBodyBasics parentBody, Vector3D jointOffset, Vector3D jointAxis)
      {
         String jointName;
         if (jointAxis == X)
            jointName = name + "_x";
         else if (jointAxis == Y)
            jointName = name + "_y";
         else
            jointName = name + "_z";
         RevoluteJoint joint = new RevoluteJoint(jointName, parentBody, jointOffset, jointAxis);
         joint.setQ(random.nextDouble());

         Vector3D comOffset = new Vector3D();
         return new RigidBody(jointName, joint, 0.005, 0.005, 0.005, 0.1, comOffset);
      }

      private RigidBodyBasics createUpperArm(RigidBodyBasics parentBody)
      {
         RevoluteJoint joint = new RevoluteJoint("shoulderPitch_y", parentBody, new Vector3D(0.0, 0.0, 0.0), Y);
         joint.setQ(random.nextDouble());

         Vector3D comOffset = new Vector3D(0.0, 0.0, THIGH_LENGTH / 2.0);

         return new RigidBody("upperArm", joint, 0.0437, 0.0437, 0.0054, THIGH_MASS, comOffset);
      }

      private RigidBodyBasics createLowerArm(String suffix, RigidBodyBasics parentBody)
      {
         RevoluteJoint joint = new RevoluteJoint("elbow_y_" + suffix, parentBody, new Vector3D(0.0, 0.0, 0.0), Y);
         joint.setQ(random.nextDouble());

         Vector3D comOffset = new Vector3D(0.0, 0.0, SHIN_LENGTH / 2.0);

         return new RigidBody("lowerArm" + suffix, joint, 0.0437, 0.0437, 0.0054, SHIN_MASS, comOffset);
      }

      private RigidBodyBasics createHand(String suffix, RigidBodyBasics parentBody)
      {
         RevoluteJoint joint = new RevoluteJoint("wristPitch_y_" + suffix, parentBody, new Vector3D(0.0, 0.0, 0.0), Y);
         joint.setQ(random.nextDouble());

         Vector3D comOffset = new Vector3D(0.0, 0.0, SHIN_LENGTH / 4.0);

         return new RigidBody("hand" + suffix, joint, 0.0437, 0.0437, 0.0054, FOOT_MASS, comOffset);
      }

      @Override
      public OneDoFJointBasics[] getControllableOneDoFJoints()
      {
         return null;
      }

      @Override
      public void getControllableOneDoFJoints(List<OneDoFJointBasics> oneDoFJointsToPack)
      {
      }

      @Override
      public OneDoFJointBasics getOneDoFJointByName(String name)
      {
         return null;
      }

      @Override
      public RigidBodyBasics getEndEffector(Enum<?> segmentEnum)
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
      public Map<String, OneDoFJointBasics> getOneDoFJointsAsMap()
      {
         return null;
      }

      @Override
      public void getOneDoFJointsFromRootToHere(OneDoFJointBasics oneDoFJointAtEndOfChain, List<OneDoFJointBasics> oneDoFJointsToPack)
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
      private final MovingReferenceFrame elevatorFrame;
      private final ReferenceFrame centerOfMassFrame;

      private final RigidBodyBasics elevator;
      private final RigidBodyBasics upperArm;
      private final RigidBodyBasics hand1;
      private final RigidBodyBasics hand2;
      private final SideDependentList<RigidBodyBasics> hands = new SideDependentList<>();

      private ExternalForcePoint externalForcePoint1;
      private ExternalForcePoint externalForcePoint2;

      private final OneDoFJointBasics[] oneDoFJoints;

      PlanarForkedRobotArm()
      {
         elevator = new RigidBody("elevator", worldFrame);
         elevatorFrame = elevator.getBodyFixedFrame();
         centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMassFrame", ReferenceFrame.getWorldFrame(), elevator);

         List<JointBasics> joints = new ArrayList<>();

         upperArm = createUpperArm(elevator);
         joints.add(upperArm.getParentJoint());

         RigidBodyBasics lowerArm1 = createLowerArm("1", upperArm);
         RigidBodyBasics lowerArm2 = createLowerArm("2", upperArm);
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

         JointBasics[] jointArray = new JointBasics[joints.size()];
         joints.toArray(jointArray);
         oneDoFJoints = MultiBodySystemTools.filterJoints(jointArray, OneDoFJointBasics.class);
         elevator.updateFramesRecursively();

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
         return new SideDependentList<>(externalForcePoint1, externalForcePoint2);
      }

      public SCSRobotFromInverseDynamicsRobotModel getSCSRobotArm()
      {
         return scsRobotArm;
      }

      @Override
      public RobotSpecificJointNames getRobotSpecificJointNames()
      {
         return null;
      }

      @Override
      public void updateFrames()
      {
         worldFrame.update();
         elevator.updateFramesRecursively();
      }

      @Override
      public MovingReferenceFrame getElevatorFrame()
      {
         return elevatorFrame;
      }

      public ReferenceFrame getCenterOfMassFrame()
      {
         return centerOfMassFrame;
      }

      @Override
      public SixDoFJoint getRootJoint()
      {
         return null;
      }

      @Override
      public RigidBodyBasics getElevator()
      {
         return elevator;
      }

      public RigidBodyBasics getHand(RobotSide robotSide)
      {
         return hands.get(robotSide);
      }

      @Override
      public OneDoFJointBasics getSpineJoint(SpineJointName spineJointName)
      {
         return null;
      }

      @Override
      public OneDoFJointBasics getNeckJoint(NeckJointName neckJointName)
      {
         return null;
      }

      @Override
      public JointBasics getLidarJoint(String lidarName)
      {
         return null;
      }

      @Override
      public RigidBodyBasics getRootBody()
      {
         return getElevator();
      }

      @Override
      public RigidBodyBasics getHead()
      {
         return null;
      }

      @Override
      public OneDoFJointBasics[] getOneDoFJoints()
      {
         return oneDoFJoints;
      }

      @Override
      public void getOneDoFJoints(List<OneDoFJointBasics> oneDoFJointsToPack)
      {
         List<OneDoFJointBasics> list = Arrays.asList(oneDoFJoints);

         for (int i = 0; i < list.size(); i++)
            oneDoFJointsToPack.set(i, list.get(i));
      }

      @Override
      public IMUDefinition[] getIMUDefinitions()
      {
         return null;
      }

      @Override
      public ForceSensorDefinition[] getForceSensorDefinitions()
      {
         return null;
      }

      @Override
      public ContactSensorDefinition[] getContactSensorDefinitions()
      {
         return null;
      }

      private RigidBodyBasics createUpperArm(RigidBodyBasics parentBody)
      {
         RevoluteJoint joint = new RevoluteJoint("shoulderPitch_y", parentBody, new Vector3D(0.0, 0.0, 0.0), Y);
         joint.setQ(random.nextDouble());

         Vector3D comOffset = new Vector3D(0.0, 0.0, THIGH_LENGTH / 2.0);
         return new RigidBody("upperArm", joint, 0.0437, 0.0437, 0.0054, THIGH_MASS, comOffset);
      }

      private RigidBodyBasics createLowerArm(String suffix, RigidBodyBasics parentBody)
      {
         RevoluteJoint joint = new RevoluteJoint("elbow_y_" + suffix, parentBody, new Vector3D(0.0, 0.0, THIGH_LENGTH), Y);
         joint.setQ(random.nextDouble());

         Vector3D comOffset = new Vector3D(0.0, 0.0, SHIN_LENGTH / 2.0);

         return new RigidBody("lowerArm" + suffix, joint, 0.0437, 0.0437, 0.0054, SHIN_MASS, comOffset);
      }

      private RigidBodyBasics createHand(String suffix, RigidBodyBasics parentBody)
      {
         RevoluteJoint joint = new RevoluteJoint("wristPitch_y_" + suffix, parentBody, new Vector3D(0.0, 0.0, SHIN_LENGTH), Y);
         joint.setQ(random.nextDouble());

         Vector3D comOffset = new Vector3D(0.0, 0.0, SHIN_LENGTH / 4.0);

         return new RigidBody("hand" + suffix, joint, 0.0437, 0.0437, 0.0054, FOOT_MASS, comOffset);
      }

      @Override
      public OneDoFJointBasics[] getControllableOneDoFJoints()
      {
         return null;
      }

      @Override
      public void getControllableOneDoFJoints(List<OneDoFJointBasics> oneDoFJointsToPack)
      {
      }

      @Override
      public OneDoFJointBasics getOneDoFJointByName(String name)
      {
         return null;
      }

      @Override
      public RigidBodyBasics getEndEffector(Enum<?> segmentEnum)
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
      public Map<String, OneDoFJointBasics> getOneDoFJointsAsMap()
      {
         return null;
      }

      @Override
      public void getOneDoFJointsFromRootToHere(OneDoFJointBasics oneDoFJointAtEndOfChain, List<OneDoFJointBasics> oneDoFJointsToPack)
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
      private RigidBodyBasics elevator;
      private RigidBodyBasics pelvis;

      private SideDependentList<RigidBodyBasics> feet = new SideDependentList<>();
      private SideDependentList<MovingReferenceFrame> soleFrames = new SideDependentList<>();
      private SixDoFJoint rootJoint;
      private OneDoFJointBasics[] joints;

      private CommonHumanoidReferenceFrames referenceFrames;
      private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      RobotLegs(String name)
      {
         super(name);
      }

      @Override
      public void updateFrames()
      {
         worldFrame.update();
         referenceFrames.updateFrames();
      }

      void setRootJoint(SixDoFJoint rootJoint)
      {
         this.rootJoint = rootJoint;
      }

      void setElevator(RigidBodyBasics elevator)
      {
         this.elevator = elevator;
      }

      void setPelvis(RigidBodyBasics pelvis)
      {
         this.pelvis = pelvis;
      }

      public void setFeet(SideDependentList<RigidBodyBasics> feet)
      {
         this.feet.set(feet);
      }

      void setSoleFrames(SideDependentList<MovingReferenceFrame> soleFrames)
      {
         this.soleFrames.set(soleFrames);
      }

      void setOneDoFJoints(OneDoFJointBasics[] joints)
      {
         this.joints = joints;
      }

      void createReferenceFrames()
      {
         referenceFrames = new LegReferenceFrames(pelvis, elevator, feet, soleFrames);
      }

      @Override
      public RobotSpecificJointNames getRobotSpecificJointNames()
      {
         return null;
      }

      @Override
      public MovingReferenceFrame getElevatorFrame()
      {
         return elevator.getBodyFixedFrame();
      }

      @Override
      public SixDoFJoint getRootJoint()
      {
         return rootJoint;
      }

      @Override
      public RigidBodyBasics getElevator()
      {
         return elevator;
      }

      @Override
      public OneDoFJointBasics getSpineJoint(SpineJointName spineJointName)
      {
         return null;
      }

      @Override
      public OneDoFJointBasics getNeckJoint(NeckJointName neckJointName)
      {
         return null;
      }

      @Override
      public JointBasics getLidarJoint(String lidarName)
      {
         return null;
      }

      @Override
      public RigidBodyBasics getRootBody()
      {
         return pelvis;
      }

      @Override
      public RigidBodyBasics getHead()
      {
         return null;
      }

      @Override
      public OneDoFJointBasics[] getOneDoFJoints()
      {
         return joints;
      }

      @Override
      public void getOneDoFJoints(List<OneDoFJointBasics> oneDoFJointsToPack)
      {
         oneDoFJointsToPack.clear();
         for (OneDoFJointBasics joint : joints)
            oneDoFJointsToPack.add(joint);
      }

      @Override
      public IMUDefinition[] getIMUDefinitions()
      {
         return null;
      }

      @Override
      public ForceSensorDefinition[] getForceSensorDefinitions()
      {
         return null;
      }

      @Override
      public ContactSensorDefinition[] getContactSensorDefinitions()
      {
         return null;
      }

      public RigidBodyBasics getFoot(RobotSide robotSide)
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

      @Override
      public OneDoFJointBasics[] getControllableOneDoFJoints()
      {
         return null;
      }

      @Override
      public void getControllableOneDoFJoints(List<OneDoFJointBasics> oneDoFJointsToPack)
      {
      }

      @Override
      public OneDoFJointBasics getOneDoFJointByName(String name)
      {
         return null;
      }

      @Override
      public RigidBodyBasics getEndEffector(Enum<?> segmentEnum)
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
      public Map<String, OneDoFJointBasics> getOneDoFJointsAsMap()
      {
         return null;
      }

      @Override
      public void getOneDoFJointsFromRootToHere(OneDoFJointBasics oneDoFJointAtEndOfChain, List<OneDoFJointBasics> oneDoFJointsToPack)
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
      private final MovingReferenceFrame pelvisFrame;
      private final ReferenceFrame centerOfMassFrame;

      private final SideDependentList<MovingReferenceFrame> footReferenceFrames = new SideDependentList<>();
      private final SideDependentList<MovingReferenceFrame> soleReferenceFrames = new SideDependentList<>();

      LegReferenceFrames(RigidBodyBasics pelvis, RigidBodyBasics elevator, SideDependentList<RigidBodyBasics> feet, SideDependentList<MovingReferenceFrame> soleFrames)
      {
         pelvisFrame = pelvis.getBodyFixedFrame();
         centerOfMassFrame = new CenterOfMassReferenceFrame("centerOfMass", ReferenceFrame.getWorldFrame(), elevator);

         for (RobotSide robotSide : RobotSide.values)
         {
            footReferenceFrames.put(robotSide, feet.get(robotSide).getBodyFixedFrame());
            soleReferenceFrames.put(robotSide, soleFrames.get(robotSide));
         }
      }

      @Override
      public void updateFrames()
      {
         centerOfMassFrame.update();
         for (RobotSide robotSide : RobotSide.values())
         {
            footReferenceFrames.get(robotSide).update();
            soleReferenceFrames.get(robotSide).update();
         }
      }

      @Override
      public MovingReferenceFrame getABodyAttachedZUpFrame()
      {
         return null;
      }

      @Override
      public MovingReferenceFrame getMidFeetZUpFrame()
      {
         return null;
      }

      @Override
      public MovingReferenceFrame getMidFeetUnderPelvisFrame()
      {
         return null;
      }

      @Override
      public MovingReferenceFrame getMidFootZUpGroundFrame()
      {
         return null;
      }

      @Override
      public SideDependentList<MovingReferenceFrame> getAnkleZUpReferenceFrames()
      {
         return null;
      }

      @Override
      public SideDependentList<MovingReferenceFrame> getFootReferenceFrames()
      {
         return footReferenceFrames;
      }

      @Override
      public SideDependentList<MovingReferenceFrame> getSoleFrames()
      {
         return soleReferenceFrames;
      }

      @Override
      public MovingReferenceFrame getPelvisFrame()
      {
         return pelvisFrame;
      }

      @Override
      public MovingReferenceFrame getAnkleZUpFrame(RobotSide robotSide)
      {
         return null;
      }

      @Override
      public MovingReferenceFrame getFootFrame(RobotSide robotSide)
      {
         return footReferenceFrames.get(robotSide);
      }

      @Override
      public MovingReferenceFrame getLegJointFrame(RobotSide robotSide, LegJointName legJointName)
      {
         return null;
      }

      @Override
      public EnumMap<LegJointName, MovingReferenceFrame> getLegJointFrames(RobotSide robotSide)
      {
         return null;
      }

      @Override
      public ReferenceFrame getIMUFrame()
      {
         return null;
      }

      @Override
      public ReferenceFrame getCenterOfMassFrame()
      {
         return centerOfMassFrame;
      }

      @Override
      public MovingReferenceFrame getPelvisZUpFrame()
      {
         return null;
      }

      @Override
      public MovingReferenceFrame getSoleFrame(RobotSide robotSide)
      {
         return soleReferenceFrames.get(robotSide);
      }

      @Override
      public MovingReferenceFrame getSoleZUpFrame(RobotSide robotSide)
      {
         return null;
      }

      @Override
      public SideDependentList<MovingReferenceFrame> getSoleZUpFrames()
      {
         return null;
      }

      @Override
      public MovingReferenceFrame getChestFrame()
      {
         return null;
      }
   }

   public static class ForcePointController implements RobotController
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

      private final YoDouble desiredLinearX;
      private final YoDouble desiredLinearY;
      private final YoDouble desiredLinearZ;
      private final YoDouble currentLinearX;
      private final YoDouble currentLinearY;
      private final YoDouble currentLinearZ;

      private final YoDouble desiredAngularX;
      private final YoDouble desiredAngularY;
      private final YoDouble desiredAngularZ;
      private final YoDouble currentAngularX;
      private final YoDouble currentAngularY;
      private final YoDouble currentAngularZ;

      private final ReferenceFrame handFrame;
      private final FramePose3D currentPose = new FramePose3D();
      private final Vector3D desiredPosition = new Vector3D();
      private final Vector3D currentPosition = new Vector3D();
      private final Quaternion desiredOrientation = new Quaternion();
      private final Quaternion currentOrientation = new Quaternion();
      private final Vector3D initialForce = new Vector3D();
      private final Vector3D initialTorque = new Vector3D();

      private final YoFrameVector3D contactForce;
      private final YoFrameVector3D contactTorque;

      private final YoGraphicVector forceVisualizer;
      private final YoGraphicsList yoGraphicsList;

      private boolean hasInitialForce = false;

      public ForcePointController(String suffix, ExternalForcePoint forcePoint, ReferenceFrame handFrame, FramePose3D desiredPose)
      {
         this.forcePoint = forcePoint;
         this.handFrame = handFrame;
         this.currentPose.setToZero(handFrame);
         desiredPosition.set(desiredPose.getPosition());
         desiredOrientation.set(desiredPose.getOrientation());

         registry = new YoVariableRegistry("forcePointController" + suffix);

         desiredLinearX = new YoDouble("desiredLinearX" + suffix, registry);
         desiredLinearY = new YoDouble("desiredLinearY" + suffix, registry);
         desiredLinearZ = new YoDouble("desiredLinearZ" + suffix, registry);
         currentLinearX = new YoDouble("currentLinearX" + suffix, registry);
         currentLinearY = new YoDouble("currentLinearY" + suffix, registry);
         currentLinearZ = new YoDouble("currentLinearZ" + suffix, registry);

         desiredAngularX = new YoDouble("desiredAngularX" + suffix, registry);
         desiredAngularY = new YoDouble("desiredAngularY" + suffix, registry);
         desiredAngularZ = new YoDouble("desiredAngularZ" + suffix, registry);
         currentAngularX = new YoDouble("currentAngularX" + suffix, registry);
         currentAngularY = new YoDouble("currentAngularY" + suffix, registry);
         currentAngularZ = new YoDouble("currentAngularZ" + suffix, registry);

         contactForce = new YoFrameVector3D("contactForce" + suffix, worldFrame, registry);
         contactTorque = new YoFrameVector3D("contactTorque" + suffix, worldFrame, registry);

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

      public void setInitialForce(Vector3D initialForce, Vector3D initialTorque)
      {
         forcePoint.setForce(initialForce);
         forcePoint.setMoment(initialTorque);
         this.initialForce.set(initialForce);
         this.initialTorque.set(initialTorque);
         hasInitialForce = true;
      }

      @Override
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

      @Override
      public void doControl()
      {
         currentPose.setToZero(handFrame);
         currentPose.changeFrame(worldFrame);
         currentPosition.set(currentPose.getPosition());
         currentOrientation.set(currentPose.getOrientation());

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
         contactTorque.set(this.contactTorque);
         contactTorque.negate();
         return contactTorque;
      }

      public Vector3D getCurrentForce()
      {
         Vector3D contactForce = new Vector3D();
         contactForce.set(this.contactForce);
         contactForce.negate();
         return contactForce;
      }

      public Quaternion getCurrentOrientation()
      {
         return currentOrientation;
      }

      @Override
      public String getName()
      {
         return "robotArmController";
      }

      @Override
      public String getDescription()
      {
         return getName();
      }

      @Override
      public YoVariableRegistry getYoVariableRegistry()
      {
         return registry;
      }
   }

   private static class DummyArmController implements RobotController
   {
      private final YoVariableRegistry registry = new YoVariableRegistry("controller");

      private final Map<JointBasics, YoDouble> yoJointTorques = new HashMap<>();

      private final SCSRobotFromInverseDynamicsRobotModel scsRobot;
      private final FullRobotModel controllerModel;
      private final OneDoFJointBasics[] controlledJoints;

      private final VirtualModelController virtualModelController;

      private Wrench desiredWrench = new Wrench();

      private List<ForcePointController> forcePointControllers = new ArrayList<>();
      private List<YoFixedFrameWrench> yoDesiredWrenches = new ArrayList<>();
      private List<RigidBodyBasics> endEffectors = new ArrayList<>();
      private final DenseMatrix64F selectionMatrix;

      DummyArmController(SCSRobotFromInverseDynamicsRobotModel scsRobot, FullRobotModel controllerModel, OneDoFJointBasics[] controlledJoints,
            List<ForcePointController> forcePointControllers, VirtualModelController virtualModelController, List<RigidBodyBasics> endEffectors,
            List<YoFixedFrameWrench> yoDesiredWrenches, DenseMatrix64F selectionMatrix)
      {
         this.scsRobot = scsRobot;
         this.controllerModel = controllerModel;
         this.controlledJoints = controlledJoints;
         this.forcePointControllers = forcePointControllers;
         this.virtualModelController = virtualModelController;
         this.endEffectors = endEffectors;
         this.selectionMatrix = selectionMatrix;
         this.yoDesiredWrenches = yoDesiredWrenches;

         for (JointBasics joint : controlledJoints)
            yoJointTorques.put(joint, new YoDouble(joint.getName() + "solutionTorque", registry));

         for (ForcePointController forcePointController : forcePointControllers)
            registry.addChild(forcePointController.getYoVariableRegistry());
      }

      @Override
      public void initialize()
      {
         for (ForcePointController forcePointController : forcePointControllers)
            forcePointController.initialize();
      }

      @Override
      public void doControl()
      {
         // copy from scs
         scsRobot.updateJointPositions_SCS_to_ID();
         scsRobot.updateJointVelocities_SCS_to_ID();
         scsRobot.update();
         controllerModel.updateFrames();

         for (ForcePointController forcePointController : forcePointControllers)
            forcePointController.doControl();

         // compute forces
         VirtualModelControlSolution virtualModelControlSolution = new VirtualModelControlSolution();

         virtualModelController.clear();
         for (int i = 0; i < endEffectors.size(); i++)
         {
            desiredWrench.setIncludingFrame(yoDesiredWrenches.get(i));
            virtualModelController.submitControlledBodyVirtualWrench(endEffectors.get(i), desiredWrench, selectionMatrix);
         }
         virtualModelController.compute(virtualModelControlSolution);

         DenseMatrix64F jointTorques = virtualModelControlSolution.getJointTorques();
         for (int i = 0; i < controlledJoints.length; i++)
         {
            OneDoFJointBasics joint = controlledJoints[i];
            double tau = jointTorques.get(i, 0);
            yoJointTorques.get(joint).set(tau);
            joint.setTau(tau);
         }

         // write to scs
         scsRobot.updateJointPositions_ID_to_SCS();
         scsRobot.updateJointVelocities_ID_to_SCS();
         scsRobot.updateJointTorques_ID_to_SCS();
      }

      Vector3D getDesiredPosition(int index)
      {
         return forcePointControllers.get(index).getDesiredPosition();
      }

      Quaternion getDesiredOrientation(int index)
      {
         return forcePointControllers.get(index).getDesiredOrientation();
      }

      Vector3D getCurrentPosition(int index)
      {
         return forcePointControllers.get(index).getCurrentPosition();
      }

      Quaternion getCurrentOrientation(int index)
      {
         return forcePointControllers.get(index).getCurrentOrientation();
      }

      Vector3D getCurrentForce(int index)
      {
         return forcePointControllers.get(index).getCurrentForce();
      }

      Vector3D getCurrentTorque(int index)
      {
         return forcePointControllers.get(index).getCurrentTorque();
      }

      @Override
      public String getName()
      {
         return "robotArmController";
      }

      @Override
      public String getDescription()
      {
         return getName();
      }

      @Override
      public YoVariableRegistry getYoVariableRegistry()
      {
         return registry;
      }
   }
}
