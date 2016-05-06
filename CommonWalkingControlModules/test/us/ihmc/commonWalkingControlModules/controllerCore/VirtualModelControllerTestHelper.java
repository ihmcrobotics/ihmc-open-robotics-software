package us.ihmc.commonWalkingControlModules.controllerCore;

import org.ejml.data.DenseMatrix64F;
import org.junit.Assert;
import us.ihmc.SdfLoader.models.FullRobotModel;
import us.ihmc.SdfLoader.partNames.LegJointName;
import us.ihmc.SdfLoader.partNames.NeckJointName;
import us.ihmc.SdfLoader.partNames.RobotSpecificJointNames;
import us.ihmc.SdfLoader.partNames.SpineJointName;
import us.ihmc.graphics3DAdapter.graphics.Graphics3DObject;
import us.ihmc.graphics3DAdapter.graphics.appearances.AppearanceDefinition;
import us.ihmc.graphics3DAdapter.graphics.appearances.YoAppearance;
import us.ihmc.robotics.Axis;
import us.ihmc.robotics.MathTools;
import us.ihmc.robotics.geometry.RigidBodyTransform;
import us.ihmc.robotics.geometry.TransformTools;
import us.ihmc.robotics.referenceFrames.CenterOfMassReferenceFrame;
import us.ihmc.robotics.referenceFrames.ReferenceFrame;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.*;
import us.ihmc.robotics.sensors.ContactSensorDefinition;
import us.ihmc.robotics.sensors.ForceSensorDefinition;
import us.ihmc.robotics.sensors.IMUDefinition;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationconstructionset.*;
import us.ihmc.simulationconstructionset.RobotTools.SCSRobotFromInverseDynamicsRobotModel;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner;
import us.ihmc.simulationconstructionset.util.simulationRunner.ControllerFailureException;
import us.ihmc.tools.testing.JUnitTools;

import javax.vecmath.Matrix3d;
import javax.vecmath.Vector3d;
import java.util.*;

public class VirtualModelControllerTestHelper
{
   private static final Vector3d X = new Vector3d(1.0, 0.0, 0.0);
   private static final Vector3d Y = new Vector3d(0.0, 1.0, 0.0);
   private static final Vector3d Z = new Vector3d(0.0, 0.0, 1.0);

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

   private final Random random = new Random(100L);


   private BlockingSimulationRunner blockingSimulationRunner;

   public VirtualModelControllerTestHelper()
   {
   }

   public void setSCSAndCreateSimulationRunner(SimulationConstructionSet scs)
   {
      blockingSimulationRunner = new BlockingSimulationRunner(scs, 10.0);
   }

   public void simulateAndBlock(double simulationTime) throws BlockingSimulationRunner.SimulationExceededMaximumTimeException, ControllerFailureException
   {
      blockingSimulationRunner.simulateAndBlock(simulationTime);
   }

   public boolean simulateAndBlockAndCatchExceptions(double simulationTime) throws BlockingSimulationRunner.SimulationExceededMaximumTimeException
   {
      try
      {
         simulateAndBlock(simulationTime);
         return true;
      }
      catch (Exception e)
      {
         System.err.println("Caught exception in " + getClass().getSimpleName() + ".simulateAndBlockAndCatchExceptions. Exception = /n" + e);
         return false;
      }
   }

   public RobotLegs createRobotLeg(double gravity)
   {
      RobotLegs robotLeg = new RobotLegs("robotLegs");
      robotLeg.setGravity(gravity);
      HashMap<InverseDynamicsJoint, Joint> jointMap = new HashMap<>();

      ReferenceFrame elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", ReferenceFrame.getWorldFrame(), new RigidBodyTransform());
      RigidBody elevator = new RigidBody("elevator", elevatorFrame);

      FloatingJoint floatingJoint = new FloatingJoint("pelvis", new Vector3d(), robotLeg);
      robotLeg.addRootJoint(floatingJoint);
      SixDoFJoint rootJoint = new SixDoFJoint("pelvis", elevator, elevatorFrame);
      jointMap.put(rootJoint, floatingJoint);

      Link pelvisLink = pelvis();
      floatingJoint.setLink(pelvisLink);
      RigidBody pelvisBody = copyLinkAsRigidBody(pelvisLink, rootJoint, "pelvis");

      Vector3d leftHipYawOffset = new Vector3d(0.0, HIP_WIDTH, 0.0);
      Vector3d rightHipYawOffset = new Vector3d(0.0, -HIP_WIDTH, 0.0);
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
      l_leg_hpz.setQ(l_hip_yaw.getQ().getDoubleValue());
      r_leg_hpz.setQ(r_hip_yaw.getQ().getDoubleValue());
      RigidBody leftHipDifferentialBody = copyLinkAsRigidBody(l_hip_differential, l_leg_hpz, "l_hip_differential");
      RigidBody rightHipDifferentialBody = copyLinkAsRigidBody(r_hip_differential, r_leg_hpz, "r_hip_differential");
      jointMap.put(l_leg_hpz, l_hip_yaw);
      jointMap.put(r_leg_hpz, r_hip_yaw);

      Vector3d leftHipRollOffset = new Vector3d();
      Vector3d rightHipRollOffset = new Vector3d();
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
      l_leg_hpx.setQ(l_hip_roll.getQ().getDoubleValue());
      r_leg_hpx.setQ(r_hip_roll.getQ().getDoubleValue());
      RigidBody leftHipDifferentialBody2 = copyLinkAsRigidBody(l_hip_differential2, l_leg_hpx, "l_hip_differential");
      RigidBody rightHipDifferentialBody2 = copyLinkAsRigidBody(r_hip_differential2, r_leg_hpx, "r_hip_differential");
      jointMap.put(l_leg_hpx, l_hip_roll);
      jointMap.put(r_leg_hpx, r_hip_roll);

      Vector3d leftHipPitchOffset = new Vector3d();
      Vector3d rightHipPitchOffset = new Vector3d();
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
      l_leg_hpy.setQ(l_hip_pitch.getQ().getDoubleValue());
      r_leg_hpy.setQ(r_hip_pitch.getQ().getDoubleValue());
      RigidBody leftThighBody = copyLinkAsRigidBody(leftThigh, l_leg_hpy, "l_thigh");
      RigidBody rightThighBody = copyLinkAsRigidBody(rightThigh, r_leg_hpy, "r_thigh");
      jointMap.put(l_leg_hpy, l_hip_pitch);
      jointMap.put(r_leg_hpy, r_hip_pitch);

      Vector3d leftKneePitchOffset = new Vector3d(0.0, 0.0, -THIGH_LENGTH);
      Vector3d rightKneePitchOffset = new Vector3d(0.0, 0.0, -THIGH_LENGTH);
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
      l_leg_kny.setQ(l_knee_pitch.getQ().getDoubleValue());
      r_leg_kny.setQ(r_knee_pitch.getQ().getDoubleValue());
      RigidBody leftShinBody = copyLinkAsRigidBody(l_shin, l_leg_kny, "l_shin");
      RigidBody rightShinBody = copyLinkAsRigidBody(r_shin, r_leg_kny, "r_shin");
      jointMap.put(l_leg_kny, l_knee_pitch);
      jointMap.put(r_leg_kny, r_knee_pitch);

      Vector3d leftAnklePitchOffset = new Vector3d(0.0, 0.0, -SHIN_LENGTH);
      Vector3d rightAnklePitchOffset = new Vector3d(0.0, 0.0, -SHIN_LENGTH);
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
      l_leg_aky.setQ(l_ankle_pitch.getQ().getDoubleValue());
      r_leg_aky.setQ(r_ankle_pitch.getQ().getDoubleValue());
      RigidBody leftAnkleDifferentialBody = copyLinkAsRigidBody(l_ankle_differential, l_leg_aky, "l_ankle_differential");
      RigidBody rightAnkleDifferentialBody = copyLinkAsRigidBody(r_ankle_differential, r_leg_aky, "r_ankle_differential");
      jointMap.put(l_leg_aky, l_ankle_pitch);
      jointMap.put(r_leg_aky, r_ankle_pitch);

      Vector3d leftAnkleRollOffset = new Vector3d();
      Vector3d rightAnkleRollOffset = new Vector3d();
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
      l_leg_akx.setQ(l_ankle_roll.getQ().getDoubleValue());
      r_leg_akx.setQ(r_ankle_roll.getQ().getDoubleValue());
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

   private Link pelvis()
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

   private Link hip_differential()
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


   private Link thigh()
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


   private Link shin()
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

   private Link ankle_differential()
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

   private Link foot()
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
   private RigidBody copyLinkAsRigidBody(Link link, InverseDynamicsJoint currentInverseDynamicsJoint, String bodyName)
   {
      Vector3d comOffset = new Vector3d();
      link.getComOffset(comOffset);
      Matrix3d momentOfInertia = new Matrix3d();
      link.getMomentOfInertia(momentOfInertia);
      ReferenceFrame nextFrame = createOffsetFrame(currentInverseDynamicsJoint, comOffset, bodyName);
      nextFrame.update();
      RigidBodyInertia inertia = new RigidBodyInertia(nextFrame, momentOfInertia, link.getMass());
      RigidBody rigidBody = new RigidBody(bodyName, inertia, currentInverseDynamicsJoint);

      return rigidBody;
   }

   public static ReferenceFrame createOffsetFrame(InverseDynamicsJoint currentInverseDynamicsJoint, Vector3d offset, String frameName)
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
      inputWrench.getExpressedInFrame().checkReferenceFrameMatch(outputWrench.getExpressedInFrame());

      double epsilon = 1e-4;
      JUnitTools.assertTuple3dEquals(inputWrench.getAngularPartCopy(), outputWrench.getAngularPartCopy(), epsilon);
      JUnitTools.assertTuple3dEquals(inputWrench.getLinearPartCopy(), outputWrench.getLinearPartCopy(), epsilon);
   }

   public RobotArm createRobotArm()
   {
      return new RobotArm();
   }

   public PlanarRobotArm createPlanarArm()
   {
      return new PlanarRobotArm();
   }

   public class PlanarRobotArm implements FullRobotModel
   {
      private final SCSRobotFromInverseDynamicsRobotModel scsRobotArm;

      private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      private final ReferenceFrame elevatorFrame;

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
         RevoluteJoint joint = ScrewTools.addRevoluteJoint("shoulderPitch_y", parentBody, new Vector3d(0.0, 0.0, 0.0), Y);
         joint.setQ(Math.toRadians(20));

         Vector3d comOffset = new Vector3d(0.0, 0.0, THIGH_LENGTH / 2.0);
         ReferenceFrame nextFrame = createOffsetFrame(joint, comOffset, "upperArmFrame");
         nextFrame.update();

         Matrix3d momentOfInertia = new Matrix3d();
         momentOfInertia.setElement(0, 0, 0.0437);
         momentOfInertia.setElement(1, 1, 0.0437);
         momentOfInertia.setElement(2, 2, 0.0054);

         RigidBodyInertia inertia = new RigidBodyInertia(nextFrame, momentOfInertia, THIGH_MASS);
         RigidBody rigidBody = new RigidBody("upperArm", inertia, joint);

         return rigidBody;
      }

      private RigidBody createLowerArm(RigidBody parentBody)
      {
         RevoluteJoint joint = ScrewTools.addRevoluteJoint("elbow_y", parentBody, new Vector3d(0.0, 0.0, THIGH_LENGTH), Y);
         joint.setQ(Math.toRadians(40));

         Vector3d comOffset = new Vector3d(0.0, 0.0, SHIN_LENGTH / 2.0);
         ReferenceFrame nextFrame = createOffsetFrame(joint, comOffset, "lowerArmFrame");
         nextFrame.update();

         Matrix3d momentOfInertia = new Matrix3d();
         momentOfInertia.setElement(0, 0, 0.0437);
         momentOfInertia.setElement(1, 1, 0.0437);
         momentOfInertia.setElement(2, 2, 0.0054);

         RigidBodyInertia inertia = new RigidBodyInertia(nextFrame, momentOfInertia, SHIN_MASS);
         RigidBody rigidBody = new RigidBody("lowerArm", inertia, joint);

         return rigidBody;
      }

      private RigidBody createHand(RigidBody parentBody)
      {
         RevoluteJoint joint = ScrewTools.addRevoluteJoint("wristPitch_y", parentBody, new Vector3d(0.0, 0.0, SHIN_LENGTH), Y);
         joint.setQ(Math.toRadians(30));

         Vector3d comOffset = new Vector3d(0.0, 0.0, SHIN_LENGTH / 4.0);
         ReferenceFrame nextFrame = createOffsetFrame(joint, comOffset, "handFrame");
         nextFrame.update();

         Matrix3d momentOfInertia = new Matrix3d();
         momentOfInertia.setElement(0, 0, 0.0437);
         momentOfInertia.setElement(1, 1, 0.0437);
         momentOfInertia.setElement(2, 2, 0.0054);

         RigidBodyInertia inertia = new RigidBodyInertia(nextFrame, momentOfInertia, FOOT_MASS);
         RigidBody rigidBody = new RigidBody("hand", inertia, joint);

         return rigidBody;
      }
   }

   public class RobotArm implements FullRobotModel
   {
      private final SCSRobotFromInverseDynamicsRobotModel scsRobotArm;

      private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
      private final ReferenceFrame elevatorFrame;

      private final RigidBody elevator;
      private final RigidBody hand;
      private final RigidBody shoulderDifferentialYaw;

      private ExternalForcePoint externalForcePoint;

      private final OneDoFJoint[] oneDoFJoints;

      public RobotArm()
      {
         elevatorFrame = ReferenceFrame.constructFrameWithUnchangingTransformToParent("elevator", worldFrame, new RigidBodyTransform());
         elevator = new RigidBody("elevator", elevatorFrame);

         shoulderDifferentialYaw = createDifferential("shoulderDifferential", elevator, new Vector3d(), Z);
         RigidBody shoulderDifferentialRoll = createDifferential("shoulderDifferential", shoulderDifferentialYaw, new Vector3d(), X);

         RigidBody upperArm = createUpperArm(shoulderDifferentialRoll);

         RigidBody lowerArm = createLowerArm(upperArm);

         RigidBody wristDifferentialRoll = createDifferential("wristDifferential", lowerArm, new Vector3d(0.0, 0.0, SHIN_LENGTH), X);
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

      private RigidBody createDifferential(String name, RigidBody parentBody, Vector3d jointOffset, Vector3d jointAxis)
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

         Vector3d comOffset = new Vector3d();
         ReferenceFrame nextFrame = createOffsetFrame(joint, comOffset, name + "Frame");
         nextFrame.update();

         Matrix3d momentOfInertia = new Matrix3d();
         momentOfInertia.setElement(0, 0, 0.005);
         momentOfInertia.setElement(1, 1, 0.005);
         momentOfInertia.setElement(2, 2, 0.005);

         RigidBodyInertia inertia = new RigidBodyInertia(nextFrame, momentOfInertia, 0.1);
         RigidBody rigidBody = new RigidBody(name, inertia, joint);

         return rigidBody;
      }

      private RigidBody createUpperArm(RigidBody parentBody)
      {
         RevoluteJoint joint = ScrewTools.addRevoluteJoint("shoulderPitch_y", parentBody, new Vector3d(0.0, 0.0, 0.0), Y);
         joint.setQ(random.nextDouble());

         Vector3d comOffset = new Vector3d(0.0, 0.0, THIGH_LENGTH / 2.0);
         ReferenceFrame nextFrame = createOffsetFrame(joint, comOffset, "upperArmFrame");
         nextFrame.update();

         Matrix3d momentOfInertia = new Matrix3d();
         momentOfInertia.setElement(0, 0, 0.0437);
         momentOfInertia.setElement(1, 1, 0.0437);
         momentOfInertia.setElement(2, 2, 0.0054);

         RigidBodyInertia inertia = new RigidBodyInertia(nextFrame, momentOfInertia, THIGH_MASS);
         RigidBody rigidBody = new RigidBody("upperArm", inertia, joint);

         return rigidBody;
      }

      private RigidBody createLowerArm(RigidBody parentBody)
      {
         RevoluteJoint joint = ScrewTools.addRevoluteJoint("elbow_y", parentBody, new Vector3d(0.0, 0.0, THIGH_LENGTH), Y);
         joint.setQ(random.nextDouble());

         Vector3d comOffset = new Vector3d(0.0, 0.0, SHIN_LENGTH / 2.0);
         ReferenceFrame nextFrame = createOffsetFrame(joint, comOffset, "lowerArmFrame");
         nextFrame.update();

         Matrix3d momentOfInertia = new Matrix3d();
         momentOfInertia.setElement(0, 0, 0.0437);
         momentOfInertia.setElement(1, 1, 0.0437);
         momentOfInertia.setElement(2, 2, 0.0054);

         RigidBodyInertia inertia = new RigidBodyInertia(nextFrame, momentOfInertia, SHIN_MASS);
         RigidBody rigidBody = new RigidBody("lowerArm", inertia, joint);

         return rigidBody;
      }

      private RigidBody createHand(RigidBody parentBody)
      {
         RevoluteJoint joint = ScrewTools.addRevoluteJoint("wristPitch_y", parentBody, new Vector3d(0.0, 0.0, 0.0), Y);
         joint.setQ(random.nextDouble());

         Vector3d comOffset = new Vector3d(0.0, 0.0, SHIN_LENGTH / 4.0);
         ReferenceFrame nextFrame = createOffsetFrame(joint, comOffset, "handFrame");
         nextFrame.update();

         Matrix3d momentOfInertia = new Matrix3d();
         momentOfInertia.setElement(0, 0, 0.0437);
         momentOfInertia.setElement(1, 1, 0.0437);
         momentOfInertia.setElement(2, 2, 0.0054);

         RigidBodyInertia inertia = new RigidBodyInertia(nextFrame, momentOfInertia, FOOT_MASS);
         RigidBody rigidBody = new RigidBody("hand", inertia, joint);

         return rigidBody;
      }
   }


   public class RobotLegs extends Robot implements FullRobotModel
   {
      private RigidBody elevator;
      private RigidBody pelvis;

      private SideDependentList<RigidBody> feet = new SideDependentList<>();
      private SideDependentList<ReferenceFrame> soleFrames = new SideDependentList<>();
      private SixDoFJoint rootJoint;
      private OneDoFJoint[] joints;

      private CommonHumanoidReferenceFrames referenceFrames;
      private final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();

      public RobotLegs(String name)
      {
         super(name);
      }

      public void updateFrames()
      {
         worldFrame.update();
         referenceFrames.updateFrames();
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
   }

   private class LegReferenceFrames implements CommonHumanoidReferenceFrames
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
}
