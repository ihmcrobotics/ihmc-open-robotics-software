package us.ihmc.valkyrie.manipulation;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static us.ihmc.robotics.Assert.assertTrue;

import org.apache.commons.math3.stat.descriptive.moment.StandardDeviation;
import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Tag;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import controller_msgs.msg.dds.HandWrenchTrajectoryMessage;
import controller_msgs.msg.dds.PrepareForLocomotionMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.testTools.EndToEndTestTools;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controlModules.rigidBody.RigidBodyControlMode;
import us.ihmc.commonWalkingControlModules.referenceFrames.WalkingTrajectoryPath;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.geometry.Pose3D;
import us.ihmc.euclid.geometry.interfaces.Pose3DReadOnly;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.tools.EuclidCoreTestTools;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.transform.interfaces.RigidBodyTransformReadOnly;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.euclid.tuple4D.Quaternion;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.EuclidCoreMissingTools;
import us.ihmc.robotics.math.trajectories.generators.MultipleWaypointsPositionTrajectoryGenerator;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.scs2.definition.controller.interfaces.Controller;
import us.ihmc.scs2.definition.robot.ExternalWrenchPointDefinition;
import us.ihmc.scs2.definition.robot.MomentOfInertiaDefinition;
import us.ihmc.scs2.definition.robot.RigidBodyDefinition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.visual.ColorDefinitions;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.definition.visual.VisualDefinitionFactory;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimFloatingJointBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimJointBasics;
import us.ihmc.scs2.simulation.robot.multiBodySystem.interfaces.SimRigidBodyBasics;
import us.ihmc.scs2.simulation.robot.trackers.ExternalWrenchPoint;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.tools.MemoryTools;
import us.ihmc.valkyrie.ValkyrieRobotModel;
import us.ihmc.yoVariables.euclid.YoVector3D;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoLong;

public class ValkyrieWalkingTrajectoryPathFrameEndToEndTest
{
   private static final boolean ADD_PENDULUM = true;
   private static final RobotSide HAND_SIDE = RobotSide.LEFT;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private SimulationTestingParameters simulationTestingParameters;
   private SCS2AvatarTestingSimulation simulationTestHelper;
   private DRCRobotModel robotModel;
   private PendulumAttachmentController pendulumAttachmentController;

   @BeforeEach
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   }

   @AfterEach
   public void tearDown()
   {
      // Do this here in case a test fails. That way the memory will be recycled.
      if (simulationTestHelper != null)
      {
         simulationTestHelper.finishTest(simulationTestingParameters.getKeepSCSUp());
         simulationTestHelper = null;
      }

      simulationTestingParameters = null;

      robotModel = null;
      pendulumAttachmentController = null;
   }

   public DRCRobotModel getRobotModel()
   {
      if (robotModel == null)
         robotModel = new ValkyrieRobotModel(RobotTarget.SCS);
      return robotModel;
   }

   public String getHandName()
   {
      return getRobotModel().getJointMap().getHandName(HAND_SIDE);
   }

   public RigidBodyTransformReadOnly getPendulumOffsetInHand()
   {
      return getRobotModel().getJointMap().getHandControlFrameToWristTransform(HAND_SIDE);
   }

   public Pose3DReadOnly getHandDesiredPoseInChestFrame()
   {
      return new Pose3D(new Point3D(0.55, 0.35, -0.25), new Quaternion());
   }

   @Tag("controller-api-2")
   @Test
   public void testWalkingInPlace()
   {
      DRCRobotModel robotModel = getRobotModel();
      SCS2AvatarTestingSimulationFactory factory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(robotModel,
                                                                                                                         simulationTestingParameters);
      simulationTestHelper = factory.createAvatarTestingSimulation();

      FreeFloatingPendulumRobotDefinition pendulumRobotDefinition = setupPendulum();

      simulationTestHelper.start();

      prepareHand(getHandName(), pendulumRobotDefinition);

      FullHumanoidRobotModel controllerFullRobotModel = simulationTestHelper.getControllerFullRobotModel();

      FootstepDataListMessage stepsInPlace = EndToEndTestTools.generateStepsInPlace(controllerFullRobotModel, 4);
      simulationTestHelper.publishToController(stepsInPlace);

      pendulumAttachmentController.oscillationCalculator.clear();
      pendulumAttachmentController.rootJoint.getJointTwist().setToZero();

      assertWalkingFrameMatchMidFeetZUpFrame();
      assertTrue(simulationTestHelper.simulateNow(2.0));
      assertCorrectControlMode();
      assertTrue(simulationTestHelper.simulateNow(EndToEndTestTools.computeWalkingDuration(stepsInPlace, robotModel.getWalkingControllerParameters())));
      assertTrue(pendulumAttachmentController.angleStandardDeviation.getValue() < 0.03);
      assertWalkingFrameMatchMidFeetZUpFrame();
   }

   private void assertWalkingFrameMatchMidFeetZUpFrame()
   {
      RigidBodyTransform midFeetZUpFrameTransform = simulationTestHelper.getControllerReferenceFrames().getMidFeetZUpFrame().getTransformToRoot();
      RigidBodyTransform walkingTrajectoryPathFrameTransform = simulationTestHelper.getHighLevelHumanoidControllerFactory()
                                                                                   .getHighLevelHumanoidControllerToolbox().getWalkingTrajectoryPath()
                                                                                   .getWalkingTrajectoryPathFrame().getTransformToRoot();

      //      Vector3D diff = new Vector3D();
      //      diff.sub(midFeetZUpFrameTransform.getTranslation(), walkingTrajectoryPathFrameTransform.getTranslation());
      //      LogTools.info("Difference w.r.t. mid feet zup frame: angle= "
      //            + midFeetZUpFrameTransform.getRotation().distance(walkingTrajectoryPathFrameTransform.getRotation()) + ", distance= " + diff.length());
      // It doesn't match the mid feet zup yaw. 
      //      EuclidCoreTestTools.assertRotationMatrixGeometricallyEquals(midFeetZUpFrameTransform.getRotation(),
      //                                                                  walkingTrajectoryPathFrameTransform.getRotation(),
      //                                                                  1.0e-3);
      EuclidCoreTestTools.assertVector3DGeometricallyEquals(midFeetZUpFrameTransform.getTranslation(),
                                                            walkingTrajectoryPathFrameTransform.getTranslation(),
                                                            1.0e-5);
   }

   @Tag("controller-api-2")
   @Test
   public void testWalkingForward()
   {
      DRCRobotModel robotModel = getRobotModel();
      SCS2AvatarTestingSimulationFactory factory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(robotModel,
                                                                                                                         simulationTestingParameters);
      simulationTestHelper = factory.createAvatarTestingSimulation();

      FreeFloatingPendulumRobotDefinition pendulumRobotDefinition = setupPendulum();

      simulationTestHelper.start();

      prepareHand(getHandName(), pendulumRobotDefinition);

      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      SteppingParameters steppingParameters = walkingControllerParameters.getSteppingParameters();
      FramePose3D startPose = new FramePose3D(simulationTestHelper.getControllerReferenceFrames().getMidFootZUpGroundFrame());
      startPose.changeFrame(worldFrame);
      FootstepDataListMessage steps = EndToEndTestTools.generateForwardSteps(RobotSide.LEFT,
                                                                             6,
                                                                             a -> steppingParameters.getDefaultStepLength(),
                                                                             steppingParameters.getInPlaceWidth(),
                                                                             0.75,
                                                                             0.25,
                                                                             startPose,
                                                                             true);
      simulationTestHelper.publishToController(steps);

      pendulumAttachmentController.oscillationCalculator.clear();
      pendulumAttachmentController.rootJoint.getJointTwist().setToZero();

      assertWalkingFrameMatchMidFeetZUpFrame();
      assertTrue(simulationTestHelper.simulateNow(2.0));
      assertCorrectControlMode();
      assertTrue(simulationTestHelper.simulateNow(EndToEndTestTools.computeWalkingDuration(steps, robotModel.getWalkingControllerParameters())));
      assertTrue(pendulumAttachmentController.angleStandardDeviation.getValue() < 0.03);
      assertWalkingFrameMatchMidFeetZUpFrame();
   }

   @Tag("controller-api-2")
   @Test
   public void testWalkingCircle()
   {
      DRCRobotModel robotModel = getRobotModel();
      SCS2AvatarTestingSimulationFactory factory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(robotModel,
                                                                                                                         new FlatGroundEnvironment(),
                                                                                                                         simulationTestingParameters);
      simulationTestHelper = factory.createAvatarTestingSimulation();

      FreeFloatingPendulumRobotDefinition pendulumRobotDefinition = setupPendulum();

      simulationTestHelper.start();

      prepareHand(getHandName(), pendulumRobotDefinition);

      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      SteppingParameters steppingParameters = walkingControllerParameters.getSteppingParameters();
      FramePose3D startPose = new FramePose3D(simulationTestHelper.getControllerReferenceFrames().getMidFootZUpGroundFrame());
      startPose.changeFrame(worldFrame);
      FootstepDataListMessage steps = EndToEndTestTools.generateCircleSteps(RobotSide.LEFT,
                                                                            10,
                                                                            a -> steppingParameters.getDefaultStepLength(),
                                                                            steppingParameters.getInPlaceWidth(),
                                                                            0.75,
                                                                            0.25,
                                                                            startPose,
                                                                            true,
                                                                            RobotSide.RIGHT,
                                                                            1.0);
      simulationTestHelper.publishToController(steps);

      pendulumAttachmentController.oscillationCalculator.clear();
      pendulumAttachmentController.rootJoint.getJointTwist().setToZero();

      assertWalkingFrameMatchMidFeetZUpFrame();
      assertTrue(simulationTestHelper.simulateNow(2.0));
      assertCorrectControlMode();
      assertTrue(simulationTestHelper.simulateNow(EndToEndTestTools.computeWalkingDuration(steps, robotModel.getWalkingControllerParameters())));
      assertTrue(pendulumAttachmentController.angleStandardDeviation.getValue() < 0.04);
      assertWalkingFrameMatchMidFeetZUpFrame();
   }

   private FreeFloatingPendulumRobotDefinition setupPendulum()
   {
      SimRigidBodyBasics hand = simulationTestHelper.getRobot().getRigidBody(getHandName());
      SimJointBasics lastWristJoint = hand.getParentJoint();

      ExternalWrenchPoint robotAttachmentPoint = lastWristJoint.getAuxialiryData()
                                                               .addExternalWrenchPoint(new ExternalWrenchPointDefinition("robotAttachmentPoint",
                                                                                                                         getPendulumOffsetInHand()));
      simulationTestHelper.getRobot().updateFrames();

      FreeFloatingPendulumRobotDefinition pendulumRobotDefinition = new FreeFloatingPendulumRobotDefinition();
      pendulumRobotDefinition.rootJoint.setInitialJointState(new SixDoFJointState(null,
                                                                                  lastWristJoint.getFrameAfterJoint().getTransformToRoot().getTranslation()));

      if (ADD_PENDULUM)
      {
         Robot pendulumRobot = simulationTestHelper.getSimulationConstructionSet().addRobot(pendulumRobotDefinition);

         pendulumAttachmentController = new PendulumAttachmentController(robotAttachmentPoint, pendulumRobot);
         pendulumRobot.getControllerManager().addController(pendulumAttachmentController);
      }

      return pendulumRobotDefinition;
   }

   private void prepareHand(String handName, FreeFloatingPendulumRobotDefinition pendulumRobotDefinition)
   {
      assertTrue(simulationTestHelper.simulateNow(0.5));

      simulationTestHelper.findVariable(handName + "TaskspaceUseBaseFrameForControl").setValueFromDouble(1.0);

      PrepareForLocomotionMessage prepareForLocomotionMessage = new PrepareForLocomotionMessage();
      prepareForLocomotionMessage.setPrepareManipulation(false);
      simulationTestHelper.publishToController(prepareForLocomotionMessage);

      if (ADD_PENDULUM)
      {
         HandWrenchTrajectoryMessage handWrenchTrajectoryMessage = new HandWrenchTrajectoryMessage();
         handWrenchTrajectoryMessage.setRobotSide(RobotSide.LEFT.toByte());
         Vector3D gravityComp = new Vector3D(0.0, 0.0, pendulumRobotDefinition.mass * 9.81);
         handWrenchTrajectoryMessage.getWrenchTrajectory().getWrenchTrajectoryPoints().add()
                                    .set(HumanoidMessageTools.createWrenchTrajectoryPointMessage(0.0, null, gravityComp));
         handWrenchTrajectoryMessage.getWrenchTrajectory().getWrenchTrajectoryPoints().add()
                                    .set(HumanoidMessageTools.createWrenchTrajectoryPointMessage(1000.0, null, gravityComp));
         handWrenchTrajectoryMessage.getWrenchTrajectory().getFrameInformation().setTrajectoryReferenceFrameId(worldFrame.hashCode());
         simulationTestHelper.publishToController(handWrenchTrajectoryMessage);
      }

      CommonHumanoidReferenceFrames referenceFrames = simulationTestHelper.getControllerReferenceFrames();
      FullHumanoidRobotModel controllerFullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      MovingReferenceFrame chestFrame = controllerFullRobotModel.getChest().getBodyFixedFrame();
      FramePose3D handPose = new FramePose3D(chestFrame, getHandDesiredPoseInChestFrame());
      handPose.changeFrame(referenceFrames.getMidFeetZUpFrame());
      HandTrajectoryMessage handTrajectoryMessage = HumanoidMessageTools.createHandTrajectoryMessage(RobotSide.LEFT,
                                                                                                     2.0,
                                                                                                     handPose,
                                                                                                     WalkingTrajectoryPath.WALKING_TRAJECTORY_FRAME_ID);
      simulationTestHelper.publishToController(handTrajectoryMessage);

      assertTrue(simulationTestHelper.simulateNow(2.5));
   }

   @SuppressWarnings("unchecked")
   private void assertCorrectControlMode()
   {
      assertEquals(RigidBodyControlMode.TASKSPACE,
                   ((YoEnum<RigidBodyControlMode>) simulationTestHelper.findVariable(getHandName() + "Manager",
                                                                                     getHandName() + "ManagerCurrentState")).getValue());
      long expectedFrameIndex = simulationTestHelper.getHighLevelHumanoidControllerFactory().getHighLevelHumanoidControllerToolbox().getWalkingTrajectoryPath()
                                                    .getWalkingTrajectoryPathFrame().getFrameIndex();
      long actualFrameIndex = ((YoLong) simulationTestHelper.findVariable(getHandName() + MultipleWaypointsPositionTrajectoryGenerator.class.getSimpleName(),
                                                                          getHandName() + "CurrentPositionFrame")).getValue();
      assertEquals(expectedFrameIndex, actualFrameIndex);
   }

   private static class PendulumAttachmentController implements Controller
   {
      private final YoRegistry registry = new YoRegistry("AttachmentController");
      private final YoDouble kp = new YoDouble("kp", registry);
      private final YoDouble kd = new YoDouble("kd", registry);
      private final YoVector3D errorPosition = new YoVector3D("errorPosition", registry);
      private final YoVector3D errorVelocity = new YoVector3D("errorVelocity", registry);
      private final YoVector3D gravityCompensation = new YoVector3D("gravityCompensation", registry);

      private final StandardDeviation oscillationCalculator = new StandardDeviation(true);
      private final YoDouble currentAngle = new YoDouble("pendulumCurrentAngle", registry);
      private final YoDouble angleStandardDeviation = new YoDouble("pendulumAngleStandardDeviation", registry);

      private final ExternalWrenchPoint robotAttachmentPoint;
      private final SimFloatingJointBasics rootJoint;
      private final SimRigidBodyBasics pendulumBody;
      private final ExternalWrenchPoint pendulumAttachmentPoint;
      private final ReferenceFrame rootFrame;

      public PendulumAttachmentController(ExternalWrenchPoint robotAttachmentPoint, Robot pendulumRobot)
      {
         this.robotAttachmentPoint = robotAttachmentPoint;
         rootJoint = (SimFloatingJointBasics) pendulumRobot.getJoint(FreeFloatingPendulumRobotDefinition.rootJointName);
         pendulumBody = rootJoint.getSuccessor();
         pendulumAttachmentPoint = rootJoint.getAuxialiryData().getExternalWrenchPoints().get(0);
         rootFrame = robotAttachmentPoint.getFrame().getRootFrame();

         kp.set(500.0);
         kd.set(50.0);
      }

      private final FramePoint3D positionA = new FramePoint3D();
      private final FramePoint3D positionB = new FramePoint3D();
      private final FrameVector3D velocityA = new FrameVector3D();
      private final FrameVector3D velocityB = new FrameVector3D();
      private final FrameVector3D force = new FrameVector3D();

      @Override
      public void doControl()
      {
         MultiBodySystemTools.getRootBody(robotAttachmentPoint.getParentJoint().getPredecessor()).updateFramesRecursively();
         positionA.setToZero(robotAttachmentPoint.getFrame());
         positionA.changeFrame(rootFrame);
         positionB.setToZero(pendulumAttachmentPoint.getFrame());
         positionB.changeFrame(rootFrame);
         errorPosition.sub(positionB, positionA);

         velocityA.setIncludingFrame(robotAttachmentPoint.getFrame().getTwistOfFrame().getLinearPart());
         velocityA.changeFrame(rootFrame);
         velocityB.setIncludingFrame(pendulumAttachmentPoint.getFrame().getTwistOfFrame().getLinearPart());
         velocityB.changeFrame(rootFrame);
         errorVelocity.sub(velocityB, velocityA);

         force.setToZero(rootFrame);
         force.setAndScale(kp.getValue(), errorPosition);
         force.scaleAdd(kd.getValue(), errorVelocity, force);

         gravityCompensation.setAndScale(-9.81 * pendulumBody.getInertia().getMass(), Axis3D.Z);
         force.add(gravityCompensation);

         robotAttachmentPoint.getWrench().getLinearPart().setMatchingFrame(force);
         pendulumAttachmentPoint.getWrench().getLinearPart().setMatchingFrame(force);
         pendulumAttachmentPoint.getWrench().getLinearPart().negate();

         FrameVector3D zUp = new FrameVector3D(pendulumAttachmentPoint.getFrame(), Axis3D.Z);
         zUp.changeFrame(rootFrame);

         double angle = EuclidCoreMissingTools.angleFromFirstToSecondVector3D(zUp, Axis3D.Z);
         currentAngle.set(angle);
         oscillationCalculator.increment(angle);
         angleStandardDeviation.set(oscillationCalculator.getResult());
      }

      @Override
      public YoRegistry getYoRegistry()
      {
         return registry;
      }
   }

   private static class FreeFloatingPendulumRobotDefinition extends RobotDefinition
   {
      private static final String rootJointName = "pendulumRootJoint";

      private final double fulcrumLength;
      private final double mass;
      private final double inertia;
      private final double sphereRadius;
      private final MaterialDefinition materialDefinition;

      private final SixDoFJointDefinition rootJoint;
      private final RigidBodyDefinition body;
      private final ExternalWrenchPointDefinition attachmentPoint;

      public FreeFloatingPendulumRobotDefinition()
      {
         this(0.3, 1.0, 0.01, 0.05, new MaterialDefinition(ColorDefinitions.CadetBlue()));
      }

      public FreeFloatingPendulumRobotDefinition(double fulcrumLength, double mass, double inertia, double sphereRadius, MaterialDefinition materialDefinition)
      {
         super("Pendulum");

         this.fulcrumLength = fulcrumLength;
         this.mass = mass;
         this.inertia = inertia;
         this.sphereRadius = sphereRadius;
         this.materialDefinition = materialDefinition;

         RigidBodyDefinition rootBody = new RigidBodyDefinition("pendulumRootBody");
         rootJoint = new SixDoFJointDefinition(rootJointName);

         attachmentPoint = new ExternalWrenchPointDefinition("attachmentPoint", new Point3D());
         rootJoint.addExternalWrenchPointDefinition(attachmentPoint);

         body = new RigidBodyDefinition("pendulumBody");
         body.getCenterOfMassOffset().set(0.0, 0.0, -fulcrumLength - sphereRadius);
         body.setMass(mass);
         body.setMomentOfInertia(new MomentOfInertiaDefinition(inertia, inertia, inertia));

         VisualDefinitionFactory pendulumVisuals = new VisualDefinitionFactory();
         pendulumVisuals.appendTranslation(0.0, 0.0, -0.5 * fulcrumLength);
         pendulumVisuals.addCylinder(fulcrumLength, 0.1 * sphereRadius, materialDefinition);
         pendulumVisuals.appendTranslation(0.0, 0.0, -0.5 * fulcrumLength - sphereRadius);
         pendulumVisuals.addSphere(sphereRadius, materialDefinition);
         body.addVisualDefinitions(pendulumVisuals.getVisualDefinitions());

         setRootBodyDefinition(rootBody);
         rootBody.addChildJoint(rootJoint);
         rootJoint.setSuccessor(body);
      }
   }
}
