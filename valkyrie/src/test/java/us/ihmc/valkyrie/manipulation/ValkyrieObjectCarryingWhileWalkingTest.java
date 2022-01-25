package us.ihmc.valkyrie.manipulation;

import static us.ihmc.robotics.Assert.assertTrue;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;

import controller_msgs.msg.dds.FootstepDataListMessage;
import controller_msgs.msg.dds.HandTrajectoryMessage;
import controller_msgs.msg.dds.HandWrenchTrajectoryMessage;
import controller_msgs.msg.dds.PrepareForLocomotionMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.manipulation.HumanoidObjectCarryingWhileWalkingTest;
import us.ihmc.avatar.testTools.EndToEndTestTools;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.configurations.SteppingParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.euclid.Axis3D;
import us.ihmc.euclid.referenceFrame.FramePoint3D;
import us.ihmc.euclid.referenceFrame.FramePose3D;
import us.ihmc.euclid.referenceFrame.FrameVector3D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.euclid.transform.RigidBodyTransform;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.euclid.tuple3D.Vector3D;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.mecano.frames.MovingReferenceFrame;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
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

public class ValkyrieObjectCarryingWhileWalkingTest extends HumanoidObjectCarryingWhileWalkingTest
{
   private static final boolean ADD_PENDULUM = true;
   private static final ReferenceFrame worldFrame = ReferenceFrame.getWorldFrame();
   private SimulationTestingParameters simulationTestingParameters;
   private SCS2AvatarTestingSimulation simulationTestHelper;

   @BeforeEach
   public void setup()
   {
      MemoryTools.printCurrentMemoryUsageAndReturnUsedMemoryInMB(getClass().getSimpleName() + " before test.");
      simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
      simulationTestingParameters.setKeepSCSUp(true);
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
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return new ValkyrieRobotModel(RobotTarget.SCS);
   }

   public String getHandName()
   {
      return getRobotModel().getJointMap().getHandName(RobotSide.LEFT);
   }

   @Test
   public void testWalkingInPlace()
   {
      DRCRobotModel robotModel = getRobotModel();
      SCS2AvatarTestingSimulationFactory factory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(robotModel,
                                                                                                                         simulationTestingParameters);
      simulationTestHelper = factory.createAvatarTestingSimulation();

      RigidBodyTransform controlFrameOffset = robotModel.getJointMap().getHandControlFrameToWristTransform(RobotSide.LEFT);

      SimRigidBodyBasics leftHand = simulationTestHelper.getRobot().getRigidBody(robotModel.getJointMap().getHandName(RobotSide.LEFT));

      ExternalWrenchPoint robotAttachmentPoint = leftHand.getParentJoint().getAuxialiryData()
                                                         .addExternalWrenchPoint(new ExternalWrenchPointDefinition("robotAttachmentPoint", controlFrameOffset));
      simulationTestHelper.getRobot().updateFrames();

      FreeFloatingPendulumRobotDefinition pendulumRobotDefinition = new FreeFloatingPendulumRobotDefinition();
      pendulumRobotDefinition.rootJoint.setInitialJointState(new SixDoFJointState(null,
                                                                                  leftHand.getParentJoint().getFrameAfterJoint().getTransformToRoot()
                                                                                          .getTranslation()));
      Robot pendulumRobot = simulationTestHelper.getSimulationSession().addRobot(pendulumRobotDefinition);

      Controller attachmentController = createAttachmentController(robotAttachmentPoint, pendulumRobot);
      pendulumRobot.getControllerManager().addController(attachmentController);

      simulationTestHelper.start();

      prepareHand(pendulumRobotDefinition);

      FullHumanoidRobotModel controllerFullRobotModel = simulationTestHelper.getControllerFullRobotModel();

      FootstepDataListMessage stepsInPlace = EndToEndTestTools.generateStepsInPlace(controllerFullRobotModel, 10);
      simulationTestHelper.publishToController(stepsInPlace);

      assertTrue(simulationTestHelper.simulateAndWait(2.0
            + EndToEndTestTools.computeWalkingDuration(stepsInPlace, robotModel.getWalkingControllerParameters())));
   }

   @Test
   public void testWalkingForward()
   {
      DRCRobotModel robotModel = getRobotModel();
      SCS2AvatarTestingSimulationFactory factory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(robotModel,
                                                                                                                         simulationTestingParameters);
      simulationTestHelper = factory.createAvatarTestingSimulation();

      RigidBodyTransform controlFrameOffset = robotModel.getJointMap().getHandControlFrameToWristTransform(RobotSide.LEFT);

      SimRigidBodyBasics leftHand = simulationTestHelper.getRobot().getRigidBody(robotModel.getJointMap().getHandName(RobotSide.LEFT));

      ExternalWrenchPoint robotAttachmentPoint = leftHand.getParentJoint().getAuxialiryData()
                                                         .addExternalWrenchPoint(new ExternalWrenchPointDefinition("robotAttachmentPoint", controlFrameOffset));
      simulationTestHelper.getRobot().updateFrames();

      FreeFloatingPendulumRobotDefinition pendulumRobotDefinition = new FreeFloatingPendulumRobotDefinition();
      pendulumRobotDefinition.rootJoint.setInitialJointState(new SixDoFJointState(null,
                                                                                  leftHand.getParentJoint().getFrameAfterJoint().getTransformToRoot()
                                                                                          .getTranslation()));

      if (ADD_PENDULUM)
      {
         Robot pendulumRobot = simulationTestHelper.getSimulationSession().addRobot(pendulumRobotDefinition);

         Controller attachmentController = createAttachmentController(robotAttachmentPoint, pendulumRobot);
         pendulumRobot.getControllerManager().addController(attachmentController);
      }

      simulationTestHelper.start();

      prepareHand(pendulumRobotDefinition);

      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      SteppingParameters steppingParameters = walkingControllerParameters.getSteppingParameters();
      FramePose3D startPose = new FramePose3D(simulationTestHelper.getReferenceFrames().getMidFootZUpGroundFrame());
      startPose.changeFrame(worldFrame);
      FootstepDataListMessage stepsInPlace = EndToEndTestTools.forwardSteps(RobotSide.LEFT,
                                                                            8,
                                                                            a -> steppingParameters.getDefaultStepLength(),
                                                                            steppingParameters.getInPlaceWidth(),
                                                                            0.75,
                                                                            0.25,
                                                                            startPose,
                                                                            true);
      simulationTestHelper.publishToController(stepsInPlace);

      assertTrue(simulationTestHelper.simulateAndWait(2.0 + EndToEndTestTools.computeWalkingDuration(stepsInPlace, walkingControllerParameters)));
   }

   @Test
   public void testWalkingCircle()
   {
      DRCRobotModel robotModel = getRobotModel();
      SCS2AvatarTestingSimulationFactory factory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(robotModel,
                                                                                                                         new FlatGroundEnvironment(),
                                                                                                                         simulationTestingParameters);
      simulationTestHelper = factory.createAvatarTestingSimulation();

      RigidBodyTransform controlFrameOffset = robotModel.getJointMap().getHandControlFrameToWristTransform(RobotSide.LEFT);

      SimRigidBodyBasics leftHand = simulationTestHelper.getRobot().getRigidBody(robotModel.getJointMap().getHandName(RobotSide.LEFT));

      ExternalWrenchPoint robotAttachmentPoint = leftHand.getParentJoint().getAuxialiryData()
                                                         .addExternalWrenchPoint(new ExternalWrenchPointDefinition("robotAttachmentPoint", controlFrameOffset));
      simulationTestHelper.getRobot().updateFrames();

      FreeFloatingPendulumRobotDefinition pendulumRobotDefinition = new FreeFloatingPendulumRobotDefinition();
      pendulumRobotDefinition.rootJoint.setInitialJointState(new SixDoFJointState(null,
                                                                                  leftHand.getParentJoint().getFrameAfterJoint().getTransformToRoot()
                                                                                          .getTranslation()));

      if (ADD_PENDULUM)
      {
         Robot pendulumRobot = simulationTestHelper.getSimulationSession().addRobot(pendulumRobotDefinition);

         Controller attachmentController = createAttachmentController(robotAttachmentPoint, pendulumRobot);
         pendulumRobot.getControllerManager().addController(attachmentController);
      }

      simulationTestHelper.start();

      prepareHand(pendulumRobotDefinition);

      WalkingControllerParameters walkingControllerParameters = robotModel.getWalkingControllerParameters();
      SteppingParameters steppingParameters = walkingControllerParameters.getSteppingParameters();
      FramePose3D startPose = new FramePose3D(simulationTestHelper.getReferenceFrames().getMidFootZUpGroundFrame());
      startPose.changeFrame(worldFrame);
      FootstepDataListMessage stepsInPlace = EndToEndTestTools.circleSteps(RobotSide.LEFT,
                                                                           20,
                                                                           a -> steppingParameters.getDefaultStepLength(),
                                                                           steppingParameters.getInPlaceWidth(),
                                                                           0.75,
                                                                           0.25,
                                                                           startPose,
                                                                           true,
                                                                           RobotSide.RIGHT,
                                                                           1.0);
      simulationTestHelper.publishToController(stepsInPlace);

      assertTrue(simulationTestHelper.simulateAndWait(2.0 + EndToEndTestTools.computeWalkingDuration(stepsInPlace, walkingControllerParameters)));
   }

   private void prepareHand(FreeFloatingPendulumRobotDefinition pendulumRobotDefinition)
   {
      assertTrue(simulationTestHelper.simulateAndWait(0.5));

      simulationTestHelper.findVariable("leftPalmTaskspaceUseBaseFrameForControl").setValueFromDouble(1.0);

      PrepareForLocomotionMessage prepareForLocomotionMessage = new PrepareForLocomotionMessage();
      prepareForLocomotionMessage.setPrepareManipulation(false);
      simulationTestHelper.publishToController(prepareForLocomotionMessage);

      CommonHumanoidReferenceFrames referenceFrames = simulationTestHelper.getReferenceFrames();
//            MovingReferenceFrame trajectoryFrame = referenceFrames.getMidFeetUnderPelvisFrame();
      MovingReferenceFrame trajectoryFrame = simulationTestHelper.getHighLevelHumanoidControllerFactory().getHighLevelHumanoidControllerToolbox()
                                                                 .getWalkingTrajectoryPath().getWalkingTrajectoryPathFrame();
      FullHumanoidRobotModel controllerFullRobotModel = simulationTestHelper.getControllerFullRobotModel();
      FramePose3D handPose = new FramePose3D(controllerFullRobotModel.getChest().getBodyFixedFrame());
      handPose.appendTranslation(0.55, 0.35, -0.25);
      handPose.changeFrame(trajectoryFrame);
      HandTrajectoryMessage handTrajectoryMessage = HumanoidMessageTools.createHandTrajectoryMessage(RobotSide.LEFT, 4.0, handPose, trajectoryFrame);
      simulationTestHelper.publishToController(handTrajectoryMessage);

      if (ADD_PENDULUM)
      {
         assertTrue(simulationTestHelper.simulateAndWait(5.0));

         HandWrenchTrajectoryMessage handWrenchTrajectoryMessage = new HandWrenchTrajectoryMessage();
         handWrenchTrajectoryMessage.setRobotSide(RobotSide.LEFT.toByte());
         Vector3D gravityComp = new Vector3D(0.0, 0.0, pendulumRobotDefinition.mass * 9.81);
         handWrenchTrajectoryMessage.getWrenchTrajectory().getWrenchTrajectoryPoints().add()
                                    .set(HumanoidMessageTools.createWrenchTrajectoryPointMessage(0.0, null, gravityComp));
         handWrenchTrajectoryMessage.getWrenchTrajectory().getWrenchTrajectoryPoints().add()
                                    .set(HumanoidMessageTools.createWrenchTrajectoryPointMessage(1000.0, null, gravityComp));
         handWrenchTrajectoryMessage.getWrenchTrajectory().getFrameInformation().setTrajectoryReferenceFrameId(worldFrame.hashCode());
         simulationTestHelper.publishToController(handWrenchTrajectoryMessage);

         assertTrue(simulationTestHelper.simulateAndWait(5.0));
      }
      else
      {
         assertTrue(simulationTestHelper.simulateAndWait(0.5));
      }
   }

   private final Controller createAttachmentController(ExternalWrenchPoint robotAttachmentPoint, Robot pendulumRobot)
   {
      SimJointBasics rootJoint = pendulumRobot.getJoint(FreeFloatingPendulumRobotDefinition.rootJointName);
      SimRigidBodyBasics pendulumBody = rootJoint.getSuccessor();
      ExternalWrenchPoint pendulumAttachmentPoint = rootJoint.getAuxialiryData().getExternalWrenchPoints().get(0);
      ReferenceFrame rootFrame = robotAttachmentPoint.getFrame().getRootFrame();

      return new Controller()
      {
         private final YoRegistry registry = new YoRegistry("AttachmentController");
         private final YoDouble kp = new YoDouble("kp", registry);
         private final YoDouble kd = new YoDouble("kd", registry);
         private final YoVector3D errorPosition = new YoVector3D("errorPosition", registry);
         private final YoVector3D errorVelocity = new YoVector3D("errorVelocity", registry);
         private final YoVector3D gravityCompensation = new YoVector3D("gravityCompensation", registry);

         {
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
         }

         @Override
         public YoRegistry getYoRegistry()
         {
            return registry;
         }
      };
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
