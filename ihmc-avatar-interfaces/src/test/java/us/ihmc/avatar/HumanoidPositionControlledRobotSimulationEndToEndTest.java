package us.ihmc.avatar;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.io.InputStream;
import java.util.Arrays;
import java.util.EnumMap;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInfo;

import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.WholeBodyJointspaceTrajectoryMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.RobotInitialSetup;
import us.ihmc.avatar.multiContact.KinematicsToolboxSnapshotDescription;
import us.ihmc.avatar.multiContact.MultiContactScriptMatcher;
import us.ihmc.avatar.multiContact.MultiContactScriptPostProcessor;
import us.ihmc.avatar.multiContact.MultiContactScriptReader;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulation;
import us.ihmc.avatar.testTools.scs2.SCS2AvatarTestingSimulationFactory;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerStateTransitionFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControllerStateFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.FreezeControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.JointspacePositionControllerState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.tuple3D.Point3D;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.SixDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateTransition;
import us.ihmc.scs2.definition.robot.RobotDefinition;
import us.ihmc.scs2.definition.robot.SixDoFJointDefinition;
import us.ihmc.scs2.definition.state.SixDoFJointState;
import us.ihmc.scs2.definition.visual.ColorDefinition;
import us.ihmc.scs2.definition.visual.MaterialDefinition;
import us.ihmc.scs2.simulation.SimulationSession;
import us.ihmc.scs2.simulation.robot.Robot;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

public abstract class HumanoidPositionControlledRobotSimulationEndToEndTest implements MultiRobotTestInterface
{
   protected static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final ColorDefinition ghostApperance = ColorDefinition.parse("#9e8329").derive(0, 1, 1, 0.25); // Some darkish orangish
   protected SCS2AvatarTestingSimulation simulationTestHelper = null;

   protected abstract HighLevelControllerParameters getPositionControlParameters(HighLevelControllerName positionControlState);

   protected abstract DRCRobotModel getGhostRobotModel();

   @AfterEach
   public void tearDown()
   {
      if (simulationTestHelper != null)
      {
         simulationTestHelper.finishTest();
         simulationTestHelper = null;
      }
   }

   @Test
   public void testFreezeController(TestInfo testInfo) throws Exception
   {
      simulationTestingParameters.setUsePefectSensors(true);

      DRCRobotModel robotModel = getRobotModel();
      FlatGroundEnvironment testEnvironment = new FlatGroundEnvironment();
      SCS2AvatarTestingSimulationFactory factory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(robotModel,
                                                                                                                         testEnvironment,
                                                                                                                         simulationTestingParameters);
      factory.getHighLevelHumanoidControllerFactory().addCustomControlState(new HighLevelControllerStateFactory()
      {
         private FreezeControllerState freezeControllerState;

         @Override
         public HighLevelControllerState getOrCreateControllerState(HighLevelControllerFactoryHelper controllerFactoryHelper)
         {
            if (freezeControllerState == null)
            {
               freezeControllerState = new FreezeControllerState(controllerFactoryHelper.getHighLevelHumanoidControllerToolbox().getControlledOneDoFJoints(),
                                                                 getPositionControlParameters(HighLevelControllerName.FREEZE_STATE),
                                                                 controllerFactoryHelper.getLowLevelControllerOutput());
            }

            return freezeControllerState;
         }

         @Override
         public HighLevelControllerName getStateEnum()
         {
            return HighLevelControllerName.FREEZE_STATE;
         }
      });

      // Automatic transition to FREEZE_STATE
      factory.getHighLevelHumanoidControllerFactory()
             .addCustomStateTransition(createImmediateTransition(HighLevelControllerName.WALKING, HighLevelControllerName.FREEZE_STATE));

      factory.setUseImpulseBasedPhysicsEngine(true);
      simulationTestHelper = factory.createAvatarTestingSimulation();
      simulationTestHelper.start();
      assertTrue(simulationTestHelper.simulateAndWait(3.0));
   }

   @Test
   public void testPositionController(TestInfo testInfo) throws Exception
   {
      createSimulation(testInfo, null, new FlatGroundEnvironment());
      simulationTestHelper.start();
      assertTrue(simulationTestHelper.simulateAndWait(1.0));

      WholeBodyJointspaceTrajectoryMessage message = new WholeBodyJointspaceTrajectoryMessage();

      FullHumanoidRobotModel controllerFullRobotModel = simulationTestHelper.getControllerFullRobotModel();

      for (OneDoFJointBasics joint : controllerFullRobotModel.getControllableOneDoFJoints())
      {
         int jointId = joint.hashCode();
         double position = joint.getQ();

         message.getJointHashCodes().add(jointId);
         message.getJointTrajectoryMessages().add().set(HumanoidMessageTools.createOneDoFJointTrajectoryMessage(0.1, position));
      }

      simulationTestHelper.publishToController(message);

      assertTrue(simulationTestHelper.simulateAndWait(3.0));
   }

   private void createSimulation(TestInfo testInfo,
                                 RobotInitialSetup<HumanoidFloatingRootJointRobot> initialSetup,
                                 CommonAvatarEnvironmentInterface environment)
   {
      createSimulation(testInfo, null, initialSetup, environment);
   }

   private void createSimulation(TestInfo testInfo,
                                 Robot ghostRobot,
                                 RobotInitialSetup<HumanoidFloatingRootJointRobot> initialSetup,
                                 CommonAvatarEnvironmentInterface environment)
   {
      simulationTestingParameters.setUsePefectSensors(true);

      DRCRobotModel robotModel = getRobotModel();
      SCS2AvatarTestingSimulationFactory simulationFactory = SCS2AvatarTestingSimulationFactory.createDefaultTestSimulationFactory(robotModel,
                                                                                                                                   environment,
                                                                                                                                   simulationTestingParameters);

      if (ghostRobot != null)
      {
         simulationFactory.addSecondaryRobot(ghostRobot);
      }

      simulationFactory.getHighLevelHumanoidControllerFactory().addCustomControlState(createControllerFactory(HighLevelControllerName.CUSTOM1));
      if (initialSetup != null)
         simulationFactory.setRobotInitialSetup(initialSetup);
      simulationFactory.setSimulationDataRecordTickPeriod(10);

      // Automatic transition to CUSTOM1
      simulationFactory.getHighLevelHumanoidControllerFactory()
                       .addCustomStateTransition(createImmediateTransition(HighLevelControllerName.WALKING, HighLevelControllerName.CUSTOM1));
      simulationFactory.setUseImpulseBasedPhysicsEngine(true);
      simulationTestHelper = simulationFactory.createAvatarTestingSimulation();
   }

   public void runRawScriptTest(TestInfo testInfo,
                                RobotInitialSetup<HumanoidFloatingRootJointRobot> initialSetup,
                                CommonAvatarEnvironmentInterface environment,
                                InputStream... scriptInputStreams)
         throws Exception
   {
      DRCRobotModel ghostRobotModel = getGhostRobotModel();
      RobotDefinition ghostRobotDefinition = ghostRobotModel.getRobotDefinition();
      ghostRobotDefinition.setName("Ghost");
      MaterialDefinition ghostMaterial = new MaterialDefinition(ghostApperance);
      ghostRobotDefinition.getAllRigidBodies()
                          .forEach(rigidBodyDefinition -> rigidBodyDefinition.getVisualDefinitions()
                                                                             .forEach(visual -> visual.setMaterialDefinition(ghostMaterial)));
      ghostRobotDefinition.ignoreAllJoints();
      ((SixDoFJointDefinition) ghostRobotDefinition.getRootJointDefinitions().get(0)).setInitialJointState(new SixDoFJointState(null,
                                                                                                                                new Point3D(-1000.0, 0, 0)));
      Robot ghostRobot = new Robot(ghostRobotDefinition, SimulationSession.DEFAULT_INERTIAL_FRAME);

      createSimulation(testInfo, ghostRobot, initialSetup, environment);
      simulationTestHelper.start();

      YoInteger totalNumberOfFrames = new YoInteger("totalNumberOfFrames", simulationTestHelper.getSimulationSession().getRootRegistry());
      YoInteger frameIndex = new YoInteger("frameIndex", simulationTestHelper.getSimulationSession().getRootRegistry());
      //      scs.setupGraph(totalNumberOfFrames.getFullNameString(), frameIndex.getFullNameString()); // TODO

      for (InputStream scriptInputStream : scriptInputStreams)
      {
         MultiContactScriptMatcher scriptMatcher = new MultiContactScriptMatcher(getRobotModel(), simulationTestHelper.getControllerFullRobotModel());
         MultiContactScriptReader scriptReader = new MultiContactScriptReader();
         assertTrue(scriptReader.loadScript(scriptInputStream), "Failed to load the script");
         assertTrue(scriptReader.hasNext(), "Script is empty");
         scriptMatcher.computeTransform(scriptReader.getFirst());
         scriptReader.applyTransform(scriptMatcher.getScriptTransform());
         totalNumberOfFrames.set(scriptReader.size());

         assertTrue(simulationTestHelper.simulateAndWait(1.0));

         OneDoFJointReadOnly[] allJoints = FullRobotModelUtils.getAllJointsExcludingHands(simulationTestHelper.getControllerFullRobotModel());

         double itemDuration = 1.0;

         while (scriptReader.hasNext())
         {
            KinematicsToolboxSnapshotDescription nextItem = scriptReader.next();
            frameIndex.increment();
            WholeBodyJointspaceTrajectoryMessage message = toWholeBodyJointspaceTrajectoryMessage(nextItem.getIkSolution(), allJoints, itemDuration);
            setSCSRobotConfiguration(nextItem.getIkSolution(), allJoints, ghostRobot);
            simulationTestHelper.publishToController(message);
            assertTrue(simulationTestHelper.simulateAndWait(itemDuration + 0.1));
         }
      }
   }

   private void setSCSRobotConfiguration(KinematicsToolboxOutputStatus ikSolution, OneDoFJointReadOnly[] allJoints, Robot ghostRobot)
   {
      assertEquals(Arrays.hashCode(allJoints), ikSolution.getJointNameHash(), "Message incompatible with robot.");

      SixDoFJointBasics rootJoint = (SixDoFJointBasics) ghostRobot.getRootBody().getChildrenJoints().get(0);
      rootJoint.setJointPosition(ikSolution.getDesiredRootTranslation());
      rootJoint.setJointOrientation(ikSolution.getDesiredRootOrientation());

      for (int i = 0; i < allJoints.length; i++)
      {
         String jointName = allJoints[i].getName();
         float q = ikSolution.getDesiredJointAngles().get(i);
         ghostRobot.getOneDoFJoint(jointName).setQ(q);
      }
      ghostRobot.updateFrames();
   }

   public void runProcessedScriptTest(TestInfo testInfo,
                                      RobotInitialSetup<HumanoidFloatingRootJointRobot> initialSetup,
                                      CommonAvatarEnvironmentInterface environment,
                                      double durationPerKeyframe,
                                      InputStream... scriptInputStreams)
         throws Exception
   {
      createSimulation(testInfo, initialSetup, environment);
      simulationTestHelper.start();

      for (InputStream scriptInputStream : scriptInputStreams)
      {
         MultiContactScriptReader scriptReader = new MultiContactScriptReader();
         assertTrue(scriptReader.loadScript(scriptInputStream), "Failed to load the script");
         assertTrue(scriptReader.hasNext(), "Script is empty");
         assertTrue(simulationTestHelper.simulateAndWait(1.0));

         MultiContactScriptPostProcessor scriptPostProcessor = new MultiContactScriptPostProcessor(getRobotModel());
         scriptPostProcessor.setDurationPerKeyframe(durationPerKeyframe);
         WholeBodyJointspaceTrajectoryMessage message = scriptPostProcessor.process1(scriptReader.getAllItems());

         simulationTestHelper.publishToController(message);
         assertTrue(simulationTestHelper.simulateAndWait(message.getJointTrajectoryMessages().get(0).getTrajectoryPoints().getLast().getTime() + 2.0));
      }
   }

   public static WholeBodyJointspaceTrajectoryMessage toWholeBodyJointspaceTrajectoryMessage(KinematicsToolboxOutputStatus ikSolution,
                                                                                             OneDoFJointReadOnly[] allJoints,
                                                                                             double trajectoryDuration)
   {
      assertEquals(Arrays.hashCode(allJoints), ikSolution.getJointNameHash(), "Message incompatible with robot.");

      WholeBodyJointspaceTrajectoryMessage message = new WholeBodyJointspaceTrajectoryMessage();

      for (int i = 0; i < allJoints.length; i++)
      {
         float q_d = ikSolution.getDesiredJointAngles().get(i);
         message.getJointHashCodes().add(allJoints[i].hashCode());
         message.getJointTrajectoryMessages().add().set(HumanoidMessageTools.createOneDoFJointTrajectoryMessage(trajectoryDuration, q_d));
      }

      return message;
   }

   private HighLevelControllerStateFactory createControllerFactory(HighLevelControllerName controllerName)
   {
      return new HighLevelControllerStateFactory()
      {
         @Override
         public HighLevelControllerName getStateEnum()
         {
            return controllerName;
         }

         @Override
         public HighLevelControllerState getOrCreateControllerState(HighLevelControllerFactoryHelper controllerFactoryHelper)
         {
            CommandInputManager commandInputManager = controllerFactoryHelper.getCommandInputManager();
            StatusMessageOutputManager statusOutputManager = controllerFactoryHelper.getStatusMessageOutputManager();
            OneDoFJointBasics[] controlledJoints = controllerFactoryHelper.getHighLevelHumanoidControllerToolbox().getControlledOneDoFJoints();
            HighLevelHumanoidControllerToolbox controllerToolbox = controllerFactoryHelper.getHighLevelHumanoidControllerToolbox();
            HighLevelControllerParameters highLevelControllerParameters = getPositionControlParameters(getStateEnum());
            JointDesiredOutputListReadOnly highLevelControllerOutput = controllerFactoryHelper.getLowLevelControllerOutput();
            return new JointspacePositionControllerState(controllerName,
                                                         commandInputManager,
                                                         statusOutputManager,
                                                         controlledJoints,
                                                         controllerToolbox,
                                                         highLevelControllerParameters,
                                                         highLevelControllerOutput);
         }
      };
   }

   private static ControllerStateTransitionFactory<HighLevelControllerName> createImmediateTransition(HighLevelControllerName from, HighLevelControllerName to)
   {
      return new ControllerStateTransitionFactory<HighLevelControllerName>()
      {
         @Override
         public HighLevelControllerName getStateToAttachEnum()
         {
            return from;
         }

         @Override
         public StateTransition<HighLevelControllerName> getOrCreateStateTransition(EnumMap<HighLevelControllerName, ? extends State> stateMap,
                                                                                    HighLevelControllerFactoryHelper controllerFactoryHelper,
                                                                                    YoRegistry parentRegistry)
         {
            return new StateTransition<>(to, t -> true);
         }
      };
   }
}
