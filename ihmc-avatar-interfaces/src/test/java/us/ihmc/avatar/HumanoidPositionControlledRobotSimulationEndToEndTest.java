package us.ihmc.avatar;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.awt.Color;
import java.io.InputStream;
import java.util.Arrays;
import java.util.Collections;
import java.util.EnumMap;
import java.util.List;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInfo;

import controller_msgs.msg.dds.KinematicsToolboxOutputStatus;
import controller_msgs.msg.dds.WholeBodyJointspaceTrajectoryMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.initialSetup.DRCRobotInitialSetup;
import us.ihmc.avatar.multiContact.KinematicsToolboxSnapshotDescription;
import us.ihmc.avatar.multiContact.MultiContactScriptMatcher;
import us.ihmc.avatar.multiContact.MultiContactScriptPostProcessor;
import us.ihmc.avatar.multiContact.MultiContactScriptReader;
import us.ihmc.avatar.networkProcessor.kinematicsToolboxModule.KinematicsToolboxControllerTest;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerStateTransitionFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControllerStateFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.FreezeControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.JointspacePositionControllerState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.graphicsDescription.appearance.YoAppearanceRGBColor;
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointReadOnly;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotModels.FullRobotModelUtils;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateTransition;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.simulationConstructionSetTools.util.HumanoidFloatingRootJointRobot;
import us.ihmc.simulationConstructionSetTools.util.environments.CommonAvatarEnvironmentInterface;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.Robot;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.ground.TerrainObject3D;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoInteger;

public abstract class HumanoidPositionControlledRobotSimulationEndToEndTest implements MultiRobotTestInterface
{
   protected static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   private static final YoAppearanceRGBColor ghostApperance = new YoAppearanceRGBColor(Color.decode("#9e8329"), 0.25); // Some darkish orangish
   protected DRCSimulationTestHelper drcSimulationTestHelper = null;

   protected abstract HighLevelControllerParameters getPositionControlParameters(HighLevelControllerName positionControlState);

   protected abstract DRCRobotModel getGhostRobotModel();

   @AfterEach
   public void tearDown()
   {
      if (simulationTestingParameters.getKeepSCSUp())
         ThreadTools.sleepForever();

      if (drcSimulationTestHelper != null)
      {
         drcSimulationTestHelper.destroySimulation();
         drcSimulationTestHelper = null;
      }
   }

   @Test
   public void testFreezeController(TestInfo testInfo) throws Exception
   {
      simulationTestingParameters.setUsePefectSensors(true);

      DRCRobotModel robotModel = getRobotModel();
      FlatGroundEnvironment testEnvironment = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, testEnvironment);
      drcSimulationTestHelper.getSimulationStarter().registerHighLevelControllerState(new HighLevelControllerStateFactory()
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
      drcSimulationTestHelper.getSimulationStarter().registerControllerStateTransition(createImmediateTransition(HighLevelControllerName.WALKING,
                                                                                                                 HighLevelControllerName.FREEZE_STATE));

      drcSimulationTestHelper.getSCSInitialSetup().setUseExperimentalPhysicsEngine(true);
      drcSimulationTestHelper.createSimulation(testInfo.getTestClass().getClass().getSimpleName() + "." + testInfo.getTestMethod().get().getName() + "()");
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0));
   }

   @Test
   public void testPositionController(TestInfo testInfo) throws Exception
   {
      createSimulation(testInfo, null, new FlatGroundEnvironment());
      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));

      WholeBodyJointspaceTrajectoryMessage message = new WholeBodyJointspaceTrajectoryMessage();

      FullHumanoidRobotModel controllerFullRobotModel = drcSimulationTestHelper.getControllerFullRobotModel();

      for (OneDoFJointBasics joint : controllerFullRobotModel.getControllableOneDoFJoints())
      {
         int jointId = joint.hashCode();
         double position = joint.getQ();

         message.getJointHashCodes().add(jointId);
         message.getJointTrajectoryMessages().add().set(HumanoidMessageTools.createOneDoFJointTrajectoryMessage(0.1, position));
      }

      drcSimulationTestHelper.publishToController(message);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0));
   }

   private void createSimulation(TestInfo testInfo,
                                 DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> initialSetup,
                                 CommonAvatarEnvironmentInterface environment)
   {
      createSimulation(testInfo, null, initialSetup, environment);
   }

   private void createSimulation(TestInfo testInfo,
                                 Robot ghostRobot,
                                 DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> initialSetup,
                                 CommonAvatarEnvironmentInterface environment)
   {
      simulationTestingParameters.setUsePefectSensors(true);

      DRCRobotModel robotModel = getRobotModel();
      if (ghostRobot == null)
      {
         drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, environment);
      }
      else
      {
         drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, new CommonAvatarEnvironmentInterface()
         {
            @Override
            public List<? extends Robot> getEnvironmentRobots()
            {
               return Collections.singletonList(ghostRobot);
            }

            @Override
            public TerrainObject3D getTerrainObject3D()
            {
               return environment.getTerrainObject3D();
            }
         });
      }

      drcSimulationTestHelper.getSimulationStarter().registerHighLevelControllerState(createControllerFactory(HighLevelControllerName.CUSTOM1));
      if (initialSetup != null)
         drcSimulationTestHelper.setInitialSetup(initialSetup);
      drcSimulationTestHelper.getSCSInitialSetup().setRecordFrequency(10);

      // Automatic transition to CUSTOM1
      drcSimulationTestHelper.getSimulationStarter()
                             .registerControllerStateTransition(createImmediateTransition(HighLevelControllerName.WALKING, HighLevelControllerName.CUSTOM1));
      drcSimulationTestHelper.getSCSInitialSetup().setUseExperimentalPhysicsEngine(true);
      drcSimulationTestHelper.createSimulation(testInfo.getTestClass().getClass().getSimpleName() + "." + testInfo.getTestMethod().get().getName() + "()");
      drcSimulationTestHelper.getSimulationConstructionSet().setFastSimulate(true, 10);
   }

   public void runRawScriptTest(TestInfo testInfo,
                                DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> initialSetup,
                                CommonAvatarEnvironmentInterface environment,
                                InputStream... scriptInputStreams)
         throws Exception
   {
      DRCRobotModel ghostRobotModel = getGhostRobotModel();
      ghostRobotModel.getRobotDescription().setName("Ghost");
      KinematicsToolboxControllerTest.recursivelyModifyGraphics(ghostRobotModel.getRobotDescription().getRootJoints().get(0), ghostApperance);
      HumanoidFloatingRootJointRobot ghostRobot = ghostRobotModel.createHumanoidFloatingRootJointRobot(false);
      ghostRobot.getRootJoint().setPosition(-1000.0, 0., 0.);
      ghostRobot.setDynamic(false);
      ghostRobot.setGravity(0);

      createSimulation(testInfo, ghostRobot, initialSetup, environment);

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      YoInteger totalNumberOfFrames = new YoInteger("totalNumberOfFrames", scs.getRootRegistry());
      YoInteger frameIndex = new YoInteger("frameIndex", scs.getRootRegistry());
      scs.setupGraph(totalNumberOfFrames.getFullNameString(), frameIndex.getFullNameString());

      for (InputStream scriptInputStream : scriptInputStreams)
      {
         MultiContactScriptMatcher scriptMatcher = new MultiContactScriptMatcher(getRobotModel(), drcSimulationTestHelper.getControllerFullRobotModel());
         MultiContactScriptReader scriptReader = new MultiContactScriptReader();
         assertTrue(scriptReader.loadScript(scriptInputStream), "Failed to load the script");
         assertTrue(scriptReader.hasNext(), "Script is empty");
         scriptMatcher.computeTransform(scriptReader.getFirst());
         scriptReader.applyTransform(scriptMatcher.getScriptTransform());
         totalNumberOfFrames.set(scriptReader.size());

         assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));

         OneDoFJointReadOnly[] allJoints = FullRobotModelUtils.getAllJointsExcludingHands(drcSimulationTestHelper.getControllerFullRobotModel());

         double itemDuration = 1.0;

         while (scriptReader.hasNext())
         {
            KinematicsToolboxSnapshotDescription nextItem = scriptReader.next();
            frameIndex.increment();
            WholeBodyJointspaceTrajectoryMessage message = toWholeBodyJointspaceTrajectoryMessage(nextItem.getIkSolution(), allJoints, itemDuration);
            setSCSRobotConfiguration(nextItem.getIkSolution(), allJoints, ghostRobot);
            drcSimulationTestHelper.publishToController(message);
            assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(itemDuration + 0.1));
         }
      }
   }

   private void setSCSRobotConfiguration(KinematicsToolboxOutputStatus ikSolution, OneDoFJointReadOnly[] allJoints, HumanoidFloatingRootJointRobot ghostRobot)
   {
      assertEquals(Arrays.hashCode(allJoints), ikSolution.getJointNameHash(), "Message incompatible with robot.");

      ghostRobot.getRootJoint().setPosition(ikSolution.getDesiredRootTranslation());
      ghostRobot.getRootJoint().setOrientation(ikSolution.getDesiredRootOrientation());

      for (int i = 0; i < allJoints.length; i++)
      {
         String jointName = allJoints[i].getName();
         float q = ikSolution.getDesiredJointAngles().get(i);
         ghostRobot.getOneDegreeOfFreedomJoint(jointName).setQ(q);
      }
      ghostRobot.update();
   }

   public void runProcessedScriptTest(TestInfo testInfo,
                                      DRCRobotInitialSetup<HumanoidFloatingRootJointRobot> initialSetup,
                                      CommonAvatarEnvironmentInterface environment,
                                      double durationPerKeyframe,
                                      InputStream... scriptInputStreams)
         throws Exception
   {
      createSimulation(testInfo, initialSetup, environment);

      for (InputStream scriptInputStream : scriptInputStreams)
      {
         MultiContactScriptReader scriptReader = new MultiContactScriptReader();
         assertTrue(scriptReader.loadScript(scriptInputStream), "Failed to load the script");
         assertTrue(scriptReader.hasNext(), "Script is empty");
         assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(1.0));

         MultiContactScriptPostProcessor scriptPostProcessor = new MultiContactScriptPostProcessor(getRobotModel());
         scriptPostProcessor.setDurationPerKeyframe(durationPerKeyframe);
         WholeBodyJointspaceTrajectoryMessage message = scriptPostProcessor.process1(scriptReader.getAllItems());

         drcSimulationTestHelper.publishToController(message);
         assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(message.getJointTrajectoryMessages().get(0).getTrajectoryPoints().getLast()
                                                                                      .getTime()
               + 2.0));
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
