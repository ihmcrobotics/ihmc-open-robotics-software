package us.ihmc.avatar;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.EnumMap;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInfo;

import controller_msgs.msg.dds.WholeBodyJointspaceTrajectoryMessage;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
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
import us.ihmc.humanoidRobotics.communication.packets.HumanoidMessageTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateTransition;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;
import us.ihmc.yoVariables.registry.YoRegistry;

public abstract class HumanoidPositionControlledRobotSimulationEndToEndTest implements MultiRobotTestInterface
{
   protected static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   protected DRCSimulationTestHelper drcSimulationTestHelper = null;

   protected abstract HighLevelControllerParameters getPositionControlParameters(HighLevelControllerName positionControlState);

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
      simulationTestingParameters.setUsePefectSensors(true);

      DRCRobotModel robotModel = getRobotModel();
      FlatGroundEnvironment testEnvironment = new FlatGroundEnvironment();
      drcSimulationTestHelper = new DRCSimulationTestHelper(simulationTestingParameters, robotModel, testEnvironment);
      drcSimulationTestHelper.getSimulationStarter().registerHighLevelControllerState(new HighLevelControllerStateFactory()
      {
         @Override
         public HighLevelControllerName getStateEnum()
         {
            return HighLevelControllerName.CUSTOM1;
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
            return new JointspacePositionControllerState(getStateEnum(),
                                                         commandInputManager,
                                                         statusOutputManager,
                                                         controlledJoints,
                                                         controllerToolbox,
                                                         highLevelControllerParameters,
                                                         highLevelControllerOutput);
         }
      });

      // Automatic transition to CUSTOM1
      drcSimulationTestHelper.getSimulationStarter()
                             .registerControllerStateTransition(createImmediateTransition(HighLevelControllerName.WALKING, HighLevelControllerName.CUSTOM1));

      drcSimulationTestHelper.getSCSInitialSetup().setUseExperimentalPhysicsEngine(true);
      drcSimulationTestHelper.createSimulation(testInfo.getTestClass().getClass().getSimpleName() + "." + testInfo.getTestMethod().get().getName() + "()");
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

   private ControllerStateTransitionFactory<HighLevelControllerName> createImmediateTransition(HighLevelControllerName from, HighLevelControllerName to)
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
