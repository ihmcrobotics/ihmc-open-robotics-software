package us.ihmc.atlas;

import org.junit.jupiter.api.Assumptions;
import org.junit.jupiter.api.Disabled;
import org.junit.jupiter.api.Test;
import org.opentest4j.TestAbortedException;
import us.ihmc.avatar.DRCFlatGroundWalkingTest;
import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.drcRobot.RobotTarget;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerStateTransitionFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControllerStateFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.RemoteControllerStateFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.JointspacePositionControllerState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.stateMachine.core.State;
import us.ihmc.robotics.stateMachine.core.StateTransition;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.simulationConstructionSetTools.bambooTools.BambooTools;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.SimulationConstructionSet;
import us.ihmc.simulationconstructionset.util.ControllerFailureException;
import us.ihmc.simulationconstructionset.util.simulationRunner.BlockingSimulationRunner.SimulationExceededMaximumTimeException;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;

import java.util.EnumMap;

import static us.ihmc.robotics.Assert.fail;

// This test is slow but very important, let's keep it in the FAST build please. (Sylvain)
public class AtlasRemoteTest extends DRCFlatGroundWalkingTest
{
   private DRCRobotModel robotModel;

   @Override
   public boolean doPelvisWarmup()
   {
      return true;
   }

   @Test
   public void testRemote() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
      DRCSimulationTestHelper drcSimulationTestHelper = new DRCSimulationTestHelper(super.getSimulationTestingParameters(), robotModel, new FlatGroundEnvironment());
      drcSimulationTestHelper.setAddFootstepMessageGenerator(true);
      drcSimulationTestHelper.setUseHeadingAndVelocityScript(true);
      drcSimulationTestHelper.setCheatWithGroundHeightAtFootstep(false);
      drcSimulationTestHelper.setWalkingScriptParameters(getWalkingScriptParameters());
      drcSimulationTestHelper.getSimulationStarter().registerHighLevelControllerState(new RemoteControllerStateFactory());
      drcSimulationTestHelper.getSimulationStarter().setInitialStateEnum(HighLevelControllerName.REMOTE);
      drcSimulationTestHelper.createSimulation(robotModel.getSimpleRobotName() + "Reset");

//      drcSimulationTestHelper.getSimulationStarter().registerHighLevelControllerState(createControllerFactory(HighLevelControllerName.CUSTOM1));
//      drcSimulationTestHelper.getSimulationStarter().registerControllerStateTransition(createImmediateTransition(HighLevelControllerName.WALKING,
//              HighLevelControllerName.FREEZE_STATE));
//      drcSimulationTestHelper.getSimulationStarter().getAvatarSimulation().getHighLevelHumanoidControllerFactory().setInitialState(HighLevelControllerName.REMOTE);

      SimulationConstructionSet scs = drcSimulationTestHelper.getSimulationConstructionSet();
      // getHighLevelHumanoidControllerFactory

      YoBoolean walk = (YoBoolean) scs.findVariable("walkCSG");
//      walk.set(true);

      double timeIncrement = 1.0;
      while (scs.getTime() < 1000)
      {
         drcSimulationTestHelper.simulateAndBlock(timeIncrement);
      }
   }

   @Override
   @Test
   public void testReset() throws SimulationExceededMaximumTimeException, ControllerFailureException
   {
      robotModel = new AtlasRobotModel(AtlasRobotVersion.ATLAS_UNPLUGGED_V5_NO_HANDS, RobotTarget.SCS, false);
      super.testReset();
   }

   @Override
   public DRCRobotModel getRobotModel()
   {
      return robotModel;
   }

   @Override
   public String getSimpleRobotName()
   {
      return BambooTools.getSimpleRobotNameFor(BambooTools.SimpleRobotNameKeys.ATLAS);
   }
//
//   private HighLevelControllerStateFactory createControllerFactory(HighLevelControllerName controllerName)
//   {
//      return new HighLevelControllerStateFactory()
//      {
//         @Override
//         public HighLevelControllerName getStateEnum()
//         {
//            return controllerName;
//         }
//
//         @Override
//         public HighLevelControllerState getOrCreateControllerState(HighLevelControllerFactoryHelper controllerFactoryHelper)
//         {
//            CommandInputManager commandInputManager = controllerFactoryHelper.getCommandInputManager();
//            StatusMessageOutputManager statusOutputManager = controllerFactoryHelper.getStatusMessageOutputManager();
//            OneDoFJointBasics[] controlledJoints = controllerFactoryHelper.getHighLevelHumanoidControllerToolbox().getControlledOneDoFJoints();
//            HighLevelHumanoidControllerToolbox controllerToolbox = controllerFactoryHelper.getHighLevelHumanoidControllerToolbox();
//            HighLevelControllerParameters highLevelControllerParameters = getPositionControlParameters(getStateEnum());
//            JointDesiredOutputListReadOnly highLevelControllerOutput = controllerFactoryHelper.getLowLevelControllerOutput();
//            return new JointspacePositionControllerState(controllerName,
//                    commandInputManager,
//                    statusOutputManager,
//                    controlledJoints,
//                    controllerToolbox,
//                    highLevelControllerParameters,
//                    highLevelControllerOutput);
//         }
//      };
//   }
//
//   private static ControllerStateTransitionFactory<HighLevelControllerName> createImmediateTransition(HighLevelControllerName from, HighLevelControllerName to)
//   {
//      return new ControllerStateTransitionFactory<HighLevelControllerName>()
//      {
//         @Override
//         public HighLevelControllerName getStateToAttachEnum()
//         {
//            return from;
//         }
//
//         @Override
//         public StateTransition<HighLevelControllerName> getOrCreateStateTransition(EnumMap<HighLevelControllerName, ? extends State> stateMap,
//                                                                                    HighLevelControllerFactoryHelper controllerFactoryHelper,
//                                                                                    YoRegistry parentRegistry)
//         {
//            return new StateTransition<>(to, t -> true);
//         }
//      };
//   }
}
