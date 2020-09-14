package us.ihmc.avatar;

import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.Collections;
import java.util.List;
import java.util.stream.Stream;

import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.TestInfo;

import us.ihmc.avatar.drcRobot.DRCRobotModel;
import us.ihmc.avatar.testTools.DRCSimulationTestHelper;
import us.ihmc.commonWalkingControlModules.configurations.GroupParameter;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.parameters.JointAccelerationIntegrationParametersReadOnly;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.HighLevelControllerFactoryHelper;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControllerStateFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.FreezeControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WholeBodySetpointParameters;
import us.ihmc.commons.thread.ThreadTools;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.interfaces.JointReadOnly;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredBehavior;
import us.ihmc.sensorProcessing.outputData.JointDesiredBehaviorReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredControlMode;
import us.ihmc.simulationConstructionSetTools.util.environments.FlatGroundEnvironment;
import us.ihmc.simulationconstructionset.util.simulationTesting.SimulationTestingParameters;

public abstract class HumanoidPositionControlledRobotSimulationEndToEndTest implements MultiRobotTestInterface
{
   protected static final SimulationTestingParameters simulationTestingParameters = SimulationTestingParameters.createFromSystemProperties();
   protected DRCSimulationTestHelper drcSimulationTestHelper = null;

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
                                                                 new PositionControlParametersWrapper(HighLevelControllerName.FREEZE_STATE),
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
      drcSimulationTestHelper.getSCSInitialSetup().setUseExperimentalPhysicsEngine(true);
      drcSimulationTestHelper.createSimulation(testInfo.getTestClass().getClass().getSimpleName() + "." + testInfo.getTestMethod().get().getName() + "()");
      // Switch to zero-torque controller.
      drcSimulationTestHelper.getAvatarSimulation().getHighLevelHumanoidControllerFactory().addRequestableTransition(HighLevelControllerName.WALKING,
                                                                                                                     HighLevelControllerName.FREEZE_STATE);
      drcSimulationTestHelper.getAvatarSimulation().getHighLevelHumanoidControllerFactory().getRequestedControlStateEnum()
                             .set(HighLevelControllerName.DO_NOTHING_BEHAVIOR);

      assertTrue(drcSimulationTestHelper.simulateAndBlockAndCatchExceptions(3.0));
   }

   private class PositionControlParametersWrapper implements HighLevelControllerParameters
   {
      private final DRCRobotModel robotModel = getRobotModel();
      private final HighLevelControllerParameters originalParameters = robotModel.getHighLevelControllerParameters();
      private final OneDoFJointBasics[] oneDoFJoints = robotModel.createFullRobotModel().getOneDoFJoints();
      private final HighLevelControllerName positionControlState;

      public PositionControlParametersWrapper(HighLevelControllerName positionControlState)
      {
         this.positionControlState = positionControlState;
      }

      @Override
      public WholeBodySetpointParameters getStandPrepParameters()
      {
         return originalParameters.getStandPrepParameters();
      }

      @Override
      public HighLevelControllerName getDefaultInitialControllerState()
      {
         return originalParameters.getDefaultInitialControllerState();
      }

      @Override
      public HighLevelControllerName getFallbackControllerState()
      {
         return originalParameters.getDefaultInitialControllerState();
      }

      @Override
      public boolean automaticallyTransitionToWalkingWhenReady()
      {
         return originalParameters.automaticallyTransitionToWalkingWhenReady();
      }

      @Override
      public double getTimeToMoveInStandPrep()
      {
         return originalParameters.getTimeToMoveInStandPrep();
      }

      @Override
      public double getMinimumTimeInStandReady()
      {
         return originalParameters.getMinimumTimeInStandReady();
      }

      @Override
      public double getTimeInStandTransition()
      {
         return originalParameters.getTimeInStandTransition();
      }

      @Override
      public double getCalibrationDuration()
      {
         return originalParameters.getCalibrationDuration();
      }

      @Override
      public List<GroupParameter<JointDesiredBehaviorReadOnly>> getDesiredJointBehaviors(HighLevelControllerName state)
      {
         if (state == positionControlState)
            return getPositionControlBehavior();
         else
            return originalParameters.getDesiredJointBehaviors(state);
      }

      @Override
      public List<GroupParameter<JointDesiredBehaviorReadOnly>> getDesiredJointBehaviorsUnderLoad(HighLevelControllerName state)
      {
         if (state == positionControlState)
            return getPositionControlBehavior();
         else
            return originalParameters.getDesiredJointBehaviorsUnderLoad(state);
      }

      @Override
      public List<GroupParameter<JointAccelerationIntegrationParametersReadOnly>> getJointAccelerationIntegrationParameters(HighLevelControllerName state)
      {
         try
         {
            return originalParameters.getJointAccelerationIntegrationParameters(state);
         }
         catch (RuntimeException e)
         {
            return null;
         }
      }

      @Override
      public List<GroupParameter<JointAccelerationIntegrationParametersReadOnly>> getJointAccelerationIntegrationParametersUnderLoad(HighLevelControllerName state)
      {
         return originalParameters.getJointAccelerationIntegrationParametersUnderLoad(state);
      }

      @Override
      public boolean deactivateAccelerationIntegrationInTheWBC()
      {
         return originalParameters.deactivateAccelerationIntegrationInTheWBC();
      }

      private List<GroupParameter<JointDesiredBehaviorReadOnly>> getPositionControlBehavior()
      {
         return Collections.singletonList(new GroupParameter<>("AllJoint",
                                                               new JointDesiredBehavior(JointDesiredControlMode.POSITION),
                                                               Stream.of(oneDoFJoints).map(JointReadOnly::getName).toArray(String[]::new)));
      }
   }
}
