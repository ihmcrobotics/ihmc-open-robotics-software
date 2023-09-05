package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController;

import us.ihmc.commonWalkingControlModules.capturePoint.LinearMomentumRateControlModule;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationDataReadOnly;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.WholeBodyControllerCoreFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.pushRecoveryController.states.PushRecoveryStateEnum;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.yoVariables.variable.YoBoolean;

public class PushRecoveryControllerState extends HighLevelControllerState
{
   private final static HighLevelControllerName controllerState = HighLevelControllerName.PUSH_RECOVERY;
   private final static FrameVector2D emptyVector = new FrameVector2D();

   private final WholeBodyControllerCore controllerCore;
   private final PushRecoveryHighLevelHumanoidController pushRecoveryController;

   private final ExecutionTimer controllerCoreTimer = new ExecutionTimer("controllerCoreTimer", 1.0, registry);

   private final boolean deactivateAccelerationIntegrationInWBC;
   private final LinearMomentumRateControlModule linearMomentumRateControlModule;

   private boolean requestIntegratorReset = false;
   private final YoBoolean yoRequestingIntegratorReset = new YoBoolean("RequestingIntegratorReset", registry);

   private final HighLevelHumanoidControllerToolbox controllerToolbox;

   public PushRecoveryControllerState(CommandInputManager commandInputManager,
                                      StatusMessageOutputManager statusOutputManager,
                                      PushRecoveryControlManagerFactory managerFactory,
                                      WholeBodyControllerCoreFactory controllerCoreFactory,
                                      HighLevelHumanoidControllerToolbox controllerToolbox,
                                      HighLevelControllerParameters highLevelControllerParameters,
                                      PushRecoveryControllerParameters pushRecoveryControllerParameters,
                                      WalkingControllerParameters walkingControllerParameters)
   {
      super(controllerState, highLevelControllerParameters, MultiBodySystemTools.filterJoints(controllerToolbox.getControlledJoints(), OneDoFJointBasics.class));
      this.controllerToolbox = controllerToolbox;

      // create push recovery controller
      pushRecoveryController = new PushRecoveryHighLevelHumanoidController(commandInputManager, statusOutputManager, managerFactory, pushRecoveryControllerParameters,
                                                                           walkingControllerParameters.getKneePrivilegedConfigurationParameters(), controllerToolbox);

      // get controller core
      controllerCoreFactory.setFeedbackControllerTemplate(managerFactory.createFeedbackControlTemplate());
      controllerCore = controllerCoreFactory.getOrCreateWholeBodyControllerCore();
      ControllerCoreOutputReadOnly controllerCoreOutput = controllerCore.getOutputForHighLevelController();
      pushRecoveryController.setControllerCoreOutput(controllerCoreOutput);

      deactivateAccelerationIntegrationInWBC = highLevelControllerParameters.deactivateAccelerationIntegrationInTheWBC();

      linearMomentumRateControlModule = controllerCoreFactory.getOrCreateLinearMomentumRateControlModule(registry);

      registry.addChild(pushRecoveryController.getYoVariableRegistry());
   }

   public void initialize()
   {
      pushRecoveryController.initialize();
      requestIntegratorReset = true;
   }

   @Override
   public void doAction(double timeInState)
   {
      pushRecoveryController.doAction();

      linearMomentumRateControlModule.setInputFromWalkingStateMachine(pushRecoveryController.getLinearMomentumRateControlModuleInput());
      if (!linearMomentumRateControlModule.computeControllerCoreCommands())
      {
         controllerToolbox.reportControllerFailureToListeners(emptyVector);
      }
      pushRecoveryController.setLinearMomentumRateControlModuleOutput(linearMomentumRateControlModule.getOutputForWalkingStateMachine());

      ControllerCoreCommand controllerCoreCommand = pushRecoveryController.getControllerCoreCommand();
      controllerCoreCommand.addInverseDynamicsCommand(linearMomentumRateControlModule.getMomentumRateCommand());

      JointDesiredOutputList stateSpecificJointSettings = getStateSpecificJointSettings();

      if (requestIntegratorReset)
      {
         stateSpecificJointSettings.requestIntegratorReset();
         requestIntegratorReset = false;
         yoRequestingIntegratorReset.set(true);
      }
      else
      {
         yoRequestingIntegratorReset.set(false);
      }

      JointAccelerationIntegrationCommand accelerationIntegrationCommand = getAccelerationIntegrationCommand();
      if (!deactivateAccelerationIntegrationInWBC)
      {
         controllerCoreCommand.addInverseDynamicsCommand(accelerationIntegrationCommand);
      }
      controllerCoreCommand.completeLowLevelJointData(stateSpecificJointSettings);

      controllerCoreTimer.startMeasurement();
      controllerCore.compute(controllerCoreCommand);
      controllerCoreTimer.stopMeasurement();

      linearMomentumRateControlModule.setInputFromControllerCore(controllerCore.getControllerCoreOutput());
      linearMomentumRateControlModule.computeAchievedCMP();
   }

   @Override
   public void onEntry()
   {
      initialize();
   }

   @Override
   public void onExit(double timeInState)
   {
      pushRecoveryController.reset();
      controllerToolbox.reportChangeOfRobotMotionStatus(RobotMotionStatus.UNKNOWN);
   }

   @Override
   public JointDesiredOutputListReadOnly getOutputForLowLevelController()
   {
      return controllerCore.getOutputForLowLevelController();
   }

   @Override
   public RootJointDesiredConfigurationDataReadOnly getOutputForRootJoint()
   {
      return controllerCore.getOutputForRootJoint();
   }

   @Override
   public boolean isJointLoadBearing(String jointName)
   {
      return pushRecoveryController.isJointLoadBearing(jointName);
   }

   @Override
   public boolean isDone(double timeInState)
   {
      if (pushRecoveryController.getRecoveringStateEnum() != PushRecoveryStateEnum.TO_STANDING)
         return false;

      return pushRecoveryController.getRecoveringState().isDone(timeInState);
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(pushRecoveryController.getSCS2YoGraphics());
      return group;
   }
}
