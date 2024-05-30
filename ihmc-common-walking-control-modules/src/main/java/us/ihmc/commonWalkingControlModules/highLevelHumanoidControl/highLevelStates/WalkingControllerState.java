package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.commonWalkingControlModules.capturePoint.LinearMomentumRateControlModule;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationDataReadOnly;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.WholeBodyControllerCoreFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
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
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.variable.YoBoolean;

public class WalkingControllerState extends HighLevelControllerState
{
   private final static HighLevelControllerName controllerState = HighLevelControllerName.WALKING;
   private final static FrameVector2D emptyVector = new FrameVector2D();

   private final WholeBodyControllerCore controllerCore;
   private final LinearMomentumRateControlModule linearMomentumRateControlModule;
   private final WalkingHighLevelHumanoidController walkingController;

   private final ExecutionTimer controllerCoreTimer = new ExecutionTimer("controllerCoreTimer", 1.0, registry);

   private final boolean deactivateAccelerationIntegrationInWBC;

   private boolean requestIntegratorReset = false;
   private final YoBoolean yoRequestingIntegratorReset = new YoBoolean("RequestingIntegratorReset", registry);

   private final BooleanParameter useCoPObjective = new BooleanParameter("UseCenterOfPressureObjectiveFromPlanner", registry, false);

   private final HighLevelHumanoidControllerToolbox controllerToolbox;

   private final KinematicsSimulationVirtualGroundReactionManager kinematicsSimulationVirtualGroundReactionManager;

   public WalkingControllerState(CommandInputManager commandInputManager,
                                 StatusMessageOutputManager statusOutputManager,
                                 HighLevelControlManagerFactory managerFactory,
                                 WholeBodyControllerCoreFactory controllerCoreFactory,
                                 HighLevelHumanoidControllerToolbox controllerToolbox,
                                 HighLevelControllerParameters highLevelControllerParameters,
                                 WalkingControllerParameters walkingControllerParameters)
   {
      super(controllerState,
            highLevelControllerParameters,
            MultiBodySystemTools.filterJoints(controllerToolbox.getControlledJoints(), OneDoFJointBasics.class));
      this.controllerToolbox = controllerToolbox;

      // create walking controller
      walkingController = new WalkingHighLevelHumanoidController(commandInputManager,
                                                                 statusOutputManager,
                                                                 managerFactory,
                                                                 walkingControllerParameters,
                                                                 controllerToolbox);

      // create controller core
      controllerCoreFactory.setFeedbackControllerTemplate(managerFactory.createFeedbackControlTemplate());
      controllerCore = controllerCoreFactory.getOrCreateWholeBodyControllerCore();
      ControllerCoreOutputReadOnly controllerCoreOutput = controllerCore.getOutputForHighLevelController();
      walkingController.setControllerCoreOutput(controllerCoreOutput);

      deactivateAccelerationIntegrationInWBC = highLevelControllerParameters.deactivateAccelerationIntegrationInTheWBC();

      //      linearMomentumRateControlModule = new LinearMomentumRateControlModule(controllerToolbox, walkingControllerParameters, registry);
      linearMomentumRateControlModule = controllerCoreFactory.getOrCreateLinearMomentumRateControlModule(registry);

      if (controllerToolbox.isKinematicsSimulation())
         kinematicsSimulationVirtualGroundReactionManager = new KinematicsSimulationVirtualGroundReactionManager(controllerToolbox, statusOutputManager, walkingController);
      else
         kinematicsSimulationVirtualGroundReactionManager = null;

      registry.addChild(walkingController.getYoVariableRegistry());
   }

   public void initialize()
   {
      if (getPreviousHighLevelControllerName() == HighLevelControllerName.PUSH_RECOVERY)
         walkingController.setShouldKeepInitialContacts(true);

      controllerCore.initialize();
      walkingController.initialize();
      linearMomentumRateControlModule.reset();
      requestIntegratorReset = true;

      if (kinematicsSimulationVirtualGroundReactionManager != null)
         kinematicsSimulationVirtualGroundReactionManager.initialize();
   }

   @Override
   public void doAction(double timeInState)
   {
      if (kinematicsSimulationVirtualGroundReactionManager != null)
      {
         kinematicsSimulationVirtualGroundReactionManager.update();
      }

      walkingController.doAction();

      linearMomentumRateControlModule.setInputFromWalkingStateMachine(walkingController.getLinearMomentumRateControlModuleInput());
      if (!linearMomentumRateControlModule.computeControllerCoreCommands())
      {
         controllerToolbox.reportControllerFailureToListeners(emptyVector);
      }
      walkingController.setLinearMomentumRateControlModuleOutput(linearMomentumRateControlModule.getOutputForWalkingStateMachine());

      ControllerCoreCommand controllerCoreCommand = walkingController.getControllerCoreCommand();
      controllerCoreCommand.addInverseDynamicsCommand(linearMomentumRateControlModule.getMomentumRateCommand());
      if (useCoPObjective.getValue())
      {
         controllerCoreCommand.addInverseDynamicsCommand(linearMomentumRateControlModule.getCenterOfPressureCommand());
      }
      if (kinematicsSimulationVirtualGroundReactionManager != null)
      {
         controllerCoreCommand.addInverseDynamicsCommand(kinematicsSimulationVirtualGroundReactionManager.getInverseDynamicsContactHolderCommandList());
      }

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
      return walkingController.isJointLoadBearing(jointName);
   }

   /**
    * Returns the currently active walking state. This is used for unit testing.
    * 
    * @return WalkingStateEnum
    */
   public WalkingStateEnum getWalkingStateEnum()
   {
      return walkingController.getWalkingStateEnum();
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(walkingController.getSCS2YoGraphics());
      if (group.isEmpty())
         return null;
      return group;
   }
}
