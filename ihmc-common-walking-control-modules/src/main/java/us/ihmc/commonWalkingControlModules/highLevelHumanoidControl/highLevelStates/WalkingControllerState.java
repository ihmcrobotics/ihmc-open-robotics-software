package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates;

import us.ihmc.commonWalkingControlModules.capturePoint.LinearMomentumRateControlModule;
import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.JointAccelerationIntegrationCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationDataReadOnly;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.WalkingStateEnum;
import us.ihmc.commonWalkingControlModules.messageHandlers.PlanarRegionsListHandler;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.FrameVector2D;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.graphicsDescription.yoGraphics.YoGraphicsListRegistry;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactableFoot;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.mecano.multiBodySystem.OneDoFJoint;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.JointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.RigidBodyBasics;
import us.ihmc.mecano.tools.MultiBodySystemTools;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.sensorProcessing.frames.CommonHumanoidReferenceFrames;
import us.ihmc.sensorProcessing.model.RobotMotionStatus;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.yoVariables.parameters.BooleanParameter;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoVariable;

public class WalkingControllerState extends HighLevelControllerState
{
   private final static HighLevelControllerName controllerState = HighLevelControllerName.WALKING;
   private final static FrameVector2D emptyVector = new FrameVector2D();

   private final WholeBodyControllerCore controllerCore;
   private final LinearMomentumRateControlModule linearMomentumRateControlModule;
   private final WalkingHighLevelHumanoidController walkingController;

   private final ExecutionTimer controllerCoreTimer = new ExecutionTimer("controllerCoreTimer", 1.0, registry);

   private boolean setupInverseDynamicsSolver = true;
   private boolean setupInverseKinematicsSolver = false;
   private boolean setupVirtualModelControlSolver = false;

   private final boolean deactivateAccelerationIntegrationInWBC;

   private boolean requestIntegratorReset = false;
   private final YoBoolean yoRequestingIntegratorReset = new YoBoolean("RequestingIntegratorReset", registry);

   private final BooleanParameter useCoPObjective = new BooleanParameter("UseCenterOfPressureObjectiveFromPlanner", registry, false);

   private final HighLevelHumanoidControllerToolbox controllerToolbox;

   public WalkingControllerState(CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager,
                                 HighLevelControlManagerFactory managerFactory, HighLevelHumanoidControllerToolbox controllerToolbox,
                                 HighLevelControllerParameters highLevelControllerParameters, WalkingControllerParameters walkingControllerParameters)
   {
      super(controllerState, highLevelControllerParameters, MultiBodySystemTools.filterJoints(controllerToolbox.getControlledJoints(), OneDoFJoint.class));
      this.controllerToolbox = controllerToolbox;

      // create walking controller
      walkingController = new WalkingHighLevelHumanoidController(commandInputManager, statusOutputManager, managerFactory, walkingControllerParameters,
                                                                 controllerToolbox);

      // create controller core
      FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
      JointBasics[] jointsToOptimizeFor = controllerToolbox.getControlledJoints();

      FloatingJointBasics rootJoint = fullRobotModel.getRootJoint();
      ReferenceFrame centerOfMassFrame = controllerToolbox.getCenterOfMassFrame();
      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(controllerToolbox.getControlDT(), controllerToolbox.getGravityZ(), rootJoint,
                                                                            jointsToOptimizeFor, centerOfMassFrame,
                                                                            walkingControllerParameters.getMomentumOptimizationSettings(),
                                                                            controllerToolbox.getYoGraphicsListRegistry(), registry);
      toolbox.setJointPrivilegedConfigurationParameters(walkingControllerParameters.getJointPrivilegedConfigurationParameters());
      toolbox.setFeedbackControllerSettings(walkingControllerParameters.getFeedbackControllerSettings());
      if (setupInverseDynamicsSolver)
         toolbox.setupForInverseDynamicsSolver(controllerToolbox.getContactablePlaneBodies());
      if (setupInverseKinematicsSolver)
         toolbox.setupForInverseKinematicsSolver();
      if (setupVirtualModelControlSolver)
      {
         toolbox.setupForVirtualModelControlSolver(fullRobotModel.getPelvis(), controllerToolbox.getContactablePlaneBodies());
      }
      FeedbackControlCommandList template = managerFactory.createFeedbackControlTemplate();
      JointDesiredOutputList lowLevelControllerOutput = new JointDesiredOutputList(controlledJoints);
      controllerCore = new WholeBodyControllerCore(toolbox, template, lowLevelControllerOutput, registry);
      ControllerCoreOutputReadOnly controllerCoreOutput = controllerCore.getOutputForHighLevelController();
      walkingController.setControllerCoreOutput(controllerCoreOutput);

      deactivateAccelerationIntegrationInWBC = highLevelControllerParameters.deactivateAccelerationIntegrationInTheWBC();

      double controlDT = controllerToolbox.getControlDT();
      double gravityZ = controllerToolbox.getGravityZ();
      RigidBodyBasics elevator = fullRobotModel.getElevator();
      CommonHumanoidReferenceFrames referenceFrames = controllerToolbox.getReferenceFrames();
      YoDouble yoTime = controllerToolbox.getYoTime();
      YoGraphicsListRegistry yoGraphicsListRegistry = controllerToolbox.getYoGraphicsListRegistry();
      SideDependentList<ContactableFoot> contactableFeet = controllerToolbox.getContactableFeet();

      linearMomentumRateControlModule = new LinearMomentumRateControlModule(referenceFrames, contactableFeet, elevator, walkingControllerParameters, yoTime,
                                                                            gravityZ, controlDT, registry, yoGraphicsListRegistry);
      PlanarRegionsListHandler planarRegionsListHandler = controllerToolbox.getWalkingMessageHandler().getPlanarRegionsListHandler();
      linearMomentumRateControlModule.setPlanarRegionsListHandler(planarRegionsListHandler);

      registry.addChild(walkingController.getYoVariableRegistry());
   }

   /**
    * Specifies whether the inverse dynamics module of the {@link WholeBodyControllerCore} should be
    * created or not.
    * <p>
    * This module is created by default as the {@link WalkingHighLevelHumanoidController} needs it.
    * </p>
    *
    * @param setup whether to setup the inverse dynamics mode or not.
    */
   public void setupControllerCoreInverseDynamicsMode(boolean setup)
   {
      setupInverseDynamicsSolver = setup;
   }

   /**
    * Specifies whether the inverse kinematics module of the {@link WholeBodyControllerCore} should
    * be created or not.
    * <p>
    * This module is not created by default to prevent creating unused {@link YoVariable}s.
    * </p>
    *
    * @param setup whether to setup the inverse kinematics mode or not.
    */
   public void setupControllerCoreInverseKinematicsMode(boolean setup)
   {
      setupInverseKinematicsSolver = setup;
   }

   /**
    * Specifies whether the virtual model control module of the {@link WholeBodyControllerCore}
    * should be created or not.
    * <p>
    * This module is not created by default to prevent creating unused {@link YoVariable}s.
    * </p>
    *
    * @param setup whether to setup the virtual model control mode or not.
    */
   public void setupControllerCoreVirtualModelControlMode(boolean setup)
   {
      setupVirtualModelControlSolver = setup;
   }

   public void initialize()
   {
      controllerCore.initialize();
      walkingController.initialize();
      linearMomentumRateControlModule.reset();
      requestIntegratorReset = true;
   }

   @Override
   public void doAction(double timeInState)
   {
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
      controllerCore.submitControllerCoreCommand(controllerCoreCommand);
      controllerCore.compute();
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
   public void onExit()
   {
      controllerToolbox.reportChangeOfRobotMotionStatus(RobotMotionStatus.UNKNOWN);
   }

   @Override
   public JointDesiredOutputListReadOnly getOutputForLowLevelController()
   {
      return controllerCore.getOutputForLowLevelController();
   }

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
    * @return WalkingStateEnum
    */
   public WalkingStateEnum getWalkingStateEnum()
   {
      return walkingController.getWalkingStateEnum();
   }
}
