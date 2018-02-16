package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl;

import java.util.ArrayList;
import java.util.EnumMap;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPTrajectoryPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.YoLowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerStateTransitionFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControllerStateFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.HighLevelControllerState;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WalkingControllerState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HighLevelControllerStateCommand;
import us.ihmc.humanoidRobotics.communication.packets.HighLevelStateChangeStatusMessage;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerName;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.GenericStateMachine;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.State;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateChangedListener;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.StateTransition;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputReadOnly;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

public class HumanoidHighLevelControllerManager implements RobotController
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final GenericStateMachine<HighLevelControllerName, HighLevelControllerState> stateMachine;
   private final HighLevelControllerName initialControllerState;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;

   private final YoBoolean isListeningToHighLevelStateMessage = new YoBoolean("isListeningToHighLevelStateMessage", registry);
   private final YoEnum<HighLevelControllerName> requestedHighLevelControllerState;
   private final YoLowLevelOneDoFJointDesiredDataHolder yoLowLevelOneDoFJointDesiredDataHolder;

   private final CenterOfPressureDataHolder centerOfPressureDataHolderForEstimator;
   private final JointDesiredOutputList lowLevelControllerOutput;
   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusMessageOutputManager;
   private final HighLevelControlManagerFactory managerFactory;

   private final HighLevelControllerFactoryHelper controllerFactoryHelper;

   private final EnumMap<HighLevelControllerName, HighLevelControllerState> highLevelControllerStates = new EnumMap<>(HighLevelControllerName.class);

   private final HighLevelStateChangeStatusMessage highLevelStateChangeStatusMessage = new HighLevelStateChangeStatusMessage();

   private final ExecutionTimer highLevelControllerTimer = new ExecutionTimer("activeHighLevelControllerTimer", 1.0, registry);

   public HumanoidHighLevelControllerManager(CommandInputManager commandInputManager, StatusMessageOutputManager statusMessageOutputManager,
                                             HighLevelControllerName initialControllerState, HighLevelControllerParameters highLevelControllerParameters,
                                             WalkingControllerParameters walkingControllerParameters, ICPTrajectoryPlannerParameters icpPlannerParameters,
                                             YoEnum<HighLevelControllerName> requestedHighLevelControllerState,
                                             EnumMap<HighLevelControllerName, HighLevelControllerStateFactory> controllerStateFactories,
                                             ArrayList<ControllerStateTransitionFactory<HighLevelControllerName>> controllerTransitionFactories,
                                             HighLevelControlManagerFactory managerFactory, HighLevelHumanoidControllerToolbox controllerToolbox,
                                             CenterOfPressureDataHolder centerOfPressureDataHolderForEstimator,
                                             ForceSensorDataHolderReadOnly forceSensorDataHolder,
                                             JointDesiredOutputList lowLevelControllerOutput)
   {
      this.commandInputManager = commandInputManager;
      this.statusMessageOutputManager = statusMessageOutputManager;
      this.controllerToolbox = controllerToolbox;
      this.requestedHighLevelControllerState = requestedHighLevelControllerState;
      this.initialControllerState = initialControllerState;
      this.managerFactory = managerFactory;
      this.centerOfPressureDataHolderForEstimator = centerOfPressureDataHolderForEstimator;
      this.lowLevelControllerOutput = lowLevelControllerOutput;

      this.requestedHighLevelControllerState.set(initialControllerState);
      registry.addChild(controllerToolbox.getYoVariableRegistry());

      controllerFactoryHelper = new HighLevelControllerFactoryHelper();
      controllerFactoryHelper.setCommandInputManager(commandInputManager);
      controllerFactoryHelper.setStatusMessageOutputManager(statusMessageOutputManager);
      controllerFactoryHelper.setParameters(highLevelControllerParameters, walkingControllerParameters, icpPlannerParameters);
      controllerFactoryHelper.setHighLevelHumanoidControllerToolbox(controllerToolbox);
      controllerFactoryHelper.setLowLevelControllerOutput(lowLevelControllerOutput);
      controllerFactoryHelper.setRequestedHighLevelControllerState(requestedHighLevelControllerState);
      controllerFactoryHelper.setForceSensorDataHolder(forceSensorDataHolder);

      stateMachine = setUpStateMachine(controllerStateFactories, controllerTransitionFactories, managerFactory, controllerToolbox.getYoTime(), registry);
      isListeningToHighLevelStateMessage.set(true);
      for (HighLevelControllerState highLevelControllerState : highLevelControllerStates.values())
      {
         this.registry.addChild(highLevelControllerState.getYoVariableRegistry());
      }

      yoLowLevelOneDoFJointDesiredDataHolder = new YoLowLevelOneDoFJointDesiredDataHolder(controllerToolbox.getFullRobotModel().getOneDoFJoints(), registry);
   }

   public void addYoVariableRegistry(YoVariableRegistry registryToAdd)
   {
      this.registry.addChild(registryToAdd);
   }

   public void requestHighLevelControllerState(HighLevelControllerName requestedHighLevelControllerName)
   {
      this.requestedHighLevelControllerState.set(requestedHighLevelControllerName);
   }

   public void setListenToHighLevelStatePackets(boolean isListening)
   {
      isListeningToHighLevelStateMessage.set(isListening);
   }

   @Override
   public void initialize()
   {
      controllerToolbox.initialize();
      stateMachine.setCurrentState(initialControllerState);
   }

   @Override
   public void doControl()
   {
      if (isListeningToHighLevelStateMessage.getBooleanValue())
      {
         if (commandInputManager.isNewCommandAvailable(HighLevelControllerStateCommand.class))
         {
            requestedHighLevelControllerState.set(commandInputManager.pollNewestCommand(HighLevelControllerStateCommand.class).getHighLevelControllerName());
         }
      }

      highLevelControllerTimer.startMeasurement();
      controllerToolbox.update();
      stateMachine.checkTransitionConditions();
      stateMachine.doAction();
      highLevelControllerTimer.stopMeasurement();

      copyJointDesiredsToJoints();
      reportDesiredCenterOfPressureForEstimator();
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   @Override
   public String getName()
   {
      return this.getClass().getSimpleName();
   }

   @Override
   public String getDescription()
   {
      return getName();
   }

   private GenericStateMachine<HighLevelControllerName, HighLevelControllerState> setUpStateMachine(EnumMap<HighLevelControllerName, HighLevelControllerStateFactory> controllerStateFactories,
                                                                                                    ArrayList<ControllerStateTransitionFactory<HighLevelControllerName>> controllerTransitionFactories,
                                                                                                    HighLevelControlManagerFactory managerFactory, YoDouble yoTime,
                                                                                                    YoVariableRegistry registry)
   {
      controllerFactoryHelper.setControllerFactories(controllerStateFactories);
      controllerFactoryHelper.setHighLevelControlManagerFactory(managerFactory);

      GenericStateMachine<HighLevelControllerName, HighLevelControllerState> highLevelStateMachine = new GenericStateMachine<>("highLevelControllerName",
                                                                                                                               "switchTimeName",
                                                                                                                               HighLevelControllerName.class, yoTime,
                                                                                                                               registry);

      // create controller states
      for (HighLevelControllerStateFactory controllerStateFactory : controllerStateFactories.values())
      {
         // create the individual state
         HighLevelControllerState highLevelControllerState = controllerStateFactory.getOrCreateControllerState(controllerFactoryHelper);

         // add the controller to the state machine
         highLevelStateMachine.addState(highLevelControllerState);

         highLevelControllerStates.put(controllerStateFactory.getStateEnum(), highLevelControllerState);

         if (controllerStateFactory.isTransitionToControllerRequested())
            requestedHighLevelControllerState.set(controllerStateFactory.getStateEnum());
      }

      // create controller transitions
      for (ControllerStateTransitionFactory<HighLevelControllerName> controllerStateTransitionFactory : controllerTransitionFactories)
      {
         StateTransition<HighLevelControllerName> stateTransition = controllerStateTransitionFactory.getOrCreateStateTransition(highLevelControllerStates,
                                                                                                                                controllerFactoryHelper, registry);

         HighLevelControllerState state = highLevelControllerStates.get(controllerStateTransitionFactory.getStateToAttachEnum());
         state.addStateTransition(stateTransition);
      }

      highLevelStateMachine.attachStateChangedListener(new StateChangedListener<HighLevelControllerName>()
      {
         @Override
         public void stateChanged(State<HighLevelControllerName> oldState, State<HighLevelControllerName> newState, double time)
         {
            byte oldStateEnum = oldState == null ? -1 : oldState.getStateEnum().toByte();
            byte newStateEnum = newState == null ? -1 : newState.getStateEnum().toByte();
            highLevelStateChangeStatusMessage.setStateChange(oldStateEnum, newStateEnum);
            statusMessageOutputManager.reportStatusMessage(highLevelStateChangeStatusMessage);
         }
      });

      return highLevelStateMachine;
   }

   private final SideDependentList<FramePoint2D> desiredFootCoPs = new SideDependentList<FramePoint2D>(new FramePoint2D(), new FramePoint2D());

   private void reportDesiredCenterOfPressureForEstimator()
   {
      SideDependentList<? extends ContactablePlaneBody> contactableFeet = controllerToolbox.getContactableFeet();
      FullHumanoidRobotModel fullHumanoidRobotModel = controllerToolbox.getFullRobotModel();
      for (RobotSide robotSide : RobotSide.values)
      {
         controllerToolbox.getDesiredCenterOfPressure(contactableFeet.get(robotSide), desiredFootCoPs.get(robotSide));
         centerOfPressureDataHolderForEstimator.setCenterOfPressure(desiredFootCoPs.get(robotSide), fullHumanoidRobotModel.getFoot(robotSide));
      }
   }

   private void copyJointDesiredsToJoints()
   {
      JointDesiredOutputListReadOnly lowLevelOneDoFJointDesiredDataHolder = stateMachine.getCurrentState().getOutputForLowLevelController();
      for (int jointIndex = 0; jointIndex < lowLevelOneDoFJointDesiredDataHolder.getNumberOfJointsWithDesiredOutput(); jointIndex++)
      {
         OneDoFJoint controlledJoint = lowLevelOneDoFJointDesiredDataHolder.getOneDoFJoint(jointIndex);
         JointDesiredOutputReadOnly lowLevelJointData = lowLevelOneDoFJointDesiredDataHolder.getJointDesiredOutput(controlledJoint);

         if (!lowLevelJointData.hasControlMode())
            throw new NullPointerException("Joint: " + controlledJoint.getName() + " has no control mode.");
      }

      yoLowLevelOneDoFJointDesiredDataHolder.overwriteWith(lowLevelOneDoFJointDesiredDataHolder);
      lowLevelControllerOutput.overwriteWith(lowLevelOneDoFJointDesiredDataHolder);
   }

   public HighLevelControllerName getCurrentHighLevelControlState()
   {
      return stateMachine.getCurrentStateEnum();
   }

   public void reinitializeWalking()
   {
      WalkingControllerState walkingControllerState = (WalkingControllerState) stateMachine.getState(HighLevelControllerName.WALKING);
      if (walkingControllerState != null)
      {
         walkingControllerState.initializeDesiredHeightToCurrent();
         walkingControllerState.reinitializePelvisOrientation(false);
      }

      if (managerFactory != null)
         managerFactory.getOrCreatePelvisOrientationManager().setToHoldCurrentInWorldFrame();
   }
}
