package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPTrajectoryPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.YoLowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.ControllerStateTransitionFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControllerStateFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates.NewHighLevelControllerState;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.FramePoint2D;
import us.ihmc.humanoidRobotics.bipedSupportPolygons.ContactablePlaneBody;
import us.ihmc.humanoidRobotics.communication.controllerAPI.command.HighLevelControllerStateCommand;
import us.ihmc.humanoidRobotics.communication.packets.HighLevelStateChangeStatusMessage;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.HighLevelControllerState;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotController.RobotController;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.robotSide.SideDependentList;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.sensors.ForceSensorDataHolderReadOnly;
import us.ihmc.robotics.stateMachines.conditionBasedStateMachine.*;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.sensorProcessing.outputData.LowLevelJointDataReadOnly;
import us.ihmc.sensorProcessing.outputData.LowLevelOneDoFJointDesiredDataHolderList;
import us.ihmc.sensorProcessing.outputData.LowLevelOneDoFJointDesiredDataHolderReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoBoolean;
import us.ihmc.yoVariables.variable.YoDouble;
import us.ihmc.yoVariables.variable.YoEnum;

import java.util.ArrayList;
import java.util.EnumMap;
import java.util.concurrent.atomic.AtomicReference;

public class NewHumanoidHighLevelControllerManager implements RobotController
{
   private final String name = getClass().getSimpleName();
   private final YoVariableRegistry registry = new YoVariableRegistry(name);

   private final GenericStateMachine<HighLevelControllerState, NewHighLevelControllerState> stateMachine;
   private final HighLevelControllerState initialControllerState;
   private final HighLevelHumanoidControllerToolbox controllerToolbox;

   private final YoBoolean isListeningToHighLevelStateMessage = new YoBoolean("isListeningToHighLevelStateMessage", registry);
   private final YoEnum<HighLevelControllerState> requestedHighLevelControllerState;
   private final AtomicReference<HighLevelControllerState> fallbackControllerForFailure;
   private final YoLowLevelOneDoFJointDesiredDataHolder yoLowLevelOneDoFJointDesiredDataHolder;

   private final ForceSensorDataHolderReadOnly forceSensorDataHolder;
   private final CenterOfPressureDataHolder centerOfPressureDataHolderForEstimator;
   private final LowLevelOneDoFJointDesiredDataHolderList lowLevelControllerOutput;
   private final CommandInputManager commandInputManager;
   private final StatusMessageOutputManager statusMessageOutputManager;

   private final HighLevelControllerFactoryHelper controllerFactoryHelper;

   private final EnumMap<HighLevelControllerState, NewHighLevelControllerState> highLevelControllerStates = new EnumMap<>(HighLevelControllerState.class);

   private final HighLevelStateChangeStatusMessage highLevelStateChangeStatusMessage = new HighLevelStateChangeStatusMessage();

   private final ExecutionTimer highLevelControllerTimer = new ExecutionTimer("activeHighLevelControllerTimer", 1.0, registry);

   public NewHumanoidHighLevelControllerManager(CommandInputManager commandInputManager, StatusMessageOutputManager statusMessageOutputManager,
                                                HighLevelControllerState initialControllerState, HighLevelControllerParameters highLevelControllerParameters,
                                                WalkingControllerParameters walkingControllerParameters, ICPTrajectoryPlannerParameters icpPlannerParameters,
                                                YoEnum<HighLevelControllerState> requestedHighLevelControllerState, AtomicReference<HighLevelControllerState> fallbackControllerForFailure,
                                                EnumMap<HighLevelControllerState, HighLevelControllerStateFactory> controllerStateFactories,
                                                ArrayList<ControllerStateTransitionFactory<HighLevelControllerState>> controllerTransitionFactories,
                                                HighLevelControlManagerFactory managerFactory, HighLevelHumanoidControllerToolbox controllerToolbox,
                                                CenterOfPressureDataHolder centerOfPressureDataHolderForEstimator, ForceSensorDataHolderReadOnly forceSensorDataHolder,
                                                LowLevelOneDoFJointDesiredDataHolderList lowLevelControllerOutput)
   {
      this.commandInputManager = commandInputManager;
      this.statusMessageOutputManager = statusMessageOutputManager;
      this.controllerToolbox = controllerToolbox;
      this.requestedHighLevelControllerState = requestedHighLevelControllerState;
      this.fallbackControllerForFailure = fallbackControllerForFailure;
      this.initialControllerState = initialControllerState;
      this.centerOfPressureDataHolderForEstimator = centerOfPressureDataHolderForEstimator;
      this.lowLevelControllerOutput = lowLevelControllerOutput;
      this.forceSensorDataHolder = forceSensorDataHolder;

      this.requestedHighLevelControllerState.set(initialControllerState);
      registry.addChild(controllerToolbox.getYoVariableRegistry());

      controllerFactoryHelper = new HighLevelControllerFactoryHelper();
      controllerFactoryHelper.setCommandInputManager(commandInputManager);
      controllerFactoryHelper.setStatusMessageOutputManager(statusMessageOutputManager);
      controllerFactoryHelper.setParameters(highLevelControllerParameters, walkingControllerParameters, icpPlannerParameters);
      controllerFactoryHelper.setHighLevelHumanoidControllerToolbox(controllerToolbox);
      controllerFactoryHelper.setLowLevelControllerOutput(lowLevelControllerOutput);
      controllerFactoryHelper.setRequestedHighLevelControllerState(requestedHighLevelControllerState);
      controllerFactoryHelper.setFallbackControllerForFailure(fallbackControllerForFailure);

      stateMachine = setUpStateMachine(controllerStateFactories, controllerTransitionFactories, managerFactory, controllerToolbox.getYoTime(), registry);
      isListeningToHighLevelStateMessage.set(true);
      for (NewHighLevelControllerState highLevelControllerState : highLevelControllerStates.values())
      {
         this.registry.addChild(highLevelControllerState.getYoVariableRegistry());
      }

      yoLowLevelOneDoFJointDesiredDataHolder = new YoLowLevelOneDoFJointDesiredDataHolder(controllerToolbox.getFullRobotModel().getOneDoFJoints(), registry);
   }

   public void addYoVariableRegistry(YoVariableRegistry registryToAdd)
   {
      this.registry.addChild(registryToAdd);
   }

   public void requestHighLevelControllerState(HighLevelControllerState requestedHighLevelControllerState)
   {
      this.requestedHighLevelControllerState.set(requestedHighLevelControllerState);
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
            requestedHighLevelControllerState.set(commandInputManager.pollNewestCommand(HighLevelControllerStateCommand.class).getHighLevelControllerState());
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

   /**
    * Warmup the walking behavior by running all states for a number of iterations.
    *
    * Also warms up the controller core
    *
    * @param iterations number of times to run a single state
    */
   public void warmup(int iterations)
   {
      PrintTools.info(this, "Starting JIT warmup routine");
      for (NewHighLevelControllerState highLevelControllerState : highLevelControllerStates.values())
      {
         highLevelControllerState.doTransitionIntoAction();
         highLevelControllerState.warmup(iterations);
         highLevelControllerState.doTransitionOutOfAction();
      }
      PrintTools.info(this, "Finished JIT warmup routine");
   }

   private GenericStateMachine<HighLevelControllerState, NewHighLevelControllerState> setUpStateMachine(EnumMap<HighLevelControllerState, HighLevelControllerStateFactory> controllerStateFactories,
                                                                                                            ArrayList<ControllerStateTransitionFactory<HighLevelControllerState>> controllerTransitionFactories,
                                                                                                            HighLevelControlManagerFactory managerFactory, YoDouble yoTime, YoVariableRegistry registry)
   {
      controllerFactoryHelper.setControllerFactories(controllerStateFactories);
      controllerFactoryHelper.setHighLevelControlManagerFactory(managerFactory);

      GenericStateMachine<HighLevelControllerState, NewHighLevelControllerState> highLevelStateMachine = new GenericStateMachine<>("highLevelControllerState", "switchTimeName",
                                                                                                                                       HighLevelControllerState.class, yoTime, registry);


      // create controller states
      for (HighLevelControllerStateFactory controllerStateFactory : controllerStateFactories.values())
      {
         // create the individual state
         NewHighLevelControllerState highLevelControllerState = controllerStateFactory.getOrCreateControllerState(controllerFactoryHelper);

         // add the controller to the state machine
         highLevelStateMachine.addState(highLevelControllerState);

         highLevelControllerStates.put(controllerStateFactory.getStateEnum(), highLevelControllerState);

         if (controllerStateFactory.isTransitionToControllerRequested())
            requestedHighLevelControllerState.set(controllerStateFactory.getStateEnum());
      }

      // create controller transitions
      for (ControllerStateTransitionFactory<HighLevelControllerState> controllerStateTransitionFactory : controllerTransitionFactories)
      {
         StateTransition<HighLevelControllerState> stateTransition = controllerStateTransitionFactory.getOrCreateStateTransition(highLevelControllerStates, controllerFactoryHelper,
                                                                                                                                     forceSensorDataHolder, registry);

         NewHighLevelControllerState state = highLevelControllerStates.get(controllerStateTransitionFactory.getStateToAttachEnum());
         state.addStateTransition(stateTransition);
      }

      highLevelStateMachine.attachStateChangedListener(new StateChangedListener<HighLevelControllerState>()
      {
         @Override
         public void stateChanged(State<HighLevelControllerState> oldState, State<HighLevelControllerState> newState, double time)
         {
            highLevelStateChangeStatusMessage.setStateChange(oldState.getStateEnum(), newState.getStateEnum());
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
      LowLevelOneDoFJointDesiredDataHolderReadOnly lowLevelOneDoFJointDesiredDataHolder = stateMachine.getCurrentState().getOutputForLowLevelController();
      for (int jointIndex = 0; jointIndex < lowLevelOneDoFJointDesiredDataHolder.getNumberOfJointsWithLowLevelData(); jointIndex++)
      {
         OneDoFJoint controlledJoint = lowLevelOneDoFJointDesiredDataHolder.getOneDoFJoint(jointIndex);
         LowLevelJointDataReadOnly lowLevelJointData = lowLevelOneDoFJointDesiredDataHolder.getLowLevelJointData(controlledJoint);

         if (!lowLevelJointData.hasControlMode())
            throw new NullPointerException("Joint: " + controlledJoint.getName() + " has no control mode.");

         // Zero out joint for testing purposes
         controlledJoint.setqDesired(Double.NaN);
         controlledJoint.setQdDesired(Double.NaN);
         controlledJoint.setQddDesired(Double.NaN);
         controlledJoint.setTau(Double.NaN);
      }

      yoLowLevelOneDoFJointDesiredDataHolder.overwriteWith(lowLevelOneDoFJointDesiredDataHolder);
      lowLevelControllerOutput.overwriteWith(lowLevelOneDoFJointDesiredDataHolder);
   }


}
