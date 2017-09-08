package us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.newHighLevelStates;

import us.ihmc.commonWalkingControlModules.configurations.HighLevelControllerParameters;
import us.ihmc.commonWalkingControlModules.configurations.ICPTrajectoryPlannerParameters;
import us.ihmc.commonWalkingControlModules.configurations.JointPrivilegedConfigurationParameters;
import us.ihmc.commonWalkingControlModules.configurations.WalkingControllerParameters;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControllerCore;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutput;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.factories.HighLevelControlManagerFactory;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.WalkingHighLevelHumanoidController;
import us.ihmc.commonWalkingControlModules.highLevelHumanoidControl.highLevelStates.walkingController.states.*;
import us.ihmc.commonWalkingControlModules.momentumBasedController.HighLevelHumanoidControllerToolbox;
import us.ihmc.commons.PrintTools;
import us.ihmc.communication.controllerAPI.CommandInputManager;
import us.ihmc.communication.controllerAPI.StatusMessageOutputManager;
import us.ihmc.euclid.referenceFrame.ReferenceFrame;
import us.ihmc.humanoidRobotics.communication.packets.dataobjects.NewHighLevelControllerStates;
import us.ihmc.robotModels.FullHumanoidRobotModel;
import us.ihmc.robotics.robotSide.RobotSide;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.InverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.RigidBody;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.sensorProcessing.outputData.LowLevelJointControlMode;
import us.ihmc.sensorProcessing.outputData.LowLevelOneDoFJointDesiredDataHolderReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;

import java.util.*;

public class NewWalkingControllerState extends NewHighLevelControllerState
{
   private final static NewHighLevelControllerStates controllerState = NewHighLevelControllerStates.WALKING_STATE;
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());

   private final HighLevelHumanoidControllerToolbox controllerToolbox;
   private final WholeBodyControllerCore controllerCore;
   private final WalkingHighLevelHumanoidController walkingController;

   private final ExecutionTimer controllerCoreTimer = new ExecutionTimer("controllerCoreTimer", 1.0, registry);

   private boolean setupInverseDynamicsSolver = true;
   private boolean setupInverseKinematicsSolver = false;
   private boolean setupVirtualModelControlSolver = false;

   public NewWalkingControllerState(CommandInputManager commandInputManager, StatusMessageOutputManager statusOutputManager, HighLevelControlManagerFactory managerFactory,
                                    HighLevelHumanoidControllerToolbox controllerToolbox, HighLevelControllerParameters highLevelControllerParameters,
                                    ICPTrajectoryPlannerParameters capturePointPlannerParameters, WalkingControllerParameters walkingControllerParameters)
   {
      super(controllerState);

      this.controllerToolbox = controllerToolbox;

      // create walking controller
      walkingController = new WalkingHighLevelHumanoidController(commandInputManager, statusOutputManager, managerFactory, walkingControllerParameters,
                                                                 capturePointPlannerParameters, controllerToolbox);

      // create controller core
      FullHumanoidRobotModel fullRobotModel = controllerToolbox.getFullRobotModel();
      InverseDynamicsJoint[] jointsToOptimizeFor = controllerToolbox.getControlledJoints();

      FloatingInverseDynamicsJoint rootJoint = fullRobotModel.getRootJoint();
      ReferenceFrame centerOfMassFrame = controllerToolbox.getCenterOfMassFrame();
      WholeBodyControlCoreToolbox toolbox = new WholeBodyControlCoreToolbox(controllerToolbox.getControlDT(), controllerToolbox.getGravityZ(), rootJoint,
                                                                            jointsToOptimizeFor, centerOfMassFrame,
                                                                            walkingControllerParameters.getMomentumOptimizationSettings(),
                                                                            controllerToolbox.getYoGraphicsListRegistry(), registry);
      toolbox.setJointPrivilegedConfigurationParameters(walkingControllerParameters.getJointPrivilegedConfigurationParameters());
      if (setupInverseDynamicsSolver)
         toolbox.setupForInverseDynamicsSolver(controllerToolbox.getContactablePlaneBodies());
      if (setupInverseKinematicsSolver)
         toolbox.setupForInverseKinematicsSolver();
      if (setupVirtualModelControlSolver)
      {
         RigidBody[] controlledBodies = {fullRobotModel.getPelvis(), fullRobotModel.getFoot(RobotSide.LEFT), fullRobotModel.getFoot(RobotSide.RIGHT)};
         toolbox.setupForVirtualModelControlSolver(fullRobotModel.getPelvis(), controlledBodies, controllerToolbox.getContactablePlaneBodies());
      }
      FeedbackControlCommandList template = managerFactory.createFeedbackControlTemplate();
      controllerCore = new WholeBodyControllerCore(toolbox, template, registry);
      ControllerCoreOutputReadOnly controllerCoreOutput = controllerCore.getOutputForHighLevelController();
      walkingController.setControllerCoreOutput(controllerCoreOutput);

      OneDoFJoint[] controlledJoints = controllerToolbox.getFullRobotModel().getOneDoFJoints();
      for (OneDoFJoint controlledJoint : controlledJoints)
      {
         LowLevelJointControlMode jointControlMode = highLevelControllerParameters.getLowLevelJointControlMode(controlledJoint.getName(), controllerState);
      }

      registry.addChild(walkingController.getYoVariableRegistry());
   }


   public void initialize()
   {
      controllerCore.initialize();
      controllerToolbox.initialize();
      walkingController.initialize();
   }

   public void initializeDesiredHeightToCurrent()
   {
      walkingController.initializeDesiredHeightToCurrent();
   }


   @Override
   public void doAction()
   {
      walkingController.doAction();

      ControllerCoreCommand controllerCoreCommand = walkingController.getControllerCoreCommand();
      controllerCoreTimer.startMeasurement();
      controllerCore.submitControllerCoreCommand(controllerCoreCommand);
      controllerCore.compute();
      controllerCoreTimer.stopMeasurement();

   }

   public void reinitializePelvisOrientation(boolean reinitialize)
   {
      walkingController.reinitializePelvisOrientation(reinitialize);
   }

   @Override
   public void doTransitionIntoAction()
   {
      initialize();
      walkingController.doTransitionIntoAction();
   }

   @Override
   public void doTransitionOutOfAction()
   {
      walkingController.doTransitionOutOfAction();
   }

   @Override
   public LowLevelOneDoFJointDesiredDataHolderReadOnly getOutputForLowLevelController()
   {
      return controllerCore.getOutputForLowLevelController();
   }

   @Override
   public YoVariableRegistry getYoVariableRegistry()
   {
      return registry;
   }

   public void warmup(int iterations)
   {
      PrintTools.info(this, "Starting JIT walking warmup routine");
      ArrayList<WalkingStateEnum> states = new ArrayList<>();
      controllerCore.initialize();
      walkingController.doTransitionIntoAction();

      walkingController.getOrderedWalkingStatesForWarmup(states);
      for(WalkingStateEnum state : states)
      {
         PrintTools.info(this, "Warming up " + state);
         for(int i = 0; i < iterations; i++)
         {
            walkingController.warmupStateIteration(state);
            ControllerCoreCommand controllerCoreCommandList = walkingController.getControllerCoreCommand();
            controllerCore.submitControllerCoreCommand(controllerCoreCommandList);
            controllerCore.compute();
         }
      }

      walkingController.doTransitionOutOfAction();
      walkingController.getControllerCoreCommand().clear();
      PrintTools.info(this, "Finished JIT walking warmup routine");
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
