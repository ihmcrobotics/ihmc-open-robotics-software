package us.ihmc.commonWalkingControlModules.controllerCore;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutput;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseKinematics.InverseKinematicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationDataBasics;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationDataReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.YoLowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.YoRootJointDesiredConfigurationData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.virtualModelControl.VirtualModelControlCommandList;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointIndexHandler;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.yoVariables.registry.YoVariableRegistry;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

public class WholeBodyControllerCore
{
   private static final boolean DEBUG = false;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final YoEnum<WholeBodyControllerCoreMode> currentMode = new YoEnum<>("currentControllerCoreMode", registry, WholeBodyControllerCoreMode.class);
   private final YoInteger numberOfFBControllerEnabled = new YoInteger("numberOfFBControllerEnabled", registry);

   private final WholeBodyFeedbackController feedbackController;
   private final WholeBodyInverseDynamicsSolver inverseDynamicsSolver;
   private final WholeBodyInverseKinematicsSolver inverseKinematicsSolver;
   private final WholeBodyVirtualModelControlSolver virtualModelControlSolver;

   private final ControllerCoreOutput controllerCoreOutput;
   private final RootJointDesiredConfigurationDataBasics rootJointDesiredConfigurationData;
   private final JointDesiredOutputListBasics jointDesiredOutputList;

   private OneDoFJointBasics[] controlledOneDoFJoints;
   private final ExecutionTimer controllerCoreComputeTimer = new ExecutionTimer("controllerCoreComputeTimer", 1.0, registry);
   private final ExecutionTimer controllerCoreSubmitTimer = new ExecutionTimer("controllerCoreSubmitTimer", 1.0, registry);

   public WholeBodyControllerCore(WholeBodyControlCoreToolbox toolbox, FeedbackControlCommandList allPossibleCommands, YoVariableRegistry parentRegistry)
   {
      this(toolbox, allPossibleCommands, null, parentRegistry);
   }

   public WholeBodyControllerCore(WholeBodyControlCoreToolbox toolbox, FeedbackControlCommandList allPossibleCommands,
                                  JointDesiredOutputList lowLevelControllerOutput, YoVariableRegistry parentRegistry)
   {
      feedbackController = new WholeBodyFeedbackController(toolbox, allPossibleCommands, registry);

      if (toolbox.isEnableInverseDynamicsModule())
         inverseDynamicsSolver = new WholeBodyInverseDynamicsSolver(toolbox, registry);
      else
         inverseDynamicsSolver = null;
      if (toolbox.isEnableInverseKinematicsModule())
         inverseKinematicsSolver = new WholeBodyInverseKinematicsSolver(toolbox, registry);
      else
         inverseKinematicsSolver = null;
      if (toolbox.isEnableVirtualModelControlModule())
         virtualModelControlSolver = new WholeBodyVirtualModelControlSolver(toolbox, registry);
      else
         virtualModelControlSolver = null;

      if (inverseDynamicsSolver == null && inverseKinematicsSolver == null && virtualModelControlSolver == null)
         throw new RuntimeException("Controller core is not properly setup, none of the control modes is enabled.");

      JointIndexHandler jointIndexHandler = toolbox.getJointIndexHandler();
      controlledOneDoFJoints = jointIndexHandler.getIndexedOneDoFJoints();
      FloatingJointBasics rootJoint = toolbox.getRootJoint();

      if (DEBUG)
      {
         if (rootJoint != null)
            rootJointDesiredConfigurationData = new YoRootJointDesiredConfigurationData(rootJoint, registry);
         else
            rootJointDesiredConfigurationData = null;
         jointDesiredOutputList = new YoLowLevelOneDoFJointDesiredDataHolder(controlledOneDoFJoints, registry);
      }
      else
      {

         if (rootJoint != null)
            rootJointDesiredConfigurationData = new RootJointDesiredConfigurationData();
         else
            rootJointDesiredConfigurationData = null;
         jointDesiredOutputList = new LowLevelOneDoFJointDesiredDataHolder();
      }

      CenterOfPressureDataHolder desiredCenterOfPressureDataHolder;

      // When running only the inverse kinematics solver, there is no notion of contact.
      if (inverseDynamicsSolver != null || virtualModelControlSolver != null)
         desiredCenterOfPressureDataHolder = toolbox.getDesiredCenterOfPressureDataHolder();
      else
         desiredCenterOfPressureDataHolder = null;

      controllerCoreOutput = new ControllerCoreOutput(desiredCenterOfPressureDataHolder, controlledOneDoFJoints, lowLevelControllerOutput);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      feedbackController.initialize();
      if (inverseDynamicsSolver != null)
         inverseDynamicsSolver.initialize();
      if (inverseKinematicsSolver != null)
         inverseKinematicsSolver.reset();
      if (virtualModelControlSolver != null)
         virtualModelControlSolver.initialize();
      jointDesiredOutputList.clear();
   }

   public void reset()
   {
      feedbackController.reset();

      switch (currentMode.getEnumValue())
      {
      case INVERSE_DYNAMICS:
         if (inverseDynamicsSolver != null)
            inverseDynamicsSolver.reset();
         else
            throw new RuntimeException("The controller core mode: " + currentMode.getEnumValue() + "is not handled.");
         break;
      case INVERSE_KINEMATICS:
         if (inverseKinematicsSolver != null)
            inverseKinematicsSolver.reset();
         else
            throw new RuntimeException("The controller core mode: " + currentMode.getEnumValue() + "is not handled.");
         break;
      case VIRTUAL_MODEL:
         if (virtualModelControlSolver != null)
            virtualModelControlSolver.reset();
         else
            throw new RuntimeException("The controller core mode: " + currentMode.getEnumValue() + "is not handled.");
         break;
      case OFF:
         break;
      default:
         throw new RuntimeException("The controller core mode: " + currentMode.getEnumValue() + " is not handled.");
      }

      jointDesiredOutputList.clear();
   }

   public void submitControllerCoreCommand(ControllerCoreCommand controllerCoreCommand)
   {
      controllerCoreSubmitTimer.startMeasurement();
      reset();

      boolean reinitializationRequested = controllerCoreCommand.isReinitializationRequested();
      currentMode.set(controllerCoreCommand.getControllerCoreMode());

      switch (currentMode.getEnumValue())
      {
      case INVERSE_DYNAMICS:
         if (inverseDynamicsSolver != null)
         {
            if (reinitializationRequested)
               inverseDynamicsSolver.initialize();
            feedbackController.submitFeedbackControlCommandList(currentMode.getValue(), controllerCoreCommand.getFeedbackControlCommandList());
            inverseDynamicsSolver.submitInverseDynamicsCommandList(controllerCoreCommand.getInverseDynamicsCommandList());
         }
         else
         {
            throw new RuntimeException("The controller core mode: " + currentMode.getEnumValue() + " is not handled.");
         }
         break;
      case INVERSE_KINEMATICS:
         if (inverseKinematicsSolver != null)
         {
            feedbackController.submitFeedbackControlCommandList(currentMode.getValue(), controllerCoreCommand.getFeedbackControlCommandList());
            inverseKinematicsSolver.submitInverseKinematicsCommandList(controllerCoreCommand.getInverseKinematicsCommandList());
         }
         else
         {
            throw new RuntimeException("The controller core mode: " + currentMode.getEnumValue() + " is not handled.");
         }
         break;
      case VIRTUAL_MODEL:
         if (virtualModelControlSolver != null)
         {
            feedbackController.submitFeedbackControlCommandList(currentMode.getValue(), controllerCoreCommand.getFeedbackControlCommandList());
            virtualModelControlSolver.submitVirtualModelControlCommandList(controllerCoreCommand.getVirtualModelControlCommandList());
         }
         else
         {
            throw new RuntimeException("The controller core mode: " + currentMode.getEnumValue() + " is not handled.");
         }
         break;
      case OFF:
         break;
      default:
         throw new RuntimeException("The controller core mode: " + currentMode.getEnumValue() + " is not handled.");
      }

      jointDesiredOutputList.overwriteWith(controllerCoreCommand.getLowLevelOneDoFJointDesiredDataHolder());
      if (rootJointDesiredConfigurationData != null)
         rootJointDesiredConfigurationData.clear();

      controllerCoreCommand.clear();
      controllerCoreSubmitTimer.stopMeasurement();
   }

   public void compute()
   {
      controllerCoreComputeTimer.startMeasurement();
      switch (currentMode.getEnumValue())
      {
      case INVERSE_DYNAMICS:
         if (inverseDynamicsSolver != null)
            doInverseDynamics();
         else
            throw new RuntimeException("The controller core mode: " + currentMode.getEnumValue() + " is not handled.");
         break;
      case INVERSE_KINEMATICS:
         if (inverseKinematicsSolver != null)
            doInverseKinematics();
         else
            throw new RuntimeException("The controller core mode: " + currentMode.getEnumValue() + " is not handled.");
         break;
      case VIRTUAL_MODEL:
         if (virtualModelControlSolver != null)
            doVirtualModelControl();
         else
            throw new RuntimeException("The controller core mode: " + currentMode.getEnumValue() + " is not handled.");
         break;
      case OFF:
         doNothing();
         break;
      default:
         throw new RuntimeException("The controller core mode: " + currentMode.getEnumValue() + " is not handled.");
      }

      if (rootJointDesiredConfigurationData != null)
         controllerCoreOutput.setRootJointDesiredConfigurationData(rootJointDesiredConfigurationData);
      controllerCoreOutput.setLowLevelOneDoFJointDesiredDataHolder(jointDesiredOutputList);
      controllerCoreComputeTimer.stopMeasurement();
   }

   private void doInverseDynamics()
   {
      feedbackController.computeInverseDynamics();
      InverseDynamicsCommandList feedbackControllerOutput = feedbackController.getInverseDynamicsOutput();
      numberOfFBControllerEnabled.set(feedbackControllerOutput.getNumberOfCommands());
      inverseDynamicsSolver.submitInverseDynamicsCommandList(feedbackControllerOutput);
      inverseDynamicsSolver.submitResetIntegratorRequests(jointDesiredOutputList);
      inverseDynamicsSolver.compute();
      feedbackController.computeAchievedAccelerations();
      LowLevelOneDoFJointDesiredDataHolder inverseDynamicsOutput = inverseDynamicsSolver.getOutput();
      RootJointDesiredConfigurationDataReadOnly inverseDynamicsOutputForRootJoint = inverseDynamicsSolver.getOutputForRootJoint();
      jointDesiredOutputList.completeWith(inverseDynamicsOutput);
      if (rootJointDesiredConfigurationData != null)
         rootJointDesiredConfigurationData.completeWith(inverseDynamicsOutputForRootJoint);
      controllerCoreOutput.setLinearMomentumRate(inverseDynamicsSolver.getAchievedMomentumRateLinear());
   }

   private void doInverseKinematics()
   {
      feedbackController.computeInverseKinematics();
      InverseKinematicsCommandList feedbackControllerOutput = feedbackController.getInverseKinematicsOutput();
      numberOfFBControllerEnabled.set(feedbackControllerOutput.getNumberOfCommands());
      inverseKinematicsSolver.submitInverseKinematicsCommandList(feedbackControllerOutput);
      inverseKinematicsSolver.compute();
      LowLevelOneDoFJointDesiredDataHolder inverseKinematicsOutput = inverseKinematicsSolver.getOutput();
      RootJointDesiredConfigurationDataReadOnly inverseKinematicsOutputForRootJoint = inverseKinematicsSolver.getOutputForRootJoint();
      jointDesiredOutputList.completeWith(inverseKinematicsOutput);
      if (rootJointDesiredConfigurationData != null)
         rootJointDesiredConfigurationData.completeWith(inverseKinematicsOutputForRootJoint);
   }

   private void doVirtualModelControl()
   {
      feedbackController.computeVirtualModelControl();
      VirtualModelControlCommandList feedbackControllerOutput = feedbackController.getVirtualModelControlOutput();
      numberOfFBControllerEnabled.set(feedbackControllerOutput.getNumberOfCommands());
      virtualModelControlSolver.submitVirtualModelControlCommandList(feedbackControllerOutput);
      virtualModelControlSolver.compute();
      LowLevelOneDoFJointDesiredDataHolder virtualModelControlOutput = virtualModelControlSolver.getOutput();
      RootJointDesiredConfigurationDataReadOnly virtualModelControlOutputForRootJoint = virtualModelControlSolver.getOutputForRootJoint();
      jointDesiredOutputList.completeWith(virtualModelControlOutput);
      if (rootJointDesiredConfigurationData != null)
         rootJointDesiredConfigurationData.completeWith(virtualModelControlOutputForRootJoint);
      controllerCoreOutput.setLinearMomentumRate(virtualModelControlSolver.getAchievedMomentumRateLinear());
   }

   private void doNothing()
   {
      numberOfFBControllerEnabled.set(0);
      jointDesiredOutputList.insertDesiredTorquesIntoOneDoFJoints(controlledOneDoFJoints);
   }

   public ControllerCoreOutput getControllerCoreOutput()
   {
      return controllerCoreOutput;
   }

   public ControllerCoreOutputReadOnly getOutputForHighLevelController()
   {
      return controllerCoreOutput;
   }

   public JointDesiredOutputListReadOnly getOutputForLowLevelController()
   {
      return jointDesiredOutputList;
   }

   public RootJointDesiredConfigurationDataReadOnly getOutputForRootJoint()
   {
      return rootJointDesiredConfigurationData;
   }

   public FeedbackControllerDataReadOnly getWholeBodyFeedbackControllerDataHolder()
   {
      return feedbackController.getWholeBodyFeedbackControllerDataHolder();
   }
}
