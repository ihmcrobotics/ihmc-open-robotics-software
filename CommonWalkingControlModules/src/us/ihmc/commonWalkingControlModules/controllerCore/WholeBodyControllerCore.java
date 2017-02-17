package us.ihmc.commonWalkingControlModules.controllerCore;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutput;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelJointDataReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolderReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationDataReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.YoLowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.YoRootJointDesiredConfigurationData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointIndexHandler;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.screwTheory.FloatingInverseDynamicsJoint;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.time.ExecutionTimer;

public class WholeBodyControllerCore
{
   private static final boolean INCLUDE_INVERSE_DYNAMICS_SOLVER = true;
   private static final boolean INCLUDE_INVERSE_KINEMATICS_SOLVER = true;
   private static final boolean INCLUDE_VIRTUAL_MODEL_CONTROL_SOLVER = true;

   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final EnumYoVariable<WholeBodyControllerCoreMode> currentMode = new EnumYoVariable<>("currentControllerCoreMode", registry, WholeBodyControllerCoreMode.class);
   private final IntegerYoVariable numberOfFBControllerEnabled = new IntegerYoVariable("numberOfFBControllerEnabled", registry);

   private final WholeBodyFeedbackController feedbackController;
   private final WholeBodyInverseDynamicsSolver inverseDynamicsSolver;
   private final WholeBodyInverseKinematicsSolver inverseKinematicsSolver;
   private final WholeBodyVirtualModelControlSolver virtualModelControlSolver;

   private final ControllerCoreOutput controllerCoreOutput;
   private final YoRootJointDesiredConfigurationData yoRootJointDesiredConfigurationData;
   private final YoLowLevelOneDoFJointDesiredDataHolder yoLowLevelOneDoFJointDesiredDataHolder;

   private OneDoFJoint[] controlledOneDoFJoints;
   private final ExecutionTimer controllerCoreComputeTimer = new ExecutionTimer("controllerCoreComputeTimer", 1.0, registry);
   private final ExecutionTimer controllerCoreSubmitTimer = new ExecutionTimer("controllerCoreSubmitTimer", 1.0, registry);

   public WholeBodyControllerCore(WholeBodyControlCoreToolbox toolbox, FeedbackControlCommandList allPossibleCommands,
         YoVariableRegistry parentRegistry)
   {

      feedbackController = new WholeBodyFeedbackController(toolbox, allPossibleCommands, registry);

      if (INCLUDE_INVERSE_DYNAMICS_SOLVER)
         inverseDynamicsSolver = new WholeBodyInverseDynamicsSolver(toolbox, registry);
      else
         inverseDynamicsSolver = null;
      if (INCLUDE_INVERSE_KINEMATICS_SOLVER)
         inverseKinematicsSolver = new WholeBodyInverseKinematicsSolver(toolbox, registry);
      else
         inverseKinematicsSolver = null;
      if (INCLUDE_VIRTUAL_MODEL_CONTROL_SOLVER)
         virtualModelControlSolver = new WholeBodyVirtualModelControlSolver(toolbox, registry);
      else
         virtualModelControlSolver = null;

      JointIndexHandler jointIndexHandler = toolbox.getJointIndexHandler();
      controlledOneDoFJoints = jointIndexHandler.getIndexedOneDoFJoints();
      FloatingInverseDynamicsJoint rootJoint = toolbox.getRobotRootJoint();
      yoRootJointDesiredConfigurationData = new YoRootJointDesiredConfigurationData(rootJoint, registry);
      yoLowLevelOneDoFJointDesiredDataHolder = new YoLowLevelOneDoFJointDesiredDataHolder(controlledOneDoFJoints, registry);

      CenterOfPressureDataHolder desiredCenterOfPressureDataHolder = inverseDynamicsSolver.getDesiredCenterOfPressureDataHolder();
      controllerCoreOutput = new ControllerCoreOutput(desiredCenterOfPressureDataHolder, controlledOneDoFJoints);

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
         virtualModelControlSolver.reset();
      yoLowLevelOneDoFJointDesiredDataHolder.clear();
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
            virtualModelControlSolver.clear();
         else
            throw new RuntimeException("The controller core mode: " + currentMode.getEnumValue() + "is not handled.");
         break;
      case OFF:
         break;
      default:
         throw new RuntimeException("The controller core mode: " + currentMode.getEnumValue() + " is not handled.");
      }

      yoLowLevelOneDoFJointDesiredDataHolder.clear();
   }

   public void submitControllerCoreCommand(ControllerCoreCommand controllerCoreCommand)
   {
      controllerCoreSubmitTimer.startMeasurement();
      reset();

      currentMode.set(controllerCoreCommand.getControllerCoreMode());

      switch (currentMode.getEnumValue())
      {
      case INVERSE_DYNAMICS:
         if (inverseDynamicsSolver != null)
         {
            feedbackController.submitFeedbackControlCommandList(controllerCoreCommand.getFeedbackControlCommandList());
            inverseDynamicsSolver.submitInverseDynamicsCommandList(controllerCoreCommand.getInverseDynamicsCommandList());
         }
         else
         {
            throw new RuntimeException("The controller core mode: " + currentMode.getEnumValue() + " is not handled.");
         }
         break;
      case INVERSE_KINEMATICS:
         if (inverseKinematicsSolver != null)
            inverseKinematicsSolver.submitInverseKinematicsCommand(controllerCoreCommand.getInverseKinematicsCommandList());
         else
            throw new RuntimeException("The controller core mode: " + currentMode.getEnumValue() + " is not handled.");
         break;
      case VIRTUAL_MODEL:
         if (virtualModelControlSolver != null)
         {
            feedbackController.submitFeedbackControlCommandList(controllerCoreCommand.getFeedbackControlCommandList());
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

      yoLowLevelOneDoFJointDesiredDataHolder.overwriteWith(controllerCoreCommand.getLowLevelOneDoFJointDesiredDataHolder());
      yoRootJointDesiredConfigurationData.clear();

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

      parseLowLevelDataInOneDoFJoints();

      controllerCoreOutput.setRootJointDesiredConfigurationData(yoRootJointDesiredConfigurationData);
      controllerCoreOutput.setLowLevelOneDoFJointDesiredDataHolder(yoLowLevelOneDoFJointDesiredDataHolder);
      controllerCoreComputeTimer.stopMeasurement();
   }

   private void doInverseDynamics()
   {
      feedbackController.compute();
      InverseDynamicsCommandList feedbackControllerOutput = feedbackController.getOutput();
      numberOfFBControllerEnabled.set(feedbackControllerOutput.getNumberOfCommands());
      inverseDynamicsSolver.submitInverseDynamicsCommandList(feedbackControllerOutput);
      inverseDynamicsSolver.compute();
      feedbackController.computeAchievedAccelerations();
      LowLevelOneDoFJointDesiredDataHolder inverseDynamicsOutput = inverseDynamicsSolver.getOutput();
      RootJointDesiredConfigurationDataReadOnly inverseDynamicsOutputForRootJoint = inverseDynamicsSolver.getOutputForRootJoint();
      yoLowLevelOneDoFJointDesiredDataHolder.completeWith(inverseDynamicsOutput);
      yoRootJointDesiredConfigurationData.completeWith(inverseDynamicsOutputForRootJoint);
      controllerCoreOutput.setAndMatchFrameLinearMomentumRate(inverseDynamicsSolver.getAchievedMomentumRateLinear());
   }

   private void doInverseKinematics()
   {
      numberOfFBControllerEnabled.set(0);
      inverseKinematicsSolver.compute();
      LowLevelOneDoFJointDesiredDataHolder inverseKinematicsOutput = inverseKinematicsSolver.getOutput();
      RootJointDesiredConfigurationDataReadOnly inverseKinematicsOutputForRootJoint = inverseKinematicsSolver.getOutputForRootJoint();
      yoLowLevelOneDoFJointDesiredDataHolder.completeWith(inverseKinematicsOutput);
      yoRootJointDesiredConfigurationData.completeWith(inverseKinematicsOutputForRootJoint);
   }

   private void doVirtualModelControl()
   {
      feedbackController.compute();
      InverseDynamicsCommandList feedbackControllerOutput = feedbackController.getOutput();
      numberOfFBControllerEnabled.set(feedbackControllerOutput.getNumberOfCommands());
      virtualModelControlSolver.submitVirtualModelControlCommandList(feedbackControllerOutput);
      virtualModelControlSolver.compute();
      feedbackController.computeAchievedAccelerations(); // FIXME
      LowLevelOneDoFJointDesiredDataHolder virtualModelControlOutput = virtualModelControlSolver.getOutput();
      RootJointDesiredConfigurationDataReadOnly virtualModelControlOutputForRootJoint = virtualModelControlSolver.getOutputForRootJoint();
      yoLowLevelOneDoFJointDesiredDataHolder.completeWith(virtualModelControlOutput);
      yoRootJointDesiredConfigurationData.completeWith(virtualModelControlOutputForRootJoint);
   }

   private void doNothing()
   {
      numberOfFBControllerEnabled.set(0);
      yoLowLevelOneDoFJointDesiredDataHolder.insertDesiredTorquesIntoOneDoFJoints(controlledOneDoFJoints);
   }

   private void parseLowLevelDataInOneDoFJoints()
   {
      for (int i = 0; i < controlledOneDoFJoints.length; i++)
      {
         OneDoFJoint joint = controlledOneDoFJoints[i];
         LowLevelJointDataReadOnly lowLevelJointData = yoLowLevelOneDoFJointDesiredDataHolder.getLowLevelJointData(joint);

         if (!lowLevelJointData.hasControlMode())
            throw new NullPointerException("Joint: " + joint.getName() + " has no control mode.");

         switch (lowLevelJointData.getControlMode())
         {
         case FORCE_CONTROL:
            joint.setUnderPositionControl(false);
            break;
         case POSITION_CONTROL:
            joint.setUnderPositionControl(true);
            break;
         default:
            throw new RuntimeException("Unhandled joint control mode: " + lowLevelJointData.getControlMode());
         }

         if (lowLevelJointData.hasDesiredPosition())
            joint.setqDesired(lowLevelJointData.getDesiredPosition());

         if (lowLevelJointData.hasDesiredVelocity())
            joint.setQdDesired(lowLevelJointData.getDesiredVelocity());

         if (lowLevelJointData.hasDesiredAcceleration())
            joint.setQddDesired(lowLevelJointData.getDesiredAcceleration());

         if (lowLevelJointData.hasDesiredTorque())
            joint.setTau(lowLevelJointData.getDesiredTorque());
      }
   }

   public ControllerCoreOutput getControllerCoreOutput()
   {
      return controllerCoreOutput;
   }

   public ControllerCoreOutputReadOnly getOutputForHighLevelController()
   {
      return controllerCoreOutput;
   }

   public LowLevelOneDoFJointDesiredDataHolderReadOnly getOutputForLowLevelController()
   {
      return yoLowLevelOneDoFJointDesiredDataHolder;
   }

   public RootJointDesiredConfigurationDataReadOnly getOutputForRootJoint()
   {
      return yoRootJointDesiredConfigurationData;
   }
}
