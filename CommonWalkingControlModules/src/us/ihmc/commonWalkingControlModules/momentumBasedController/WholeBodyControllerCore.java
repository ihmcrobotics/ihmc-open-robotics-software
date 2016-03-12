package us.ihmc.commonWalkingControlModules.momentumBasedController;

import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyControlCoreToolbox;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyFeedbackController;
import us.ihmc.commonWalkingControlModules.controllerCore.WholeBodyInverseDynamicsSolver;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOuput;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.inverseDynamics.InverseDynamicsCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolderInterface;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationDataReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.YoLowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.YoRootJointDesiredConfigurationData;
import us.ihmc.commonWalkingControlModules.inverseKinematics.WholeBodyInverseKinematicsSolver;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.MomentumOptimizationSettings;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.robotics.dataStructures.registry.YoVariableRegistry;
import us.ihmc.robotics.dataStructures.variable.EnumYoVariable;
import us.ihmc.robotics.dataStructures.variable.IntegerYoVariable;
import us.ihmc.robotics.screwTheory.OneDoFJoint;
import us.ihmc.robotics.screwTheory.ScrewTools;
import us.ihmc.robotics.screwTheory.SixDoFJoint;

public class WholeBodyControllerCore
{
   private final YoVariableRegistry registry = new YoVariableRegistry(getClass().getSimpleName());
   private final EnumYoVariable<WholeBodyControllerCoreMode> currentMode = new EnumYoVariable<>("currentWholeBodyControllerCoreMode", registry, WholeBodyControllerCoreMode.class);
   private final IntegerYoVariable numberOfFBControllerEnabled = new IntegerYoVariable("numberOfFBControllerEnabled", registry);

   private final WholeBodyFeedbackController feedbackController;
   private final WholeBodyInverseDynamicsSolver inverseDynamicsSolver;
   private final WholeBodyInverseKinematicsSolver inverseKinematicsSolver;

   private final ControllerCoreOuput controllerCoreOuput;
   private final YoRootJointDesiredConfigurationData yoRootJointDesiredConfigurationData;
   private final YoLowLevelOneDoFJointDesiredDataHolder yoLowLevelOneDoFJointDesiredDataHolder;

   private OneDoFJoint[] oneDoFJoints;

   public WholeBodyControllerCore(WholeBodyControlCoreToolbox toolbox, MomentumOptimizationSettings momentumOptimizationSettings,
         FeedbackControlCommandList allPossibleCommands, YoVariableRegistry parentRegistry)
   {

      feedbackController = new WholeBodyFeedbackController(toolbox, allPossibleCommands, registry);
      inverseDynamicsSolver = new WholeBodyInverseDynamicsSolver(toolbox, momentumOptimizationSettings, registry);
      inverseKinematicsSolver = new WholeBodyInverseKinematicsSolver(toolbox, momentumOptimizationSettings, registry);
      oneDoFJoints = ScrewTools.filterJoints(inverseDynamicsSolver.getJointsToOptimizeFors(), OneDoFJoint.class);
      SixDoFJoint rootJoint = toolbox.getRobotRootJoint();
      yoRootJointDesiredConfigurationData = new YoRootJointDesiredConfigurationData(rootJoint, registry);
      yoLowLevelOneDoFJointDesiredDataHolder = new YoLowLevelOneDoFJointDesiredDataHolder(oneDoFJoints, registry);

      CenterOfPressureDataHolder desiredCenterOfPressureDataHolder = inverseDynamicsSolver.getDesiredCenterOfPressureDataHolder();
      controllerCoreOuput = new ControllerCoreOuput(desiredCenterOfPressureDataHolder);

      parentRegistry.addChild(registry);
   }

   public void initialize()
   {
      feedbackController.initialize();
      inverseDynamicsSolver.initialize();
      inverseKinematicsSolver.reset();
      yoLowLevelOneDoFJointDesiredDataHolder.clear();
   }

   public void reset()
   {
      feedbackController.reset();
      inverseDynamicsSolver.reset();
      inverseKinematicsSolver.reset();
      yoLowLevelOneDoFJointDesiredDataHolder.clear();
   }

   public void submitControllerCoreCommand(ControllerCoreCommand controllerCoreCommand)
   {
      reset();

      currentMode.set(controllerCoreCommand.getControllerCoreMode());

      switch (currentMode.getEnumValue())
      {
      case INVERSE_DYNAMICS:
         feedbackController.submitFeedbackControlCommandList(controllerCoreCommand.getFeedbackControlCommandList());
         inverseDynamicsSolver.submitInverseDynamicsCommand(controllerCoreCommand.getInverseDynamicsCommandList());
         break;
      case INVERSE_KINEMATICS:
         inverseKinematicsSolver.submitInverseKinematicsCommand(controllerCoreCommand.getInverseKinematicsCommandList());
         break;
      case OFF:
         break;
      default:
         throw new RuntimeException("The controller core mode: " + currentMode.getEnumValue() + " is not handled.");
      }
      yoLowLevelOneDoFJointDesiredDataHolder.overwriteWith(controllerCoreCommand.getLowLevelOneDoFJointDesiredDataHolder());
      yoRootJointDesiredConfigurationData.clear();
   }

   public void compute()
   {
      switch (currentMode.getEnumValue())
      {
      case INVERSE_DYNAMICS:
         feedbackController.compute();
         InverseDynamicsCommandList feedbackControllerOutput = feedbackController.getOutput();
         numberOfFBControllerEnabled.set(feedbackControllerOutput.getNumberOfCommands());
         inverseDynamicsSolver.submitInverseDynamicsCommand(feedbackControllerOutput);
         inverseDynamicsSolver.compute();
         LowLevelOneDoFJointDesiredDataHolder solverOutput = inverseDynamicsSolver.getOutput();
         yoLowLevelOneDoFJointDesiredDataHolder.completeWith(solverOutput);
         break;
      case INVERSE_KINEMATICS:
         numberOfFBControllerEnabled.set(0);
         inverseKinematicsSolver.compute();
         LowLevelOneDoFJointDesiredDataHolder inverseKinematicsOutput = inverseKinematicsSolver.getOutput();
         RootJointDesiredConfigurationDataReadOnly inverseKinematicsOutputForRootJoint = inverseKinematicsSolver.getOutputForRootJoint();
         yoLowLevelOneDoFJointDesiredDataHolder.completeWith(inverseKinematicsOutput);
         yoRootJointDesiredConfigurationData.completeWith(inverseKinematicsOutputForRootJoint);
      case OFF:
         numberOfFBControllerEnabled.set(0);
         yoLowLevelOneDoFJointDesiredDataHolder.insertDesiredTorquesIntoOneDoFJoints(oneDoFJoints);
         break;
      default:
         throw new RuntimeException("The controller core mode: " + currentMode.getEnumValue() + " is not handled.");
      }
   }

   public ControllerCoreOuput getOutputForHighLevelController()
   {
      return controllerCoreOuput;
   }

   public LowLevelOneDoFJointDesiredDataHolderInterface getOutputForLowLevelController()
   {
      return yoLowLevelOneDoFJointDesiredDataHolder;
   }

   public RootJointDesiredConfigurationDataReadOnly getOutputForRootJoint()
   {
      return yoRootJointDesiredConfigurationData;
   }
}
