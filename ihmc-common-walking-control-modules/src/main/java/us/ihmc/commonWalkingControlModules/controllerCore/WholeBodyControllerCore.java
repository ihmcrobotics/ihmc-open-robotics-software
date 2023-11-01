package us.ihmc.commonWalkingControlModules.controllerCore;

import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommand;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreCommandInterface;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutput;
import us.ihmc.commonWalkingControlModules.controllerCore.command.ControllerCoreOutputReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.DesiredExternalWrenchHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.feedbackController.FeedbackControlCommandList;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.LowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationData;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationDataBasics;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.RootJointDesiredConfigurationDataReadOnly;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.YoLowLevelOneDoFJointDesiredDataHolder;
import us.ihmc.commonWalkingControlModules.controllerCore.command.lowLevel.YoRootJointDesiredConfigurationData;
import us.ihmc.commonWalkingControlModules.momentumBasedController.optimization.JointIndexHandler;
import us.ihmc.humanoidRobotics.model.CenterOfPressureDataHolder;
import us.ihmc.mecano.multiBodySystem.interfaces.FloatingJointBasics;
import us.ihmc.mecano.multiBodySystem.interfaces.OneDoFJointBasics;
import us.ihmc.robotics.SCS2YoGraphicHolder;
import us.ihmc.robotics.time.ExecutionTimer;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicDefinition;
import us.ihmc.scs2.definition.yoGraphic.YoGraphicGroupDefinition;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputList;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListBasics;
import us.ihmc.sensorProcessing.outputData.JointDesiredOutputListReadOnly;
import us.ihmc.yoVariables.registry.YoRegistry;
import us.ihmc.yoVariables.variable.YoEnum;
import us.ihmc.yoVariables.variable.YoInteger;

public class WholeBodyControllerCore implements SCS2YoGraphicHolder
{
   public static boolean REDUCE_YOVARIABLES = false;

   private final YoRegistry registry = new YoRegistry(getClass().getSimpleName());
   private final YoEnum<WholeBodyControllerCoreMode> currentMode = new YoEnum<>("currentControllerCoreMode", registry, WholeBodyControllerCoreMode.class);
   private final YoInteger numberOfFBControllerEnabled = new YoInteger("numberOfFBControllerEnabled", registry);

   private final WholeBodyControlCoreToolbox toolbox;
   private final WholeBodyFeedbackController feedbackController;
   private final WholeBodyInverseDynamicsSolver inverseDynamicsSolver;
   private final WholeBodyInverseKinematicsSolver inverseKinematicsSolver;
   private final WholeBodyVirtualModelControlSolver virtualModelControlSolver;

   private final ControllerCoreCommand internalCommandInput = new ControllerCoreCommand();
   private final ControllerCoreOutput controllerCoreOutput;
   private final RootJointDesiredConfigurationDataBasics rootJointDesiredConfigurationData;
   private final JointDesiredOutputListBasics jointDesiredOutputList;

   private OneDoFJointBasics[] controlledOneDoFJoints;
   private final ExecutionTimer controllerCoreComputeTimer = new ExecutionTimer("controllerCoreComputeTimer", 1.0, registry);
   private final ExecutionTimer controllerCoreFeedbackControlTimer = new ExecutionTimer("controllerCoreFeedbackControlTimer", 1.0, registry);
   private final ExecutionTimer controllerCoreSubmissionTimer = new ExecutionTimer("controllerCoreSubmissionTimer", 1.0, registry);

   @Deprecated
   public WholeBodyControllerCore(WholeBodyControlCoreToolbox toolbox, FeedbackControlCommandList allPossibleCommands, YoRegistry parentRegistry)
   {
      this(toolbox, allPossibleCommands, null, parentRegistry);
   }

   @Deprecated
   public WholeBodyControllerCore(WholeBodyControlCoreToolbox toolbox,
                                  FeedbackControlCommandList allPossibleCommands,
                                  JointDesiredOutputList lowLevelControllerOutput,
                                  YoRegistry parentRegistry)
   {
      this(toolbox, new FeedbackControllerTemplate(allPossibleCommands), lowLevelControllerOutput, parentRegistry);
   }

   public WholeBodyControllerCore(WholeBodyControlCoreToolbox toolbox, FeedbackControllerTemplate feedbackControllerTemplate, YoRegistry parentRegistry)
   {
      this(toolbox, feedbackControllerTemplate, null, parentRegistry);
   }

   public WholeBodyControllerCore(WholeBodyControlCoreToolbox toolbox,
                                  FeedbackControllerTemplate feedbackControllerTemplate,
                                  JointDesiredOutputList lowLevelControllerOutput,
                                  YoRegistry parentRegistry)
   {
      this.toolbox = toolbox;
      feedbackController = new WholeBodyFeedbackController(toolbox, feedbackControllerTemplate, registry);

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

      if (REDUCE_YOVARIABLES)
      {
         if (rootJoint != null)
            rootJointDesiredConfigurationData = new RootJointDesiredConfigurationData();
         else
            rootJointDesiredConfigurationData = null;
         jointDesiredOutputList = new LowLevelOneDoFJointDesiredDataHolder();
      }
      else
      {
         if (rootJoint != null)
            rootJointDesiredConfigurationData = new YoRootJointDesiredConfigurationData(rootJoint, registry);
         else
            rootJointDesiredConfigurationData = null;
         jointDesiredOutputList = new YoLowLevelOneDoFJointDesiredDataHolder(controlledOneDoFJoints, registry);
      }

      CenterOfPressureDataHolder desiredCenterOfPressureDataHolder;
      DesiredExternalWrenchHolder desiredExternalWrenchHolder;

      // When running only the inverse kinematics solver, there is no notion of contact.
      if (inverseDynamicsSolver != null || virtualModelControlSolver != null)
      {
         desiredCenterOfPressureDataHolder = toolbox.getDesiredCenterOfPressureDataHolder();
         desiredExternalWrenchHolder = toolbox.getDesiredExternalWrenchHolder();
      }
      else
      {
         desiredCenterOfPressureDataHolder = null;
         desiredExternalWrenchHolder = null;
      }

      controllerCoreOutput = new ControllerCoreOutput(desiredCenterOfPressureDataHolder,
                                                      desiredExternalWrenchHolder,
                                                      controlledOneDoFJoints,
                                                      lowLevelControllerOutput);

      parentRegistry.addChild(registry);
   }

   public void registerAdditionalControllers(FeedbackControllerTemplate template)
   {
      feedbackController.registerControllers(template);
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
      internalCommandInput.clear();
      feedbackController.reset();

      checkControlModeHandled();

      switch (currentMode.getEnumValue())
      {
         case INVERSE_DYNAMICS:
            inverseDynamicsSolver.reset();
            break;
         case INVERSE_KINEMATICS:
            inverseKinematicsSolver.reset();
            break;
         case VIRTUAL_MODEL:
            virtualModelControlSolver.reset();
            break;
         case OFF:
            break;
      }

      jointDesiredOutputList.clear();
   }

   /**
    * @deprecated Use {@link #compute(ControllerCoreCommandInterface)} instead, note that it also makes
    *             {@link #compute()} obsolete.
    */
   public void submitControllerCoreCommand(ControllerCoreCommandInterface controllerCoreCommand)
   {
      reset();
      currentMode.set(controllerCoreCommand.getControllerCoreMode());
      internalCommandInput.set(controllerCoreCommand);
      jointDesiredOutputList.overwriteWith(controllerCoreCommand.getLowLevelOneDoFJointDesiredDataHolder());

      if (rootJointDesiredConfigurationData != null)
         rootJointDesiredConfigurationData.clear();

      controllerCoreCommand.clear();
      checkControlModeHandled();
   }

   private void checkControlModeHandled()
   {
      switch (currentMode.getEnumValue())
      {
         case INVERSE_DYNAMICS:
            if (inverseDynamicsSolver != null)
               return;
         case INVERSE_KINEMATICS:
            if (inverseKinematicsSolver != null)
               return;
         case VIRTUAL_MODEL:
            if (virtualModelControlSolver != null)
               return;
         case OFF:
            return;
      }
      throw new RuntimeException("The controller core mode: " + currentMode.getEnumValue() + " is not handled.");
   }

   /**
    * @deprecated Use {@link #compute(ControllerCoreCommandInterface)} instead, note that it also makes
    *             {@link #submitControllerCoreCommand(ControllerCoreCommandInterface)} obsolete.
    */
   public void compute()
   {
      controllerCoreComputeTimer.startMeasurement();

      computeFeedbackControllers();

      switch (currentMode.getEnumValue())
      {
         case INVERSE_DYNAMICS:
            doInverseDynamics();
            break;
         case INVERSE_KINEMATICS:
            doInverseKinematics();
            break;
         case VIRTUAL_MODEL:
            doVirtualModelControl();
            break;
         case OFF:
            doNothing();
            break;
      }

      if (rootJointDesiredConfigurationData != null)
         controllerCoreOutput.setRootJointDesiredConfigurationData(rootJointDesiredConfigurationData);
      controllerCoreOutput.setLowLevelOneDoFJointDesiredDataHolder(jointDesiredOutputList);
      controllerCoreComputeTimer.stopMeasurement();
   }

   // TODO Clean me up once compute() and submitControllerCoreCommand(ControllerCoreCommandInterface) have been removed.
   public void compute(ControllerCoreCommandInterface controllerCoreCommand)
   {
      submitControllerCoreCommand(controllerCoreCommand);
      compute();
   }

   private void computeFeedbackControllers()
   {
      controllerCoreFeedbackControlTimer.startMeasurement();
      FeedbackControlCommandList feedbackControlCommandList = internalCommandInput.getFeedbackControlCommandList();

      switch (currentMode.getEnumValue())
      {
         case INVERSE_DYNAMICS:
            feedbackController.submitFeedbackControlCommandList(currentMode.getValue(), feedbackControlCommandList);
            feedbackController.computeInverseDynamics();
            internalCommandInput.getInverseDynamicsCommandList().addCommandList(feedbackController.getInverseDynamicsOutput());
            numberOfFBControllerEnabled.set(feedbackController.getInverseDynamicsOutput().getNumberOfCommands());
            break;

         case INVERSE_KINEMATICS:
            feedbackController.submitFeedbackControlCommandList(currentMode.getValue(), feedbackControlCommandList);
            feedbackController.computeInverseKinematics();
            internalCommandInput.getInverseKinematicsCommandList().addCommandList(feedbackController.getInverseKinematicsOutput());
            numberOfFBControllerEnabled.set(feedbackController.getInverseKinematicsOutput().getNumberOfCommands());
            break;

         case VIRTUAL_MODEL:
            feedbackController.submitFeedbackControlCommandList(currentMode.getValue(), feedbackControlCommandList);
            feedbackController.computeVirtualModelControl();
            internalCommandInput.getVirtualModelControlCommandList().addCommandList(feedbackController.getVirtualModelControlOutput());
            numberOfFBControllerEnabled.set(feedbackController.getVirtualModelControlOutput().getNumberOfCommands());
            break;

         case OFF:
            break;
      }

      feedbackControlCommandList.clear();
      controllerCoreFeedbackControlTimer.stopMeasurement();
   }

   private void doInverseDynamics()
   {
      if (internalCommandInput.isReinitializationRequested())
         inverseDynamicsSolver.initialize();

      controllerCoreSubmissionTimer.startMeasurement();
      inverseDynamicsSolver.submitInverseDynamicsCommandList(internalCommandInput.getInverseDynamicsCommandList());
      inverseDynamicsSolver.submitResetIntegratorRequests(jointDesiredOutputList);
      controllerCoreSubmissionTimer.stopMeasurement();
      inverseDynamicsSolver.compute();
      feedbackController.computeAchievedAccelerations();

      jointDesiredOutputList.completeWith(inverseDynamicsSolver.getOutput());
      if (rootJointDesiredConfigurationData != null)
         rootJointDesiredConfigurationData.completeWith(inverseDynamicsSolver.getOutputForRootJoint());
      controllerCoreOutput.setLinearMomentumRate(inverseDynamicsSolver.getAchievedMomentumRateLinear());
      controllerCoreOutput.setAngularMomentumRate(inverseDynamicsSolver.getAchievedMomentumRateAngular());
   }

   private void doInverseKinematics()
   {
      controllerCoreSubmissionTimer.startMeasurement();
      inverseKinematicsSolver.submitInverseKinematicsCommandList(internalCommandInput.getInverseKinematicsCommandList());
      controllerCoreSubmissionTimer.stopMeasurement();
      inverseKinematicsSolver.compute();

      jointDesiredOutputList.completeWith(inverseKinematicsSolver.getOutput());
      if (rootJointDesiredConfigurationData != null)
         rootJointDesiredConfigurationData.completeWith(inverseKinematicsSolver.getOutputForRootJoint());
   }

   private void doVirtualModelControl()
   {
      controllerCoreSubmissionTimer.startMeasurement();
      virtualModelControlSolver.submitVirtualModelControlCommandList(internalCommandInput.getVirtualModelControlCommandList());
      controllerCoreSubmissionTimer.stopMeasurement();
      virtualModelControlSolver.compute();

      jointDesiredOutputList.completeWith(virtualModelControlSolver.getOutput());
      if (rootJointDesiredConfigurationData != null)
         rootJointDesiredConfigurationData.completeWith(virtualModelControlSolver.getOutputForRootJoint());
      controllerCoreOutput.setLinearMomentumRate(virtualModelControlSolver.getAchievedMomentumRateLinear());
      controllerCoreOutput.setAngularMomentumRate(virtualModelControlSolver.getAchievedMomentumRateAngular());
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

   public FeedbackControllerDataHolderReadOnly getWholeBodyFeedbackControllerDataHolder()
   {
      return feedbackController.getWholeBodyFeedbackControllerDataHolder();
   }

   @Override
   public YoGraphicDefinition getSCS2YoGraphics()
   {
      YoGraphicGroupDefinition group = new YoGraphicGroupDefinition(getClass().getSimpleName());
      group.addChild(toolbox.getSCS2YoGraphics());
      group.addChild(feedbackController.getSCS2YoGraphics());
      if (inverseDynamicsSolver != null)
         group.addChild(inverseDynamicsSolver.getSCS2YoGraphics());
      if (inverseKinematicsSolver != null)
         group.addChild(inverseKinematicsSolver.getSCS2YoGraphics());
      if (virtualModelControlSolver != null)
         group.addChild(virtualModelControlSolver.getSCS2YoGraphics());
      return group;
   }
}
